#include "bool_problem.h"
#include "algorithm/bsp.h"
#include "algorithm/WNV_tracing.h"
#include "core/logging.h"

#include <algorithm>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemSolveScope = "BoolProblem::solve";
        constexpr const char *kBoolProblemRootReferenceScope = "BoolProblem::initializeRootReference";
        constexpr const char *kBoolProblemSolveRecursiveScope = "BoolProblem::solveRecursive";
        constexpr const char *kBoolProblemLeafScope = "BoolProblem::solveLeafArrangement";
        constexpr const char *kBoolProblemClassifyScope = "BoolProblem::classifyLeafFragmentsAndCollectResults";
        constexpr const char *kBoolProblemChildrenScope = "BoolProblem::createChildrenFromSplit";
        constexpr const char *kBoolProblemChildReferenceScope = "BoolProblem::makeChildReference";

        template <typename Builder>
        void logBoolInfo(const char *scope, Builder &&builder)
        {
            emitLogLazy(LogLevel::Info, LogCategory::BoolProblem, scope, std::forward<Builder>(builder));
        }

        template <typename Builder>
        void logBoolDebug(const char *scope, Builder &&builder)
        {
            emitLogLazy(LogLevel::Debug, LogCategory::BoolProblem, scope, std::forward<Builder>(builder));
        }

        template <typename Builder>
        void logTracingDebug(const char *scope, Builder &&builder)
        {
            emitLogLazy(LogLevel::Debug, LogCategory::Tracing, scope, std::forward<Builder>(builder));
        }

        void logBoolError(const char *scope, const std::string &message)
        {
            emitLog(LogLevel::Error, LogCategory::BoolProblem, scope, message);
        }

        std::size_t computeWNVSize(const std::vector<Polygon256> &polygons) noexcept
        {
            std::size_t dimension = 0;
            for (const Polygon256 &polygon : polygons)
            {
                dimension = std::max(dimension, polygon.WNTV.size());
            }

            return dimension;
        }

        const char *traceStatusName(traceStatus status) noexcept
        {
            switch (status)
            {
            case SUCCESS:
                return "SUCCESS";
            case PATH_INVALID:
                return "PATH_INVALID";
            case INPUT_INVALID:
                return "INPUT_INVALID";
            case FAIL:
                return "FAIL";
            }

            return "UNKNOWN";
        }

        const char *boolOpName(BoolOp op) noexcept
        {
            switch (op)
            {
            case BoolOp::Union:
                return "union";
            case BoolOp::Intersection:
                return "intersection";
            case BoolOp::Difference:
                return "difference";
            }

            return "unknown";
        }

        std::string formatAABB(const AABB3i &box)
        {
            std::ostringstream message;
            message << "{valid=" << box.valid
                    << ", x=[" << box.xMin << ", " << box.xMax
                    << "], y=[" << box.yMin << ", " << box.yMax
                    << "], z=[" << box.zMin << ", " << box.zMax
                    << "]}";
            return message.str();
        }
    }

    BoolProblem::BoolProblem(std::size_t leafPolygonThreshold) noexcept
        : leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold)
    {
    }

    void BoolProblem::clear() noexcept
    {
        polygons_.clear();
        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setOperation(BoolOp op) noexcept
    {
        if (op_ == op)
        {
            return;
        }

        op_ = op;
        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setLeafPolygonThreshold(std::size_t threshold) noexcept
    {
        const std::size_t sanitized = (threshold == 0) ? 1 : threshold;
        if (leafPolygonThreshold_ == sanitized)
        {
            return;
        }

        leafPolygonThreshold_ = sanitized;
        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::addPolygon(const Polygon256 &polygon)
    {
        Polygon256 copy = polygon;
        copy.WNVF.clear();
        copy.WNVB.clear();
        polygons_.push_back(std::move(copy));

        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setPolygons(const std::vector<Polygon256> &polygons)
    {
        polygons_ = polygons;
        clearClassificationState(polygons_);

        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs)
    {
        std::vector<Polygon256> lhsCopy = lhs;
        std::vector<Polygon256> rhsCopy = rhs;

        assignOperandWNTV(lhsCopy, 2, 0);
        assignOperandWNTV(rhsCopy, 2, 1);

        polygons_.clear();
        polygons_.reserve(lhsCopy.size() + rhsCopy.size());
        polygons_.insert(polygons_.end(), lhsCopy.begin(), lhsCopy.end());
        polygons_.insert(polygons_.end(), rhsCopy.begin(), rhsCopy.end());

        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::solve()
    {
        depth_ = 0;
        resetSubdivisionState();

        logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                std::ostringstream message;
                message << "Starting solve operation=" << boolOpName(op_)
                        << " polygon_count=" << polygons_.size()
                        << " leaf_threshold=" << leafPolygonThreshold_
                        << ".";
                return message.str();
            });

        if (polygons_.empty())
        {
            discarded_ = true;
            solved_ = true;
            emitLog(
                LogLevel::Info,
                LogCategory::BoolProblem,
                kBoolProblemSolveScope,
                "Solve ended early because the input polygon soup is empty.");
            return;
        }

        aabb_ = computeAABB(polygons_);
        if (!isValidAABB(aabb_))
        {
            discarded_ = true;
            solved_ = true;
            logBoolInfo(
                kBoolProblemSolveScope,
                [this]()
                {
                    return "Solve ended early because root AABB is invalid root_aabb=" + formatAABB(aabb_) + ".";
                });
            return;
        }

        logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                return "Computed valid root AABB root_aabb=" + formatAABB(aabb_) + ".";
            });

        initializeRootReference();
        solveRecursive();

        logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                std::ostringstream message;
                message << "Solve finished discarded=" << discarded_
                        << " result_fragments=" << resultFragments_.size()
                        << ".";
                return message.str();
            });
    }

    bool BoolProblem::isLeaf() const noexcept
    {
        return isLeaf_;
    }

    bool BoolProblem::isDiscarded() const noexcept
    {
        return discarded_;
    }

    bool BoolProblem::isSolved() const noexcept
    {
        return solved_;
    }

    std::size_t BoolProblem::polygonCount() const noexcept
    {
        return polygons_.size();
    }

    std::size_t BoolProblem::depth() const noexcept
    {
        return depth_;
    }

    const AABB3i &BoolProblem::aabb() const noexcept
    {
        return aabb_;
    }

    const Plane3i &BoolProblem::splitPlane() const noexcept
    {
        return splitPlane_;
    }

    const SubdivisionRefState &BoolProblem::reference() const noexcept
    {
        return reference_;
    }

    const std::vector<Polygon256> &BoolProblem::polygons() const noexcept
    {
        return polygons_;
    }

    const std::vector<Polygon256> &BoolProblem::leafFragments() const noexcept
    {
        return leafFragments_;
    }

    const std::vector<Polygon256> &BoolProblem::resultFragments() const noexcept
    {
        return resultFragments_;
    }

    const BoolProblem *BoolProblem::leftChild() const noexcept
    {
        return leftChild_.get();
    }

    const BoolProblem *BoolProblem::rightChild() const noexcept
    {
        return rightChild_.get();
    }

    BoolProblem *BoolProblem::leftChild() noexcept
    {
        return leftChild_.get();
    }

    BoolProblem *BoolProblem::rightChild() noexcept
    {
        return rightChild_.get();
    }

    void BoolProblem::collectLeafProblems(std::vector<const BoolProblem *> &outLeaves) const
    {
        collectLeafProblemsRecursive(this, outLeaves);
    }

    void BoolProblem::collectLeafProblems(std::vector<BoolProblem *> &outLeaves)
    {
        collectLeafProblemsRecursive(this, outLeaves);
    }

    // 重置由求解流程派生出的状态，保留调用方配置和输入 polygon soup。
    void BoolProblem::resetSubdivisionState() noexcept
    {
        isLeaf_ = true;
        discarded_ = false;
        solved_ = false;
        splitPlane_ = Plane3i();
        aabb_ = AABB3i();
        reference_ = SubdivisionRefState();
        leafFragments_.clear();
        resultFragments_.clear();
        leftChild_.reset();
        rightChild_.reset();
    }

    // 使用根 AABB 的最小角点建立初始参考点，并按输入 WNTV 维度置零。
    void BoolProblem::initializeRootReference()
    {
        reference_.point = getAABBCornerPoint(aabb_, false, false, false);
        reference_.wnv.assign(computeWNVSize(polygons_), 0);

        logBoolDebug(
            kBoolProblemRootReferenceScope,
            [this]()
            {
                std::ostringstream message;
                message << "Initialized root reference depth=" << depth_
                        << " wnv_dimension=" << reference_.wnv.size()
                        << " point=" << reference_.point
                        << ".";
                return message.str();
            });
    }

    // 递归推进 subdivision；到达叶子后立即执行局部 BSP 和 WNV 分类。
    void BoolProblem::solveRecursive()
    {
        if (polygons_.empty() || !isValidAABB(aabb_))
        {
            discarded_ = true;
            isLeaf_ = true;
            solved_ = true;
            logBoolInfo(
                kBoolProblemSolveRecursiveScope,
                [this]()
                {
                    std::ostringstream message;
                    message << "Discarded node depth=" << depth_
                            << " polygon_count=" << polygons_.size()
                            << " aabb=" << formatAABB(aabb_)
                            << ".";
                    return message.str();
                });
            return;
        }

        const bool thresholdReached = polygons_.size() <= leafPolygonThreshold_;
        const bool aabbSplittable = hasSplittableAxis(aabb_);
        if (thresholdReached || !aabbSplittable)
        {
            isLeaf_ = true;
            logBoolInfo(
                kBoolProblemSolveRecursiveScope,
                [this, thresholdReached]()
                {
                    std::ostringstream message;
                    message << "Stopping subdivision depth=" << depth_
                            << " polygon_count=" << polygons_.size()
                            << " reason=" << (thresholdReached ? "leaf_threshold" : "aabb_not_splittable")
                            << ".";
                    return message.str();
                });
            solveLeafArrangement();
            classifyLeafFragmentsAndCollectResults();
            solved_ = true;
            return;
        }

        AABBSplit3i split;
        if (!splitAABBAtMidpoint(aabb_, split))
        {
            isLeaf_ = true;
            logBoolInfo(
                kBoolProblemSolveRecursiveScope,
                [this]()
                {
                    std::ostringstream message;
                    message << "Stopping subdivision depth=" << depth_
                            << " polygon_count=" << polygons_.size()
                            << " reason=split_aabb_failed.";
                    return message.str();
                });
            solveLeafArrangement();
            classifyLeafFragmentsAndCollectResults();
            solved_ = true;
            return;
        }

        logBoolDebug(
            kBoolProblemSolveRecursiveScope,
            [this, &split]()
            {
                std::ostringstream message;
                message << "Split node depth=" << depth_
                        << " polygon_count=" << polygons_.size()
                        << " split_plane=" << split.splitPlane
                        << " left_aabb=" << formatAABB(split.left)
                        << " right_aabb=" << formatAABB(split.right)
                        << ".";
                return message.str();
            });

        splitPlane_ = split.splitPlane;
        if (!createChildrenFromSplit(split))
        {
            splitPlane_ = Plane3i();
            isLeaf_ = true;
            logBoolInfo(
                kBoolProblemSolveRecursiveScope,
                [this]()
                {
                    std::ostringstream message;
                    message << "Stopping subdivision depth=" << depth_
                            << " polygon_count=" << polygons_.size()
                            << " reason=child_creation_failed.";
                    return message.str();
                });
            solveLeafArrangement();
            classifyLeafFragmentsAndCollectResults();
            solved_ = true;
            return;
        }

        isLeaf_ = false;

        if (leftChild_)
        {
            leftChild_->solveRecursive();
        }
        if (rightChild_)
        {
            rightChild_->solveRecursive();
        }

        resultFragments_.clear();
        if (leftChild_ && !leftChild_->discarded_)
        {
            resultFragments_.insert(
                resultFragments_.end(),
                leftChild_->resultFragments_.begin(),
                leftChild_->resultFragments_.end());
        }
        if (rightChild_ && !rightChild_->discarded_)
        {
            resultFragments_.insert(
                resultFragments_.end(),
                rightChild_->resultFragments_.begin(),
                rightChild_->resultFragments_.end());
        }

        discarded_ =
            (!leftChild_ || leftChild_->discarded_) &&
            (!rightChild_ || rightChild_->discarded_);
        solved_ = true;

        logBoolDebug(
            kBoolProblemSolveRecursiveScope,
            [this]()
            {
                std::ostringstream message;
                message << "Merged child results depth=" << depth_
                        << " result_fragments=" << resultFragments_.size()
                        << " discarded=" << discarded_
                        << ".";
                return message.str();
            });
    }

    // 对叶子节点内的每个 polygon 建立局部 BSP，并收集启用的 fragment。
    void BoolProblem::solveLeafArrangement()
    {
        leafFragments_.clear();
        if (discarded_ || polygons_.empty())
        {
            return;
        }

        const std::size_t polygonCount = polygons_.size();
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            BSPTree tree;
            tree.setBasePolygon(polygons_[i], i);

            for (std::size_t j = 0; j < polygonCount; ++j)
            {
                if (i == j)
                {
                    continue;
                }

                tree.insert(polygons_[j], j);
            }

            std::vector<Polygon256> localFragments = tree.collectLeafGeometries();
            clearClassificationState(localFragments);
            leafFragments_.insert(
                leafFragments_.end(),
                std::make_move_iterator(localFragments.begin()),
                std::make_move_iterator(localFragments.end()));
        }

        logBoolDebug(
            kBoolProblemLeafScope,
            [this]()
            {
                std::ostringstream message;
                message << "Leaf arrangement finished depth=" << depth_
                        << " input_polygons=" << polygons_.size()
                        << " leaf_fragments=" << leafFragments_.size()
                        << ".";
                return message.str();
            });
    }

    // 对每个 leaf fragment 追踪到严格内部点；分类失败属于不可恢复错误。
    void BoolProblem::classifyLeafFragmentsAndCollectResults()
    {
        resultFragments_.clear();
        if (discarded_ || leafFragments_.empty())
        {
            return;
        }

        const refPoint localReference(reference_.point, reference_.wnv);
        for (std::size_t fragmentIndex = 0; fragmentIndex < leafFragments_.size(); ++fragmentIndex)
        {
            Polygon256 &fragment = leafFragments_[fragmentIndex];
            const std::vector<LeafClassificationPathCandidate> candidates =
                enumerateLeafClassificationPathCandidates(reference_.point, fragment, aabb_);

            bool classified = false;
            traceStatus lastStatus = FAIL;
            for (std::size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex)
            {
                const LeafClassificationPathCandidate &candidate = candidates[candidateIndex];
                WNV frontWNV;
                WNV backWNV;
                const traceStatus status = tracePathWNVToSurfacePoint(
                    localReference,
                    candidate.path,
                    polygons_,
                    fragment.plane,
                    frontWNV,
                    backWNV);

                logTracingDebug(
                    kBoolProblemClassifyScope,
                    [fragmentIndex, candidateIndex, &candidate, status]()
                    {
                        std::ostringstream message;
                        message << "Leaf fragment trace attempt fragment_index=" << fragmentIndex
                                << " candidate_index=" << candidateIndex
                                << " path_segments=" << candidate.path.size()
                                << " status=" << traceStatusName(status)
                                << ".";
                        return message.str();
                    });

                if (status == SUCCESS)
                {
                    fragment.WNVF = std::move(frontWNV);
                    fragment.WNVB = std::move(backWNV);
                    classified = true;
                    break;
                }

                lastStatus = status;
                if (status != PATH_INVALID)
                {
                    break;
                }
            }

            if (!classified)
            {
                std::ostringstream message;
                message << "BoolProblem failed to classify leaf fragment "
                        << fragmentIndex
                        << " after "
                        << candidates.size()
                        << " candidates; last trace status = "
                        << traceStatusName(lastStatus)
                        << ".";
                logBoolError(kBoolProblemClassifyScope, message.str());
                throw std::runtime_error(message.str());
            }

            const BoolStatus frontStatus = evaluateBooleanIndicator(fragment.WNVF);
            const BoolStatus backStatus = evaluateBooleanIndicator(fragment.WNVB);
            if (frontStatus == OUT && backStatus == IN)
            {
                resultFragments_.push_back(fragment);
            }
            else if (frontStatus == IN && backStatus == OUT)
            {
                resultFragments_.push_back(reversePolygonOrientation(fragment));
            }
        }

        logBoolDebug(
            kBoolProblemClassifyScope,
            [this]()
            {
                std::ostringstream message;
                message << "Leaf classification finished depth=" << depth_
                        << " result_fragments=" << resultFragments_.size()
                        << ".";
                return message.str();
            });
    }

    // 叶子阈值和 AABB 可切分性共同决定当前节点是否停止递归。
    bool BoolProblem::shouldStopSubdivision() const noexcept
    {
        return polygons_.size() <= leafPolygonThreshold_ || !hasSplittableAxis(aabb_);
    }

    // 裁剪当前 polygon soup 到左右子 AABB，并为每个非空子问题建立参考状态。
    bool BoolProblem::createChildrenFromSplit(const AABBSplit3i &split)
    {
        std::vector<Polygon256> leftPolygons;
        std::vector<Polygon256> rightPolygons;
        leftPolygons.reserve(polygons_.size());
        rightPolygons.reserve(polygons_.size());

        for (const Polygon256 &polygon : polygons_)
        {
            Polygon256 leftPolygon;
            if (detail::clipPolygonToHalfSpace(polygon, split.splitPlane, true, leftPolygon))
            {
                leftPolygons.push_back(std::move(leftPolygon));
            }

            Polygon256 rightPolygon;
            if (detail::clipPolygonToHalfSpace(polygon, split.splitPlane, false, rightPolygon))
            {
                rightPolygons.push_back(std::move(rightPolygon));
            }
        }

        if (leftPolygons.empty() && rightPolygons.empty())
        {
            emitLog(
                LogLevel::Debug,
                LogCategory::BoolProblem,
                kBoolProblemChildrenScope,
                "Split produced no child polygons.");
            return false;
        }

        logBoolDebug(
            kBoolProblemChildrenScope,
            [this, &leftPolygons, &rightPolygons]()
            {
                std::ostringstream message;
                message << "Prepared child polygon soups depth=" << depth_
                        << " left_polygons=" << leftPolygons.size()
                        << " right_polygons=" << rightPolygons.size()
                        << ".";
                return message.str();
            });

        SubdivisionRefState leftReference;
        SubdivisionRefState rightReference;
        if (!leftPolygons.empty() && !makeChildReference(split.left, leftPolygons, leftReference))
        {
            return false;
        }
        if (!rightPolygons.empty() && !makeChildReference(split.right, rightPolygons, rightReference))
        {
            return false;
        }

        if (!leftPolygons.empty())
        {
            leftChild_ = std::make_unique<BoolProblem>(leafPolygonThreshold_);
            leftChild_->op_ = op_;
            leftChild_->depth_ = depth_ + 1;
            leftChild_->polygons_ = std::move(leftPolygons);
            leftChild_->aabb_ = split.left;
            leftChild_->reference_ = std::move(leftReference);
        }

        if (!rightPolygons.empty())
        {
            rightChild_ = std::make_unique<BoolProblem>(leafPolygonThreshold_);
            rightChild_->op_ = op_;
            rightChild_->depth_ = depth_ + 1;
            rightChild_->polygons_ = std::move(rightPolygons);
            rightChild_->aabb_ = split.right;
            rightChild_->reference_ = std::move(rightReference);
        }

        logBoolDebug(
            kBoolProblemChildrenScope,
            [this]()
            {
                std::ostringstream message;
                message << "Created child nodes depth=" << depth_
                        << " has_left=" << (leftChild_ != nullptr)
                        << " has_right=" << (rightChild_ != nullptr)
                        << ".";
                return message.str();
            });

        return true;
    }

    // 优先复用仍在子 AABB 内且不落在表面上的参考点，否则枚举 AABB 路径传播 WNV。
    bool BoolProblem::makeChildReference(
        const AABB3i &childBox,
        const std::vector<Polygon256> &childPolygons,
        SubdivisionRefState &outReference) const
    {
        if (isPointInsideOrOnAABB(reference_.point, childBox))
        {
            bool onSurface = false;
            for (const Polygon256 &polygon : childPolygons)
            {
                if (polygon.classify(reference_.point) == 0)
                {
                    onSurface = true;
                    break;
                }
            }

            if (!onSurface)
            {
                outReference = reference_;
                logTracingDebug(
                    kBoolProblemChildReferenceScope,
                    [this, &childBox]()
                    {
                        std::ostringstream message;
                        message << "Reused reference depth=" << depth_
                                << " child_aabb=" << formatAABB(childBox)
                                << ".";
                        return message.str();
                    });
                return true;
            }
        }

        const refPoint sourceRef(reference_.point, reference_.wnv);
        const std::vector<AABBPathCandidate> candidates = enumerateAABBPathCandidates(reference_.point, childBox);
        for (std::size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex)
        {
            const AABBPathCandidate &candidate = candidates[candidateIndex];
            bool onSurface = false;
            for (const Polygon256 &polygon : childPolygons)
            {
                if (polygon.classify(candidate.targetPoint) == 0)
                {
                    onSurface = true;
                    break;
                }
            }
            if (onSurface)
            {
                logTracingDebug(
                    kBoolProblemChildReferenceScope,
                    [this, candidateIndex, &candidate]()
                    {
                        std::ostringstream message;
                        message << "Skipped child reference candidate depth=" << depth_
                                << " candidate_index=" << candidateIndex
                                << " path_segments=" << candidate.path.size()
                                << " reason=target_on_surface.";
                        return message.str();
                    });
                continue;
            }

            WNV propagatedWNV;
            const traceStatus status = tracePathWNV(sourceRef, candidate.path, polygons_, propagatedWNV);
            logTracingDebug(
                kBoolProblemChildReferenceScope,
                [this, candidateIndex, &candidate, status]()
                {
                    std::ostringstream message;
                    message << "Child reference trace depth=" << depth_
                            << " candidate_index=" << candidateIndex
                            << " path_segments=" << candidate.path.size()
                            << " status=" << traceStatusName(status)
                            << ".";
                    return message.str();
                });
            if (status == SUCCESS)
            {
                outReference.point = candidate.targetPoint;
                outReference.wnv = std::move(propagatedWNV);
                return true;
            }

            if (status != PATH_INVALID)
            {
                break;
            }
        }

        outReference = SubdivisionRefState();
        logTracingDebug(
            kBoolProblemChildReferenceScope,
            [this, &childBox, &candidates]()
            {
                std::ostringstream message;
                message << "Failed to propagate child reference depth=" << depth_
                        << " child_aabb=" << formatAABB(childBox)
                        << " candidate_count=" << candidates.size()
                        << ".";
                return message.str();
            });
        return false;
    }

    // 将 WNV 交给当前布尔运算的二元指示函数，返回内外状态。
    BoolStatus BoolProblem::evaluateBooleanIndicator(const WNV &wnv) const noexcept
    {
        WNV tmp = wnv;
        switch (op_)
        {
        case BoolOp::Union:
            return f_union(tmp, 0, 1);
        case BoolOp::Intersection:
            return f_intersection(tmp, 0, 1);
        case BoolOp::Difference:
            return f_diff(tmp, 0, 1);
        }

        return OUT;
    }

    // 写入基础 WNTV，并清理可能来自上游的前后侧分类缓存。
    void BoolProblem::assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t dimension, std::size_t hotIndex)
    {
        for (Polygon256 &polygon : polygons)
        {
            polygon.WNTV.assign(dimension, 0);
            if (hotIndex < dimension)
            {
                polygon.WNTV[hotIndex] = 1;
            }

            polygon.WNVF.clear();
            polygon.WNVB.clear();
        }
    }

    // 清理 polygon 集合上的 WNVF/WNVB，使后续分类不会读取陈旧结果。
    void BoolProblem::clearClassificationState(std::vector<Polygon256> &polygons)
    {
        for (Polygon256 &polygon : polygons)
        {
            polygon.WNVF.clear();
            polygon.WNVB.clear();
        }
    }

    // 只读遍历版本：跳过空节点和已丢弃节点，只返回有效叶子。
    void BoolProblem::collectLeafProblemsRecursive(const BoolProblem *node, std::vector<const BoolProblem *> &outLeaves)
    {
        if (!node || node->discarded_)
        {
            return;
        }

        if (node->isLeaf_)
        {
            outLeaves.push_back(node);
            return;
        }

        collectLeafProblemsRecursive(node->leftChild_.get(), outLeaves);
        collectLeafProblemsRecursive(node->rightChild_.get(), outLeaves);
    }

    // 可变遍历版本：与只读版本保持相同过滤规则。
    void BoolProblem::collectLeafProblemsRecursive(BoolProblem *node, std::vector<BoolProblem *> &outLeaves)
    {
        if (!node || node->discarded_)
        {
            return;
        }

        if (node->isLeaf_)
        {
            outLeaves.push_back(node);
            return;
        }

        collectLeafProblemsRecursive(node->leftChild_.get(), outLeaves);
        collectLeafProblemsRecursive(node->rightChild_.get(), outLeaves);
    }
}
