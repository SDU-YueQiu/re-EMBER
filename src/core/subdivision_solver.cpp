/**
 * @file subdivision_solver.cpp
 * @brief 实现递归空间细分与子节点参考点传播。
 */
#include "core/subdivision_solver.h"

#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/logging.h"
#include "geometry/polygon_ops.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemSolveScope = "SubdivisionSolver::solve";
        constexpr const char *kBoolProblemRootReferenceScope = "SubdivisionSolver::initializeRootReference";
        constexpr const char *kBoolProblemSolveRecursiveScope = "SubdivisionSolver::solveRecursive";
        constexpr const char *kBoolProblemChildrenScope = "SubdivisionSolver::createChildrenFromSplit";
        constexpr const char *kBoolProblemChildReferenceScope = "SubdivisionSolver::makeChildReference";

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

        std::size_t computeWNVSize(const std::vector<Polygon256> &polygons) noexcept
        {
            std::size_t dimension = 0;
            for (const Polygon256 &polygon : polygons)
            {
                dimension = std::max(dimension, polygon.WNTV.size());
            }

            return dimension;
        }

        bool isPointStrictlyInsideAABB(const PlanePoint3i &point, const AABB3i &box) noexcept
        {
            Integer x;
            Integer y;
            Integer z;
            return detail::tryExtractExactIntegerPoint(point, x, y, z) &&
                   x > box.xMin && x < box.xMax &&
                   y > box.yMin && y < box.yMax &&
                   z > box.zMin && z < box.zMax;
        }
    }

    SubdivisionSolver::SubdivisionSolver(
        BoolOp op,
        std::size_t leafPolygonThreshold,
        const std::vector<Polygon256> &polygons)
        : op_(op),
          leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold),
          polygons_(polygons)
    {
    }

    SubdivisionSolver::SubdivisionSolver(
        BoolOp op,
        std::size_t leafPolygonThreshold,
        std::size_t depth,
        std::vector<Polygon256> polygons,
        const AABB3i &aabb,
        SubdivisionRefState reference)
        : op_(op),
          leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold),
          depth_(depth),
          aabb_(aabb),
          reference_(std::move(reference)),
          polygons_(std::move(polygons))
    {
    }

    void SubdivisionSolver::solve()
    {
        resetSolveState();
        if (polygons_.empty())
        {
            discarded_ = true;
            solved_ = true;
            return;
        }

        aabb_ = computeAABB(polygons_);
        if (!isValidAABB(aabb_))
        {
            std::ostringstream message;
            message << "SubdivisionSolver failed to compute a valid root AABB root_aabb="
                    << formatAABB(aabb_)
                    << ".";
            emitLog(LogLevel::Error, LogCategory::BoolProblem, kBoolProblemSolveScope, message.str());
            throw std::runtime_error(message.str());
        }

        logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                return "Computed valid root AABB root_aabb=" + formatAABB(aabb_) + ".";
            });

        initializeRootReference();
        solveRecursive();
        leafSummaries_.clear();
        collectLeafSummaries(leafSummaries_);
    }

    bool SubdivisionSolver::isDiscarded() const noexcept
    {
        return discarded_;
    }

    const std::vector<Polygon256> &SubdivisionSolver::resultFragments() const noexcept
    {
        return resultFragments_;
    }

    const std::vector<BoolLeafSummary> &SubdivisionSolver::leafSummaries() const noexcept
    {
        return leafSummaries_;
    }

    void SubdivisionSolver::resetSolveState() noexcept
    {
        depth_ = 0;
        isLeaf_ = true;
        discarded_ = false;
        solved_ = false;
        splitPlane_ = Plane3i();
        aabb_ = AABB3i();
        reference_ = SubdivisionRefState();
        leafFragments_.clear();
        classifiedFragments_.clear();
        resultFragments_.clear();
        leafSummaries_.clear();
        leftChild_.reset();
        rightChild_.reset();
    }

    void SubdivisionSolver::initializeRootReference()
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
    void SubdivisionSolver::solveRecursive()
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

        if (shouldStopSubdivision())
        {
            isLeaf_ = true;
            logBoolInfo(
                kBoolProblemSolveRecursiveScope,
                [this]()
                {
                    std::ostringstream message;
                    message << "Stopping subdivision depth=" << depth_
                            << " polygon_count=" << polygons_.size()
                            << " reason=" << (polygons_.size() <= leafPolygonThreshold_ ? "leaf_threshold" : "aabb_not_splittable")
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
            std::ostringstream message;
            message << "Failed to create child subdivision references depth=" << depth_
                    << " polygon_count=" << polygons_.size()
                    << ".";
            logBoolError(kBoolProblemSolveRecursiveScope, message.str());
            throw std::runtime_error(message.str());
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

    // 叶子阈值和 AABB 可切分性共同决定当前节点是否停止递归。
    bool SubdivisionSolver::shouldStopSubdivision() const noexcept
    {
        return polygons_.size() <= leafPolygonThreshold_ || !hasSplittableAxis(aabb_);
    }

    // 裁剪当前多边形集合到左右子 AABB，并为每个非空子问题建立参考状态。
    bool SubdivisionSolver::createChildrenFromSplit(const AABBSplit3i &split)
    {
        std::vector<Polygon256> leftPolygons;
        std::vector<Polygon256> rightPolygons;
        leftPolygons.reserve(polygons_.size());
        rightPolygons.reserve(polygons_.size());

        for (const Polygon256 &polygon : polygons_)
        {
            Polygon256 leftPolygon;
            if (detail::clipPolygonToHalfSpace(
                    polygon,
                    split.splitPlane,
                    true,
                    leftPolygon,
                    PolygonEdgeProvenance::SubdivisionClip))
            {
                leftPolygons.push_back(std::move(leftPolygon));
            }

            Polygon256 rightPolygon;
            if (detail::clipPolygonToHalfSpace(
                    polygon,
                    split.splitPlane,
                    false,
                    rightPolygon,
                    PolygonEdgeProvenance::SubdivisionClip))
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
            std::ostringstream message;
            message << "Failed to propagate left child reference depth=" << depth_
                    << " candidate_polygons=" << leftPolygons.size()
                    << ".";
            throw std::runtime_error(message.str());
        }
        if (!rightPolygons.empty() && !makeChildReference(split.right, rightPolygons, rightReference))
        {
            std::ostringstream message;
            message << "Failed to propagate right child reference depth=" << depth_
                    << " candidate_polygons=" << rightPolygons.size()
                    << ".";
            throw std::runtime_error(message.str());
        }

        if (!leftPolygons.empty())
        {
            leftChild_.reset(new SubdivisionSolver(
                op_,
                leafPolygonThreshold_,
                depth_ + 1,
                std::move(leftPolygons),
                split.left,
                std::move(leftReference)));
        }

        if (!rightPolygons.empty())
        {
            rightChild_.reset(new SubdivisionSolver(
                op_,
                leafPolygonThreshold_,
                depth_ + 1,
                std::move(rightPolygons),
                split.right,
                std::move(rightReference)));
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
    bool SubdivisionSolver::makeChildReference(
        const AABB3i &childBox,
        const std::vector<Polygon256> &childPolygons,
        SubdivisionRefState &outReference) const
    {
        const bool sourceReferenceIsStrictInterior = isPointStrictlyInsideAABB(reference_.point, childBox);
        if (sourceReferenceIsStrictInterior)
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
            if (!sourceReferenceIsStrictInterior &&
                candidate.path.empty() &&
                areSamePlanePoint(candidate.targetPoint, reference_.point))
            {
                bool onSupportPlane = false;
                for (const Polygon256 &polygon : childPolygons)
                {
                    if (candidate.targetPoint.classify(polygon.plane) == 0)
                    {
                        onSupportPlane = true;
                        break;
                    }
                }

                if (onSupportPlane)
                {
                    logTracingDebug(
                        kBoolProblemChildReferenceScope,
                        [this, candidateIndex]()
                        {
                            std::ostringstream message;
                            message << "Skipped child reference candidate depth=" << depth_
                                    << " candidate_index=" << candidateIndex
                                    << " path_segments=0 reason=source_on_child_surface_plane.";
                            return message.str();
                        });
                    continue;
                }
            }

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
            const traceStatus status = detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                sourceRef,
                candidate.path,
                polygons_,
                propagatedWNV);
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

    void SubdivisionSolver::collectLeafSummaries(std::vector<BoolLeafSummary> &outSummaries) const
    {
        if (discarded_)
        {
            return;
        }

        if (isLeaf_)
        {
            outSummaries.push_back(BoolLeafSummary{depth_, polygons_.size(), aabb_, discarded_});
            return;
        }

        if (leftChild_)
        {
            leftChild_->collectLeafSummaries(outSummaries);
        }
        if (rightChild_)
        {
            rightChild_->collectLeafSummaries(outSummaries);
        }
    }
}
