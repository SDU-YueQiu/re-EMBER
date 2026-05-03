#include "core/bool_problem.h"

#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/logging.h"
#include "geometry/polygon_ops.h"

#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemSolveRecursiveScope = "BoolProblem::solveRecursive";
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
}
