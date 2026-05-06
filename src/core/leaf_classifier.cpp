/**
 * @file leaf_classifier.cpp
 * @brief 实现细分求解器的叶片编排分类。
 */
#include "core/subdivision_solver.h"

#include "algorithm/leaf_arrangement.h"
#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/logging.h"
#include "core/perf_tracing.h"
#include "core/solver_shared.h"
#include "geometry/polygon_ops.h"

#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemLeafScope = "SubdivisionSolver::solveLeafArrangement";
        constexpr const char *kBoolProblemClassifyScope = "SubdivisionSolver::classifyLeafFragmentsAndCollectResults";

        bool isSameStrictSideForDiagnostics(int lhs, int rhs) noexcept
        {
            return lhs == rhs && (lhs == -1 || lhs == 1);
        }

        std::string describeSurfaceTraceFailure(
            const refPoint &refpoint,
            const Path &path,
            const std::vector<Polygon256> &polygons,
            const Plane3i &referencePlane)
        {
            if (path.empty())
            {
                return "empty_path";
            }
            if (!areSamePlanePoint(path.front().getStartPoint(), refpoint.point))
            {
                return "path_start_mismatch";
            }

            const PlanePoint3i targetPoint = path.back().getEndPoint();
            if (!targetPoint.hasUniqueIntersection() || targetPoint.classify(referencePlane) != 0)
            {
                return "target_not_on_reference_plane";
            }

            for (std::size_t polygonIndex = 0; polygonIndex < polygons.size(); ++polygonIndex)
            {
                const Polygon256 &poly = polygons[polygonIndex];
                PlanePoint3i startPoint = path[0].getStartPoint();
                int pcs = poly.classify(startPoint);
                if (pcs == 0)
                {
                    std::ostringstream message;
                    message << "start_on_surface polygon_index=" << polygonIndex;
                    return message.str();
                }

                for (std::size_t segmentIndex = 0; segmentIndex < path.size(); ++segmentIndex)
                {
                    const Segment256 &seg = path[segmentIndex];
                    const PlanePoint3i endPoint = seg.getEndPoint();
                    const int pce = poly.classify(endPoint);
                    const bool isLastSegment = (segmentIndex + 1 == path.size());

                    if (!isLastSegment && pce == 0)
                    {
                        std::ostringstream message;
                        message << "intermediate_endpoint_on_surface polygon_index=" << polygonIndex
                                << " segment_index=" << segmentIndex;
                        return message.str();
                    }
                    if (isSameStrictSideForDiagnostics(pcs, pce))
                    {
                        pcs = pce;
                        continue;
                    }

                    const PlanePoint3i intersectPoint = intersect(seg.direction, poly.plane);
                    if (intersectPoint.hasUniqueIntersection())
                    {
                        const detail::PolygonSurfaceLocation hitLocation =
                            detail::classifyPolygonSurfacePointUnchecked(poly, intersectPoint);
                        if (hitLocation == detail::PolygonSurfaceLocation::Boundary)
                        {
                            std::ostringstream message;
                            message << "boundary_hit polygon_index=" << polygonIndex
                                    << " segment_index=" << segmentIndex;
                            return message.str();
                        }

                        pcs = pce;
                        continue;
                    }

                    if (detail::isSegmentTouchPolygonEdgeUnchecked(seg, poly))
                    {
                        std::ostringstream message;
                        message << "edge_touch polygon_index=" << polygonIndex
                                << " segment_index=" << segmentIndex;
                        return message.str();
                    }

                    pcs = pce;
                }
            }

            const PlanePoint3i lastStartPoint = path.back().getStartPoint();
            if (lastStartPoint.classify(referencePlane) == 0)
            {
                return "last_start_on_reference_plane";
            }

            return "unknown_path_invalid";
        }

        struct LeafClassificationAttemptStats
        {
            std::size_t pointCandidateCount = 0;
            std::size_t normalCandidateCount = 0;
            std::size_t fastCandidateCount = 0;
            std::size_t interiorBridgeCandidateCount = 0;
            std::size_t fallbackCandidateCount = 0;
            std::size_t primaryPointCandidateCount = 0;
            std::size_t expandedPointCandidateCount = 0;
            std::string lastFailureReason = "not_collected";
            bool classified = false;
            traceStatus lastStatus = FAIL;
        };

        enum class LeafClassificationResult
        {
            Success,
            RetryPathInvalid,
            Failure
        };

        struct LeafClassificationContext
        {
            const refPoint &localReference;
            const std::vector<Polygon256> &polygons;
            const AABB3i &aabb;
            BoolSolveMetrics &solveMetrics;
            std::vector<ClassifiedFragment> &classifiedFragments;
            std::size_t depth = 0;
            bool collectFailureDiagnostics = false;
        };

        traceStatus traceLeafClassificationCandidate(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            LeafClassificationAttemptStats &attemptStats,
            const LeafClassificationPathCandidate &candidate,
            const char *candidateLayer,
            std::size_t candidateIndex)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::traceCandidate");

            ++context.solveMetrics.leafClassificationTraceAttemptCount;
            WNV frontWNV;
            WNV backWNV;
            const traceStatus status = detail::tracePathWNVToSurfacePointTrusted(
                context.localReference,
                candidate.path,
                context.polygons,
                fragment.plane,
                frontWNV,
                backWNV);
            if (status != SUCCESS && context.collectFailureDiagnostics)
            {
                attemptStats.lastFailureReason = describeSurfaceTraceFailure(
                    context.localReference,
                    candidate.path,
                    context.polygons,
                    fragment.plane);
            }

            detail::logTracingDebug(
                kBoolProblemClassifyScope,
                [fragmentIndex, candidateLayer, candidateIndex, &candidate, status, &attemptStats]()
                {
                    std::ostringstream message;
                    message << "Leaf fragment trace attempt fragment_index=" << fragmentIndex
                            << " candidate_layer=" << candidateLayer
                            << " candidate_index=" << candidateIndex
                            << " path_segments=" << candidate.path.size()
                            << " status=" << detail::traceStatusName(status);
                    if (status != SUCCESS)
                    {
                        message << " reason=" << attemptStats.lastFailureReason;
                    }
                    message << ".";
                    return message.str();
                });

            attemptStats.lastStatus = status;
            if (status == SUCCESS)
            {
                context.classifiedFragments.push_back(
                    ClassifiedFragment{fragment, std::move(frontWNV), std::move(backWNV)});
                attemptStats.classified = true;
            }

            return status;
        }

        bool attemptFastPointCandidates(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            LeafClassificationAttemptStats &attemptStats,
            const std::vector<PlanePoint3i> &pointCandidates)
        {
            bool allowFallback = true;
            enumerateLeafClassificationFastPathCandidatesFromPoints(
                context.localReference.point,
                pointCandidates,
                context.aabb,
                [&](LeafClassificationPathCandidate candidate)
                {
                    const std::size_t candidateIndex = attemptStats.fastCandidateCount;
                    ++attemptStats.fastCandidateCount;
                    ++context.solveMetrics.leafClassificationFastCandidateCount;
                    const traceStatus status = traceLeafClassificationCandidate(
                        context,
                        fragmentIndex,
                        fragment,
                        attemptStats,
                        candidate,
                        "fast",
                        candidateIndex);
                    if (status == SUCCESS)
                    {
                        return false;
                    }

                    if (status != PATH_INVALID)
                    {
                        allowFallback = false;
                        return false;
                    }

                    allowFallback = true;
                    return true;
                });
            return allowFallback;
        }

        bool attemptFallbackPointCandidates(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            LeafClassificationAttemptStats &attemptStats,
            const std::vector<PlanePoint3i> &pointCandidates)
        {
            bool allowFallback = true;
            enumerateLeafClassificationFallbackPathCandidatesFromPoints(
                context.localReference.point,
                pointCandidates,
                context.aabb,
                [&](LeafClassificationPathCandidate candidate)
                {
                    const std::size_t candidateIndex = attemptStats.fallbackCandidateCount;
                    ++attemptStats.fallbackCandidateCount;
                    ++context.solveMetrics.leafClassificationFallbackCandidateCount;
                    const traceStatus status = traceLeafClassificationCandidate(
                        context,
                        fragmentIndex,
                        fragment,
                        attemptStats,
                        candidate,
                        "fallback",
                        candidateIndex);
                    if (status == SUCCESS)
                    {
                        return false;
                    }

                    if (status != PATH_INVALID)
                    {
                        allowFallback = false;
                        return false;
                    }

                    return true;
                });
            return allowFallback;
        }

        bool attemptNormalApproachCandidates(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            LeafClassificationAttemptStats &attemptStats,
            const std::vector<PlanePoint3i> &pointCandidates)
        {
            bool allowFallback = true;
            enumerateLeafClassificationNormalApproachCandidatesFromPoints(
                context.localReference.point,
                pointCandidates,
                fragment.plane,
                context.aabb,
                [&](LeafClassificationPathCandidate candidate)
                {
                    const std::size_t candidateIndex = attemptStats.normalCandidateCount;
                    ++attemptStats.normalCandidateCount;
                    ++context.solveMetrics.leafClassificationNormalCandidateCount;
                    const traceStatus status = traceLeafClassificationCandidate(
                        context,
                        fragmentIndex,
                        fragment,
                        attemptStats,
                        candidate,
                        "normal",
                        candidateIndex);
                    if (status == SUCCESS)
                    {
                        return false;
                    }

                    if (status != PATH_INVALID)
                    {
                        allowFallback = false;
                        return false;
                    }

                    allowFallback = true;
                    return true;
                });
            return allowFallback;
        }

        bool attemptInteriorBridgeCandidates(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            LeafClassificationAttemptStats &attemptStats,
            const std::vector<PlanePoint3i> &pointCandidates)
        {
            bool allowFallback = true;
            enumerateLeafClassificationInteriorBridgeCandidatesFromPoints(
                context.localReference.point,
                pointCandidates,
                context.aabb,
                [&](LeafClassificationPathCandidate candidate)
                {
                    const std::size_t candidateIndex = attemptStats.interiorBridgeCandidateCount;
                    ++attemptStats.interiorBridgeCandidateCount;
                    ++context.solveMetrics.leafClassificationInteriorBridgeCandidateCount;
                    const traceStatus status = traceLeafClassificationCandidate(
                        context,
                        fragmentIndex,
                        fragment,
                        attemptStats,
                        candidate,
                        "interior-bridge",
                        candidateIndex);
                    if (status == SUCCESS)
                    {
                        return false;
                    }

                    if (status != PATH_INVALID)
                    {
                        allowFallback = false;
                        return false;
                    }

                    allowFallback = true;
                    return true;
                });
            return allowFallback;
        }

        bool attemptLeafClassificationPointCandidates(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            LeafClassificationAttemptStats &attemptStats,
            const std::vector<PlanePoint3i> &pointCandidates)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::attemptPointCandidates");

            attemptStats.pointCandidateCount += pointCandidates.size();
            context.solveMetrics.leafClassificationPointCandidateCount += pointCandidates.size();

            bool allowFallback = attemptFastPointCandidates(
                context,
                fragmentIndex,
                fragment,
                attemptStats,
                pointCandidates);
            if (!attemptStats.classified && allowFallback)
            {
                allowFallback = attemptFallbackPointCandidates(
                    context,
                    fragmentIndex,
                    fragment,
                    attemptStats,
                    pointCandidates);
            }
            if (!attemptStats.classified && allowFallback)
            {
                allowFallback = attemptNormalApproachCandidates(
                    context,
                    fragmentIndex,
                    fragment,
                    attemptStats,
                    pointCandidates);
            }
            if (!attemptStats.classified && allowFallback)
            {
                allowFallback = attemptInteriorBridgeCandidates(
                    context,
                    fragmentIndex,
                    fragment,
                    attemptStats,
                    pointCandidates);
            }

            return !attemptStats.classified && allowFallback;
        }

        LeafClassificationResult classifyLeafFragment(
            LeafClassificationContext &context,
            std::size_t fragmentIndex,
            Polygon256 &fragment,
            bool allowRetryFallback,
            LeafClassificationAttemptStats &attemptStats)
        {
            const std::vector<PlanePoint3i> primaryPointCandidates =
                detail::enumerateLeafClassificationPrimaryPointCandidatesUnchecked(fragment);
            attemptStats.primaryPointCandidateCount = primaryPointCandidates.size();

            const bool shouldTryExpandedPoints = attemptLeafClassificationPointCandidates(
                context,
                fragmentIndex,
                fragment,
                attemptStats,
                primaryPointCandidates);
            if (!attemptStats.classified && shouldTryExpandedPoints)
            {
                const std::vector<PlanePoint3i> expandedPointCandidates =
                    detail::enumerateLeafClassificationPointCandidatesUnchecked(fragment);
                attemptStats.expandedPointCandidateCount = expandedPointCandidates.size();
                attemptLeafClassificationPointCandidates(
                    context,
                    fragmentIndex,
                    fragment,
                    attemptStats,
                    expandedPointCandidates);
            }

            if (attemptStats.classified)
            {
                return LeafClassificationResult::Success;
            }
            if (allowRetryFallback && attemptStats.lastStatus == PATH_INVALID)
            {
                return LeafClassificationResult::RetryPathInvalid;
            }

            return LeafClassificationResult::Failure;
        }

        void logLeafClassificationFailure(
            const LeafClassificationContext &context,
            std::size_t fragmentIndex,
            const Polygon256 &fragment,
            const LeafClassificationAttemptStats &attemptStats)
        {
            detail::logBoolDebug(
                kBoolProblemClassifyScope,
                [&context, fragmentIndex, &fragment, &attemptStats]()
                {
                    std::ostringstream message;
                    message << "Leaf classification failure diagnostic fragment_index="
                            << fragmentIndex
                            << " depth="
                            << context.depth
                            << " fragment_edges="
                            << fragment.edgeCount()
                            << " target_point_candidates="
                            << attemptStats.pointCandidateCount
                            << " primary_point_candidates="
                            << attemptStats.primaryPointCandidateCount
                            << " expanded_point_candidates="
                            << attemptStats.expandedPointCandidateCount
                            << " normal_candidates="
                            << attemptStats.normalCandidateCount
                            << " fast_candidates="
                            << attemptStats.fastCandidateCount
                            << " interior_bridge_candidates="
                            << attemptStats.interiorBridgeCandidateCount
                            << " fallback_candidates="
                            << attemptStats.fallbackCandidateCount
                            << " last_trace_status="
                            << detail::traceStatusName(attemptStats.lastStatus)
                            << " last_failure_reason="
                            << attemptStats.lastFailureReason
                            << " fragment_plane="
                            << fragment.plane
                            << " aabb={x=["
                            << context.aabb.xMin
                            << ", "
                            << context.aabb.xMax
                            << "], y=["
                            << context.aabb.yMin
                            << ", "
                            << context.aabb.yMax
                            << "], z=["
                            << context.aabb.zMin
                            << ", "
                            << context.aabb.zMax
                            << "]}";
                    return message.str();
                });
        }
    }

    // 对叶子节点内的每个多边形建立局部 BSP，并收集启用的片段。
    void SubdivisionSolver::solveLeafArrangement()
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::solveLeafArrangement");

        leafFragments_.clear();
        classifiedFragments_.clear();
        if (discarded_ || polygons_.empty())
        {
            return;
        }

        BoolOperandAssumptions assumptions;
        if (tryGetSingleOperandAssumptions(assumptions) && assumptions.noSelfIntersections)
        {
            ++solveMetrics_.singleOperandLeafBspSkipCount;
            leafFragments_ = polygons_;
            detail::logBoolDebug(
                kBoolProblemLeafScope,
                [this, assumptions]()
                {
                    std::ostringstream message;
                    message << "Skipped leaf BSP by operand assumption depth=" << depth_
                            << " input_polygons=" << polygons_.size()
                            << " leaf_fragments=" << leafFragments_.size()
                            << " no_nested_components=" << assumptions.noNestedComponents
                            << ".";
                    return message.str();
                });
            return;
        }

        ++solveMetrics_.leafBspBuildCount;
        leafFragments_ = buildLeafArrangement(polygons_);

        detail::logBoolDebug(
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

    void SubdivisionSolver::appendResultFragmentFromClassification(const ClassifiedFragment &classifiedFragment)
    {
        const BoolStatus frontStatus = evaluateBooleanIndicator(classifiedFragment.frontWNV);
        const BoolStatus backStatus = evaluateBooleanIndicator(classifiedFragment.backWNV);
        if (frontStatus == OUT && backStatus == IN)
        {
            resultFragments_.push_back(classifiedFragment.polygon);
        }
        else if (frontStatus == IN && backStatus == OUT)
        {
            resultFragments_.push_back(reversePolygonOrientation(classifiedFragment.polygon));
        }
    }

    bool SubdivisionSolver::tryReuseSingleOperandFragmentClassification(
        const Polygon256 &fragment,
        bool reuseSingleOperandClassification,
        bool hasReusableClassification,
        const WNV &reusableFrontWNV,
        const WNV &reusableBackWNV)
    {
        if (!reuseSingleOperandClassification || !hasReusableClassification)
        {
            return false;
        }

        ++solveMetrics_.singleOperandClassificationReuseCount;
        classifiedFragments_.push_back(ClassifiedFragment{fragment, reusableFrontWNV, reusableBackWNV});
        appendResultFragmentFromClassification(classifiedFragments_.back());
        return true;
    }

    // 对每个叶片片段追踪到严格内部点；分类失败属于不可恢复错误。
    void SubdivisionSolver::classifyLeafFragmentsAndCollectResults()
    {
        (void)classifyLeafFragmentsAndCollectResults(false);
    }

    bool SubdivisionSolver::classifyLeafFragmentsAndCollectResults(bool allowRetryFallback)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::classifyLeafFragmentsAndCollectResults");

        resultFragments_.clear();
        classifiedFragments_.clear();
        if (discarded_ || leafFragments_.empty())
        {
            return true;
        }

        const refPoint localReference(reference_.point, reference_.wnv);
        LeafClassificationContext context{
            localReference,
            polygons_,
            aabb_,
            solveMetrics_,
            classifiedFragments_,
            depth_,
            detail::isLogEnabled(LogLevel::Debug)};
        BoolOperandAssumptions assumptions;
        const bool reuseSingleOperandClassification =
            tryGetSingleOperandAssumptions(assumptions) &&
            assumptions.noSelfIntersections &&
            assumptions.noNestedComponents;
        bool hasReusableClassification = false;
        WNV reusableFrontWNV;
        WNV reusableBackWNV;
        for (std::size_t fragmentIndex = 0; fragmentIndex < leafFragments_.size(); ++fragmentIndex)
        {
            Polygon256 &fragment = leafFragments_[fragmentIndex];
            if (tryReuseSingleOperandFragmentClassification(
                    fragment,
                    reuseSingleOperandClassification,
                    hasReusableClassification,
                    reusableFrontWNV,
                    reusableBackWNV))
            {
                continue;
            }

            LeafClassificationAttemptStats attemptStats;
            const LeafClassificationResult classificationResult =
                classifyLeafFragment(context, fragmentIndex, fragment, allowRetryFallback, attemptStats);
            if (classificationResult == LeafClassificationResult::RetryPathInvalid)
            {
                resultFragments_.clear();
                classifiedFragments_.clear();
                return false;
            }
            if (classificationResult == LeafClassificationResult::Failure)
            {
                std::ostringstream summary;
                summary << "BoolProblem failed to classify leaf fragment "
                        << fragmentIndex
                        << " at depth "
                        << depth_
                        << ".";
                logLeafClassificationFailure(context, fragmentIndex, fragment, attemptStats);

                const std::string summaryMessage = summary.str();
                detail::logBoolError(kBoolProblemClassifyScope, summaryMessage);
                throw std::runtime_error(summaryMessage);
            }

            if (reuseSingleOperandClassification && !hasReusableClassification)
            {
                const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
                reusableFrontWNV = classifiedFragment.frontWNV;
                reusableBackWNV = classifiedFragment.backWNV;
                hasReusableClassification = true;
                detail::logBoolDebug(
                    kBoolProblemClassifyScope,
                    [this, &reusableFrontWNV, &reusableBackWNV]()
                    {
                        std::ostringstream message;
                        message << "Cached single-operand leaf classification depth=" << depth_
                                << " front_wnv_size=" << reusableFrontWNV.size()
                                << " back_wnv_size=" << reusableBackWNV.size()
                                << ".";
                        return message.str();
                    });
            }

            appendResultFragmentFromClassification(classifiedFragments_.back());
        }

        detail::logBoolDebug(
            kBoolProblemClassifyScope,
            [this]()
            {
                std::ostringstream message;
                message << "Leaf classification finished depth=" << depth_
                        << " result_fragments=" << resultFragments_.size()
                        << ".";
                return message.str();
            });
        return true;
    }

    bool SubdivisionSolver::trySolveSingleOperandAssumptionLeaf()
    {
        BoolOperandAssumptions assumptions;
        if (!tryGetSingleOperandAssumptions(assumptions) ||
            !assumptions.noSelfIntersections ||
            !assumptions.noNestedComponents ||
            polygons_.size() <= leafPolygonThreshold_)
        {
            return false;
        }

        const BoolSolveMetrics metricsSnapshot = solveMetrics_;
        isLeaf_ = true;
        solveLeafArrangement();
        if (!classifyLeafFragmentsAndCollectResults(true))
        {
            solveMetrics_ = metricsSnapshot;
            ++solveMetrics_.singleOperandAssumptionFallbackCount;
            isLeaf_ = true;
            leafFragments_.clear();
            classifiedFragments_.clear();
            resultFragments_.clear();
            detail::logBoolDebug(
                kBoolProblemClassifyScope,
                [this]()
                {
                    std::ostringstream message;
                    message << "Single-operand assumption stop fallback depth=" << depth_
                            << " polygon_count=" << polygons_.size()
                            << ".";
                    return message.str();
                });
            return false;
        }

        ++solveMetrics_.singleOperandAssumptionStopCount;
        detail::logBoolDebug(
            kBoolProblemClassifyScope,
            [this]()
            {
                std::ostringstream message;
                message << "Stopped subdivision by single-operand assumptions depth=" << depth_
                        << " polygon_count=" << polygons_.size()
                        << " result_fragments=" << resultFragments_.size()
                        << ".";
                return message.str();
            });
        return true;
    }

    // 将 WNV 交给当前布尔运算的二元指示函数，返回内外状态。
    BoolStatus SubdivisionSolver::evaluateBooleanIndicator(const WNV &wnv) const noexcept
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
}
