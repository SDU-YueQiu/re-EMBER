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
            logBoolDebug(
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
        const bool collectFailureDiagnostics = detail::isLogEnabled(LogLevel::Debug);
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
            if (reuseSingleOperandClassification && hasReusableClassification)
            {
                ++solveMetrics_.singleOperandClassificationReuseCount;
                classifiedFragments_.push_back(ClassifiedFragment{fragment, reusableFrontWNV, reusableBackWNV});
                const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
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
                continue;
            }

            std::size_t pointCandidateCount = 0;
            std::size_t normalCandidateCount = 0;
            std::size_t fastCandidateCount = 0;
            std::size_t interiorBridgeCandidateCount = 0;
            std::size_t fallbackCandidateCount = 0;
            std::string lastFailureReason = "not_collected";

            bool classified = false;
            traceStatus lastStatus = FAIL;
            auto traceCandidate =
                [&](const LeafClassificationPathCandidate &candidate, const char *candidateLayer, std::size_t candidateIndex)
                {
                    REEMBER_PROFILE_ZONE("LeafClassification::traceCandidate");

                    ++solveMetrics_.leafClassificationTraceAttemptCount;
                    WNV frontWNV;
                    WNV backWNV;
                    const traceStatus status = detail::tracePathWNVToSurfacePointTrusted(
                        localReference,
                        candidate.path,
                        polygons_,
                        fragment.plane,
                        frontWNV,
                        backWNV);
                    if (status != SUCCESS && collectFailureDiagnostics)
                    {
                        lastFailureReason = describeSurfaceTraceFailure(
                            localReference,
                            candidate.path,
                            polygons_,
                            fragment.plane);
                    }

                    logTracingDebug(
                        kBoolProblemClassifyScope,
                        [fragmentIndex, candidateLayer, candidateIndex, &candidate, status, &lastFailureReason]()
                        {
                            std::ostringstream message;
                            message << "Leaf fragment trace attempt fragment_index=" << fragmentIndex
                                    << " candidate_layer=" << candidateLayer
                                    << " candidate_index=" << candidateIndex
                                    << " path_segments=" << candidate.path.size()
                                    << " status=" << traceStatusName(status);
                            if (status != SUCCESS)
                            {
                                message << " reason=" << lastFailureReason;
                            }
                            message
                                    << ".";
                            return message.str();
                        });

                    if (status == SUCCESS)
                    {
                        classifiedFragments_.push_back(
                            ClassifiedFragment{fragment, std::move(frontWNV), std::move(backWNV)});
                        classified = true;
                    }

                    return status;
                };

            auto attemptPointCandidates =
                [&](const std::vector<PlanePoint3i> &pointCandidates) -> bool
                {
                    REEMBER_PROFILE_ZONE("LeafClassification::attemptPointCandidates");

                    pointCandidateCount += pointCandidates.size();
                    solveMetrics_.leafClassificationPointCandidateCount += pointCandidates.size();

                    bool allowFallback = true;
                    enumerateLeafClassificationFastPathCandidatesFromPoints(
                        reference_.point,
                        pointCandidates,
                        aabb_,
                        [&](LeafClassificationPathCandidate candidate)
                        {
                            const std::size_t candidateIndex = fastCandidateCount;
                            ++fastCandidateCount;
                            ++solveMetrics_.leafClassificationFastCandidateCount;
                            const traceStatus status = traceCandidate(candidate, "fast", candidateIndex);
                            if (status == SUCCESS)
                            {
                                return false;
                            }

                            lastStatus = status;
                            if (status != PATH_INVALID)
                            {
                                allowFallback = false;
                                return false;
                            }

                            allowFallback = true;
                            return true;
                        });

                    if (!classified && allowFallback)
                    {
                        enumerateLeafClassificationFallbackPathCandidatesFromPoints(
                            reference_.point,
                            pointCandidates,
                            aabb_,
                            [&](LeafClassificationPathCandidate candidate)
                            {
                                const std::size_t candidateIndex = fallbackCandidateCount;
                                ++fallbackCandidateCount;
                                ++solveMetrics_.leafClassificationFallbackCandidateCount;
                                const traceStatus status = traceCandidate(candidate, "fallback", candidateIndex);
                                if (status == SUCCESS)
                                {
                                    return false;
                                }

                                lastStatus = status;
                                if (status != PATH_INVALID)
                                {
                                    allowFallback = false;
                                    return false;
                                }

                                return true;
                            });
                    }

                    if (!classified && allowFallback)
                    {
                        enumerateLeafClassificationNormalApproachCandidatesFromPoints(
                            reference_.point,
                            pointCandidates,
                            fragment.plane,
                            aabb_,
                            [&](LeafClassificationPathCandidate candidate)
                            {
                                const std::size_t candidateIndex = normalCandidateCount;
                                ++normalCandidateCount;
                                ++solveMetrics_.leafClassificationNormalCandidateCount;
                                const traceStatus status = traceCandidate(candidate, "normal", candidateIndex);
                                if (status == SUCCESS)
                                {
                                    return false;
                                }

                                lastStatus = status;
                                if (status != PATH_INVALID)
                                {
                                    allowFallback = false;
                                    return false;
                                }

                                allowFallback = true;
                                return true;
                            });
                    }

                    if (!classified && allowFallback)
                    {
                        enumerateLeafClassificationInteriorBridgeCandidatesFromPoints(
                            reference_.point,
                            pointCandidates,
                            aabb_,
                            [&](LeafClassificationPathCandidate candidate)
                            {
                                const std::size_t candidateIndex = interiorBridgeCandidateCount;
                                ++interiorBridgeCandidateCount;
                                ++solveMetrics_.leafClassificationInteriorBridgeCandidateCount;
                                const traceStatus status = traceCandidate(candidate, "interior-bridge", candidateIndex);
                                if (status == SUCCESS)
                                {
                                    return false;
                                }

                                lastStatus = status;
                                if (status != PATH_INVALID)
                                {
                                    allowFallback = false;
                                    return false;
                                }

                                allowFallback = true;
                                return true;
                            });
                    }

                    return !classified && allowFallback;
                };

            const std::vector<PlanePoint3i> primaryPointCandidates =
                detail::enumerateLeafClassificationPrimaryPointCandidatesUnchecked(fragment);
            const std::size_t primaryPointCandidateCount = primaryPointCandidates.size();
            std::size_t expandedPointCandidateCount = 0;

            const bool shouldTryExpandedPoints = attemptPointCandidates(primaryPointCandidates);
            if (!classified && shouldTryExpandedPoints)
            {
                const std::vector<PlanePoint3i> expandedPointCandidates =
                    detail::enumerateLeafClassificationPointCandidatesUnchecked(fragment);
                expandedPointCandidateCount = expandedPointCandidates.size();
                attemptPointCandidates(expandedPointCandidates);
            }

            if (!classified)
            {
                if (allowRetryFallback && lastStatus == PATH_INVALID)
                {
                    resultFragments_.clear();
                    classifiedFragments_.clear();
                    return false;
                }

                std::ostringstream summary;
                summary << "BoolProblem failed to classify leaf fragment "
                        << fragmentIndex
                        << " at depth "
                        << depth_
                        << ".";

                logBoolDebug(
                    kBoolProblemClassifyScope,
                    [this,
                     fragmentIndex,
                     normalCandidateCount,
                     fastCandidateCount,
                     interiorBridgeCandidateCount,
                     fallbackCandidateCount,
                     pointCandidateCount,
                     primaryPointCandidateCount,
                     expandedPointCandidateCount,
                     lastStatus,
                     &lastFailureReason,
                     &fragment]()
                    {
                        std::ostringstream message;
                        message << "Leaf classification failure diagnostic fragment_index="
                                << fragmentIndex
                                << " depth="
                                << depth_
                                << " fragment_edges="
                                << fragment.edgeCount()
                                << " target_point_candidates="
                                << pointCandidateCount
                                << " primary_point_candidates="
                                << primaryPointCandidateCount
                                << " expanded_point_candidates="
                                << expandedPointCandidateCount
                                << " normal_candidates="
                                << normalCandidateCount
                                << " fast_candidates="
                                << fastCandidateCount
                                << " interior_bridge_candidates="
                                << interiorBridgeCandidateCount
                                << " fallback_candidates="
                                << fallbackCandidateCount
                                << " last_trace_status="
                                << traceStatusName(lastStatus)
                                << " last_failure_reason="
                                << lastFailureReason
                                << " fragment_plane="
                                << fragment.plane
                                << " aabb={x=["
                                << aabb_.xMin
                                << ", "
                                << aabb_.xMax
                                << "], y=["
                                << aabb_.yMin
                                << ", "
                                << aabb_.yMax
                                << "], z=["
                                << aabb_.zMin
                                << ", "
                                << aabb_.zMax
                                << "]}";
                        return message.str();
                    });

                const std::string summaryMessage = summary.str();
                logBoolError(kBoolProblemClassifyScope, summaryMessage);
                throw std::runtime_error(summaryMessage);
            }

            if (reuseSingleOperandClassification && !hasReusableClassification)
            {
                const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
                reusableFrontWNV = classifiedFragment.frontWNV;
                reusableBackWNV = classifiedFragment.backWNV;
                hasReusableClassification = true;
                logBoolDebug(
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

            const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
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
            logBoolDebug(
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
        logBoolDebug(
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
