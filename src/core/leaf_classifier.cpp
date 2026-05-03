#include "core/bool_problem.h"

#include "algorithm/leaf_arrangement.h"
#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/logging.h"
#include "geometry/polygon_ops.h"

#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemLeafScope = "BoolProblem::solveLeafArrangement";
        constexpr const char *kBoolProblemClassifyScope = "BoolProblem::classifyLeafFragmentsAndCollectResults";

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

    // 对叶子节点内的每个 polygon 建立局部 BSP，并收集启用的 fragment。
    void BoolProblem::solveLeafArrangement()
    {
        leafFragments_.clear();
        classifiedFragments_.clear();
        if (discarded_ || polygons_.empty())
        {
            return;
        }

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

    // 对每个 leaf fragment 追踪到严格内部点；分类失败属于不可恢复错误。
    void BoolProblem::classifyLeafFragmentsAndCollectResults()
    {
        resultFragments_.clear();
        classifiedFragments_.clear();
        if (discarded_ || leafFragments_.empty())
        {
            return;
        }

        const refPoint localReference(reference_.point, reference_.wnv);
        const bool collectFailureDiagnostics = detail::isLogEnabled(LogLevel::Debug);
        for (std::size_t fragmentIndex = 0; fragmentIndex < leafFragments_.size(); ++fragmentIndex)
        {
            Polygon256 &fragment = leafFragments_[fragmentIndex];
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
                    pointCandidateCount += pointCandidates.size();

                    bool allowFallback = true;
                    enumerateLeafClassificationFastPathCandidatesFromPoints(
                        reference_.point,
                        pointCandidates,
                        aabb_,
                        [&](LeafClassificationPathCandidate candidate)
                        {
                            const std::size_t candidateIndex = fastCandidateCount;
                            ++fastCandidateCount;
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

            const bool shouldTryExpandedPoints = attemptPointCandidates(primaryPointCandidates);
            if (!classified && shouldTryExpandedPoints)
            {
                const std::vector<PlanePoint3i> expandedPointCandidates =
                    detail::enumerateLeafClassificationPointCandidatesUnchecked(fragment);
                attemptPointCandidates(expandedPointCandidates);
            }

            if (!classified)
            {
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
                     lastStatus,
                     &lastFailureReason,
                     &fragment]()
                    {
                        std::ostringstream message;
                        message << "Leaf classification failure diagnostic fragment_index="
                                << fragmentIndex
                                << " depth="
                                << depth_
                                << " target_point_candidates="
                                << pointCandidateCount
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
}
