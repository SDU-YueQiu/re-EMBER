/**
 * @file leaf_classifier.cpp
 * @brief 实现细分求解器的叶片编排分类。
 */
#include "core/subdivision_solver.h"

#include "algorithm/leaf_arrangement.h"
#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/perf_tracing.h"
#include "core/solver_shared.h"

#include "geometry/polygon_ops.h"

#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace ember
{
namespace
{
constexpr bool kLeafClassificationDebug = false;

struct LeafClassificationAttemptStats
{
    bool classified = false;
    traceStatus lastStatus = FAIL;
    bool centroidPointFound = false;
    PlanePoint3i centroidTargetPoint;
    std::size_t insetPointAttemptCount = 0;
    std::size_t insetPointCandidateCount = 0;
    std::size_t axisPathAttemptCount = 0;
    std::size_t planeReplacementPathAttemptCount = 0;
    std::vector<PlanePoint3i> bridgeRescueTargets;
    std::ostringstream debugLog;
};

enum class LeafClassificationResult
{
    Success,
    Failure
};

struct LeafClassificationContext
{
    const refPoint &localReference;
    const std::vector<Polygon256> &polygons;
    const AABB3i &aabb;
    std::mt19937 &leafRng;
    BoolSolveMetrics &solveMetrics;
    std::vector<ClassifiedFragment> &classifiedFragments;
    std::size_t depth = 0;
};

[[noreturn]] void throwLeafClassificationFailure(
    std::size_t fragmentIndex,
    std::size_t depth,
    const Polygon256 &fragment,
    const LeafClassificationAttemptStats &attemptStats)
{
    std::ostringstream summary;
    summary << "BoolProblem failed to classify leaf fragment "
            << fragmentIndex
            << " at depth "
            << depth
            << ". edge_count=" << fragment.edgeCount()
            << " centroid_point_found=" << (attemptStats.centroidPointFound ? 1 : 0)
            << " inset_point_attempt_count=" << attemptStats.insetPointAttemptCount
            << " inset_point_candidate_count=" << attemptStats.insetPointCandidateCount
            << " axis_path_attempt_count=" << attemptStats.axisPathAttemptCount
            << " plane_replacement_path_attempt_count=" << attemptStats.planeReplacementPathAttemptCount
            << " last_status=" << static_cast<int>(attemptStats.lastStatus)
            << ".";
    if constexpr (kLeafClassificationDebug)
    {
        if (!attemptStats.debugLog.str().empty())
        {
            std::cerr << "[leaf-classification-debug] fragment_index=" << fragmentIndex
                      << " depth=" << depth << "\n"
                      << "support_plane=" << detail::formatPlaneForDebug(fragment.plane) << "\n";
            for (std::size_t edgeIndex = 0; edgeIndex < fragment.edgePlanes.size(); ++edgeIndex)
            {
                std::cerr << "edge[" << edgeIndex << "]="
                          << detail::formatPlaneForDebug(fragment.edgePlanes[edgeIndex])
                          << " provenance=" << static_cast<int>(fragment.edgeProvenance(edgeIndex))
                          << "\n";
            }
            std::cerr
                << attemptStats.debugLog.str();
        }
    }
    throw std::runtime_error(summary.str());
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

std::string formatPathForDebug(const std::vector<Segment256> &path)
{
    std::ostringstream out;
    out << "segments=" << path.size();
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        out << " ["
            << i
            << "] "
            << detail::formatPlanePointForDebug(path[i].getStartPointRef())
            << " -> "
            << detail::formatPlanePointForDebug(path[i].getEndPointRef());
    }
    return out.str();
}

void appendPlaneReplacementFailureDebug(
    std::ostringstream &debugLog,
    const PlanePoint3i &referencePoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box)
{
    const std::array<Plane3i, 3> targetPlanes = {targetPoint.p, targetPoint.q, targetPoint.r};
    std::array<int, 3> targetPlaneOrder = {0, 1, 2};
    do
    {
        debugLog << "  plane_replacement target_plane_order=("
                 << targetPlaneOrder[0] << ","
                 << targetPlaneOrder[1] << ","
                 << targetPlaneOrder[2] << ")";

        const PlanePoint3i permutedTargetPoint = detail::makePointFromPlanes({
            targetPlanes[targetPlaneOrder[0]],
            targetPlanes[targetPlaneOrder[1]],
            targetPlanes[targetPlaneOrder[2]]});
        if (!permutedTargetPoint.hasUniqueIntersection())
        {
            debugLog << " invalid_permuted_target\n";
            continue;
        }
        if (!areSamePlanePoint(permutedTargetPoint, targetPoint))
        {
            debugLog << " permuted_target_not_same\n";
            continue;
        }

        std::vector<int> changedPlaneIndices;
        if (!detail::areSamePlaneEquation(referencePoint.p, permutedTargetPoint.p))
            changedPlaneIndices.push_back(0);
        if (!detail::areSamePlaneEquation(referencePoint.q, permutedTargetPoint.q))
            changedPlaneIndices.push_back(1);
        if (!detail::areSamePlaneEquation(referencePoint.r, permutedTargetPoint.r))
            changedPlaneIndices.push_back(2);
        if (changedPlaneIndices.empty())
        {
            debugLog << " no_changed_planes\n";
            continue;
        }

        std::sort(changedPlaneIndices.begin(), changedPlaneIndices.end());
        do
        {
            std::vector<Segment256> path;
            const bool ok = detail::buildPlaneReplacementPath(
                referencePoint,
                permutedTargetPoint,
                box,
                changedPlaneIndices,
                path);
            debugLog << " changed_planes=(";
            for (std::size_t i = 0; i < changedPlaneIndices.size(); ++i)
            {
                if (i != 0)
                    debugLog << ",";
                debugLog << changedPlaneIndices[i];
            }
            debugLog << ") build_ok=" << (ok ? 1 : 0);
            if (ok)
                debugLog << " " << formatPathForDebug(path);
            debugLog << "\n";
        } while (std::next_permutation(changedPlaneIndices.begin(), changedPlaneIndices.end()));
    } while (std::next_permutation(targetPlaneOrder.begin(), targetPlaneOrder.end()));
}

traceStatus traceLeafClassificationCandidate(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats,
    const LeafClassificationPathCandidate &candidate)
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

    attemptStats.lastStatus = status;
    if (status == SUCCESS)
    {
        context.classifiedFragments.push_back(
            ClassifiedFragment{fragment, std::move(frontWNV), std::move(backWNV)});
        attemptStats.classified = true;
    }

    return status;
}

bool attemptCentroidAxisPathCandidate(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats)
{
    REEMBER_PROFILE_ZONE("LeafClassification::attemptCentroidAxisPath");

    PlanePoint3i targetPoint;
    if (!detail::buildLeafClassificationCentroidTargetPoint(fragment, targetPoint))
    {
        if constexpr (kLeafClassificationDebug)
            attemptStats.debugLog << "centroid_target=none\n";
        return true;
    }

    attemptStats.centroidPointFound = true;
    attemptStats.centroidTargetPoint = targetPoint;
    if constexpr (kLeafClassificationDebug)
        attemptStats.debugLog << "centroid_target=" << detail::formatPlanePointForDebug(targetPoint) << "\n";
    ++context.solveMetrics.leafClassificationCentroidPointCount;
    {
        REEMBER_PROFILE_ZONE("LeafClassification::centroidPoint");
    }

    bool allowFallback = true;
    enumerateLeafClassificationAxisPathCandidatesFromPoints(
        context.localReference.point,
        std::vector<PlanePoint3i>{targetPoint},
        context.aabb,
        [&](LeafClassificationPathCandidate candidate)
    {
        REEMBER_PROFILE_ZONE("LeafClassification::axisPathCandidate");

        ++attemptStats.axisPathAttemptCount;
        ++context.solveMetrics.leafClassificationAxisPathAttemptCount;
        if constexpr (kLeafClassificationDebug)
        {
            attemptStats.debugLog << "axis_path_candidate "
                                  << formatPathForDebug(candidate.path);
        }
        const traceStatus status = traceLeafClassificationCandidate(
                                       context,
                                       fragmentIndex,
                                       fragment,
                                       attemptStats,
                                       candidate);
        if constexpr (kLeafClassificationDebug)
            attemptStats.debugLog << " status=" << traceStatusName(status) << "\n";
        if (status == SUCCESS)
            return false;

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

bool attemptInsetPlaneReplacementCandidates(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats)
{
    REEMBER_PROFILE_ZONE("LeafClassification::attemptInsetPlaneReplacementCandidates");

    const detail::LeafClassificationInsetPointSequence insetPoints =
        detail::enumerateLeafClassificationInsetPointCandidates(fragment, context.leafRng, kLeafClassificationDebug);
    attemptStats.insetPointAttemptCount += insetPoints.attemptCount;
    attemptStats.insetPointCandidateCount += insetPoints.candidates.size();
    if constexpr (kLeafClassificationDebug)
    {
        attemptStats.debugLog << "inset_attempts=" << insetPoints.attemptCount
                              << " inset_candidates=" << insetPoints.candidates.size() << "\n";
        for (const std::string &line : insetPoints.debugLines)
            attemptStats.debugLog << "  inset " << line << "\n";
    }
    context.solveMetrics.leafClassificationInsetPointAttemptCount += insetPoints.attemptCount;
    std::vector<PlanePoint3i> planeReplacementTargets = insetPoints.candidates;
    if (planeReplacementTargets.empty())
    {
        if constexpr (kLeafClassificationDebug)
            attemptStats.debugLog << "exact_fallback_point_generation=1\n";
        detail::appendHomogeneousVertexAverageInteriorPointCandidate(fragment, planeReplacementTargets);
        if (planeReplacementTargets.empty())
            detail::appendEqualizedEdgeInteriorPointCandidates(fragment, planeReplacementTargets);
        if (planeReplacementTargets.empty())
        {
            if (!attemptStats.centroidPointFound)
                return true;

            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << "reusing_centroid_target_for_plane_replacement=1\n";
            planeReplacementTargets.push_back(attemptStats.centroidTargetPoint);
        }
    }

    if constexpr (kLeafClassificationDebug)
    {
        for (const PlanePoint3i &candidatePoint : planeReplacementTargets)
            attemptStats.debugLog << "  inset_candidate_point=" << detail::formatPlanePointForDebug(candidatePoint) << "\n";
    }
    attemptStats.bridgeRescueTargets = planeReplacementTargets;

    bool allowFallback = true;
    const auto tryPlaneReplacementTargets =
        [&](const std::vector<PlanePoint3i> &targets, const char *label)
    {
        enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints(
            context.localReference.point,
            targets,
            context.aabb,
            [&](LeafClassificationPathCandidate candidate)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::planeReplacementPathCandidate");

            ++attemptStats.planeReplacementPathAttemptCount;
            ++context.solveMetrics.leafClassificationPlaneReplacementPathAttemptCount;
            attemptStats.debugLog << label << " "
                                  << formatPathForDebug(candidate.path);
            const traceStatus status = traceLeafClassificationCandidate(
                                           context,
                                           fragmentIndex,
                                           fragment,
                                           attemptStats,
                                           candidate);
            attemptStats.debugLog << " status=" << traceStatusName(status) << "\n";
            if (status == SUCCESS)
                return false;

            if (status != PATH_INVALID)
            {
                allowFallback = false;
                return false;
            }

            return true;
        });
    };

    constexpr std::size_t kMaxDirectInsetTargets = 3;
    if (planeReplacementTargets.size() > kMaxDirectInsetTargets)
    {
        if constexpr (kLeafClassificationDebug)
            attemptStats.debugLog << "direct_inset_target_cap=" << kMaxDirectInsetTargets << "\n";
        std::vector<PlanePoint3i> directTargets(
            planeReplacementTargets.begin(),
            planeReplacementTargets.begin() + static_cast<std::ptrdiff_t>(kMaxDirectInsetTargets));
        tryPlaneReplacementTargets(directTargets, "plane_replacement_candidate");
        if (!attemptStats.classified && allowFallback)
        {
            std::vector<PlanePoint3i> remainingTargets(
                planeReplacementTargets.begin() + static_cast<std::ptrdiff_t>(kMaxDirectInsetTargets),
                planeReplacementTargets.end());
            if (!remainingTargets.empty())
            {
                if constexpr (kLeafClassificationDebug)
                    attemptStats.debugLog << "expanding_inset_target_set=" << remainingTargets.size() << "\n";
                tryPlaneReplacementTargets(remainingTargets, "plane_replacement_candidate_expanded");
            }
        }
    }
    else
    {
        tryPlaneReplacementTargets(planeReplacementTargets, "plane_replacement_candidate");
    }

    if (attemptStats.planeReplacementPathAttemptCount == 0)
    {
        if constexpr (kLeafClassificationDebug)
        {
            attemptStats.debugLog << "plane_replacement_candidates=0\n";
            for (const PlanePoint3i &candidatePoint : planeReplacementTargets)
                appendPlaneReplacementFailureDebug(
                    attemptStats.debugLog,
                    context.localReference.point,
                    candidatePoint,
                    context.aabb);
        }
    }
    return allowFallback;
}

bool attemptBridgeRescueCandidates(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats)
{
    REEMBER_PROFILE_ZONE("LeafClassification::attemptBridgeRescueCandidates");

    if (attemptStats.bridgeRescueTargets.empty())
        return true;

    const std::vector<PlanePoint3i> bridgePoints =
        detail::enumerateAABBInteriorBridgePoints(context.localReference.point, context.aabb);
    if constexpr (kLeafClassificationDebug)
    {
        attemptStats.debugLog << "bridge_point_count=" << bridgePoints.size()
                              << " aabb=["
                              << integerToString(context.aabb.xMin) << ","
                              << integerToString(context.aabb.xMax) << "]x["
                              << integerToString(context.aabb.yMin) << ","
                              << integerToString(context.aabb.yMax) << "]x["
                              << integerToString(context.aabb.zMin) << ","
                              << integerToString(context.aabb.zMax) << "]\n";
    }
    if (bridgePoints.empty())
        return true;

    bool allowFallback = true;
    for (const PlanePoint3i &bridgePoint : bridgePoints)
    {
        std::vector<Segment256> prefix;
        if (!detail::appendBridgePath(prefix, context.localReference.point, bridgePoint, context.aabb))
            continue;

        if constexpr (kLeafClassificationDebug)
        {
            attemptStats.debugLog << "bridge_point=" << detail::formatPlanePointForDebug(bridgePoint)
                                  << " prefix_" << formatPathForDebug(prefix) << "\n";
        }

        const auto traceWithPrefix =
            [&](LeafClassificationPathCandidate candidate,
                const char *label,
                std::size_t &pathAttemptCounter,
                std::size_t &metricCounter) -> bool
        {
            std::vector<Segment256> fullPath;
            fullPath.reserve(prefix.size() + candidate.path.size());
            fullPath.insert(fullPath.end(), prefix.begin(), prefix.end());
            fullPath.insert(
                fullPath.end(),
                std::make_move_iterator(candidate.path.begin()),
                std::make_move_iterator(candidate.path.end()));

            ++pathAttemptCounter;
            ++metricCounter;
            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << label << " " << formatPathForDebug(fullPath);
            const traceStatus status = traceLeafClassificationCandidate(
                                           context,
                                           fragmentIndex,
                                           fragment,
                                           attemptStats,
                                           LeafClassificationPathCandidate{
                                               candidate.targetPoint,
                                               std::move(fullPath)});
            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << " status=" << traceStatusName(status) << "\n";
            if (status == SUCCESS)
                return false;

            if (status != PATH_INVALID)
            {
                allowFallback = false;
                return false;
            }

            return true;
        };

        enumerateLeafClassificationAxisPathCandidatesFromPoints(
            bridgePoint,
            attemptStats.bridgeRescueTargets,
            context.aabb,
            [&](LeafClassificationPathCandidate candidate)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::bridgeRescueAxisCandidate");
            return traceWithPrefix(
                std::move(candidate),
                "bridge_axis_candidate",
                attemptStats.axisPathAttemptCount,
                context.solveMetrics.leafClassificationAxisPathAttemptCount);
        });

        if (attemptStats.classified || !allowFallback)
            return allowFallback;

        enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints(
            bridgePoint,
            attemptStats.bridgeRescueTargets,
            context.aabb,
            [&](LeafClassificationPathCandidate candidate)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::bridgeRescuePlaneReplacementCandidate");
            return traceWithPrefix(
                std::move(candidate),
                "bridge_plane_replacement_candidate",
                attemptStats.planeReplacementPathAttemptCount,
                context.solveMetrics.leafClassificationPlaneReplacementPathAttemptCount);
        });

        if (attemptStats.classified || !allowFallback)
            return allowFallback;
    }

    return allowFallback;
}

LeafClassificationResult classifyLeafFragment(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats)
{
    REEMBER_PROFILE_ZONE("LeafClassification::classifyLeafFragment");

    bool allowFallback = attemptCentroidAxisPathCandidate(
                             context,
                             fragmentIndex,
                             fragment,
                             attemptStats);
    if (!attemptStats.classified && allowFallback)
    {
        allowFallback = attemptInsetPlaneReplacementCandidates(
                            context,
                            fragmentIndex,
                            fragment,
                            attemptStats);
    }
    if (!attemptStats.classified && allowFallback)
    {
        allowFallback = attemptBridgeRescueCandidates(
                            context,
                            fragmentIndex,
                            fragment,
                            attemptStats);
    }

    if (attemptStats.classified)
        return LeafClassificationResult::Success;
    return LeafClassificationResult::Failure;
}

}

// 对叶子节点内的每个多边形建立局部 BSP，并收集启用的片段。
void SubdivisionSolver::solveLeafArrangement()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::solveLeafArrangement");

    leafFragments_.clear();
    classifiedFragments_.clear();
    if (discarded_ || polygons_.empty())
        return;

    if (singleOperandPolicy_.maySkipLeafBsp)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::skipLeafBspBySingleOperandAssumption");
        ++solveMetrics_.singleOperandLeafBspSkipCount;
        leafFragments_ = polygons_;
        return;
    }

    ++solveMetrics_.leafBspBuildCount;
    leafFragments_ = buildLeafArrangement(polygons_);

}

void SubdivisionSolver::appendResultFragmentFromClassification(const ClassifiedFragment &classifiedFragment)
{
    const BoolStatus frontStatus = evaluateBooleanIndicator(classifiedFragment.frontWNV);
    const BoolStatus backStatus = evaluateBooleanIndicator(classifiedFragment.backWNV);
    if (frontStatus == OUT && backStatus == IN)
        resultFragments_.push_back(classifiedFragment.polygon);
    else if (frontStatus == IN && backStatus == OUT)
        resultFragments_.push_back(reversePolygonOrientation(classifiedFragment.polygon));
}

bool SubdivisionSolver::tryReuseSingleOperandFragmentClassification(
    const Polygon256 &fragment,
    bool hasReusableClassification,
    const WNV &reusableFrontWNV,
    const WNV &reusableBackWNV)
{
    if (!singleOperandPolicy_.mayReuseLeafClassification || !hasReusableClassification)
        return false;

    REEMBER_PROFILE_ZONE("LeafClassification::reuseSingleOperandClassification");
    ++solveMetrics_.singleOperandClassificationReuseCount;
    classifiedFragments_.push_back(ClassifiedFragment{fragment, reusableFrontWNV, reusableBackWNV});
    appendResultFragmentFromClassification(classifiedFragments_.back());
    return true;
}

// 对每个叶片片段追踪到严格内部点；分类失败属于不可恢复错误。
void SubdivisionSolver::classifyLeafFragmentsAndCollectResults()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::classifyLeafFragmentsAndCollectResults");

    resultFragments_.clear();
    classifiedFragments_.clear();
    if (discarded_ || leafFragments_.empty())
        return;

    const refPoint localReference(reference_.point, reference_.wnv);
    std::mt19937 leafRng(42u);
    LeafClassificationContext context{
        localReference,
        polygons_,
        aabb_,
        leafRng,
        solveMetrics_,
        classifiedFragments_,
        depth_};
    WNV reusableFrontWNV;
    WNV reusableBackWNV;
    bool hasReusableClassification = false;

    for (std::size_t fragmentIndex = 0; fragmentIndex < leafFragments_.size(); ++fragmentIndex)
    {
        Polygon256 &fragment = leafFragments_[fragmentIndex];
        if (tryReuseSingleOperandFragmentClassification(
                    fragment,
                    hasReusableClassification,
                    reusableFrontWNV,
                    reusableBackWNV))
            continue;

        LeafClassificationAttemptStats attemptStats;
        const LeafClassificationResult classificationResult =
            classifyLeafFragment(context, fragmentIndex, fragment, attemptStats);
        if (classificationResult == LeafClassificationResult::Failure)
            throwLeafClassificationFailure(fragmentIndex, depth_, fragment, attemptStats);

        if (singleOperandPolicy_.mayReuseLeafClassification && !hasReusableClassification)
        {
            const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
            reusableFrontWNV = classifiedFragment.frontWNV;
            reusableBackWNV = classifiedFragment.backWNV;
            hasReusableClassification = true;
        }

        appendResultFragmentFromClassification(classifiedFragments_.back());
    }
}

bool SubdivisionSolver::trySolveSingleOperandAssumptionLeaf()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::trySolveSingleOperandAssumptionLeaf");
    if (!singleOperandPolicy_.mayProbeEarlyLeaf)
        return false;

    isLeaf_ = true;
    solveLeafArrangement();
    classifyLeafFragmentsAndCollectResults();
    ++solveMetrics_.singleOperandAssumptionStopCount;
    finalizeLeafNode();
    return true;
}

// 将 WNV 交给当前布尔运算的二元指示函数，返回内外状态。
BoolStatus SubdivisionSolver::evaluateBooleanIndicator(const WNV &wnv) const noexcept
{
    switch (op_)
    {
    case BoolOp::Union:
        return f_union(wnv, static_cast<int>(detail::kLhsOperandIndex), static_cast<int>(detail::kRhsOperandIndex));
    case BoolOp::Intersection:
        return f_intersection(wnv, static_cast<int>(detail::kLhsOperandIndex), static_cast<int>(detail::kRhsOperandIndex));
    case BoolOp::Difference:
        return f_diff(wnv, static_cast<int>(detail::kLhsOperandIndex), static_cast<int>(detail::kRhsOperandIndex));
    }

    return OUT;
}
}
