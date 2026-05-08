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

#include <algorithm>
#include <array>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace ember
{
namespace
{
constexpr bool kLeafClassificationDebug = false;

std::size_t mixHashValue(std::size_t seed, std::size_t value) noexcept
{
    return seed ^ (value + 0x9e3779b97f4a7c15ull + (seed << 6u) + (seed >> 2u));
}

std::size_t hashInteger(const Integer &value)
{
    constexpr std::size_t kBitsPerChunk = 64;
    const Integer mask = (Integer(1) << kBitsPerChunk) - 1;
    Integer magnitude = value < 0 ? -value : value;
    std::size_t seed = value < 0 ? 0x517cc1b727220a95ull : 0x243f6a8885a308d3ull;
    for (std::size_t shift = 0; shift < 256u; shift += kBitsPerChunk)
    {
        const Integer chunk = (magnitude >> shift) & mask;
        seed = mixHashValue(seed, static_cast<std::size_t>(chunk.convert_to<std::uint64_t>()));
    }
    return seed;
}

std::size_t hashPlane(const Plane3i &plane)
{
    std::size_t seed = hashInteger(plane.a);
    seed = mixHashValue(seed, hashInteger(plane.b));
    seed = mixHashValue(seed, hashInteger(plane.c));
    seed = mixHashValue(seed, hashInteger(plane.d));
    return seed;
}

struct PlaneReplacementBuildSignatureHash
{
    std::size_t operator()(const detail::PlaneReplacementBuildSignature &signature) const
    {
        std::size_t seed = signature.planeReplacementCount;
        for (const Plane3i &plane : signature.startPlanes)
            seed = mixHashValue(seed, hashPlane(plane));
        for (const Plane3i &plane : signature.targetPlanes)
            seed = mixHashValue(seed, hashPlane(plane));
        for (std::size_t i = 0; i < signature.planeReplacementCount; ++i)
            seed = mixHashValue(seed, static_cast<std::size_t>(signature.planeReplacementOrder[i]));
        return seed;
    }
};

struct PlaneReplacementBuildSignatureEqual
{
    bool operator()(
        const detail::PlaneReplacementBuildSignature &lhs,
        const detail::PlaneReplacementBuildSignature &rhs) const noexcept
    {
        return detail::samePlaneReplacementBuildSignature(lhs, rhs);
    }
};

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
    std::vector<std::vector<HomPoint4i>> uniquePathSignatures;
    std::unordered_set<
        detail::PlaneReplacementBuildSignature,
        PlaneReplacementBuildSignatureHash,
        PlaneReplacementBuildSignatureEqual> seenPlaneReplacementBuildSignatures;
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

enum class LeafClassificationTraceStage
{
    CentroidAxis,
    InsetReplacement,
    BridgeRescue
};

enum class LeafClassificationCandidateKind
{
    AxisPath,
    PlaneReplacementPath
};

struct LeafClassificationTraceResult
{
    bool traced = false;
    traceStatus status = PATH_INVALID;
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

bool isRecoverableClassificationTraceStatus(traceStatus status) noexcept
{
    return status == PATH_INVALID;
}

void recordLeafClassificationStatus(
    BoolSolveMetrics &metrics,
    LeafClassificationTraceStage stage,
    traceStatus status) noexcept
{
    std::size_t *successCount = nullptr;
    std::size_t *pathInvalidCount = nullptr;
    std::size_t *inputInvalidCount = nullptr;
    std::size_t *failCount = nullptr;

    switch (stage)
    {
    case LeafClassificationTraceStage::CentroidAxis:
        successCount = &metrics.leafClassificationCentroidAxisSuccessCount;
        pathInvalidCount = &metrics.leafClassificationCentroidAxisPathInvalidCount;
        inputInvalidCount = &metrics.leafClassificationCentroidAxisInputInvalidCount;
        failCount = &metrics.leafClassificationCentroidAxisFailCount;
        break;
    case LeafClassificationTraceStage::InsetReplacement:
        successCount = &metrics.leafClassificationInsetReplacementSuccessCount;
        pathInvalidCount = &metrics.leafClassificationInsetReplacementPathInvalidCount;
        inputInvalidCount = &metrics.leafClassificationInsetReplacementInputInvalidCount;
        failCount = &metrics.leafClassificationInsetReplacementFailCount;
        break;
    case LeafClassificationTraceStage::BridgeRescue:
        successCount = &metrics.leafClassificationBridgeRescueSuccessCount;
        pathInvalidCount = &metrics.leafClassificationBridgeRescuePathInvalidCount;
        inputInvalidCount = &metrics.leafClassificationBridgeRescueInputInvalidCount;
        failCount = &metrics.leafClassificationBridgeRescueFailCount;
        break;
    }

    switch (status)
    {
    case SUCCESS:
        ++(*successCount);
        break;
    case PATH_INVALID:
        ++(*pathInvalidCount);
        break;
    case INPUT_INVALID:
        ++(*inputInvalidCount);
        break;
    case FAIL:
        ++(*failCount);
        break;
    }
}

bool samePathSignature(
    const std::vector<HomPoint4i> &lhs,
    const std::vector<HomPoint4i> &rhs) noexcept
{
    if (lhs.size() != rhs.size())
        return false;

    for (std::size_t i = 0; i < lhs.size(); ++i)
    {
        if (!lhs[i].hasSameComponents(rhs[i]))
            return false;
    }

    return true;
}

std::vector<HomPoint4i> makePathSignature(const std::vector<Segment256> &path)
{
    std::vector<HomPoint4i> signature;
    if (path.empty())
        return signature;

    signature.reserve(path.size() + 1u);
    signature.push_back(path.front().getStartPointRef().x);
    for (const Segment256 &segment : path)
        signature.push_back(segment.getEndPointRef().x);
    return signature;
}

bool hasSeenPathSignature(
    const LeafClassificationAttemptStats &attemptStats,
    const std::vector<HomPoint4i> &signature) noexcept
{
    for (const std::vector<HomPoint4i> &existing : attemptStats.uniquePathSignatures)
    {
        if (samePathSignature(existing, signature))
            return true;
    }

    return false;
}

bool shouldSkipRepeatedPlaneReplacementBuild(
    LeafClassificationAttemptStats &attemptStats,
    const detail::PlaneReplacementBuildSignature &signature)
{
    const auto [_, inserted] = attemptStats.seenPlaneReplacementBuildSignatures.insert(signature);
    return !inserted;
}

bool isTraceableSurfaceCandidatePath(
    const PlanePoint3i &referencePoint,
    const PlanePoint3i &targetPoint,
    const std::vector<Segment256> &path) noexcept
{
    if (!targetPoint.hasUniqueIntersection() || path.empty())
        return false;

    if (!areSamePlanePoint(path.front().getStartPointRef(), referencePoint))
        return false;

    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (!path[i].isValid())
            return false;
        if (i > 0u && !areSamePlanePoint(path[i - 1u].getEndPointRef(), path[i].getStartPointRef()))
            return false;
    }

    const PlanePoint3i &pathTargetPoint = path.back().getEndPointRef();
    if (pathTargetPoint.x.hasSameComponents(targetPoint.x))
        return true;

    return areSamePlanePoint(pathTargetPoint, targetPoint);
}

bool buildAxisRepairPath(
    const PlanePoint3i &referencePoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    std::vector<Segment256> &outPath)
{
    const std::array<Plane3i, 3> referenceCoordinatePlanes = {
        detail::makeCoordinatePlaneFromPoint(referencePoint, SplitAxis3i::X),
        detail::makeCoordinatePlaneFromPoint(referencePoint, SplitAxis3i::Y),
        detail::makeCoordinatePlaneFromPoint(referencePoint, SplitAxis3i::Z)
    };
    const std::array<Plane3i, 3> targetCoordinatePlanes = {
        detail::makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::X),
        detail::makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Y),
        detail::makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Z)
    };

    std::vector<SplitAxis3i> changedAxes;
    changedAxes.reserve(3);
    if (!detail::areSamePlaneEquation(referenceCoordinatePlanes[0], targetCoordinatePlanes[0]))
        changedAxes.push_back(SplitAxis3i::X);
    if (!detail::areSamePlaneEquation(referenceCoordinatePlanes[1], targetCoordinatePlanes[1]))
        changedAxes.push_back(SplitAxis3i::Y);
    if (!detail::areSamePlaneEquation(referenceCoordinatePlanes[2], targetCoordinatePlanes[2]))
        changedAxes.push_back(SplitAxis3i::Z);
    if (changedAxes.empty())
        return false;

    std::sort(
        changedAxes.begin(),
        changedAxes.end(),
        detail::axisOrderLess);
    do
    {
        std::vector<Segment256> path;
        if (detail::buildAxisAlignedCoordinatePath(referencePoint, targetPoint, box, changedAxes, path) &&
                isTraceableSurfaceCandidatePath(referencePoint, targetPoint, path))
        {
            outPath = std::move(path);
            return true;
        }
    } while (std::next_permutation(
                 changedAxes.begin(),
                 changedAxes.end(),
                 detail::axisOrderLess));

    return false;
}

bool buildPlaneReplacementRepairPath(
    const PlanePoint3i &referencePoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    std::vector<Segment256> &outPath)
{
    std::array<int, 3> changedPlaneIndices = {};
    std::size_t changedPlaneCount = 0;
    if (!detail::areSamePlaneEquation(referencePoint.p, targetPoint.p))
        changedPlaneIndices[changedPlaneCount++] = 0;
    if (!detail::areSamePlaneEquation(referencePoint.q, targetPoint.q))
        changedPlaneIndices[changedPlaneCount++] = 1;
    if (!detail::areSamePlaneEquation(referencePoint.r, targetPoint.r))
        changedPlaneIndices[changedPlaneCount++] = 2;
    if (changedPlaneCount == 0)
        return false;

    std::sort(
        changedPlaneIndices.begin(),
        changedPlaneIndices.begin() + static_cast<std::ptrdiff_t>(changedPlaneCount));
    std::array<int, 3> planeReplacementOrder = changedPlaneIndices;
    do
    {
        std::vector<Segment256> path;
        if (detail::buildPlaneReplacementPath(
                    referencePoint,
                    targetPoint,
                    box,
                    planeReplacementOrder,
                    changedPlaneCount,
                    path) &&
                isTraceableSurfaceCandidatePath(referencePoint, targetPoint, path))
        {
            outPath = std::move(path);
            return true;
        }
    } while (std::next_permutation(
                 planeReplacementOrder.begin(),
                 planeReplacementOrder.begin() + static_cast<std::ptrdiff_t>(changedPlaneCount)));

    return false;
}

bool repairSurfaceCandidatePath(
    const LeafClassificationContext &context,
    const Polygon256 &fragment,
    const LeafClassificationPathCandidate &candidate,
    std::vector<Segment256> &outPath)
{
    outPath.clear();
    if (!candidate.targetPoint.hasUniqueIntersection() ||
            !fragment.containsStrictly(candidate.targetPoint))
        return false;

    if (buildAxisRepairPath(
                context.localReference.point,
                candidate.targetPoint,
                context.aabb,
                outPath))
        return true;

    return buildPlaneReplacementRepairPath(
               context.localReference.point,
               candidate.targetPoint,
               context.aabb,
               outPath);
}

bool registerUniqueLeafClassificationCandidatePath(
    LeafClassificationContext &context,
    LeafClassificationAttemptStats &attemptStats,
    const std::vector<Segment256> &path)
{
    std::vector<HomPoint4i> signature = makePathSignature(path);
    if (signature.empty())
    {
        ++context.solveMetrics.leafClassificationCandidateRejectedCount;
        return false;
    }

    if (hasSeenPathSignature(attemptStats, signature))
    {
        ++context.solveMetrics.leafClassificationCandidateDuplicateSkipCount;
        return false;
    }

    attemptStats.uniquePathSignatures.push_back(std::move(signature));
    ++context.solveMetrics.leafClassificationCandidateUniqueCount;
    return true;
}

bool prepareLeafClassificationCandidate(
    LeafClassificationContext &context,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats,
    LeafClassificationPathCandidate &candidate)
{
    ++context.solveMetrics.leafClassificationCandidateGeneratedCount;
    if (!candidate.targetPoint.hasUniqueIntersection() ||
            !fragment.containsStrictly(candidate.targetPoint))
    {
        ++context.solveMetrics.leafClassificationCandidateRejectedCount;
        return false;
    }

    if (!isTraceableSurfaceCandidatePath(
                context.localReference.point,
                candidate.targetPoint,
                candidate.path))
    {
        REEMBER_PROFILE_ZONE("LeafClassification::candidateRepair");
        ++context.solveMetrics.leafClassificationCandidateRepairAttemptCount;

        std::vector<Segment256> repairedPath;
        if (!repairSurfaceCandidatePath(context, fragment, candidate, repairedPath))
        {
            ++context.solveMetrics.leafClassificationCandidateRejectedCount;
            return false;
        }

        candidate.path = std::move(repairedPath);
        ++context.solveMetrics.leafClassificationCandidateRepairSuccessCount;
    }

    return registerUniqueLeafClassificationCandidatePath(context, attemptStats, candidate.path);
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
    LeafClassificationTraceStage stage,
    LeafClassificationCandidateKind kind,
    const LeafClassificationPathCandidate &candidate)
{
    REEMBER_PROFILE_ZONE("LeafClassification::traceCandidate");

    ++context.solveMetrics.leafClassificationTraceAttemptCount;
    if (kind == LeafClassificationCandidateKind::AxisPath)
    {
        ++attemptStats.axisPathAttemptCount;
        ++context.solveMetrics.leafClassificationAxisPathAttemptCount;
    }
    else
    {
        ++attemptStats.planeReplacementPathAttemptCount;
        ++context.solveMetrics.leafClassificationPlaneReplacementPathAttemptCount;
    }

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
    recordLeafClassificationStatus(context.solveMetrics, stage, status);
    if (status == SUCCESS)
    {
        context.classifiedFragments.push_back(
            ClassifiedFragment{fragment, std::move(frontWNV), std::move(backWNV)});
        attemptStats.classified = true;
    }

    return status;
}

LeafClassificationTraceResult prepareAndTraceLeafClassificationCandidate(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats,
    LeafClassificationTraceStage stage,
    LeafClassificationCandidateKind kind,
    LeafClassificationPathCandidate candidate)
{
    if (!prepareLeafClassificationCandidate(context, fragment, attemptStats, candidate))
        return LeafClassificationTraceResult{};

    LeafClassificationTraceResult result{
        true,
        traceLeafClassificationCandidate(
            context,
            fragmentIndex,
            fragment,
            attemptStats,
            stage,
            kind,
            candidate)};
    if (result.status != INPUT_INVALID)
        return result;

    // `INPUT_INVALID` 表示候选构造违反 trace 前置条件，只修复当前候选；
    // 若局部重建仍不可用，继续尝试后续候选，而不是把它当作路径退化全局放大。
    REEMBER_PROFILE_ZONE("LeafClassification::candidateRepair");
    ++context.solveMetrics.leafClassificationCandidateRepairAttemptCount;

    std::vector<Segment256> repairedPath;
    if (!repairSurfaceCandidatePath(context, fragment, candidate, repairedPath))
    {
        ++context.solveMetrics.leafClassificationCandidateRejectedCount;
        return LeafClassificationTraceResult{false, INPUT_INVALID};
    }

    if (!registerUniqueLeafClassificationCandidatePath(context, attemptStats, repairedPath))
        return LeafClassificationTraceResult{false, INPUT_INVALID};

    candidate.path = std::move(repairedPath);
    ++context.solveMetrics.leafClassificationCandidateRepairSuccessCount;
    return LeafClassificationTraceResult{
        true,
        traceLeafClassificationCandidate(
            context,
            fragmentIndex,
            fragment,
            attemptStats,
            stage,
            kind,
            candidate)};
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

        if constexpr (kLeafClassificationDebug)
        {
            attemptStats.debugLog << "axis_path_candidate "
                                  << formatPathForDebug(candidate.path);
        }
        const LeafClassificationTraceResult traceResult =
            prepareAndTraceLeafClassificationCandidate(
                context,
                fragmentIndex,
                fragment,
                attemptStats,
                LeafClassificationTraceStage::CentroidAxis,
                LeafClassificationCandidateKind::AxisPath,
                std::move(candidate));
        if (!traceResult.traced)
            return true;

        const traceStatus status = traceResult.status;
        if constexpr (kLeafClassificationDebug)
            attemptStats.debugLog << " status=" << traceStatusName(status) << "\n";
        if (status == SUCCESS)
            return false;

        if (!isRecoverableClassificationTraceStatus(status))
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
            [&](const detail::PlaneReplacementBuildSignature &signature, const PlanePoint3i &)
        {
            return shouldSkipRepeatedPlaneReplacementBuild(attemptStats, signature);
        },
            [&](LeafClassificationPathCandidate candidate)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::planeReplacementPathCandidate");

            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << label << " "
                                      << formatPathForDebug(candidate.path);
            const LeafClassificationTraceResult traceResult =
                prepareAndTraceLeafClassificationCandidate(
                    context,
                    fragmentIndex,
                    fragment,
                    attemptStats,
                    LeafClassificationTraceStage::InsetReplacement,
                    LeafClassificationCandidateKind::PlaneReplacementPath,
                    std::move(candidate));
            if (!traceResult.traced)
                return true;

            const traceStatus status = traceResult.status;
            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << " status=" << traceStatusName(status) << "\n";
            if (status == SUCCESS)
                return false;

            if (!isRecoverableClassificationTraceStatus(status))
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
                LeafClassificationCandidateKind kind) -> bool
        {
            std::vector<Segment256> fullPath;
            fullPath.reserve(prefix.size() + candidate.path.size());
            fullPath.insert(fullPath.end(), prefix.begin(), prefix.end());
            fullPath.insert(
                fullPath.end(),
                std::make_move_iterator(candidate.path.begin()),
                std::make_move_iterator(candidate.path.end()));

            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << label << " " << formatPathForDebug(fullPath);
            const LeafClassificationTraceResult traceResult =
                prepareAndTraceLeafClassificationCandidate(
                    context,
                    fragmentIndex,
                    fragment,
                    attemptStats,
                    LeafClassificationTraceStage::BridgeRescue,
                    kind,
                    LeafClassificationPathCandidate{
                        candidate.targetPoint,
                        std::move(fullPath)});
            if (!traceResult.traced)
                return true;

            const traceStatus status = traceResult.status;
            if constexpr (kLeafClassificationDebug)
                attemptStats.debugLog << " status=" << traceStatusName(status) << "\n";
            if (status == SUCCESS)
                return false;

            if (!isRecoverableClassificationTraceStatus(status))
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
                LeafClassificationCandidateKind::AxisPath);
        });

        if (attemptStats.classified || !allowFallback)
            return allowFallback;

        enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints(
            bridgePoint,
            attemptStats.bridgeRescueTargets,
            context.aabb,
            [&](const detail::PlaneReplacementBuildSignature &signature, const PlanePoint3i &)
        {
            return shouldSkipRepeatedPlaneReplacementBuild(attemptStats, signature);
        },
            [&](LeafClassificationPathCandidate candidate)
        {
            REEMBER_PROFILE_ZONE("LeafClassification::bridgeRescuePlaneReplacementCandidate");
            return traceWithPrefix(
                std::move(candidate),
                "bridge_plane_replacement_candidate",
                LeafClassificationCandidateKind::PlaneReplacementPath);
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

    leafFragmentsAliasPolygons_ = false;
    leafFragments_.clear();
    classifiedFragments_.clear();
    leafFragmentCount_ = 0;
    classifiedFragmentCount_ = 0;
    if (discarded_ || polygons_.empty())
        return;

    if (singleOperandPolicy_.maySkipLeafBsp)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::skipLeafBspBySingleOperandAssumption");
        ++solveMetrics_.singleOperandLeafBspSkipCount;
        leafFragmentsAliasPolygons_ = true;
        leafFragmentCount_ = polygonCount_;
        return;
    }

    ++solveMetrics_.leafBspBuildCount;
    leafFragments_ = buildLeafArrangement(polygons_);
    leafFragmentCount_ = leafFragments_.size();

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

bool SubdivisionSolver::tryClassifySingleOperandLeafByBulkReuse()
{
    if (!singleOperandPolicy_.mayReuseLeafClassification || !leafFragmentsAliasPolygons_ || polygons_.empty())
        return false;

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

    LeafClassificationAttemptStats attemptStats;
    Polygon256 &representativeFragment = polygons_.front();
    const LeafClassificationResult classificationResult =
        classifyLeafFragment(context, 0u, representativeFragment, attemptStats);
    if (classificationResult == LeafClassificationResult::Failure)
        throwLeafClassificationFailure(0u, depth_, representativeFragment, attemptStats);

    // bulk reuse 只真正分类一个代表片段，其余片段沿用同一前后侧 WNV；
    // 这里补回诊断计数，避免 fast path 让 reuse 指标失真。
    solveMetrics_.singleOperandClassificationReuseCount += (polygonCount_ > 0u) ? (polygonCount_ - 1u) : 0u;
    classifiedFragmentCount_ = polygonCount_;
    const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
    const BoolStatus frontStatus = evaluateBooleanIndicator(classifiedFragment.frontWNV);
    const BoolStatus backStatus = evaluateBooleanIndicator(classifiedFragment.backWNV);

    resultFragments_.clear();
    if (frontStatus == OUT && backStatus == IN)
    {
        resultFragments_.swap(polygons_);
    }
    else if (frontStatus == IN && backStatus == OUT)
    {
        resultFragments_.reserve(polygons_.size());
        for (const Polygon256 &polygon : polygons_)
            resultFragments_.push_back(reversePolygonOrientation(polygon));
        polygons_.clear();
    }
    else
    {
        polygons_.clear();
    }

    return true;
}

// 对每个叶片片段追踪到严格内部点；分类失败属于不可恢复错误。
void SubdivisionSolver::classifyLeafFragmentsAndCollectResults()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::classifyLeafFragmentsAndCollectResults");

    resultFragments_.clear();
    classifiedFragments_.clear();
    const std::size_t sourceFragmentCount = leafFragmentsAliasPolygons_ ? polygons_.size() : leafFragments_.size();
    if (discarded_ || sourceFragmentCount == 0)
        return;

    if (tryClassifySingleOperandLeafByBulkReuse())
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

    for (std::size_t fragmentIndex = 0; fragmentIndex < sourceFragmentCount; ++fragmentIndex)
    {
        Polygon256 &fragment = leafFragmentsAliasPolygons_ ? polygons_[fragmentIndex] : leafFragments_[fragmentIndex];
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

    classifiedFragmentCount_ = sourceFragmentCount;
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
