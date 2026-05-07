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

#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace ember
{
namespace
{
struct LeafClassificationAttemptStats
{
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
};

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
        REEMBER_PROFILE_ZONE("LeafClassification::fastCandidate");

        ++context.solveMetrics.leafClassificationFastCandidateCount;
        const traceStatus status = traceLeafClassificationCandidate(
                                       context,
                                       fragmentIndex,
                                       fragment,
                                       attemptStats,
                                       candidate);
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
        REEMBER_PROFILE_ZONE("LeafClassification::fallbackCandidate");

        ++context.solveMetrics.leafClassificationFallbackCandidateCount;
        const traceStatus status = traceLeafClassificationCandidate(
                                       context,
                                       fragmentIndex,
                                       fragment,
                                       attemptStats,
                                       candidate);
        if (status == SUCCESS)
            return false;

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
        REEMBER_PROFILE_ZONE("LeafClassification::normalCandidate");

        ++context.solveMetrics.leafClassificationNormalCandidateCount;
        const traceStatus status = traceLeafClassificationCandidate(
                                       context,
                                       fragmentIndex,
                                       fragment,
                                       attemptStats,
                                       candidate);
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
        REEMBER_PROFILE_ZONE("LeafClassification::interiorBridgeCandidate");

        ++context.solveMetrics.leafClassificationInteriorBridgeCandidateCount;
        const traceStatus status = traceLeafClassificationCandidate(
                                       context,
                                       fragmentIndex,
                                       fragment,
                                       attemptStats,
                                       candidate);
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

bool attemptLeafClassificationPointCandidates(
    LeafClassificationContext &context,
    std::size_t fragmentIndex,
    Polygon256 &fragment,
    LeafClassificationAttemptStats &attemptStats,
    const std::vector<PlanePoint3i> &pointCandidates)
{
    REEMBER_PROFILE_ZONE("LeafClassification::attemptPointCandidates");

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
    REEMBER_PROFILE_ZONE("LeafClassification::classifyLeafFragment");

    std::vector<PlanePoint3i> primaryPointCandidates;
    {
        REEMBER_PROFILE_ZONE("LeafClassification::enumeratePrimaryPointCandidates");
        primaryPointCandidates =
            detail::enumerateLeafClassificationPrimaryPointCandidatesUnchecked(fragment);
    }
    context.solveMetrics.leafClassificationPrimaryPointCandidateCount += primaryPointCandidates.size();
    for (const PlanePoint3i &primaryPointCandidate : primaryPointCandidates)
    {
        (void)primaryPointCandidate;
        REEMBER_PROFILE_ZONE("LeafClassification::primaryPointCandidate");
    }

    const bool shouldTryExpandedPoints = attemptLeafClassificationPointCandidates(
            context,
            fragmentIndex,
            fragment,
            attemptStats,
            primaryPointCandidates);
    if (!attemptStats.classified && shouldTryExpandedPoints)
    {
        std::vector<PlanePoint3i> expandedPointCandidates;
        {
            REEMBER_PROFILE_ZONE("LeafClassification::enumerateExpandedPointCandidates");
            expandedPointCandidates =
                detail::enumerateLeafClassificationPointCandidatesUnchecked(fragment);
        }
        context.solveMetrics.leafClassificationExpandedPointCandidateCount += expandedPointCandidates.size();
        for (const PlanePoint3i &expandedPointCandidate : expandedPointCandidates)
        {
            (void)expandedPointCandidate;
            REEMBER_PROFILE_ZONE("LeafClassification::expandedPointCandidate");
        }
        attemptLeafClassificationPointCandidates(
            context,
            fragmentIndex,
            fragment,
            attemptStats,
            expandedPointCandidates);
    }

    if (attemptStats.classified)
        return LeafClassificationResult::Success;
    if (allowRetryFallback && attemptStats.lastStatus == PATH_INVALID)
        return LeafClassificationResult::RetryPathInvalid;

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

    BoolOperandAssumptions assumptions;
    if (tryGetSingleOperandAssumptions(assumptions) && assumptions.noSelfIntersections)
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
    bool reuseSingleOperandClassification,
    bool hasReusableClassification,
    const WNV &reusableFrontWNV,
    const WNV &reusableBackWNV)
{
    if (!reuseSingleOperandClassification || !hasReusableClassification)
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
    (void)classifyLeafFragmentsAndCollectResults(false);
}

bool SubdivisionSolver::classifyLeafFragmentsAndCollectResults(bool allowRetryFallback)
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::classifyLeafFragmentsAndCollectResults");

    resultFragments_.clear();
    classifiedFragments_.clear();
    if (discarded_ || leafFragments_.empty())
        return true;

    const refPoint localReference(reference_.point, reference_.wnv);
    LeafClassificationContext context{
        localReference,
        polygons_,
        aabb_,
        solveMetrics_,
        classifiedFragments_,
        depth_};
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
            continue;

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
            throw std::runtime_error(summary.str());
        }

        if (reuseSingleOperandClassification && !hasReusableClassification)
        {
            const ClassifiedFragment &classifiedFragment = classifiedFragments_.back();
            reusableFrontWNV = classifiedFragment.frontWNV;
            reusableBackWNV = classifiedFragment.backWNV;
            hasReusableClassification = true;
        }

        appendResultFragmentFromClassification(classifiedFragments_.back());
    }
    return true;
}

bool SubdivisionSolver::trySolveSingleOperandAssumptionLeaf()
{
    BoolOperandAssumptions assumptions;
    if (!tryGetSingleOperandAssumptions(assumptions) ||
            !assumptions.noSelfIntersections ||
            !assumptions.noNestedComponents ||
            polygons_.size() <= leafPolygonThreshold_)
        return false;

    const BoolSolveMetrics metricsSnapshot = solveMetrics_;
    isLeaf_ = true;
    solveLeafArrangement();
    if (!classifyLeafFragmentsAndCollectResults(true))
    {
        solveMetrics_ = metricsSnapshot;
        REEMBER_PROFILE_ZONE("SubdivisionSolver::singleOperandAssumptionFallback");
        ++solveMetrics_.singleOperandAssumptionFallbackCount;
        isLeaf_ = true;
        leafFragments_.clear();
        classifiedFragments_.clear();
        resultFragments_.clear();
        return false;
    }

    REEMBER_PROFILE_ZONE("SubdivisionSolver::singleOperandAssumptionStop");
    ++solveMetrics_.singleOperandAssumptionStopCount;
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
