/**
 * @file bool_problem.cpp
 * @brief 实现公开布尔问题门面及输入校验。
 */
#include "bool_problem.h"

#include "core/perf_tracing.h"
#include "core/parallel_solve_context.h"
#include "core/solver_shared.h"
#include "core/subdivision_solver.h"

#include <algorithm>
#include <limits>
#include <oneapi/tbb/info.h>
#include <oneapi/tbb/task_arena.h>
#include <oneapi/tbb/task_group.h>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
namespace
{
void validateSolveInputPolygons(const std::vector<Polygon256> &polygons)
{
    REEMBER_PROFILE_ZONE("validateSolveInputPolygons");
    for (std::size_t polygonIndex = 0; polygonIndex < polygons.size(); ++polygonIndex)
    {
        const Polygon256 &polygon = polygons[polygonIndex];
        if (!polygon.isValid())
        {
            std::ostringstream message;
            message << "BoolProblem received an invalid polygon at index "
                    << polygonIndex
                    << ".";
            throw std::runtime_error(message.str());
        }
        if (!detail::isCanonicalBinaryOperandWNTV(polygon.WNTV))
        {
            std::ostringstream message;
            message << "BoolProblem expects canonical binary WNTV tags at polygon index "
                    << polygonIndex
                    << ".";
            throw std::runtime_error(message.str());
        }
    }

}

void validateSceneAABB(const AABB3i &sceneAABB)
{
    if (isValidAABB(sceneAABB))
        return;

    std::ostringstream message;
    message << "BoolProblem requires a valid input scene AABB.";
    throw std::runtime_error(message.str());
}

void throwIfSolveAttempted(bool solveAttempted, const char *method)
{
    if (!solveAttempted)
        return;

    std::ostringstream message;
    message << "BoolProblem::" << method
            << "() cannot be called after solve(); each BoolProblem instance is single-use.";
    throw std::runtime_error(message.str());
}

std::size_t resolveEffectiveThreadCount(std::size_t requestedThreadCount) noexcept
{
    if (requestedThreadCount > 0)
        return requestedThreadCount;

    const int defaultConcurrency = oneapi::tbb::info::default_concurrency();
    return static_cast<std::size_t>(std::max(1, defaultConcurrency));
}

int toTaskArenaConcurrency(std::size_t threadCount) noexcept
{
    const std::size_t maxInt = static_cast<std::size_t>(std::numeric_limits<int>::max());
    return static_cast<int>(std::min(threadCount, maxInt));
}
}

BoolProblem::BoolProblem(std::size_t leafPolygonThreshold) noexcept
    : leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold)
{
}

void BoolProblem::setOperation(BoolOp op)
{
    throwIfSolveAttempted(solveAttempted_, "setOperation");
    op_ = op;
}

void BoolProblem::setOperandAssumptions(
    BoolOperandAssumptions lhsAssumptions,
    BoolOperandAssumptions rhsAssumptions)
{
    throwIfSolveAttempted(solveAttempted_, "setOperandAssumptions");
    lhsAssumptions_ = lhsAssumptions;
    rhsAssumptions_ = rhsAssumptions;
}

void BoolProblem::setThreadCount(std::size_t threadCount)
{
    throwIfSolveAttempted(solveAttempted_, "setThreadCount");
    threadCount_ = threadCount;
}

void BoolProblem::setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs)
{
    throwIfSolveAttempted(solveAttempted_, "setOperands");

    std::vector<Polygon256> lhsCopy = lhs;
    std::vector<Polygon256> rhsCopy = rhs;

    assignOperandWNTV(lhsCopy, detail::kLhsOperandIndex);

    assignOperandWNTV(rhsCopy, detail::kRhsOperandIndex);

    polygons_.clear();
    polygons_.reserve(lhsCopy.size() + rhsCopy.size());
    polygons_.insert(polygons_.end(), lhsCopy.begin(), lhsCopy.end());
    polygons_.insert(polygons_.end(), rhsCopy.begin(), rhsCopy.end());
}

void BoolProblem::solve(const AABB3i &sceneAABB)
{
    REEMBER_PROFILE_ZONE("BoolProblem::solve");

    throwIfSolveAttempted(solveAttempted_, "solve");
    solveAttempted_ = true;

    const std::size_t effectiveThreadCount = resolveEffectiveThreadCount(threadCount_);
    solveMetrics_.effectiveThreadCount = effectiveThreadCount;

    if (polygons_.empty())
    {
        discarded_ = true;
        return;
    }

    validateSceneAABB(sceneAABB);
    validateSolveInputPolygons(polygons_);

    if (effectiveThreadCount > 1)
    {
        oneapi::tbb::task_group_context taskGroupContext;
        ParallelSolveContext parallelContext{
            true,
            effectiveThreadCount,
            &taskGroupContext};
        SubdivisionSolver solver(
            op_,
            leafPolygonThreshold_,
            polygons_,
            sceneAABB,
            &parallelContext,
            lhsAssumptions_,
            rhsAssumptions_);
        oneapi::tbb::task_arena arena(toTaskArenaConcurrency(effectiveThreadCount));
        arena.execute([&solver]()
        {
            solver.solve();
        });
        discarded_ = solver.isDiscarded();
        solver.extractResultFragments(resultFragments_);
        if (resultFragments_.empty())
            discarded_ = true;
        solver.extractLeafSummaries(leafSummaries_);
        solveMetrics_ = solver.solveMetrics();
        return;
    }

    SubdivisionSolver solver(
        op_,
        leafPolygonThreshold_,
        polygons_,
        sceneAABB,
        nullptr,
        lhsAssumptions_,
        rhsAssumptions_);
    solver.solve();
    discarded_ = solver.isDiscarded();
    solver.extractResultFragments(resultFragments_);
    if (resultFragments_.empty())
        discarded_ = true;
    solver.extractLeafSummaries(leafSummaries_);
    solveMetrics_ = solver.solveMetrics();
}

bool BoolProblem::isDiscarded() const noexcept
{
    return discarded_;
}

const std::vector<Polygon256> &BoolProblem::resultFragments() const noexcept
{
    return resultFragments_;
}

const std::vector<BoolLeafSummary> &BoolProblem::leafSummaries() const noexcept
{
    return leafSummaries_;
}

const BoolSolveMetrics &BoolProblem::solveMetrics() const noexcept
{
    return solveMetrics_;
}

void BoolProblem::assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t hotIndex)
{
    for (Polygon256 &polygon : polygons)
    {
        polygon.WNTV.assign(detail::kBinaryWnvDimension, 0);
        if (hotIndex < detail::kBinaryWnvDimension)
            polygon.WNTV[hotIndex] = 1;
    }
}
}
