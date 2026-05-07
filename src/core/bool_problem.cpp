/**
 * @file bool_problem.cpp
 * @brief 实现公开布尔问题门面及输入校验。
 */
#include "bool_problem.h"

#include "core/perf_tracing.h"
#include "core/solver_shared.h"
#include "core/subdivision_solver.h"

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

void preprocessSolveInputPolygons(std::vector<Polygon256> &polygons)
{
    REEMBER_PROFILE_ZONE("preprocessSolveInputPolygons");
    for (Polygon256 &polygon : polygons)
        polygon.precomputeVertices();
}
}

BoolProblem::BoolProblem(std::size_t leafPolygonThreshold) noexcept
    : leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold)
{
}

void BoolProblem::clear() noexcept
{
    polygons_.clear();
    resetSolveState();
}

void BoolProblem::setOperation(BoolOp op) noexcept
{
    if (op_ == op)
        return;

    op_ = op;
    resetSolveState();
}

void BoolProblem::setLeafPolygonThreshold(std::size_t threshold) noexcept
{
    const std::size_t sanitized = (threshold == 0) ? 1 : threshold;
    if (leafPolygonThreshold_ == sanitized)
        return;

    leafPolygonThreshold_ = sanitized;
    resetSolveState();
}

void BoolProblem::setOperandAssumptions(
    BoolOperandAssumptions lhsAssumptions,
    BoolOperandAssumptions rhsAssumptions) noexcept
{
    lhsAssumptions_ = lhsAssumptions;
    rhsAssumptions_ = rhsAssumptions;
    resetSolveState();
}

void BoolProblem::setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs)
{
    std::vector<Polygon256> lhsCopy = lhs;
    std::vector<Polygon256> rhsCopy = rhs;

    assignOperandWNTV(lhsCopy, detail::kLhsOperandIndex);
    assignOperandWNTV(rhsCopy, detail::kRhsOperandIndex);

    polygons_.clear();
    polygons_.reserve(lhsCopy.size() + rhsCopy.size());
    polygons_.insert(polygons_.end(), lhsCopy.begin(), lhsCopy.end());
    polygons_.insert(polygons_.end(), rhsCopy.begin(), rhsCopy.end());

    resetSolveState();
}

void BoolProblem::solve()
{
    REEMBER_PROFILE_ZONE("BoolProblem::solve");

    resetSolveState();

    if (polygons_.empty())
    {
        discarded_ = true;
        solved_ = true;
        return;
    }

    preprocessSolveInputPolygons(polygons_);
    validateSolveInputPolygons(polygons_);

    SubdivisionSolver solver(op_, leafPolygonThreshold_, polygons_, lhsAssumptions_, rhsAssumptions_);
    solver.solve();

    discarded_ = solver.isDiscarded();
    resultFragments_ = solver.resultFragments();
    leafSummaries_ = solver.leafSummaries();
    solveMetrics_ = solver.solveMetrics();
    solved_ = true;
}

bool BoolProblem::isDiscarded() const noexcept
{
    return discarded_;
}

bool BoolProblem::isSolved() const noexcept
{
    return solved_;
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

void BoolProblem::resetSolveState() noexcept
{
    discarded_ = false;
    solved_ = false;
    resultFragments_.clear();
    leafSummaries_.clear();
    solveMetrics_ = BoolSolveMetrics();
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
