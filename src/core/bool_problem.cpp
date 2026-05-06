/**
 * @file bool_problem.cpp
 * @brief 实现公开布尔问题门面及输入校验。
 */
#include "bool_problem.h"

#include "core/logging.h"
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
        const char *kBoolProblemSolveScope = "BoolProblem::solve";

        void validateSolveInputPolygons(const std::vector<Polygon256> &polygons)
        {
            const std::size_t wnvDimension = detail::computeWNVSize(polygons);
            if (wnvDimension < 2)
            {
                throw std::runtime_error("BoolProblem requires at least two WNV dimensions.");
            }

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
                if (polygon.WNTV.size() != wnvDimension)
                {
                    std::ostringstream message;
                    message << "BoolProblem received inconsistent WNTV dimension at polygon index "
                            << polygonIndex
                            << ": expected "
                            << wnvDimension
                            << ", got "
                            << polygon.WNTV.size()
                            << ".";
                    throw std::runtime_error(message.str());
                }
            }
        }

        void preprocessSolveInputPolygons(std::vector<Polygon256> &polygons)
        {
            for (Polygon256 &polygon : polygons)
            {
                polygon.precomputeVertices();
            }
        }

        const char *boolOpName(BoolOp op) noexcept
        {
            switch (op)
            {
            case BoolOp::Union:
                return "union";
            case BoolOp::Intersection:
                return "intersection";
            case BoolOp::Difference:
                return "difference";
            }

            return "unknown";
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
        {
            return;
        }

        op_ = op;
        resetSolveState();
    }

    void BoolProblem::setLeafPolygonThreshold(std::size_t threshold) noexcept
    {
        const std::size_t sanitized = (threshold == 0) ? 1 : threshold;
        if (leafPolygonThreshold_ == sanitized)
        {
            return;
        }

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

    void BoolProblem::addPolygon(const Polygon256 &polygon)
    {
        Polygon256 copy = polygon;
        polygons_.push_back(std::move(copy));

        resetSolveState();
    }

    void BoolProblem::setPolygons(const std::vector<Polygon256> &polygons)
    {
        polygons_ = polygons;
        resetSolveState();
    }

    void BoolProblem::setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs)
    {
        std::vector<Polygon256> lhsCopy = lhs;
        std::vector<Polygon256> rhsCopy = rhs;

        assignOperandWNTV(lhsCopy, 2, 0);
        assignOperandWNTV(rhsCopy, 2, 1);

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

        detail::logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                std::ostringstream message;
                message << "Starting solve operation=" << boolOpName(op_)
                        << " polygon_count=" << polygons_.size()
                        << " leaf_threshold=" << leafPolygonThreshold_
                        << ".";
                return message.str();
            });

        if (polygons_.empty())
        {
            discarded_ = true;
            solved_ = true;
            emitLog(
                LogLevel::Info,
                LogCategory::BoolProblem,
                kBoolProblemSolveScope,
                "Solve ended early because the input polygon soup is empty.");
            return;
        }

        preprocessSolveInputPolygons(polygons_);

        try
        {
            validateSolveInputPolygons(polygons_);
        }
        catch (const std::runtime_error &ex)
        {
            emitLog(LogLevel::Error, LogCategory::BoolProblem, kBoolProblemSolveScope, ex.what());
            throw;
        }

        SubdivisionSolver solver(op_, leafPolygonThreshold_, polygons_, lhsAssumptions_, rhsAssumptions_);
        solver.solve();

        discarded_ = solver.isDiscarded();
        resultFragments_ = solver.resultFragments();
        leafSummaries_ = solver.leafSummaries();
        solveMetrics_ = solver.solveMetrics();
        solved_ = true;

        detail::logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                std::ostringstream message;
                message << "Solve finished discarded=" << discarded_
                        << " result_fragments=" << resultFragments_.size()
                        << ".";
                return message.str();
            });
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

    void BoolProblem::assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t dimension, std::size_t hotIndex)
    {
        for (Polygon256 &polygon : polygons)
        {
            polygon.WNTV.assign(dimension, 0);
            if (hotIndex < dimension)
            {
                polygon.WNTV[hotIndex] = 1;
            }
        }
    }
}
