#include "bool_problem.h"
#include "core/logging.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kBoolProblemSolveScope = "BoolProblem::solve";
        constexpr const char *kBoolProblemRootReferenceScope = "BoolProblem::initializeRootReference";

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

        std::size_t computeWNVSize(const std::vector<Polygon256> &polygons) noexcept
        {
            std::size_t dimension = 0;
            for (const Polygon256 &polygon : polygons)
            {
                dimension = std::max(dimension, polygon.WNTV.size());
            }

            return dimension;
        }

        void validateSolveInputPolygons(const std::vector<Polygon256> &polygons)
        {
            const std::size_t wnvDimension = computeWNVSize(polygons);
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

    BoolProblem::BoolProblem(std::size_t leafPolygonThreshold) noexcept
        : leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold)
    {
    }

    void BoolProblem::clear() noexcept
    {
        polygons_.clear();
        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setOperation(BoolOp op) noexcept
    {
        if (op_ == op)
        {
            return;
        }

        op_ = op;
        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setLeafPolygonThreshold(std::size_t threshold) noexcept
    {
        const std::size_t sanitized = (threshold == 0) ? 1 : threshold;
        if (leafPolygonThreshold_ == sanitized)
        {
            return;
        }

        leafPolygonThreshold_ = sanitized;
        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::addPolygon(const Polygon256 &polygon)
    {
        Polygon256 copy = polygon;
        polygons_.push_back(std::move(copy));

        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setPolygons(const std::vector<Polygon256> &polygons)
    {
        polygons_ = polygons;

        depth_ = 0;
        resetSubdivisionState();
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

        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::solve()
    {
        depth_ = 0;
        resetSubdivisionState();

        logBoolInfo(
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

        try
        {
            validateSolveInputPolygons(polygons_);
        }
        catch (const std::runtime_error &ex)
        {
            emitLog(LogLevel::Error, LogCategory::BoolProblem, kBoolProblemSolveScope, ex.what());
            throw;
        }

        aabb_ = computeAABB(polygons_);
        if (!isValidAABB(aabb_))
        {
            std::ostringstream message;
            message << "BoolProblem failed to compute a valid root AABB root_aabb="
                    << formatAABB(aabb_)
                    << ".";
            emitLog(LogLevel::Error, LogCategory::BoolProblem, kBoolProblemSolveScope, message.str());
            throw std::runtime_error(message.str());
        }

        logBoolInfo(
            kBoolProblemSolveScope,
            [this]()
            {
                return "Computed valid root AABB root_aabb=" + formatAABB(aabb_) + ".";
            });

        initializeRootReference();
        solveRecursive();

        logBoolInfo(
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

    bool BoolProblem::isLeaf() const noexcept
    {
        return isLeaf_;
    }

    bool BoolProblem::isDiscarded() const noexcept
    {
        return discarded_;
    }

    bool BoolProblem::isSolved() const noexcept
    {
        return solved_;
    }

    std::size_t BoolProblem::polygonCount() const noexcept
    {
        return polygons_.size();
    }

    std::size_t BoolProblem::depth() const noexcept
    {
        return depth_;
    }

    const AABB3i &BoolProblem::aabb() const noexcept
    {
        return aabb_;
    }

    const Plane3i &BoolProblem::splitPlane() const noexcept
    {
        return splitPlane_;
    }

    const SubdivisionRefState &BoolProblem::reference() const noexcept
    {
        return reference_;
    }

    const std::vector<Polygon256> &BoolProblem::polygons() const noexcept
    {
        return polygons_;
    }

    const std::vector<Polygon256> &BoolProblem::leafFragments() const noexcept
    {
        return leafFragments_;
    }

    const std::vector<Polygon256> &BoolProblem::resultFragments() const noexcept
    {
        return resultFragments_;
    }

    const BoolProblem *BoolProblem::leftChild() const noexcept
    {
        return leftChild_.get();
    }

    const BoolProblem *BoolProblem::rightChild() const noexcept
    {
        return rightChild_.get();
    }

    BoolProblem *BoolProblem::leftChild() noexcept
    {
        return leftChild_.get();
    }

    BoolProblem *BoolProblem::rightChild() noexcept
    {
        return rightChild_.get();
    }

    void BoolProblem::collectLeafProblems(std::vector<const BoolProblem *> &outLeaves) const
    {
        collectLeafProblemsRecursive(this, outLeaves);
    }

    void BoolProblem::collectLeafProblems(std::vector<BoolProblem *> &outLeaves)
    {
        collectLeafProblemsRecursive(this, outLeaves);
    }

    // 重置由求解流程派生出的状态，保留调用方配置和输入 polygon soup。
    void BoolProblem::resetSubdivisionState() noexcept
    {
        isLeaf_ = true;
        discarded_ = false;
        solved_ = false;
        splitPlane_ = Plane3i();
        aabb_ = AABB3i();
        reference_ = SubdivisionRefState();
        leafFragments_.clear();
        classifiedFragments_.clear();
        resultFragments_.clear();
        leftChild_.reset();
        rightChild_.reset();
    }

    // 使用根 AABB 的最小角点建立初始参考点，并按输入 WNTV 维度置零。
    void BoolProblem::initializeRootReference()
    {
        reference_.point = getAABBCornerPoint(aabb_, false, false, false);
        reference_.wnv.assign(computeWNVSize(polygons_), 0);

        logBoolDebug(
            kBoolProblemRootReferenceScope,
            [this]()
            {
                std::ostringstream message;
                message << "Initialized root reference depth=" << depth_
                        << " wnv_dimension=" << reference_.wnv.size()
                        << " point=" << reference_.point
                        << ".";
                return message.str();
            });
    }

    // 写入基础 WNTV。
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

    // 只读遍历版本：跳过空节点和已丢弃节点，只返回有效叶子。
    void BoolProblem::collectLeafProblemsRecursive(const BoolProblem *node, std::vector<const BoolProblem *> &outLeaves)
    {
        if (!node || node->discarded_)
        {
            return;
        }

        if (node->isLeaf_)
        {
            outLeaves.push_back(node);
            return;
        }

        collectLeafProblemsRecursive(node->leftChild_.get(), outLeaves);
        collectLeafProblemsRecursive(node->rightChild_.get(), outLeaves);
    }

    // 可变遍历版本：与只读版本保持相同过滤规则。
    void BoolProblem::collectLeafProblemsRecursive(BoolProblem *node, std::vector<BoolProblem *> &outLeaves)
    {
        if (!node || node->discarded_)
        {
            return;
        }

        if (node->isLeaf_)
        {
            outLeaves.push_back(node);
            return;
        }

        collectLeafProblemsRecursive(node->leftChild_.get(), outLeaves);
        collectLeafProblemsRecursive(node->rightChild_.get(), outLeaves);
    }
}
