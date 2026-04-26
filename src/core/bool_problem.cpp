#include "bool_problem.h"

#include <algorithm>
#include <utility>

namespace ember
{
    namespace
    {
        std::size_t computeWNVSize(const std::vector<Polygon256> &polygons) noexcept
        {
            std::size_t dimension = 0;
            for (const Polygon256 &polygon : polygons)
            {
                dimension = std::max(dimension, polygon.WNTV.size());
            }

            return dimension;
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
        copy.WNVF.clear();
        copy.WNVB.clear();
        polygons_.push_back(std::move(copy));

        depth_ = 0;
        resetSubdivisionState();
    }

    void BoolProblem::setPolygons(const std::vector<Polygon256> &polygons)
    {
        polygons_ = polygons;
        clearClassificationState(polygons_);

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

        if (polygons_.empty())
        {
            discarded_ = true;
            solved_ = true;
            return;
        }

        aabb_ = computeAABB(polygons_);
        if (!isValidAABB(aabb_))
        {
            discarded_ = true;
            solved_ = true;
            return;
        }

        initializeRootReference();
        solveRecursive();
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

    void BoolProblem::resetSubdivisionState() noexcept
    {
        isLeaf_ = true;
        discarded_ = false;
        solved_ = false;
        splitPlane_ = Plane3i();
        aabb_ = AABB3i();
        reference_ = SubdivisionRefState();
        leftChild_.reset();
        rightChild_.reset();
    }

    void BoolProblem::initializeRootReference()
    {
        reference_.point = getAABBCornerPoint(aabb_, false, false, false);
        reference_.wnv.assign(computeWNVSize(polygons_), 0);
        reference_.hasExactWNV = true;
    }

    void BoolProblem::solveRecursive()
    {
        if (polygons_.empty() || !isValidAABB(aabb_))
        {
            discarded_ = true;
            isLeaf_ = true;
            solved_ = true;
            return;
        }

        if (shouldStopSubdivision())
        {
            isLeaf_ = true;
            solved_ = true;
            return;
        }

        AABBSplit3i split;
        if (!splitAABBAtMidpoint(aabb_, split))
        {
            isLeaf_ = true;
            solved_ = true;
            return;
        }

        splitPlane_ = split.splitPlane;
        if (!createChildrenFromSplit(split))
        {
            discarded_ = true;
            isLeaf_ = true;
            solved_ = true;
            return;
        }

        isLeaf_ = false;

        if (leftChild_)
        {
            leftChild_->solveRecursive();
        }
        if (rightChild_)
        {
            rightChild_->solveRecursive();
        }

        discarded_ =
            (!leftChild_ || leftChild_->discarded_) &&
            (!rightChild_ || rightChild_->discarded_);
        solved_ = true;
    }

    bool BoolProblem::shouldStopSubdivision() const noexcept
    {
        return polygons_.size() <= leafPolygonThreshold_ || !hasSplittableAxis(aabb_);
    }

    bool BoolProblem::createChildrenFromSplit(const AABBSplit3i &split)
    {
        std::vector<Polygon256> leftPolygons;
        std::vector<Polygon256> rightPolygons;
        leftPolygons.reserve(polygons_.size());
        rightPolygons.reserve(polygons_.size());

        for (const Polygon256 &polygon : polygons_)
        {
            Polygon256 leftPolygon;
            if (detail::clipPolygonToHalfSpace(polygon, split.splitPlane, true, leftPolygon))
            {
                leftPolygons.push_back(std::move(leftPolygon));
            }

            Polygon256 rightPolygon;
            if (detail::clipPolygonToHalfSpace(polygon, split.splitPlane, false, rightPolygon))
            {
                rightPolygons.push_back(std::move(rightPolygon));
            }
        }

        if (leftPolygons.empty() && rightPolygons.empty())
        {
            return false;
        }

        if (!leftPolygons.empty())
        {
            leftChild_ = std::make_unique<BoolProblem>(leafPolygonThreshold_);
            leftChild_->op_ = op_;
            leftChild_->depth_ = depth_ + 1;
            leftChild_->polygons_ = std::move(leftPolygons);
            leftChild_->aabb_ = split.left;
            leftChild_->reference_ = makeChildReference(split.left);
        }

        if (!rightPolygons.empty())
        {
            rightChild_ = std::make_unique<BoolProblem>(leafPolygonThreshold_);
            rightChild_->op_ = op_;
            rightChild_->depth_ = depth_ + 1;
            rightChild_->polygons_ = std::move(rightPolygons);
            rightChild_->aabb_ = split.right;
            rightChild_->reference_ = makeChildReference(split.right);
        }

        return true;
    }

    SubdivisionRefState BoolProblem::makeChildReference(const AABB3i &childBox) const
    {
        if (isPointInsideOrOnAABB(reference_.point, childBox))
        {
            return reference_;
        }

        SubdivisionRefState childRef = reference_;
        childRef.point = projectPointToAABB(reference_.point, childBox);
        if (!childRef.point.hasUniqueIntersection())
        {
            childRef.point = getAABBCornerPoint(childBox, false, false, false);
        }

        // TODO：后续接入路径传播后，这里需要真正更新 childRef.wnv。
        childRef.hasExactWNV = false;
        return childRef;
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

            polygon.WNVF.clear();
            polygon.WNVB.clear();
        }
    }

    void BoolProblem::clearClassificationState(std::vector<Polygon256> &polygons)
    {
        for (Polygon256 &polygon : polygons)
        {
            polygon.WNVF.clear();
            polygon.WNVB.clear();
        }
    }

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
