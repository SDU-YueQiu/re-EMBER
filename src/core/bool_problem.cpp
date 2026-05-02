#include "bool_problem.h"
#include "algorithm/bsp.h"
#include "algorithm/WNV_tracing.h"

#include <algorithm>
#include <iterator>
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

    void BoolProblem::resetSubdivisionState() noexcept
    {
        isLeaf_ = true;
        discarded_ = false;
        solved_ = false;
        splitPlane_ = Plane3i();
        aabb_ = AABB3i();
        reference_ = SubdivisionRefState();
        leafFragments_.clear();
        resultFragments_.clear();
        leftChild_.reset();
        rightChild_.reset();
    }

    void BoolProblem::initializeRootReference()
    {
        reference_.point = getAABBCornerPoint(aabb_, false, false, false);
        reference_.wnv.assign(computeWNVSize(polygons_), 0);
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
            solveLeafArrangement();
            classifyLeafFragmentsAndCollectResults();
            solved_ = true;
            return;
        }

        AABBSplit3i split;
        if (!splitAABBAtMidpoint(aabb_, split))
        {
            isLeaf_ = true;
            solveLeafArrangement();
            classifyLeafFragmentsAndCollectResults();
            solved_ = true;
            return;
        }

        splitPlane_ = split.splitPlane;
        if (!createChildrenFromSplit(split))
        {
            splitPlane_ = Plane3i();
            isLeaf_ = true;
            solveLeafArrangement();
            classifyLeafFragmentsAndCollectResults();
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

        resultFragments_.clear();
        if (leftChild_ && !leftChild_->discarded_)
        {
            resultFragments_.insert(
                resultFragments_.end(),
                leftChild_->resultFragments_.begin(),
                leftChild_->resultFragments_.end());
        }
        if (rightChild_ && !rightChild_->discarded_)
        {
            resultFragments_.insert(
                resultFragments_.end(),
                rightChild_->resultFragments_.begin(),
                rightChild_->resultFragments_.end());
        }

        discarded_ =
            (!leftChild_ || leftChild_->discarded_) &&
            (!rightChild_ || rightChild_->discarded_);
        solved_ = true;
    }

    void BoolProblem::solveLeafArrangement()
    {
        leafFragments_.clear();
        if (discarded_ || polygons_.empty())
        {
            return;
        }

        const std::size_t polygonCount = polygons_.size();
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            BSPTree tree;
            tree.setBasePolygon(polygons_[i], i);

            for (std::size_t j = 0; j < polygonCount; ++j)
            {
                if (i == j)
                {
                    continue;
                }

                tree.insert(polygons_[j], j);
            }

            std::vector<Polygon256> localFragments = tree.collectLeafGeometries();
            clearClassificationState(localFragments);
            leafFragments_.insert(
                leafFragments_.end(),
                std::make_move_iterator(localFragments.begin()),
                std::make_move_iterator(localFragments.end()));
        }
    }

    void BoolProblem::classifyLeafFragmentsAndCollectResults()
    {
        resultFragments_.clear();
        if (discarded_ || leafFragments_.empty())
        {
            return;
        }

        const refPoint localReference(reference_.point, reference_.wnv);
        for (Polygon256 &fragment : leafFragments_)
        {
            const std::vector<LeafClassificationPathCandidate> candidates =
                enumerateLeafClassificationPathCandidates(reference_.point, fragment, aabb_);

            bool classified = false;
            for (const LeafClassificationPathCandidate &candidate : candidates)
            {
                WNV frontWNV;
                WNV backWNV;
                const traceStatus status = tracePathWNVToSurfacePoint(
                    localReference,
                    candidate.path,
                    polygons_,
                    fragment.plane,
                    frontWNV,
                    backWNV);

                if (status == SUCCESS)
                {
                    fragment.WNVF = std::move(frontWNV);
                    fragment.WNVB = std::move(backWNV);
                    classified = true;
                    break;
                }

                if (status != PATH_INVALID)
                {
                    break;
                }
            }

            if (!classified)
            {
                fragment.WNVF.clear();
                fragment.WNVB.clear();
                continue;
            }

            const BoolStatus frontStatus = evaluateBooleanIndicator(fragment.WNVF);
            const BoolStatus backStatus = evaluateBooleanIndicator(fragment.WNVB);
            if (frontStatus == OUT && backStatus == IN)
            {
                resultFragments_.push_back(fragment);
            }
            else if (frontStatus == IN && backStatus == OUT)
            {
                resultFragments_.push_back(reversePolygonOrientation(fragment));
            }
        }
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

        SubdivisionRefState leftReference;
        SubdivisionRefState rightReference;
        if (!leftPolygons.empty() && !makeChildReference(split.left, leftPolygons, leftReference))
        {
            return false;
        }
        if (!rightPolygons.empty() && !makeChildReference(split.right, rightPolygons, rightReference))
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
            leftChild_->reference_ = std::move(leftReference);
        }

        if (!rightPolygons.empty())
        {
            rightChild_ = std::make_unique<BoolProblem>(leafPolygonThreshold_);
            rightChild_->op_ = op_;
            rightChild_->depth_ = depth_ + 1;
            rightChild_->polygons_ = std::move(rightPolygons);
            rightChild_->aabb_ = split.right;
            rightChild_->reference_ = std::move(rightReference);
        }

        return true;
    }

    bool BoolProblem::makeChildReference(
        const AABB3i &childBox,
        const std::vector<Polygon256> &childPolygons,
        SubdivisionRefState &outReference) const
    {
        if (isPointInsideOrOnAABB(reference_.point, childBox))
        {
            bool onSurface = false;
            for (const Polygon256 &polygon : childPolygons)
            {
                if (polygon.classify(reference_.point) == 0)
                {
                    onSurface = true;
                    break;
                }
            }

            if (!onSurface)
            {
                outReference = reference_;
                return true;
            }
        }

        const refPoint sourceRef(reference_.point, reference_.wnv);
        const std::vector<AABBPathCandidate> candidates = enumerateAABBPathCandidates(reference_.point, childBox);
        for (const AABBPathCandidate &candidate : candidates)
        {
            bool onSurface = false;
            for (const Polygon256 &polygon : childPolygons)
            {
                if (polygon.classify(candidate.targetPoint) == 0)
                {
                    onSurface = true;
                    break;
                }
            }
            if (onSurface)
            {
                continue;
            }

            WNV propagatedWNV;
            const traceStatus status = tracePathWNV(sourceRef, candidate.path, polygons_, propagatedWNV);
            if (status == SUCCESS)
            {
                outReference.point = candidate.targetPoint;
                outReference.wnv = std::move(propagatedWNV);
                return true;
            }

            if (status != PATH_INVALID)
            {
                break;
            }
        }

        outReference = SubdivisionRefState();
        return false;
    }

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

