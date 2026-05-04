/**
 * @file subdivision_solver.h
 * @brief Declares the internal subdivision solver used by BoolProblem.
 */
#pragma once

#include "core/bool_problem.h"

#include <cstddef>
#include <memory>
#include <vector>

namespace ember
{
    /**
     * @brief 细分阶段保存的局部参考点状态。
     */
    struct SubdivisionRefState
    {
        PlanePoint3i point;
        WNV wnv;
    };

    /**
     * @brief 叶片片段及用于决定是否输出的前后侧 WNV。
     */
    struct ClassifiedFragment
    {
        Polygon256 polygon;
        WNV frontWNV;
        WNV backWNV;
    };

    /**
     * @brief Internal recursive solver for subdivision, local arrangement, and WNV classification.
     *
     * Public callers use `BoolProblem`; this class owns the temporary node tree and
     * keeps those recursive details out of the public API.
     */
    class SubdivisionSolver
    {
    public:
        SubdivisionSolver(
            BoolOp op,
            std::size_t leafPolygonThreshold,
            const std::vector<Polygon256> &polygons);

        SubdivisionSolver(const SubdivisionSolver &) = delete;
        SubdivisionSolver &operator=(const SubdivisionSolver &) = delete;
        SubdivisionSolver(SubdivisionSolver &&) noexcept = default;
        SubdivisionSolver &operator=(SubdivisionSolver &&) noexcept = default;

        /**
         * @brief Run root initialization and recursive subdivision.
         */
        void solve();

        /**
         * @brief Return whether the solved subtree contains no active output.
         */
        bool isDiscarded() const noexcept;

        /**
         * @brief Return the result fragments collected from this subtree.
         */
        const std::vector<Polygon256> &resultFragments() const noexcept;

        /**
         * @brief Return leaf diagnostics collected from this subtree.
         */
        const std::vector<BoolLeafSummary> &leafSummaries() const noexcept;

    private:
        SubdivisionSolver(
            BoolOp op,
            std::size_t leafPolygonThreshold,
            std::size_t depth,
            std::vector<Polygon256> polygons,
            const AABB3i &aabb,
            SubdivisionRefState reference);

        /**
         * @brief Reset runtime state derived by a previous solve.
         */
        void resetSolveState() noexcept;

        /**
         * @brief Initialize the root reference point and zero WNV.
         */
        void initializeRootReference();

        /**
         * @brief Recursively subdivide this node and classify leaves.
         */
        void solveRecursive();

        /**
         * @brief Build local BSP arrangements for the current leaf node.
         */
        void solveLeafArrangement();

        /**
         * @brief Classify leaf fragments and collect boolean-result faces.
         */
        void classifyLeafFragmentsAndCollectResults();

        /**
         * @brief Return whether this node should stop subdivision.
         */
        bool shouldStopSubdivision() const noexcept;

        /**
         * @brief Create child solver nodes from an AABB split.
         */
        bool createChildrenFromSplit(const AABBSplit3i &split);

        /**
         * @brief Propagate or rebuild a reference point for a child AABB.
         */
        bool makeChildReference(
            const AABB3i &childBox,
            const std::vector<Polygon256> &childPolygons,
            SubdivisionRefState &outReference) const;

        /**
         * @brief Evaluate the configured boolean indicator for a WNV.
         */
        BoolStatus evaluateBooleanIndicator(const WNV &wnv) const noexcept;

        /**
         * @brief Append non-discarded leaf summaries from this subtree.
         */
        void collectLeafSummaries(std::vector<BoolLeafSummary> &outSummaries) const;

        BoolOp op_ = BoolOp::Intersection;
        std::size_t leafPolygonThreshold_ = 25;
        std::size_t depth_ = 0;
        bool isLeaf_ = true;
        bool discarded_ = false;
        bool solved_ = false;
        Plane3i splitPlane_;
        AABB3i aabb_;
        SubdivisionRefState reference_;
        std::vector<Polygon256> polygons_;
        std::vector<Polygon256> leafFragments_;
        std::vector<ClassifiedFragment> classifiedFragments_;
        std::vector<Polygon256> resultFragments_;
        std::vector<BoolLeafSummary> leafSummaries_;
        std::unique_ptr<SubdivisionSolver> leftChild_;
        std::unique_ptr<SubdivisionSolver> rightChild_;
    };
}
