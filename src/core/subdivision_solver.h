/**
 * @file subdivision_solver.h
 * @brief 声明 BoolProblem 使用的内部细分求解器。
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
     * @brief 负责细分、局部编排和 WNV 分类的内部递归求解器。
     *
     * 公开调用方只使用 `BoolProblem`；该类独占临时节点树，
     * 避免递归细节泄露到公共 API。
     */
    class SubdivisionSolver
    {
    public:
        SubdivisionSolver(
            BoolOp op,
            std::size_t leafPolygonThreshold,
            const std::vector<Polygon256> &polygons,
            BoolOperandAssumptions lhsAssumptions,
            BoolOperandAssumptions rhsAssumptions);

        SubdivisionSolver(const SubdivisionSolver &) = delete;
        SubdivisionSolver &operator=(const SubdivisionSolver &) = delete;
        SubdivisionSolver(SubdivisionSolver &&) noexcept = default;
        SubdivisionSolver &operator=(SubdivisionSolver &&) noexcept = default;

        /**
         * @brief 执行根节点初始化和递归细分。
         */
        void solve();

        /**
         * @brief 返回已求解子树是否没有活跃输出。
         */
        bool isDiscarded() const noexcept;

        /**
         * @brief 返回从当前子树收集到的结果片段。
         */
        const std::vector<Polygon256> &resultFragments() const noexcept;

        /**
         * @brief 返回从当前子树收集到的叶子诊断信息。
         */
        const std::vector<BoolLeafSummary> &leafSummaries() const noexcept;

        /**
         * @brief 返回最近一次求解的高层统计信息。
         */
        const BoolSolveMetrics &solveMetrics() const noexcept;

    private:
        SubdivisionSolver(
            BoolOp op,
            std::size_t leafPolygonThreshold,
            std::size_t depth,
            std::vector<Polygon256> polygons,
            const AABB3i &aabb,
            SubdivisionRefState reference,
            BoolOperandAssumptions lhsAssumptions,
            BoolOperandAssumptions rhsAssumptions);

        /**
         * @brief 重置上一次求解派生出的运行时状态。
         */
        void resetSolveState() noexcept;

        /**
         * @brief 初始化根节点参考点和零 WNV。
         */
        void initializeRootReference();

        /**
         * @brief 递归细分当前节点并分类叶子节点。
         */
        void solveRecursive();

        /**
         * @brief 为当前叶子节点构建局部 BSP 编排。
         */
        void solveLeafArrangement();

        /**
         * @brief 分类叶片片段并收集布尔结果面。
         */
        void classifyLeafFragmentsAndCollectResults();

        /**
         * @brief 尝试用 NSI/NNC 单操作数假设提前把当前节点作为叶子求解。
         */
        bool trySolveSingleOperandAssumptionLeaf();

        /**
         * @brief 返回当前节点是否应停止继续细分。
         */
        bool shouldStopSubdivision() const noexcept;

        /**
         * @brief 判断当前子问题的布尔指示函数是否已退化为常量。
         */
        bool shouldDiscardSubproblemEarly(BoolStatus &constantStatus) const noexcept;

        /**
         * @brief 若当前节点只含一个二元操作数 WNTV 类，返回该类假设配置。
         */
        bool tryGetSingleOperandAssumptions(BoolOperandAssumptions &outAssumptions) const noexcept;

        /**
         * @brief 分类叶片片段并收集结果，可选择在分类失败时回退。
         */
        bool classifyLeafFragmentsAndCollectResults(bool allowRetryFallback);

        /**
         * @brief 根据 AABB 切分创建子求解器节点。
         */
        bool createChildrenFromSplit(const AABBSplit3i &split);

        /**
         * @brief 为子 AABB 传播或重建参考点。
         */
        bool makeChildReference(
            const AABB3i &childBox,
            const std::vector<Polygon256> &childPolygons,
            SubdivisionRefState &outReference);

        /**
         * @brief 对一个 WNV 计算当前配置的布尔指示函数。
         */
        BoolStatus evaluateBooleanIndicator(const WNV &wnv) const noexcept;

        /**
         * @brief 追加当前子树中未丢弃的叶子摘要。
         */
        void collectLeafSummaries(std::vector<BoolLeafSummary> &outSummaries) const;

        /**
         * @brief 递归收集当前子树的高层统计信息。
         */
        void collectSolveMetrics(BoolSolveMetrics &outMetrics) const;

        BoolOp op_ = BoolOp::Intersection;
        std::size_t leafPolygonThreshold_ = 25;
        BoolOperandAssumptions lhsAssumptions_;
        BoolOperandAssumptions rhsAssumptions_;
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
        BoolSolveMetrics solveMetrics_;
        std::unique_ptr<SubdivisionSolver> leftChild_;
        std::unique_ptr<SubdivisionSolver> rightChild_;
    };
}
