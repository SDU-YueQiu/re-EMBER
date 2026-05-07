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
 * @brief 当前节点 polygon soup 的二元操作数扫描结果。
 */
enum class BinarySingleOperand
{
    None,
    Lhs,
    Rhs
};

/**
 * @brief 缓存当前节点是否出现 lhs/rhs 以及是否退化为单操作数。
 */
struct BinaryPolygonScanSummary
{
    bool hasLhs = false;
    bool hasRhs = false;
    bool isSingleOperand = false;
    BinarySingleOperand singleOperand = BinarySingleOperand::None;
};

/**
 * @brief 负责细分、局部编排和 WNV 分类的内部递归求解器。
 *
 * 公开调用方只使用 `BoolProblem`；该类独占临时节点树，
 * 注意polygons会被swap到求解器内部，调用者的polygons会被置空
 * 避免递归细节泄露到公共 API。
 */
class SubdivisionSolver
{
public:
    SubdivisionSolver(
        BoolOp op,
        std::size_t leafPolygonThreshold,
        std::vector<Polygon256> &polygons,
        const AABB3i &rootAABB,
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
     * @brief 从当前子树收集到的结果片段移动到 out 中。
     */
    void extractResultFragments(std::vector<Polygon256>& out) noexcept;

    /**
     * @brief 提取从当前子树收集到的叶子诊断信息。
     */
    void extractLeafSummaries(std::vector<BoolLeafSummary>& out) noexcept;

    /**
     * @brief 返回最近一次求解的高层统计信息。
     */
    const BoolSolveMetrics &solveMetrics() const noexcept;

private:
    /**
     * @brief 当前节点基于 4.5.1 单操作数输入假设得到的共享策略。
     *
     * 该策略在节点构造时计算一次，统一决定递归入口探测、
     * 叶子 BSP 跳过和叶片分类复用，避免把同一语义拆散到多个局部判断里。
     */
    struct SingleOperandAssumptionPolicy
    {
        bool maySkipLeafBsp = false;
        bool mayReuseLeafClassification = false;
        bool mayProbeEarlyLeaf = false;
    };

    SubdivisionSolver(
        BoolOp op,
        std::size_t leafPolygonThreshold,
        std::size_t depth,
        std::vector<Polygon256> polygons,
        const AABB3i &aabb,
        SubdivisionRefState reference,
        BinaryPolygonScanSummary polygonScan,
        BoolOperandAssumptions lhsAssumptions,
        BoolOperandAssumptions rhsAssumptions);

    /**
     * @brief 初始化根节点参考点和零 WNV。
     */
    void initializeRootReference();

    /**
     * @brief 递归细分当前节点并分类叶子节点。
     */
    void solveRecursive();

    /**
     * @brief 为当前叶子节点构建局部 BSP 编排，必要时按共享单操作数策略跳过。
     */
    void solveLeafArrangement();

    /**
     * @brief 分类叶片片段并收集布尔结果面。
     */
    void classifyLeafFragmentsAndCollectResults();

    /**
     * @brief 按共享单操作数策略尝试提前把当前节点作为叶子求解。
     */
    bool trySolveSingleOperandAssumptionLeaf();

    /**
     * @brief 返回当前节点是否应停止继续细分。
     */
    bool shouldStopSubdivision() const noexcept;

    /**
     * @brief 按当前节点的二元扫描结果与输入假设计算共享单操作数策略。
     */
    static SingleOperandAssumptionPolicy buildSingleOperandAssumptionPolicy(
        const BinaryPolygonScanSummary &polygonScan,
        const BoolOperandAssumptions &lhsAssumptions,
        const BoolOperandAssumptions &rhsAssumptions,
        std::size_t polygonCount,
        std::size_t leafPolygonThreshold) noexcept;

    /**
     * @brief 将一个已分类叶片按 indicator 结果写入输出集合。
     */
    void appendResultFragmentFromClassification(const ClassifiedFragment &classifiedFragment);

    /**
     * @brief 以叶子模式完成当前节点求解。
     */
    void finishCurrentNodeAsLeaf();

    /**
     * @brief 当当前节点满足停止条件时按叶子完成求解。
     */
    bool tryFinishStoppedSubdivisionNode();

    /**
     * @brief 合并已求解子节点的结果片段与丢弃状态。
     */
    void mergeSolvedChildren();

    /**
     * @brief 根据 AABB 切分创建子求解器节点。
     */
    bool createChildrenFromSplit(const AABBSplit3i &split);

    /**
     * @brief 将当前多边形集合裁剪为左右子节点 polygon soup。
     */
    bool buildSplitChildPolygonSoups(
        const AABBSplit3i &split,
        std::vector<Polygon256> &leftPolygons,
        std::vector<Polygon256> &rightPolygons) const;

    /**
     * @brief 判断某个子节点是否可被常量指示函数直接丢弃。
     */
    bool shouldCreateChildNode(
        const BinaryPolygonScanSummary &childPolygonScan,
        const SubdivisionRefState &childReference);

    /**
     * @brief 构造一个子求解器实例并接管其输入状态。
     */
    void createChildSolver(
        std::unique_ptr<SubdivisionSolver> &child,
        const AABB3i &childBox,
        std::vector<Polygon256> childPolygons,
        SubdivisionRefState childReference,
        BinaryPolygonScanSummary childPolygonScan);

    /**
     * @brief 为子 AABB 传播或重建参考点。
     */
    bool makeChildReference(
        const AABB3i &childBox,
        const std::vector<Polygon256> &childPolygons,
        SubdivisionRefState &outReference);

    /**
     * @brief 若当前参考点仍可安全用于子节点，则直接复用。
     */
    bool tryReuseChildReference(
        const AABB3i &childBox,
        const std::vector<Polygon256> &childPolygons,
        SubdivisionRefState &outReference);

    /**
     * @brief 当共享单操作数策略允许时，直接复用上一片段的分类结果。
     */
    bool tryReuseSingleOperandFragmentClassification(
        const Polygon256 &fragment,
        bool hasReusableClassification,
        const WNV &reusableFrontWNV,
        const WNV &reusableBackWNV);

    /**
     * @brief 对一个 WNV 计算当前配置的布尔指示函数。
     */
    BoolStatus evaluateBooleanIndicator(const WNV &wnv) const noexcept;

    /**
     * @brief 以当前节点的轻量统计更新子树级高层指标。
     */
    void finalizeNodeMetrics(bool isLeafNode) noexcept;

    /**
     * @brief 叶子求解完成后写出摘要、封账并释放中间容器。
     */
    void finalizeLeafNode();

    /**
     * @brief 内部节点合并完子树后封账。
     */
    void finalizeInternalNode();

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
    BinaryPolygonScanSummary polygonScan_;
    SingleOperandAssumptionPolicy singleOperandPolicy_;
    std::vector<Polygon256> polygons_;
    std::vector<Polygon256> leafFragments_;
    std::vector<ClassifiedFragment> classifiedFragments_;
    std::vector<Polygon256> resultFragments_;
    std::vector<BoolLeafSummary> leafSummaries_;
    BoolSolveMetrics solveMetrics_;
    std::size_t polygonCount_ = 0;
    std::size_t leafFragmentCount_ = 0;
    std::size_t classifiedFragmentCount_ = 0;
    std::unique_ptr<SubdivisionSolver> leftChild_;
    std::unique_ptr<SubdivisionSolver> rightChild_;
};
}
