/**
 * @file bool_problem.h
 * @brief 声明应用和测试使用的公开布尔问题门面。
 */
#pragma once

#include "geometry/aabb.h"
#include "geometry/geometry256.h"

#include <cstddef>
#include <vector>

namespace ember
{
/**
 * @brief 二元布尔运算类型。
 */
enum class BoolOp
{
    Union,        ///< 并集。
    Intersection, ///< 交集。
    Difference    ///< 差集，按 `lhs - rhs` 解释。
};

/**
 * @brief 已求解叶子节点的只读诊断摘要。
 *
 * 该类型只暴露调试和测试所需的结构信息，不暴露内部递归节点对象。
 */
struct BoolLeafSummary
{
    std::size_t depth = 0;       ///< 叶子在细分树中的深度。
    std::size_t polygonCount = 0; ///< 叶子局部多边形集合的大小。
    AABB3i aabb;                 ///< 叶子局部 AABB。
    bool discarded = false;      ///< 该叶子是否被求解流程丢弃。
};

/**
 * @brief 最近一次求解的高层统计信息。
 */
struct BoolSolveMetrics
{
    std::size_t inputPolygonCount = 0;              ///< 输入多边形总数。
    std::size_t effectiveThreadCount = 1;           ///< 本次求解实际参与的总线程数，包含调用 `solve()` 的线程。
    std::size_t nodeCount = 0;                      ///< 递归节点总数。
    std::size_t internalNodeCount = 0;              ///< 发生进一步细分的节点数。
    std::size_t leafNodeCount = 0;                  ///< 叶节点数。
    std::size_t discardedNodeCount = 0;             ///< 被丢弃的节点数。
    std::size_t maxDepth = 0;                       ///< 递归树最大深度。
    std::size_t totalPolygonCount = 0;              ///< 访问到的节点多边形数量累计值。
    std::size_t leafFragmentCount = 0;              ///< 所有活跃叶子的叶片片段数累计值。
    std::size_t classifiedFragmentCount = 0;        ///< 所有活跃叶子的已分类片段数累计值。
    std::size_t resultFragmentCount = 0;            ///< 最终结果片段数。
    std::size_t constantDiscardCount = 0;           ///< 子节点在创建前被常量布尔指示函数剪枝的次数。
    std::size_t invalidOrEmptyDiscardCount = 0;     ///< 因空节点或非法 AABB 被直接丢弃的节点数。
    std::size_t leafThresholdStopCount = 0;         ///< 因叶子阈值停止细分的节点数。
    std::size_t aabbNotSplittableStopCount = 0;     ///< 因 AABB 不可再切分停止细分的节点数。
    std::size_t splitFailureStopCount = 0;          ///< 因切分候选失败停止细分的节点数。
    std::size_t wntvAwareSplitCount = 0;            ///< 命中 WNTV 感知切分的次数。
    std::size_t centerRangeSplitCount = 0;          ///< 命中中心范围切分的次数。
    std::size_t midpointSplitCount = 0;             ///< 回退到中点切分的次数。
    std::size_t parallelSiblingSpawnCount = 0;      ///< 递归中把 sibling 子树作为并行任务提交的次数。
    std::size_t childReferenceReuseCount = 0;       ///< 直接复用子参考点的次数。
    std::size_t childReferenceTraceCount = 0;       ///< 通过路径追踪传播子参考点的次数。
    std::size_t childReferenceCandidateCount = 0;   ///< 子参考点传播阶段生成的候选总数。
    std::size_t childReferenceFastCandidateCount = 0; ///< 子参考点快速候选阶段生成的候选总数。
    std::size_t childReferenceExhaustiveCandidateCount = 0; ///< 子参考点穷举候选阶段生成的候选总数。
    std::size_t childReferenceCandidateTriedCount = 0; ///< 子参考点传播阶段实际尝试追踪的候选数。
    std::size_t childReferenceFastCandidateTriedCount = 0; ///< 子参考点快速候选阶段实际尝试追踪的候选数。
    std::size_t childReferenceExhaustiveCandidateTriedCount = 0; ///< 子参考点穷举候选阶段实际尝试追踪的候选数。
    std::size_t tracePathStartPointOnBoundaryCount = 0; ///< WNV 追踪中起点落在多边形边界上的次数。
    std::size_t tracePathEndPointOnBoundaryCount = 0; ///< WNV 追踪中终点落在多边形边界上的次数。
    std::size_t tracePathEndpointOnBoundaryContactCount = 0; ///< WNV 追踪中命中端点边界接触的次数。
    std::size_t tracePathEdgeOverlapCount = 0; ///< WNV 追踪中命中边重叠的次数。
    std::size_t tracePathBoundaryHitRejectedRegularEdgeCount = 0; ///< WNV 追踪中命中原始边而被拒绝的次数。
    std::size_t tracePathBoundaryHitRejectedSubdivisionClipEdgeCount = 0; ///< WNV 追踪中命中 subdivision 裁剪边而被拒绝的次数。
    std::size_t tracePathBoundaryHitRejectedMixedEdgeCount = 0; ///< WNV 追踪中命中原始边与裁剪边混合边界而被拒绝的次数。
    std::size_t tracePathBoundaryHitRejectedUnknownCount = 0; ///< WNV 追踪中命中边界但无法归类边来源而被拒绝的次数。
    std::size_t tracePathBoundaryHitAllowedSubdivisionClipEdgeCount = 0; ///< WNV 追踪中允许通过 subdivision 裁剪边的次数。
    std::size_t tracePathNonStrictIntersectionCount = 0; ///< WNV 追踪中交点不在严格内部而被拒绝的次数。
    std::size_t tracePathBoundaryContactWithoutIntersectionCount = 0; ///< WNV 追踪中存在边界接触但没有唯一交点的次数。
    std::size_t singleOperandAssumptionStopCount = 0; ///< NSI/NNC 单操作数假设提前停止细分的次数。
    std::size_t singleOperandAssumptionFallbackCount = 0; ///< 保留兼容的历史计数；当前实现不再在单操作数快路径中回退细分。
    std::size_t singleOperandLeafBspSkipCount = 0;  ///< 单操作数叶子跳过局部 BSP 的次数。
    std::size_t singleOperandClassificationReuseCount = 0; ///< 单操作数叶子复用分类结果的次数。
    std::size_t leafBspBuildCount = 0;              ///< 真实执行局部 BSP 构建的次数。
    std::size_t leafClassificationCentroidPointCount = 0; ///< 叶片分类阶段命中的重心启发式目标点总数。
    std::size_t leafClassificationInsetPointAttemptCount = 0; ///< 叶片分类阶段按论文 inset 规则尝试构点的总次数。
    std::size_t leafClassificationTraceAttemptCount = 0; ///< 叶片分类阶段实际尝试的路径总数。
    std::size_t leafClassificationAxisPathAttemptCount = 0; ///< 叶片分类阶段 axis-aligned 路径尝试总数。
    std::size_t leafClassificationPlaneReplacementPathAttemptCount = 0; ///< 叶片分类阶段换平面路径尝试总数。
    std::size_t leafClassificationCandidateGeneratedCount = 0; ///< 叶片分类阶段由枚举器生成的候选路径总数。
    std::size_t leafClassificationCandidateUniqueCount = 0; ///< 叶片分类阶段去重后保留的候选路径总数。
    std::size_t leafClassificationCandidateDuplicateSkipCount = 0; ///< 叶片分类阶段因路径重复跳过的候选数。
    std::size_t leafClassificationCandidateRejectedCount = 0; ///< 叶片分类阶段结构非法且无法局部修复的候选数。
    std::size_t leafClassificationCandidateRepairAttemptCount = 0; ///< 叶片分类阶段尝试局部重建候选路径的次数。
    std::size_t leafClassificationCandidateRepairSuccessCount = 0; ///< 叶片分类阶段局部重建候选路径成功的次数。
    std::size_t leafClassificationCentroidAxisSuccessCount = 0; ///< 重心 axis 阶段 trace 成功次数。
    std::size_t leafClassificationCentroidAxisPathInvalidCount = 0; ///< 重心 axis 阶段返回 `PATH_INVALID` 的次数。
    std::size_t leafClassificationCentroidAxisInputInvalidCount = 0; ///< 重心 axis 阶段返回 `INPUT_INVALID` 的次数。
    std::size_t leafClassificationCentroidAxisFailCount = 0; ///< 重心 axis 阶段返回 `FAIL` 的次数。
    std::size_t leafClassificationInsetReplacementSuccessCount = 0; ///< inset 换平面阶段 trace 成功次数。
    std::size_t leafClassificationInsetReplacementPathInvalidCount = 0; ///< inset 换平面阶段返回 `PATH_INVALID` 的次数。
    std::size_t leafClassificationInsetReplacementInputInvalidCount = 0; ///< inset 换平面阶段返回 `INPUT_INVALID` 的次数。
    std::size_t leafClassificationInsetReplacementFailCount = 0; ///< inset 换平面阶段返回 `FAIL` 的次数。
    std::size_t leafClassificationBridgeRescueSuccessCount = 0; ///< bridge rescue 阶段 trace 成功次数。
    std::size_t leafClassificationBridgeRescuePathInvalidCount = 0; ///< bridge rescue 阶段返回 `PATH_INVALID` 的次数。
    std::size_t leafClassificationBridgeRescueInputInvalidCount = 0; ///< bridge rescue 阶段返回 `INPUT_INVALID` 的次数。
    std::size_t leafClassificationBridgeRescueFailCount = 0; ///< bridge rescue 阶段返回 `FAIL` 的次数。
};

/**
 * @brief 调用方显式声明的单个二元操作数输入假设。
 *
 * 该结构只表示调用方承诺，求解器不会主动验证输入是否真的满足。
 */
struct BoolOperandAssumptions
{
    bool noSelfIntersections = false; ///< 当前操作数没有自相交。
    bool noNestedComponents = false;  ///< 当前操作数没有嵌套连通组件。
};

/**
 * @brief EMBER 顶层布尔问题门面。
 *
 * `BoolProblem` 只负责接收输入、保存布尔配置并暴露最终结果。
 * 空间细分、叶子局部编排和 WNV 分类的运行时状态由内部求解器持有。
 */
class BoolProblem
{
public:
    explicit BoolProblem(std::size_t leafPolygonThreshold = 25) noexcept;
    ~BoolProblem() noexcept = default;

    BoolProblem(const BoolProblem &) = delete;
    BoolProblem &operator=(const BoolProblem &) = delete;
    BoolProblem(BoolProblem &&) noexcept = default;
    BoolProblem &operator=(BoolProblem &&) noexcept = default;

    /**
     * @brief 设置当前布尔运算类型。
     *
     * @param[in] op 目标布尔运算。
     */
    void setOperation(BoolOp op);

    /**
     * @brief 设置左右操作数可用于 4.5.x 快路径与单操作数 early-stop 的输入假设。
     *
     * @param[in] lhsAssumptions 左操作数假设，对应 WNTV `{1, 0}`。
     * @param[in] rhsAssumptions 右操作数假设，对应 WNTV `{0, 1}`。
     * @note 这些假设不会被验证；错误声明会破坏布尔结果正确性。
     */
    void setOperandAssumptions(
        BoolOperandAssumptions lhsAssumptions,
        BoolOperandAssumptions rhsAssumptions);

    /**
     * @brief 设置本次布尔求解允许使用的总线程数。
     *
     * @param[in] threadCount 总线程数；`0` 表示自动并发度，`1` 表示强制串行。
     * @note 线程数包含调用 `solve()` 的线程本身，而不是只统计后台 worker。
     */
    void setThreadCount(std::size_t threadCount);

    /**
     * @brief 设置二元布尔的左右输入，并自动写入基础 WNTV。
     *
     * @param[in] lhs 左操作数多边形集合。
     * @param[in] rhs 右操作数多边形集合。
     * @note 该接口会覆盖输入多边形现有的 `WNTV`，统一按二元约定写入：
     *       `lhs -> (1, 0)`，`rhs -> (0, 1)`。
     * @note 一旦调用过 `solve()`，同一个 `BoolProblem` 就不能再修改输入。
     */
    void setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs);

    /**
     * @brief 使用调用方提供的场景 AABB 执行当前布尔问题。
     *
     * @param[in] sceneAABB 全部输入多边形所在的整数场景包围盒，调用方应在读入/量化阶段生成。
     * @note `sceneAABB` 应包含足够 margin，使根参考点可取在输入几何外侧。
     *       如果输入、细分、leaf arrangement 或 WNV 分类无法保证正确，
     *       当前实现会抛出 `std::runtime_error`，避免输出不可信结果。
     * @note 每个 `BoolProblem` 实例只允许执行一次 `solve()`；无论成功还是抛错，
     *       后续都不能再次 `solve()` 或修改配置。
     */
    void solve(const AABB3i &sceneAABB);

    /**
     * @brief 判断当前问题是否已被判定为空。
     *
     * @return 当前问题没有任何可输出结果时返回 `true`。
     */
    bool isDiscarded() const noexcept;

    /**
     * @brief 读取最终布尔结果多边形集合。
     *
     * @return 仅包含 `(OUT, IN)` 或 `(IN, OUT)` 过渡的结果面集合。
     */
    const std::vector<Polygon256> &resultFragments() const noexcept;

    /**
     * @brief 读取最近一次求解产生的叶子诊断摘要。
     *
     * @return 每个未丢弃叶子的深度、多边形数量和局部 AABB。
     */
    const std::vector<BoolLeafSummary> &leafSummaries() const noexcept;

    /**
     * @brief 读取最近一次求解的高层统计信息。
     */
    const BoolSolveMetrics &solveMetrics() const noexcept;

private:
    /**
     * @brief 为一组输入多边形写入二元操作数基础 WNTV。
     *
     * @param[in,out] polygons 待标注的多边形集合。
     * @param[in] hotIndex 当前集合对应的非零 WNTV 分量。
     */
    static void assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t hotIndex);

    /// 当前布尔运算类型。
    BoolOp op_ = BoolOp::Intersection;

    /// 叶子阶段停止继续细分的多边形数量阈值。
    std::size_t leafPolygonThreshold_ = 25;

    /// 左操作数调用方声明的输入假设。
    BoolOperandAssumptions lhsAssumptions_;

    /// 右操作数调用方声明的输入假设。
    BoolOperandAssumptions rhsAssumptions_;

    /// 本次求解允许使用的总线程数；`0` 表示自动并发度。
    std::size_t threadCount_ = 0;

    /// 当前问题是否已被判定为空。
    bool discarded_ = false;

    /// 当前实例是否已经执行过一次 `solve()` 尝试。
    bool solveAttempted_ = false;

    /// 当前问题的输入多边形集合，所有元素都必须属于 lhs `{1,0}` 或 rhs `{0,1}`。
    std::vector<Polygon256> polygons_;

    /// 最近一次求解筛选出的布尔结果面。
    std::vector<Polygon256> resultFragments_;

    /// 最近一次求解产生的叶子诊断摘要。
    std::vector<BoolLeafSummary> leafSummaries_;

    /// 最近一次求解的高层统计信息。
    BoolSolveMetrics solveMetrics_;
};
}
