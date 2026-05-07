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
    std::size_t nodeCount = 0;                      ///< 递归节点总数。
    std::size_t internalNodeCount = 0;              ///< 发生进一步细分的节点数。
    std::size_t leafNodeCount = 0;                  ///< 叶节点数。
    std::size_t discardedNodeCount = 0;             ///< 被丢弃的节点数。
    std::size_t maxDepth = 0;                       ///< 递归树最大深度。
    std::size_t totalPolygonCount = 0;              ///< 访问到的节点多边形数量累计值。
    std::size_t leafFragmentCount = 0;              ///< 所有活跃叶子的叶片片段数累计值。
    std::size_t classifiedFragmentCount = 0;        ///< 所有活跃叶子的已分类片段数累计值。
    std::size_t resultFragmentCount = 0;            ///< 最终结果片段数。
    std::size_t constantDiscardCount = 0;           ///< 由常量布尔指示函数直接剪枝的节点数。
    std::size_t invalidOrEmptyDiscardCount = 0;     ///< 因空节点或非法 AABB 被直接丢弃的节点数。
    std::size_t childConstantDiscardCount = 0;      ///< 子节点在创建前被常量布尔指示函数剪枝的次数。
    std::size_t leafThresholdStopCount = 0;         ///< 因叶子阈值停止细分的节点数。
    std::size_t aabbNotSplittableStopCount = 0;     ///< 因 AABB 不可再切分停止细分的节点数。
    std::size_t splitFailureStopCount = 0;          ///< 因切分候选失败停止细分的节点数。
    std::size_t wntvAwareSplitCount = 0;            ///< 命中 WNTV 感知切分的次数。
    std::size_t centerRangeSplitCount = 0;          ///< 命中中心范围切分的次数。
    std::size_t midpointSplitCount = 0;             ///< 回退到中点切分的次数。
    std::size_t childReferenceReuseCount = 0;       ///< 直接复用子参考点的次数。
    std::size_t childReferenceTraceCount = 0;       ///< 通过路径追踪传播子参考点的次数。
    std::size_t childReferenceCandidateCount = 0;   ///< 子参考点传播阶段生成的候选总数。
    std::size_t childReferenceFastCandidateCount = 0; ///< 子参考点快速候选阶段生成的候选总数。
    std::size_t childReferenceExhaustiveCandidateCount = 0; ///< 子参考点穷举候选阶段生成的候选总数。
    std::size_t childReferenceCandidateTriedCount = 0; ///< 子参考点传播阶段实际尝试追踪的候选数。
    std::size_t childReferenceFastCandidateTriedCount = 0; ///< 子参考点快速候选阶段实际尝试追踪的候选数。
    std::size_t childReferenceExhaustiveCandidateTriedCount = 0; ///< 子参考点穷举候选阶段实际尝试追踪的候选数。
    std::size_t singleOperandAssumptionStopCount = 0; ///< NSI/NNC 单操作数假设提前停止细分的次数。
    std::size_t singleOperandAssumptionFallbackCount = 0; ///< NSI/NNC 单操作数提前停止探测失败并回退细分的次数。
    std::size_t singleOperandLeafBspSkipCount = 0;  ///< 单操作数叶子跳过局部 BSP 的次数。
    std::size_t singleOperandClassificationReuseCount = 0; ///< 单操作数叶子复用分类结果的次数。
    std::size_t leafBspBuildCount = 0;              ///< 真实执行局部 BSP 构建的次数。
    std::size_t leafClassificationPointCandidateCount = 0; ///< 叶片分类阶段枚举的目标点总数。
    std::size_t leafClassificationPrimaryPointCandidateCount = 0; ///< 叶片分类阶段 primary 点候选总数。
    std::size_t leafClassificationExpandedPointCandidateCount = 0; ///< 叶片分类阶段 expanded 点候选总数。
    std::size_t leafClassificationTraceAttemptCount = 0; ///< 叶片分类阶段实际尝试的路径总数。
    std::size_t leafClassificationFastCandidateCount = 0; ///< fast layer 候选总数。
    std::size_t leafClassificationFallbackCandidateCount = 0; ///< fallback layer 候选总数。
    std::size_t leafClassificationNormalCandidateCount = 0; ///< normal layer 候选总数。
    std::size_t leafClassificationInteriorBridgeCandidateCount = 0; ///< interior bridge layer 候选总数。
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
     * @brief 清空当前问题的输入与求解结果。
     *
     * @note 该操作不会重置 `BoolOp` 和叶子阈值配置。
     */
    void clear() noexcept;

    /**
     * @brief 设置当前布尔运算类型。
     *
     * @param[in] op 目标布尔运算。
     */
    void setOperation(BoolOp op) noexcept;

    /**
     * @brief 设置叶子停止细分阈值。
     *
     * @param[in] threshold 当子问题多边形数量不超过该值时停止继续细分。
     * @note 传入 `0` 时会自动收敛到 `1`。
     */
    void setLeafPolygonThreshold(std::size_t threshold) noexcept;

    /**
     * @brief 设置左右操作数可用于 4.5.x 快路径与单操作数 early-stop 的输入假设。
     *
     * @param[in] lhsAssumptions 左操作数假设，对应 WNTV `{1, 0}`。
     * @param[in] rhsAssumptions 右操作数假设，对应 WNTV `{0, 1}`。
     * @note 这些假设不会被验证；错误声明会破坏布尔结果正确性。
     */
    void setOperandAssumptions(
        BoolOperandAssumptions lhsAssumptions,
        BoolOperandAssumptions rhsAssumptions) noexcept;

    /**
     * @brief 设置二元布尔的左右输入，并自动写入基础 WNTV。
     *
     * @param[in] lhs 左操作数多边形集合。
     * @param[in] rhs 右操作数多边形集合。
     * @note 该接口会覆盖输入多边形现有的 `WNTV`，统一按二元约定写入：
     *       `lhs -> (1, 0)`，`rhs -> (0, 1)`。
     */
    void setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs);

    /**
     * @brief 使用调用方提供的场景 AABB 执行当前布尔问题。
     *
     * @param[in] sceneAABB 全部输入多边形所在的整数场景包围盒，调用方应在读入/量化阶段生成。
     * @note `sceneAABB` 应包含足够 margin，使根参考点可取在输入几何外侧。
     *       如果输入、细分、leaf arrangement 或 WNV 分类无法保证正确，
     *       当前实现会抛出 `std::runtime_error`，避免输出不可信结果。
     */
    void solve(const AABB3i &sceneAABB);

    /**
     * @brief 判断当前问题是否已被判定为空。
     *
     * @return 当前问题没有任何可输出结果时返回 `true`。
     */
    bool isDiscarded() const noexcept;

    /**
     * @brief 判断当前问题是否已经完成求解。
     *
     * @return `solve()` 成功运行完成时返回 `true`。
     */
    bool isSolved() const noexcept;

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
     * @brief 重置求解派生状态，保留输入多边形集合和配置。
     */
    void resetSolveState() noexcept;

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

    /// 当前问题是否已被判定为空。
    bool discarded_ = false;

    /// 当前问题是否已完成求解流程。
    bool solved_ = false;

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
