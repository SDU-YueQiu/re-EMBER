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
        AABB3i aabb;                ///< 叶子局部 AABB。
        bool discarded = false;      ///< 该叶子是否被求解流程丢弃。
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
         * @brief 追加一个已带好 WNTV 的输入多边形。
         *
         * @param[in] polygon 待加入的问题多边形。
         * @note 该接口不会重写 `polygon.WNTV`。
         */
        void addPolygon(const Polygon256 &polygon);

        /**
         * @brief 直接设置当前问题的多边形集合。
         *
         * @param[in] polygons 输入多边形集合。
         * @note 该接口假定调用方已经准备好每个多边形的 `WNTV`。
         */
        void setPolygons(const std::vector<Polygon256> &polygons);

        /**
         * @brief 设置二元布尔的左右输入，并自动写入基础 WNTV。
         *
         * @param[in] lhs 左操作数多边形集合。
         * @param[in] rhs 右操作数多边形集合。
         * @note 当前按两维 WNV 约定写入：`lhs -> (1, 0)`，`rhs -> (0, 1)`。
         */
        void setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs);

        /**
         * @brief 执行当前布尔问题。
         *
         * 如果输入、细分、leaf arrangement 或 WNV 分类无法保证正确，
         * 当前实现会抛出 `std::runtime_error`，避免输出不可信结果。
         */
        void solve();

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

    private:
        /**
         * @brief 重置求解派生状态，保留输入多边形集合和配置。
         */
        void resetSolveState() noexcept;

        /**
         * @brief 为一组输入多边形写入指定维度的基础 WNTV。
         *
         * @param[in,out] polygons 待标注的多边形集合。
         * @param[in] dimension WNV/WNTV 总维度。
         * @param[in] hotIndex 当前集合对应的非零 WNTV 分量。
         */
        static void assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t dimension, std::size_t hotIndex);

        /// 当前布尔运算类型。
        BoolOp op_ = BoolOp::Intersection;

        /// 叶子阶段停止继续细分的多边形数量阈值。
        std::size_t leafPolygonThreshold_ = 25;

        /// 当前问题是否已被判定为空。
        bool discarded_ = false;

        /// 当前问题是否已完成求解流程。
        bool solved_ = false;

        /// 当前问题的输入多边形集合。
        std::vector<Polygon256> polygons_;

        /// 最近一次求解筛选出的布尔结果面。
        std::vector<Polygon256> resultFragments_;

        /// 最近一次求解产生的叶子诊断摘要。
        std::vector<BoolLeafSummary> leafSummaries_;
    };
}
