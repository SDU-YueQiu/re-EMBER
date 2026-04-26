#pragma once

#include "algorithm/shit_wrapper.h"

#include <cstddef>
#include <memory>
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
     * @brief subdivision 阶段保存的局部参考点状态。
     *
     * `point` 始终表示当前子问题中选取的几何参考位置，
     * `wnv` 表示该点的精确已知 WNV。
     * 当前实现要求所有继续递归的活动节点都必须满足该状态精确有效。
     */
    struct SubdivisionRefState
    {
        PlanePoint3i point;
        WNV wnv;
    };

    /**
     * @brief EMBER 顶层布尔问题及递归子问题容器。
     *
     * 根节点负责接收两组输入多边形并启动空间细分；
     * 递归子节点则保存局部 polygon soup、局部 AABB 和参考点状态。
     *
     * @note 当前实现已覆盖 subdivision、子问题 reference 传播，以及叶子阶段的局部 BSP
     *       编排；面分类与布尔结果筛选也已接入当前主流程。
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
         * @brief 清空当前问题的输入与递归状态。
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
         * @brief 直接设置当前问题的 polygon soup。
         *
         * @param[in] polygons 输入多边形集合。
         * @note 该接口假定调用方已经准备好每个多边形的 `WNTV`。
         */
        void setPolygons(const std::vector<Polygon256> &polygons);

        /**
         * @brief 设置二元布尔的左右输入，并自动写入基础 WNTV。
         *
         * @param[in] lhs 左操作数 polygon soup。
         * @param[in] rhs 右操作数 polygon soup。
         * @note 当前按两维 WNV 约定写入：`lhs -> (1, 0)`，`rhs -> (0, 1)`。
         */
        void setOperands(const std::vector<Polygon256> &lhs, const std::vector<Polygon256> &rhs);

        /**
         * @brief 执行当前阶段的 subdivision。
         *
         * 该过程会初始化根 AABB、根参考点，并递归细分到叶子子问题。
         * 如果某次切分无法为子问题建立精确的参考点 WNV，
         * 当前节点会停止继续细分并保留为叶子节点。
         */
        void solve();

        /**
         * @brief 判断当前节点是否为叶子子问题。
         *
         * @return 若当前节点没有继续细分则返回 `true`。
         */
        bool isLeaf() const noexcept;

        /**
         * @brief 判断当前节点是否已被判定为空子问题。
         *
         * @return 若当前子问题没有任何可继续保留的多边形则返回 `true`。
         */
        bool isDiscarded() const noexcept;

        /**
         * @brief 判断当前节点是否已经完成当前阶段求解。
         *
         * @return `solve()` 或内部递归已运行完成时返回 `true`。
         */
        bool isSolved() const noexcept;

        /**
         * @brief 返回当前节点保存的多边形数量。
         *
         * @return 当前节点 polygon soup 的大小。
         */
        std::size_t polygonCount() const noexcept;

        /**
         * @brief 返回当前节点在 subdivision 树中的深度。
         *
         * @return 根节点为 `0`，子节点依次递增。
         */
        std::size_t depth() const noexcept;

        /**
         * @brief 读取当前节点的局部 AABB。
         *
         * @return 当前节点的包围盒。
         */
        const AABB3i &aabb() const noexcept;

        /**
         * @brief 读取当前节点的切分平面。
         *
         * @return 当前节点最近一次细分所使用的平面；叶子节点返回默认值。
         */
        const Plane3i &splitPlane() const noexcept;

        /**
         * @brief 读取当前节点的局部参考点状态。
         *
         * @return 当前节点保存的参考点和 WNV 状态。
         */
        const SubdivisionRefState &reference() const noexcept;

        /**
         * @brief 读取当前节点的 polygon soup。
         *
         * @return 当前节点保存的多边形集合。
         */
        const std::vector<Polygon256> &polygons() const noexcept;

        /**
         * @brief 读取当前叶子节点求出的局部 arrangement 结果。
         *
         * @return 当前节点在叶子阶段收集到的全部启用 leaf polygon。
         * @note 对非叶节点该数组为空。
         */
        const std::vector<Polygon256> &leafFragments() const noexcept;

        /**
         * @brief 读取当前节点筛选出的布尔结果面。
         *
         * @return 仅包含 `(OUT, IN)` 或 `(IN, OUT)` 过渡的结果面集合。
         * @note 对叶节点，这是当前局部子问题的结果面；对内部节点，这是其全部子树结果的汇总。
         */
        const std::vector<Polygon256> &resultFragments() const noexcept;

        /**
         * @brief 读取左子问题。
         *
         * @return 若左子问题存在则返回其指针，否则返回 `nullptr`。
         */
        const BoolProblem *leftChild() const noexcept;

        /**
         * @brief 读取右子问题。
         *
         * @return 若右子问题存在则返回其指针，否则返回 `nullptr`。
         */
        const BoolProblem *rightChild() const noexcept;

        /**
         * @brief 读取左子问题。
         *
         * @return 若左子问题存在则返回其指针，否则返回 `nullptr`。
         */
        BoolProblem *leftChild() noexcept;

        /**
         * @brief 读取右子问题。
         *
         * @return 若右子问题存在则返回其指针，否则返回 `nullptr`。
         */
        BoolProblem *rightChild() noexcept;

        /**
         * @brief 收集全部未丢弃的叶子子问题。
         *
         * @param[out] outLeaves 追加写入叶子节点指针的容器。
         */
        void collectLeafProblems(std::vector<const BoolProblem *> &outLeaves) const;

        /**
         * @brief 收集全部未丢弃的叶子子问题。
         *
         * @param[out] outLeaves 追加写入叶子节点指针的容器。
         */
        void collectLeafProblems(std::vector<BoolProblem *> &outLeaves);

    private:
        void resetSubdivisionState() noexcept;
        void initializeRootReference();
        void solveRecursive();
        void solveLeafArrangement();
        void classifyLeafFragmentsAndCollectResults();
        bool shouldStopSubdivision() const noexcept;
        bool createChildrenFromSplit(const AABBSplit3i &split);
        bool makeChildReference(const AABB3i &childBox, SubdivisionRefState &outReference) const;
        BoolStatus evaluateBooleanIndicator(const WNV &wnv) const noexcept;

        static void assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t dimension, std::size_t hotIndex);
        static void clearClassificationState(std::vector<Polygon256> &polygons);
        static void collectLeafProblemsRecursive(const BoolProblem *node, std::vector<const BoolProblem *> &outLeaves);
        static void collectLeafProblemsRecursive(BoolProblem *node, std::vector<BoolProblem *> &outLeaves);

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
        std::vector<Polygon256> resultFragments_;
        std::unique_ptr<BoolProblem> leftChild_;
        std::unique_ptr<BoolProblem> rightChild_;
    };
}

