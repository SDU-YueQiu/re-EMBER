#pragma once

#include "geometry/aabb.h"
#include "geometry/geometry256.h"

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
     * @brief 细分阶段保存的局部参考点状态。
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
     * @brief 叶片片段及用于决定是否输出的前后侧 WNV。
     */
    struct ClassifiedFragment
    {
        Polygon256 polygon;
        WNV frontWNV;
        WNV backWNV;
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
         * 如果输入、细分、leaf arrangement 或 WNV 分类无法保证正确，
         * 当前实现会抛出 `std::runtime_error`，避免输出不可信结果。
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
        /**
         * @brief 重置 subdivision 派生状态。
         *
         * 清空子树、局部 AABB、参考点、leaf fragment 与结果面，
         * 但保留输入 polygon soup、布尔运算类型和叶子阈值配置。
         */
        void resetSubdivisionState() noexcept;

        /**
         * @brief 为根问题初始化局部参考点及其 WNV。
         *
         * 当前实现选择根 AABB 的最小角点作为外部参考点，并按输入 WNTV
         * 最大维度初始化零 WNV。
         */
        void initializeRootReference();

        /**
         * @brief 递归执行当前子问题的 subdivision、leaf arrangement 与分类。
         */
        void solveRecursive();

        /**
         * @brief 在叶子节点内为每个输入 polygon 构造局部 BSP arrangement。
         */
        void solveLeafArrangement();

        /**
         * @brief 分类 leaf fragments 并收集通过当前布尔指示函数的结果面。
         *
         * @throws std::runtime_error 当任一叶片片段无法成功计算前后侧 WNV 时抛出。
         */
        void classifyLeafFragmentsAndCollectResults();

        /**
         * @brief 判断当前子问题是否应停止继续空间细分。
         *
         * @return 多边形数量不超过阈值，或局部 AABB 已无可切分轴时返回 `true`。
         */
        bool shouldStopSubdivision() const noexcept;

        /**
         * @brief 根据一次 AABB 切分创建左右子问题。
         *
         * @param[in] split 已计算出的左右 AABB 与切分平面。
         * @return 至少创建一个子问题且其参考点可传播时返回 `true`。
         */
        bool createChildrenFromSplit(const AABBSplit3i &split);

        /**
         * @brief 为目标子 AABB 构造局部参考点并传播 WNV。
         *
         * @param[in] childBox 目标子问题的 AABB。
         * @param[in] childPolygons 裁剪到目标子问题内的 polygon soup。
         * @param[out] outReference 成功时写入子问题参考点与 WNV。
         * @return 成功复用或传播参考点时返回 `true`。
         */
        bool makeChildReference(
            const AABB3i &childBox,
            const std::vector<Polygon256> &childPolygons,
            SubdivisionRefState &outReference) const;

        /**
         * @brief 将 WNV 代入当前布尔运算的指示函数。
         *
         * @param[in] wnv 待判断的 winding-number 状态。
         * @return 当前布尔运算下该状态对应的内外分类。
         */
        BoolStatus evaluateBooleanIndicator(const WNV &wnv) const noexcept;

        /**
         * @brief 为一组输入 polygon 写入指定维度的基础 WNTV。
         *
         * @param[in,out] polygons 待标注的 polygon 集合。
         * @param[in] dimension WNV/WNTV 总维度。
         * @param[in] hotIndex 当前集合对应的非零 WNTV 分量。
         */
        static void assignOperandWNTV(std::vector<Polygon256> &polygons, std::size_t dimension, std::size_t hotIndex);

        /**
         * @brief 递归收集未丢弃的只读叶子子问题。
         *
         * @param[in] node 当前遍历节点。
         * @param[out] outLeaves 追加写入叶子节点指针的容器。
         */
        static void collectLeafProblemsRecursive(const BoolProblem *node, std::vector<const BoolProblem *> &outLeaves);

        /**
         * @brief 递归收集未丢弃的可变叶子子问题。
         *
         * @param[in] node 当前遍历节点。
         * @param[out] outLeaves 追加写入叶子节点指针的容器。
         */
        static void collectLeafProblemsRecursive(BoolProblem *node, std::vector<BoolProblem *> &outLeaves);

        /// 当前布尔运算类型。
        BoolOp op_ = BoolOp::Intersection;

        /// 叶子阶段停止继续细分的 polygon 数量阈值。
        std::size_t leafPolygonThreshold_ = 25;

        /// 当前节点在 subdivision 树中的深度。
        std::size_t depth_ = 0;

        /// 当前节点是否为叶子子问题。
        bool isLeaf_ = true;

        /// 当前节点是否已被判定为空子问题。
        bool discarded_ = false;

        /// 当前节点是否已完成求解流程。
        bool solved_ = false;

        /// 当前节点最近一次细分使用的切分平面。
        Plane3i splitPlane_;

        /// 当前节点的局部 AABB。
        AABB3i aabb_;

        /// 当前节点保存的局部参考点及其 WNV。
        SubdivisionRefState reference_;

        /// 当前节点持有的局部 polygon soup。
        std::vector<Polygon256> polygons_;

        /// 叶子阶段局部 BSP 生成的全部 leaf fragments。
        std::vector<Polygon256> leafFragments_;

        /// 叶子阶段成功分类出的 fragment 及其前后侧 WNV。
        std::vector<ClassifiedFragment> classifiedFragments_;

        /// 当前节点或其子树筛选出的布尔结果面。
        std::vector<Polygon256> resultFragments_;

        /// 左子问题；不存在或为空时为 `nullptr`。
        std::unique_ptr<BoolProblem> leftChild_;

        /// 右子问题；不存在或为空时为 `nullptr`。
        std::unique_ptr<BoolProblem> rightChild_;
    };
}

