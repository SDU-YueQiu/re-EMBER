/**
 * @file bsp.h
 * @brief 声明用其他多边形切分单个基底多边形的局部 BSP 树。
 */
#include "geometry/geometry256.h"
#include "geometry/clipping.h"
#include <memory>
#include <vector>

namespace ember
{
    struct BSPNode
    {
        bool isLeaf;
        bool disabled;
        Plane3i splitPlane;
        Polygon256 leafGeometry;
        std::unique_ptr<BSPNode> front;
        std::unique_ptr<BSPNode> back;

        BSPNode() noexcept;
        explicit BSPNode(const Polygon256& polygon) noexcept;
    };

    class BSPTree
    {
    public:

        BSPTree() noexcept = default;
        ~BSPTree() noexcept = default;
        BSPTree(const BSPTree&) = delete;
        BSPTree& operator=(const BSPTree&) = delete;
        BSPTree(BSPTree&&) noexcept = default;
        BSPTree& operator=(BSPTree&&) noexcept = default;

        /**
         * @brief 设置局部 BSP 的基底多边形。
         *
         * @param[in] polygon 当前局部 BSP 所对应的输入多边形。
         * @param[in] orderKey 用于重叠区域去重的稳定顺序键。
         * @note 该操作会重置树结构，并以 `polygon` 作为初始唯一叶子。
         */
        void setBasePolygon(const Polygon256& polygon, std::size_t orderKey = 0);
        
        /**
         * @brief 将另一输入多边形对当前基底多边形的影响插入局部 BSP。
         *
         * @param[in] polygon 待插入多边形。
         * @param[in] incomingOrder 待插入多边形的稳定顺序键。
         */
        void insert(const Polygon256& polygon, std::size_t incomingOrder = 0);

        /**
         * @brief 插入已由调用方验证过的多边形。
         *
         * @param[in] polygon 待插入多边形。
         * @param[in] incomingOrder 待插入多边形的稳定顺序键。
         * @pre `setBasePolygon()` 已设置有效基底多边形，且 `polygon.isValid()` 为真。
         */
        void insertTrusted(const Polygon256& polygon, std::size_t incomingOrder = 0);

        /**
         * @brief 插入已确认与基底多边形共面的多边形。
         *
         * @param[in] polygon 待插入的共面多边形。
         * @param[in] incomingOrder 待插入多边形的稳定顺序键。
         * @pre `polygon` 与当前基底多边形支撑平面共面。
         */
        void insertCoplanarPolygonTrusted(const Polygon256& polygon, std::size_t incomingOrder = 0);

        /**
         * @brief 收集当前局部 BSP 的全部启用叶子几何。
         *
         * @return 按树遍历顺序输出的叶片多边形集合。
         */
        std::vector<Polygon256> collectLeafGeometries() const;

        /**
         * @brief 直接向当前局部 BSP 插入一条已构造好的切分线段。
         *
         * @param[in] v0 线段第一个端点约束平面。
         * @param[in] v1 线段第二个端点约束平面。
         * @param[in] splitPlane 与基底平面共同定义该线段支撑线的切分平面。
         */
        void addSegment(const Plane3i& v0, const Plane3i& v1, const Plane3i& splitPlane);
        bool contains(const PlanePoint3i& point) const noexcept;

    private:
        // 向节点中递归插入以基底多边形平面、切分平面、v0 和 v1 定义的线段。
        void addSegmentRecursive(BSPNode& node, const Plane3i& v0, const Plane3i& v1, const Plane3i& insertPlane);
        
        // 插入共面重叠多边形，会将该多边形的所有边插入到 BSP 中。
        void insertCoplanarPolygonEdges(const Polygon256& polygon);
        void disableOverlapLeaves(const Polygon256& polygon);

        // 递归检测每片叶子多边形的内部点是否在指定多边形中；若在其中则禁用该叶片。
        static void disableOverlapLeavesRecursive(BSPNode* node, const Polygon256& polygon);
        static bool containsRecursive(const BSPNode* node, const PlanePoint3i& point) noexcept;
        static void collectLeafGeometriesRecursive(const BSPNode* node, std::vector<Polygon256>& outLeafGeometries);
        std::unique_ptr<BSPNode> root;
		Polygon256 basePolygon;
        std::size_t baseOrderKey = 0;
    };
}

