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
         * @param[in] orderKey 用于 overlap 去重的稳定顺序键。
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
         * @pre `setBasePolygon()` 已设置有效 base polygon，且 `polygon.isValid()` 为真。
         */
        void insertTrusted(const Polygon256& polygon, std::size_t incomingOrder = 0);

        /**
         * @brief 收集当前局部 BSP 的全部启用叶子几何。
         *
         * @return 按树遍历顺序输出的 leaf polygon 集合。
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
        //向node中递归插入以basepolygon.plane和insertPlane、v0、v1定义的线段
        void addSegmentRecursive(BSPNode& node, const Plane3i& v0, const Plane3i& v1, const Plane3i& insertPlane);
        
        //插入重叠多边形，会将polygon的所有边插入到bsp中
        void insertCoplanarPolygonEdges(const Polygon256& polygon);
        void disableOverlapLeaves(const Polygon256& polygon);

        //递归检测每片叶子多边形的内部点在不在polygon中，在说明叶子多边形在polygon中，禁用掉
        static void disableOverlapLeavesRecursive(BSPNode* node, const Polygon256& polygon);
        static bool containsRecursive(const BSPNode* node, const PlanePoint3i& point) noexcept;
        static void collectLeafGeometriesRecursive(const BSPNode* node, std::vector<Polygon256>& outLeafGeometries);
        std::unique_ptr<BSPNode> root;
		Polygon256 basePolygon;
        std::size_t baseOrderKey = 0;
    };
}

