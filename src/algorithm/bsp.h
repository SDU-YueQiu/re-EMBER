#include "geometry/geometry256.h"
#include "algorithm/clipping.h"
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

        //仅设置要分割的多边形
        void setBasePolygon(const Polygon256& polygon, std::size_t orderKey = 0);
        
        //插入平面并求交、分割
        void insert(const Polygon256& polygon, std::size_t incomingOrder = 0);
        void addSegment(const Plane3i& v0, const Plane3i& v1, const Plane3i& splitPlane);
        bool contains(const PlanePoint3i& point) const noexcept;
        std::vector<Polygon256> collectLeafGeometries() const;

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
