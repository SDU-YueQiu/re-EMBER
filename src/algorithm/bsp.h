#include "geometry/geometry256.h"
#include "algorithm/clipping.h"
#include <memory>
#include <vector>

namespace ember
{
    struct BSPNode
    {
        bool isLeaf;
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
        void setBasePolygon(const Polygon256& polygon);
        
        //插入平面并求交、分割
        void insert(const Polygon256& polygon);
        void addSegment(const Plane3i& v0, const Plane3i& v1, const Plane3i& splitPlane);
        bool contains(const PlanePoint3i& point) const noexcept;
        std::vector<Polygon256> collectLeafGeometries() const;

    private:
        void addSegmentRecursive(BSPNode& node, const Plane3i& v0, const Plane3i& v1, const Plane3i& insertPlane);
        static bool containsRecursive(const BSPNode* node, const PlanePoint3i& point) noexcept;
        static void collectLeafGeometriesRecursive(const BSPNode* node, std::vector<Polygon256>& outLeafGeometries);
        std::unique_ptr<BSPNode> root;
		Polygon256 basePolygon;

    };
}