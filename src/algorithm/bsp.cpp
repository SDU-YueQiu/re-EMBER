#include "bsp.h"

namespace ember
{
    bool areCoplanarPolygons(const Polygon256 &lhs, const Polygon256 &rhs) noexcept
    {
        // 因为直接公式判断的精度未测试，所以先用这种查顶点在不在两面上的别扭实现
        // TODO：有性能问题再改

        if (!arePlaneNormalsParallel(lhs.plane, rhs.plane))
        {
            return false;
        }

        PlanePoint3i vertex(lhs.plane, lhs.edgePlanes[0], lhs.edgePlanes[1]);

        if (!vertex.hasUniqueIntersection())
            return false;

        return vertex.classify(rhs.plane) == 0;
    }

    BSPNode::BSPNode() noexcept
        : isLeaf(true), disabled(false), splitPlane(), leafGeometry(), front(), back()
    {
    }

    BSPNode::BSPNode(const Polygon256 &polygon) noexcept
        : isLeaf(true), disabled(false), splitPlane(), leafGeometry(polygon), front(), back()
    {
    }

    void BSPTree::setBasePolygon(const Polygon256 &polygon, std::size_t orderKey)
    {
        basePolygon = polygon;
        baseOrderKey = orderKey;
        root.reset();
    }

    void BSPTree::insert(const Polygon256 &polygon, std::size_t incomingOrder)
    {
        if (!polygon.isValid())
        {
            std::cout << "BSPTree::insert !polygon.isValid()" << std::endl;
            return;
        }

        if (!basePolygon.isValid())
        {
            std::cout << "BSPTree::insert !basePolygon.isValid()" << std::endl;
            return;
        }

        // 相当于初始化
        // TODO:在这儿进行有点儿丑陋
        if (!root)
        {
            root = std::make_unique<BSPNode>(basePolygon);
        }

        Plane3i segmentPlane;
        Plane3i v0;
        Plane3i v1;
        if (computePolygonIntersectionCarrier(basePolygon, polygon, segmentPlane, v0, v1))
        {
            addSegmentRecursive(*root, v0, v1, segmentPlane);
            return;
        }

        if (!areCoplanarPolygons(basePolygon, polygon))
        {
            return;
        }

        insertCoplanarPolygonEdges(polygon);

        if (baseOrderKey > incomingOrder)
        {
            disableOverlapLeaves(polygon);
        }
    }

    void BSPTree::addSegment(const Plane3i &v0, const Plane3i &v1, const Plane3i &splitPlane)
    {
        if (!root)
        {
            return;
        }

        addSegmentRecursive(*root, v0, v1, splitPlane);
    }

    bool BSPTree::contains(const PlanePoint3i &point) const noexcept
    {
        return containsRecursive(root.get(), point);
    }

    std::vector<Polygon256> BSPTree::collectLeafGeometries() const
    {
        std::vector<Polygon256> leafGeometries;
        collectLeafGeometriesRecursive(root.get(), leafGeometries);
        return leafGeometries;
    }

    void BSPTree::addSegmentRecursive(BSPNode &node, const Plane3i &v0, const Plane3i &v1, const Plane3i &insertPlane)
    {

        if (node.isLeaf)
        {
            Polygon256 frontGeometry;
            Polygon256 backGeometry;

            // 分割时用平面，对线段端点v0 v1(p0 p1)的分类讨论在进入叶节点之前就进行
            if (!clipLeafGeometryByPlane(node.leafGeometry, insertPlane, frontGeometry, backGeometry))
            {
                return;
            }

            node.isLeaf = false;
            node.splitPlane = insertPlane;
            node.leafGeometry = Polygon256();
            node.front = std::make_unique<BSPNode>(frontGeometry);
            node.back = std::make_unique<BSPNode>(backGeometry);
            node.front->disabled = node.disabled;
            node.back->disabled = node.disabled;
            return;
        }

        if (!node.front || !node.back)
        {
            std::cout << "[BSPTree::addSegmentRecursive] !node.front || !node.back" << std::endl;
            return;
        }

        PlanePoint3i p0(basePolygon.plane, v0, insertPlane);
        PlanePoint3i p1(basePolygon.plane, v1, insertPlane);

        int side0 = p0.classify(node.splitPlane);
        int side1 = p1.classify(node.splitPlane);

        if (side0 == 0 && side1 == 0)
        {
            return;
        }

        if (side0 == 1 && side1 == -1)
        {
            const Plane3i &v2 = node.splitPlane;
            addSegmentRecursive(*node.front, v0, v2, insertPlane);
            addSegmentRecursive(*node.back, v2, v1, insertPlane);
            return;
        }

        if (side0 == -1 && side1 == 1)
        {
            const Plane3i &v2 = node.splitPlane;
            addSegmentRecursive(*node.front, v1, v2, insertPlane);
            addSegmentRecursive(*node.back, v2, v0, insertPlane);
            return;
        }

        if (side0 == 1 || side1 == 1)
        {
            addSegmentRecursive(*node.front, v0, v1, insertPlane);
            return;
        }

        if (side0 == -1 || side1 == -1)
        {
            addSegmentRecursive(*node.back, v0, v1, insertPlane);
            return;
        }
    }

    void BSPTree::insertCoplanarPolygonEdges(const Polygon256 &polygon)
    {
        if (!root || polygon.edgeCount() < 3)
        {
            return;
        }

        const std::size_t n = polygon.edgeCount();
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const std::size_t next = (i + 1 == n) ? 0 : (i + 1);
            addSegmentRecursive(*root, polygon.edgePlanes[prev], polygon.edgePlanes[next], polygon.edgePlanes[i]);
        }
    }

    void BSPTree::disableOverlapLeaves(const Polygon256 &polygon)
    {
        disableOverlapLeavesRecursive(root.get(), polygon);
    }

    void BSPTree::disableOverlapLeavesRecursive(BSPNode *node, const Polygon256 &polygon)
    {
        if (!node)
        {
            return;
        }

        if (node->isLeaf)
        {
            PlanePoint3i interiorPoint(node->leafGeometry.plane, node->leafGeometry.plane, node->leafGeometry.plane);
            if (!node->leafGeometry.findStrictInteriorPoint(interiorPoint))
            {
                return;
            }

            if (polygon.containsOrOnBoundary(interiorPoint))
            {
                node->disabled = true;
            }
            return;
        }

        disableOverlapLeavesRecursive(node->front.get(), polygon);
        disableOverlapLeavesRecursive(node->back.get(), polygon);
    }

    bool BSPTree::containsRecursive(const BSPNode *node, const PlanePoint3i &point) noexcept
    {
        if (!node)
        {
            return false;
        }

        if (node->isLeaf)
        {
            return node->leafGeometry.containsOrOnBoundary(point);
        }

        const int side = point.classify(node->splitPlane);
        if (side > 0)
        {
            return containsRecursive(node->front.get(), point);
        }
        if (side < 0)
        {
            return containsRecursive(node->back.get(), point);
        }

        return containsRecursive(node->front.get(), point) || containsRecursive(node->back.get(), point);
    }

    void BSPTree::collectLeafGeometriesRecursive(const BSPNode *node, std::vector<Polygon256> &outLeafGeometries)
    {
        if (!node)
        {
            return;
        }

        if (node->isLeaf)
        {
            // TODO：多线程隐患
            if (!node->disabled)
            {
                outLeafGeometries.push_back(node->leafGeometry);
            }
            return;
        }

        collectLeafGeometriesRecursive(node->front.get(), outLeafGeometries);
        collectLeafGeometriesRecursive(node->back.get(), outLeafGeometries);
    }
}
