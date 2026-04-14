#include "bsp.h"

namespace ember
{
    BSPNode::BSPNode() noexcept
        : isLeaf(true), splitPlane(), leafGeometry(), front(), back()
    {
    }

    BSPNode::BSPNode(const Polygon256& polygon) noexcept
        : isLeaf(true), splitPlane(), leafGeometry(polygon), front(), back()
    {
    }

    void BSPTree::setBasePolygon(const Polygon256& polygon)
    {
        basePolygon = polygon;
    }

    void BSPTree::insert(const Polygon256& polygon)
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

        //相当于初始化
        //TODO:在这儿进行有点儿丑陋
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
        }
        else {
            //TODO: 重合的情况按ember论文里的描述处理
        }
    }

    void BSPTree::addSegment(const Plane3i& v0, const Plane3i& v1, const Plane3i& splitPlane)
    {
        if (!root)
        {
            return;
        }

        addSegmentRecursive(*root, v0, v1, splitPlane);
    }

    bool BSPTree::contains(const PlanePoint3i& point) const noexcept
    {
        return containsRecursive(root.get(), point);
    }

    std::vector<Polygon256> BSPTree::collectLeafGeometries() const
    {
        std::vector<Polygon256> leafGeometries;
        collectLeafGeometriesRecursive(root.get(), leafGeometries);
        return leafGeometries;
    }

    void BSPTree::addSegmentRecursive(BSPNode& node, const Plane3i& v0, const Plane3i& v1, const Plane3i& insertPlane)
    {

        if (node.isLeaf)
        {
            Polygon256 frontGeometry;
            Polygon256 backGeometry;
            if (!clipLeafGeometryByPlane(node.leafGeometry, insertPlane, frontGeometry, backGeometry))
            {
                return;
            }

            node.isLeaf = false;
            node.splitPlane = insertPlane;
            node.leafGeometry = Polygon256();
            node.front = std::make_unique<BSPNode>(frontGeometry);
            node.back = std::make_unique<BSPNode>(backGeometry);
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
            const Plane3i& v2 = node.splitPlane;
            addSegmentRecursive(*node.front, v0, v2, insertPlane);
            addSegmentRecursive(*node.back, v2, v1, insertPlane);
            return;
        }

        if (side0 == -1 && side1 == 1) 
        {
            const Plane3i& v2 = node.splitPlane;
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

    bool BSPTree::containsRecursive(const BSPNode* node, const PlanePoint3i& point) noexcept
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

    void BSPTree::collectLeafGeometriesRecursive(const BSPNode* node, std::vector<Polygon256>& outLeafGeometries)
    {
        if (!node)
        {
            return;
        }

        if (node->isLeaf)
        {
            outLeafGeometries.push_back(node->leafGeometry);
            return;
        }

        collectLeafGeometriesRecursive(node->front.get(), outLeafGeometries);
        collectLeafGeometriesRecursive(node->back.get(), outLeafGeometries);
    }
}
