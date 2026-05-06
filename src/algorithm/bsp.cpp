/**
 * @file bsp.cpp
 * @brief 实现局部 BSP 插入和多边形编排的叶片收集。
 */
#include "bsp.h"

#include "core/perf_tracing.h"
#include "geometry/polygon_ops.h"

#include <stdexcept>

namespace ember
{
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
        REEMBER_PROFILE_ZONE("BSPTree::setBasePolygon");

        basePolygon = polygon;
        baseOrderKey = orderKey;
        root = std::make_unique<BSPNode>(basePolygon);
    }

    void BSPTree::insert(const Polygon256 &polygon, std::size_t incomingOrder)
    {
        REEMBER_PROFILE_ZONE("BSPTree::insert");

        if (!polygon.isValid())
        {
            throw std::runtime_error("BSPTree::insert received an invalid incoming polygon.");
        }

        if (!basePolygon.isValid())
        {
            throw std::runtime_error("BSPTree::insert cannot run because the base polygon is invalid.");
        }

        insertTrusted(polygon, incomingOrder);
    }

    void BSPTree::insertTrusted(const Polygon256 &polygon, std::size_t incomingOrder)
    {
        REEMBER_PROFILE_ZONE("BSPTree::insertTrusted");

        Plane3i segmentPlane;
        Plane3i v0;
        Plane3i v1;
        if (detail::computePolygonIntersectionCarrierTrusted(basePolygon, polygon, segmentPlane, v0, v1))
        {
            addSegmentRecursive(*root, v0, v1, segmentPlane);
            return;
        }

        if (!areCoplanarPolygons(basePolygon, polygon))
        {
            return;
        }

        insertCoplanarPolygonTrusted(polygon, incomingOrder);
    }

    void BSPTree::insertCoplanarPolygonTrusted(const Polygon256 &polygon, std::size_t incomingOrder)
    {
        REEMBER_PROFILE_ZONE("BSPTree::insertCoplanarPolygonTrusted");

        insertCoplanarPolygonEdges(polygon);

        if (baseOrderKey > incomingOrder)
        {
            disableOverlapLeaves(polygon);
        }
    }

    void BSPTree::addSegment(const Plane3i &v0, const Plane3i &v1, const Plane3i &splitPlane)
    {
        REEMBER_PROFILE_ZONE("BSPTree::addSegment");

        if (!root)
        {
            throw std::runtime_error("BSPTree::addSegment called before a base polygon was set.");
        }

        addSegmentRecursive(*root, v0, v1, splitPlane);
    }

    bool BSPTree::contains(const PlanePoint3i &point) const noexcept
    {
        return containsRecursive(root.get(), point);
    }

    std::vector<Polygon256> BSPTree::collectLeafGeometries() const
    {
        REEMBER_PROFILE_ZONE("BSPTree::collectLeafGeometries");

        std::vector<Polygon256> leafGeometries;
        collectLeafGeometriesRecursive(root.get(), leafGeometries);
        return leafGeometries;
    }

    void BSPTree::addSegmentRecursive(BSPNode &node, const Plane3i &v0, const Plane3i &v1, const Plane3i &insertPlane)
    {
        REEMBER_PROFILE_ZONE("BSPTree::addSegmentRecursive");

        if (node.isLeaf)
        {
            Polygon256 frontGeometry;
            Polygon256 backGeometry;

            // 分割时使用平面；线段端点 v0/v1（p0/p1）的分类讨论在进入叶节点前完成。
            if (!detail::clipLeafGeometryByPlaneTrusted(node.leafGeometry, insertPlane, frontGeometry, backGeometry))
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
            throw std::runtime_error("BSPTree invariant violation: non-leaf node is missing a child.");
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
                throw std::runtime_error("BSPTree failed to find a strict interior point while disabling overlap leaves.");
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
            // TODO：如果后续并行化，需要重新处理这里的收集写入。
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

