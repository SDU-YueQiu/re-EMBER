/**
 * @file bsp.cpp
 * @brief 实现局部 BSP 插入和多边形编排的叶片收集。
 */
#include "bsp.h"

#include "core/perf_tracing.h"
#include "geometry/polygon_ops.h"

#include <stdexcept>
#include <utility>

namespace ember
{
namespace
{
bool isLeafCoveredByPolygon(const Polygon256 &leaf, const Polygon256 &polygon)
{
    if (!leaf.isValid())
        throw std::runtime_error("BSPTree encountered an invalid leaf while disabling overlap leaves.");

    const std::vector<PlanePoint3i> &vertices = leaf.vertices();
    if (vertices.size() != leaf.edgeCount())
        throw std::runtime_error("BSPTree encountered an inconsistent leaf while disabling overlap leaves.");

    // 共面重叠时，incoming polygon 的边已经全部插入 BSP。叶片不再跨越 incoming 的边，
    // 因而只需检查凸叶片的所有顶点是否位于 incoming polygon 内或边界上。
    for (const PlanePoint3i &vertex : vertices)
    {
        if (!polygon.containsOrOnBoundary(vertex))
            return false;
    }

    return true;
}

int classifyPlaneTripleIntersectionAgainstPlane(
    const Plane3i &p,
    const Plane3i &q,
    const Plane3i &r,
    const Plane3i &s) noexcept
{
    // 只需要侧别时，直接使用未约分齐次交点；sign(dot) * sign(w) 对非零比例缩放不变。
    const HomPoint4i point = intersectHomogeneousUnnormalized(p, q, r);
    if (isZero(point.w))
        return 0;

    const DotInteger dotValue =
        DotInteger(point.x) * DotInteger(s.a) +
        DotInteger(point.y) * DotInteger(s.b) +
        DotInteger(point.z) * DotInteger(s.c) +
        DotInteger(point.w) * DotInteger(s.d);
    return ((dotValue > 0) - (dotValue < 0)) * signum(point.w);
}
}

BSPNode::BSPNode() noexcept
    : isLeaf(true), disabled(false), splitPlane(), leafGeometry(), front(), back()
{
}

BSPNode::BSPNode(const Polygon256 &polygon) noexcept
    : isLeaf(true), disabled(false), splitPlane(), leafGeometry(polygon), front(), back()
{
}

BSPNode::BSPNode(Polygon256 &&polygon) noexcept
    : isLeaf(true), disabled(false), splitPlane(), leafGeometry(std::move(polygon)), front(), back()
{
}

void BSPTree::setBasePolygon(const Polygon256 &polygon, std::size_t orderKey)
{
    REEMBER_PROFILE_ZONE("BSPTree::setBasePolygon");

    basePolygon = polygon;
    basePlane = basePolygon.plane;
    hasFullBasePolygon = true;
    baseOrderKey = orderKey;
    root = std::make_unique<BSPNode>(basePolygon);
}

void BSPTree::setBasePolygonForPrecomputedRelations(const Polygon256 &polygon, std::size_t orderKey)
{
    REEMBER_PROFILE_ZONE("BSPTree::setBasePolygonForPrecomputedRelations");

    basePolygon = Polygon256();
    basePlane = polygon.plane;
    hasFullBasePolygon = false;
    baseOrderKey = orderKey;
    root = std::make_unique<BSPNode>(polygon);
}

void BSPTree::insert(const Polygon256 &polygon, std::size_t incomingOrder)
{
    REEMBER_PROFILE_ZONE("BSPTree::insert");

    if (!polygon.isValid())
        throw std::runtime_error("BSPTree::insert received an invalid incoming polygon.");

    if (!hasFullBasePolygon || !basePolygon.isValid())
        throw std::runtime_error("BSPTree::insert cannot run because the base polygon is invalid.");

    insertTrusted(polygon, incomingOrder);
}

void BSPTree::insertTrusted(const Polygon256 &polygon, std::size_t incomingOrder)
{
    REEMBER_PROFILE_ZONE("BSPTree::insertTrusted");

    if (!hasFullBasePolygon)
        throw std::runtime_error("BSPTree::insertTrusted requires a full base polygon.");

    if (!doAABBsOverlap(basePolygon.aabb(), polygon.aabb()))
        return;

    Plane3i segmentPlane;
    Plane3i v0;
    Plane3i v1;
    if (detail::computePolygonIntersectionCarrierTrusted(basePolygon, polygon, segmentPlane, v0, v1))
    {
        addSegmentRecursive(*root, v0, v1, segmentPlane);
        return;
    }

    if (!areCoplanarPolygons(basePolygon, polygon))
        return;

    insertCoplanarPolygonTrusted(polygon, incomingOrder);
}

void BSPTree::insertCoplanarPolygonTrusted(const Polygon256 &polygon, std::size_t incomingOrder)
{
    REEMBER_PROFILE_ZONE("BSPTree::insertCoplanarPolygonTrusted");

    insertCoplanarPolygonEdges(polygon);

    if (baseOrderKey > incomingOrder)
        disableOverlapLeaves(polygon);
}

void BSPTree::addSegment(const Plane3i &v0, const Plane3i &v1, const Plane3i &splitPlane)
{
    REEMBER_PROFILE_ZONE("BSPTree::addSegment");

    if (!root)
        throw std::runtime_error("BSPTree::addSegment called before a base polygon was set.");

    addSegmentRecursive(*root, v0, v1, splitPlane);
}

void BSPTree::extractLeafGeometriesInto(std::vector<Polygon256> &outLeafGeometries)
{
    REEMBER_PROFILE_ZONE("BSPTree::extractLeafGeometriesInto");

    extractLeafGeometriesRecursive(root.get(), outLeafGeometries);
}

std::size_t BSPTree::visitLeafGeometries(void *userData, BSPLeafGeometryVisitor visitor)
{
    REEMBER_PROFILE_ZONE("BSPTree::visitLeafGeometries");

    if (visitor == nullptr)
        throw std::runtime_error("BSPTree::visitLeafGeometries received a null visitor.");

    return visitLeafGeometriesRecursive(root.get(), userData, visitor);
}

void BSPTree::addSegmentRecursive(BSPNode &node, const Plane3i &v0, const Plane3i &v1, const Plane3i &insertPlane)
{
    REEMBER_PROFILE_ZONE("BSPTree::addSegmentRecursive");

    if (node.isLeaf)
    {
        Polygon256 frontGeometry;
        Polygon256 backGeometry;

        // 分割时使用平面；线段端点 v0/v1（p0/p1）的分类讨论在进入叶节点前完成。
        if (!detail::clipLeafGeometryByPlaneTrusted(
                    node.leafGeometry,
                    insertPlane,
                    frontGeometry,
                    backGeometry,
                    PolygonEdgeProvenance::ArrangementClip))
            return;

        node.isLeaf = false;
        node.splitPlane = insertPlane;
        node.leafGeometry = Polygon256();
        node.front = std::make_unique<BSPNode>(std::move(frontGeometry));
        node.back = std::make_unique<BSPNode>(std::move(backGeometry));
        node.front->disabled = node.disabled;
        node.back->disabled = node.disabled;
        return;
    }

    if (!node.front || !node.back)
        throw std::runtime_error("BSPTree invariant violation: non-leaf node is missing a child.");

    if (areSamePlaneEquation(node.splitPlane, insertPlane))
        return;

    const int side0 = classifyPlaneTripleIntersectionAgainstPlane(basePlane, v0, insertPlane, node.splitPlane);
    const int side1 = classifyPlaneTripleIntersectionAgainstPlane(basePlane, v1, insertPlane, node.splitPlane);
#ifndef NDEBUG
    const PlanePoint3i p0(basePlane, v0, insertPlane);
    const PlanePoint3i p1(basePlane, v1, insertPlane);
    if (side0 != p0.classify(node.splitPlane) || side1 != p1.classify(node.splitPlane))
        throw std::runtime_error("BSPTree optimized endpoint classification disagrees with PlanePoint3i.");
#endif

    if (side0 == 0 && side1 == 0)
        return;

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
        return;

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
        return;

    if (node->isLeaf)
    {
        if (isLeafCoveredByPolygon(node->leafGeometry, polygon))
            node->disabled = true;
        return;
    }

    disableOverlapLeavesRecursive(node->front.get(), polygon);
    disableOverlapLeavesRecursive(node->back.get(), polygon);
}

void BSPTree::extractLeafGeometriesRecursive(BSPNode *node, std::vector<Polygon256> &outLeafGeometries)
{
    if (!node)
        return;

    if (node->isLeaf)
    {
        // TODO：如果后续并行化，需要重新处理这里的收集写入。
        if (!node->disabled)
            outLeafGeometries.push_back(std::move(node->leafGeometry));
        return;
    }

    extractLeafGeometriesRecursive(node->front.get(), outLeafGeometries);
    extractLeafGeometriesRecursive(node->back.get(), outLeafGeometries);
}

std::size_t BSPTree::visitLeafGeometriesRecursive(
    BSPNode *node,
    void *userData,
    BSPLeafGeometryVisitor visitor)
{
    if (!node)
        return 0u;

    if (node->isLeaf)
    {
        if (node->disabled)
            return 0u;

        visitor(userData, std::move(node->leafGeometry));
        return 1u;
    }

    const std::size_t frontCount = visitLeafGeometriesRecursive(node->front.get(), userData, visitor);
    const std::size_t backCount = visitLeafGeometriesRecursive(node->back.get(), userData, visitor);
    return frontCount + backCount;
}
}

