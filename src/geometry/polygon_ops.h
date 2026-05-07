/**
 * @file polygon_ops.h
 * @brief 提供内联的多边形定向与 AABB 裁剪辅助函数。
 */
#pragma once

#include "geometry/aabb.h"
#include "geometry/clipping.h"

#include <algorithm>
#include <utility>
#include <vector>

namespace ember
{
/**
 * @brief 从支撑平面和相邻边平面读取一个多边形顶点。
 */
inline const PlanePoint3i &getPolygonVertex(const Polygon256 &poly, std::size_t index) noexcept
{
    return poly.vertex(index);
}

/**
 * @brief 判断两个有效多边形是否支撑在同一平面上。
 *
 * @pre `lhs` 与 `rhs` 都是可取顶点的有效多边形。
 */
inline bool areCoplanarPolygons(const Polygon256 &lhs, const Polygon256 &rhs) noexcept
{
    if (!arePlaneNormalsParallel(lhs.plane, rhs.plane))
        return false;

    const PlanePoint3i &firstVertex = getPolygonVertex(lhs, 0);
    return firstVertex.hasUniqueIntersection() && firstVertex.classify(rhs.plane) == 0;
}

namespace detail
{
/**
 * @brief 将凸多边形裁剪到指定平面的一侧。
 *
 * @pre `source` 已满足 `Polygon256::isValid()`。
 */
inline bool clipPolygonToHalfSpace(
    const Polygon256 &source,
    const Plane3i &clipPlane,
    bool keepNonPositive,
    Polygon256 &outPolygon,
    PolygonEdgeProvenance insertedEdgeProvenance = PolygonEdgeProvenance::Regular)
{
    const std::size_t n = source.edgeCount();
    bool hasPositive = false;
    bool hasNegative = false;
    bool hasZero = false;

    for (std::size_t i = 0; i < n; ++i)
    {
        const PlanePoint3i &vertex = getPolygonVertex(source, i);
        const int side = vertex.classify(clipPlane);
        if (side > 0)
            hasPositive = true;
        else if (side < 0)
            hasNegative = true;
        else
            hasZero = true;
    }

    if (keepNonPositive)
    {
        if (!hasPositive)
        {
            outPolygon = source;
            return true;
        }
        if (!hasNegative)
            return false;
    }
    else
    {
        if (!hasNegative)
        {
            if (!hasPositive && hasZero)
                return false;
            outPolygon = source;
            return true;
        }
        if (!hasPositive)
            return false;
    }

    Polygon256 frontClipped;
    Polygon256 backClipped;
    if (!clipLeafGeometryByPlaneTrusted(
                source,
                clipPlane,
                frontClipped,
                backClipped,
                insertedEdgeProvenance))
        return false;

    outPolygon = keepNonPositive ? backClipped : frontClipped;
    return true;
}
}

inline bool clipPolygonToAABB(const Polygon256 &polygon, const AABB3i &box, Polygon256 &outPolygon)
{
    if (!polygon.isValid() || !isValidAABB(box))
        return false;

    Polygon256 current = polygon;
    const auto planes = makeAABBPlanes(box);
    for (const Plane3i &plane : planes)
    {
        Polygon256 clipped;
        if (!detail::clipPolygonToHalfSpace(current, plane, true, clipped))
            return false;
        current = clipped;
    }

    outPolygon = current;
    return true;
}

inline Polygon256 reversePolygonOrientation(const Polygon256 &polygon)
{
    std::vector<Plane3i> reversedEdges = polygon.edgePlanes;
    std::vector<PolygonEdgeProvenance> reversedProvenances = polygon.edgeProvenances;
    std::reverse(reversedEdges.begin(), reversedEdges.end());
    std::reverse(reversedProvenances.begin(), reversedProvenances.end());

    Polygon256 reversedPolygon(
        Plane3i(-polygon.plane.a, -polygon.plane.b, -polygon.plane.c, -polygon.plane.d),
        std::move(reversedEdges),
        std::move(reversedProvenances));
    reversedPolygon.WNTV = polygon.WNTV;
    return reversedPolygon;
}

}
