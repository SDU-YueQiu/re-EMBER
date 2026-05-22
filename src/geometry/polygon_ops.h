/**
 * @file polygon_ops.h
 * @brief 提供内联的多边形定向辅助函数。
 */
#pragma once

#include "geometry/geometry256.h"

#include <algorithm>
#include <cassert>
#include <vector>

namespace ember
{
/**
 * @brief 从支撑平面和相邻边平面读取一个多边形顶点。
 */
inline const PlanePoint3i &getPolygonVertex(const Polygon256 &poly, std::size_t index)
{
    return poly.vertex(index);
}

/**
 * @brief 判断两个有效多边形是否支撑在同一平面上。
 *
 * @pre `lhs` 与 `rhs` 都是可取顶点的有效多边形。
 */
inline bool areCoplanarPolygons(const Polygon256 &lhs, const Polygon256 &rhs)
{
    if (!arePlaneNormalsParallel(lhs.plane, rhs.plane))
        return false;

    const PlanePoint3i &firstVertex = getPolygonVertex(lhs, 0);
    return firstVertex.hasUniqueIntersection() && firstVertex.classify(rhs.plane) == 0;
}

inline Polygon256 reversePolygonOrientation(const Polygon256 &polygon)
{
    Polygon256 reversedPolygon;
    reversedPolygon.plane = reversedPlaneOrientationPreservingScale(polygon.plane);
    reversedPolygon.edgePlanes = polygon.edgePlanes;
    reversedPolygon.edgeProvenances = polygon.edgeProvenances;
    std::reverse(reversedPolygon.edgePlanes.begin(), reversedPolygon.edgePlanes.end());
    std::reverse(reversedPolygon.edgeProvenances.begin(), reversedPolygon.edgeProvenances.end());
    reversedPolygon.WNTV = polygon.WNTV;
#ifndef NDEBUG
    assert(reversedPolygon.isValid());
#endif
    return reversedPolygon;
}

}
