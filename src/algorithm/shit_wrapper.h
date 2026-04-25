// TODO：本文件是临时抽象层，存放所有新添加的重构前难以分类的几何处理等辅助函数，用于阻断前面的实现屎山外溢

#pragma once

#include "geometry/geometry256.h"

namespace ember
{
    inline PlanePoint3i getPolygonVertex(const Polygon256 &poly, std::size_t index) noexcept
    {
        const std::size_t n = poly.edgePlanes.size();
        if (n == 0 || index >= n)
        {
            return PlanePoint3i();
        }

        const std::size_t prev = (index == 0) ? (n - 1) : (index - 1);
        return PlanePoint3i(poly.plane, poly.edgePlanes[index], poly.edgePlanes[prev]);
    }

    inline constexpr bool areSamePlanePoint(const PlanePoint3i &lhs, const PlanePoint3i &rhs) noexcept
    {
        if (!lhs.hasUniqueIntersection() || !rhs.hasUniqueIntersection())
        {
            return false;
        }

        return areSameHomPoint(lhs.x, rhs.x);
    }

    // 点在线段上，端点也算
    inline constexpr bool isPointOnSegment(const PlanePoint3i &point, const Segment256 &seg) noexcept
    {
        if (!point.hasUniqueIntersection() || !seg.direction.contains(point))
        {
            return false;
        }

        int pcs = point.classify(seg.start);
        int pec = point.classify(seg.end);
        return (pcs != 1) && (pec != 1);
    }

    // 只判断是否接触顶点
    inline bool isSegmentTouchPolygonVertex(const Segment256 &seg, const Polygon256 &poly) noexcept
    {
        if (!seg.isValid() || !poly.isValid())
        {
            return false;
        }

        const std::size_t n = poly.edgePlanes.size();
        for (std::size_t i = 0; i < n; ++i)
        {
            const PlanePoint3i vertex = getPolygonVertex(poly, i);
            if (vertex.hasUniqueIntersection() && isPointOnSegment(vertex, seg))
            {
                return true;
            }
        }

        return false;
    }

    // 判断线段是否接触边，顶点也算
    inline bool isSegmentTouchPolygonEdge(const Segment256 &seg, const Polygon256 &poly) noexcept
    {
        if (!seg.isValid() || !poly.isValid())
        {
            return false;
        }

        const PlanePoint3i startPoint = seg.getStartPoint();
        const PlanePoint3i endPoint = seg.getEndPoint();

        const std::size_t n = poly.edgePlanes.size();
        for (std::size_t i = 0; i < n; ++i)
        {
            const Plane3i &edgePlane = poly.edgePlanes[i];
            const auto &nextEdgePlane = poly.edgePlanes[(i + 1) % n];
            const auto &prevEdgePlane = poly.edgePlanes[(i - 1 + n) % n];

            const Segment256 edge(prevEdgePlane, nextEdgePlane, {poly.plane, edgePlane});

            const PlanePoint3i edgeVertex0 = edge.getStartPoint();
            const PlanePoint3i edgeVertex1 = edge.getEndPoint();

            const PlanePoint3i edgeHit = intersect(seg.direction, edgePlane);
            if (edgeHit.hasUniqueIntersection())
            {
                if (isPointOnSegment(edgeHit, seg) && isPointOnSegment(edgeHit, edge))
                    return true;
                continue;
            }

            // 唯一求交失败时，只额外处理“线段与该边共线”的情况。
            if (startPoint.classify(poly.plane) != 0 || endPoint.classify(poly.plane) != 0 ||
                startPoint.classify(edgePlane) != 0 || endPoint.classify(edgePlane) != 0)
            {
                continue;
            }

            if (isPointOnSegment(edgeVertex0, seg) || isPointOnSegment(edgeVertex1, seg))
                return true;
            if (isPointOnSegment(startPoint, edge) || isPointOnSegment(endPoint, edge))
                return true;
        }

        return false;
    }

    
    std::vector<Plane3i> computeAABBPlanes(std::vector<Polygon256>& polygons)
    {
        if (polygons.empty())
        {
            return {};
        }

        bool initialized = false;
        Integer xMin, xMax, yMin, yMax, zMin, zMax;

        for (const auto &poly : polygons)
        {
            const std::size_t n = poly.edgePlanes.size();
            for (std::size_t i = 0; i < n; ++i)
            {
                const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
                const HomPoint4i v = intersectHomogeneous(poly.plane, poly.edgePlanes[i], poly.edgePlanes[prev]);

                if (isZero(v.w))
                {
                    continue;
                }

                const Integer fx = floorDiv(v.x, v.w);
                const Integer cx = ceilDiv(v.x, v.w);
                const Integer fy = floorDiv(v.y, v.w);
                const Integer cy = ceilDiv(v.y, v.w);
                const Integer fz = floorDiv(v.z, v.w);
                const Integer cz = ceilDiv(v.z, v.w);

                if (!initialized)
                {
                    xMin = fx;
                    xMax = cx;
                    yMin = fy;
                    yMax = cy;
                    zMin = fz;
                    zMax = cz;
                    initialized = true;
                }
                else
                {
                    if (fx < xMin)
                        xMin = fx;
                    if (cx > xMax)
                        xMax = cx;
                    if (fy < yMin)
                        yMin = fy;
                    if (cy > yMax)
                        yMax = cy;
                    if (fz < zMin)
                        zMin = fz;
                    if (cz > zMax)
                        zMax = cz;
                }
            }
        }

        if (!initialized)
        {
            return {};
        }

        const Integer margin = 1;
        xMin -= margin;
        xMax += margin;
        yMin -= margin;
        yMax += margin;
        zMin -= margin;
        zMax += margin;

        return {
            Plane3i(-1, 0, 0, xMin),
            Plane3i(1, 0, 0, -xMax),
            Plane3i(0, -1, 0, yMin),
            Plane3i(0, 1, 0, -yMax),
            Plane3i(0, 0, -1, zMin),
            Plane3i(0, 0, 1, -zMax),
        };
    }
}
