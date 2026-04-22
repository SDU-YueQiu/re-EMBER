// TODO：本文件是临时抽象层，用于阻断前面的实现屎山外溢

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
}
