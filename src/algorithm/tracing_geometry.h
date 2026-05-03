#pragma once

#include "geometry/polygon_ops.h"

namespace ember
{
    /**
     * @brief 比较两个由平面组定义的点，输入无唯一交点时直接判为不相等。
     *
     * @note 该函数只服务当前精确谓词链路，调用方不能把它当作任意齐次坐标的通用相等谓词。
     */
    inline constexpr bool areSamePlanePoint(const PlanePoint3i &lhs, const PlanePoint3i &rhs) noexcept
    {
        if (!lhs.hasUniqueIntersection() || !rhs.hasUniqueIntersection())
        {
            return false;
        }
        return areSameHomPoint(lhs.x, rhs.x);
    }

    inline constexpr bool isPointOnSegment(const PlanePoint3i &point, const Segment256 &seg) noexcept
    {
        if (!point.hasUniqueIntersection() || !seg.direction.contains(point))
        {
            return false;
        }

        const int pcs = point.classify(seg.start);
        const int pec = point.classify(seg.end);
        return (pcs != 1) && (pec != 1);
    }

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

    inline bool isSegmentTouchPolygonEdge(const Segment256 &seg, const Polygon256 &poly) noexcept
    {
        if (!seg.isValid() || !poly.isValid())
        {
            return false;
        }

        const PlanePoint3i startPoint = seg.getStartPoint();
        const PlanePoint3i endPoint = seg.getEndPoint();
        const std::size_t edgeCount = poly.edgePlanes.size();

        for (std::size_t edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
        {
            const std::size_t nextIndex = (edgeIndex + 1 == edgeCount) ? 0 : (edgeIndex + 1);
            const std::size_t prevIndex = (edgeIndex == 0) ? (edgeCount - 1) : (edgeIndex - 1);

            const Plane3i &edgePlane = poly.edgePlanes[edgeIndex];
            const Plane3i &nextEdgePlane = poly.edgePlanes[nextIndex];
            const Plane3i &prevEdgePlane = poly.edgePlanes[prevIndex];

            const Line256 polygonEdgeLine = Line256(poly.plane, edgePlane);
            const Segment256 polygonEdge = Segment256(prevEdgePlane, nextEdgePlane, polygonEdgeLine);
            const PlanePoint3i edgeVertex0 = polygonEdge.getStartPoint();
            const PlanePoint3i edgeVertex1 = polygonEdge.getEndPoint();

            const PlanePoint3i edgeHit = intersect(seg.direction, edgePlane);
            if (edgeHit.hasUniqueIntersection())
            {
                if (isPointOnSegment(edgeHit, seg) && isPointOnSegment(edgeHit, polygonEdge))
                {
                    return true;
                }
                continue;
            }

            if (startPoint.classify(poly.plane) != 0 || endPoint.classify(poly.plane) != 0)
            {
                continue;
            }
            if (startPoint.classify(edgePlane) != 0 || endPoint.classify(edgePlane) != 0)
            {
                continue;
            }

            if (isPointOnSegment(edgeVertex0, seg) || isPointOnSegment(edgeVertex1, seg))
            {
                return true;
            }
            if (isPointOnSegment(startPoint, polygonEdge) || isPointOnSegment(endPoint, polygonEdge))
            {
                return true;
            }
        }
        return false;
    }
}
