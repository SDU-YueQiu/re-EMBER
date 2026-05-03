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

    namespace detail
    {
        enum class PolygonSurfaceLocation
        {
            Outside,
            Boundary,
            StrictInterior
        };

        /**
         * @brief 在已知点落在 polygon 支撑平面上时分类其面内位置。
         */
        inline PolygonSurfaceLocation classifyPolygonSurfacePointUnchecked(
            const Polygon256 &poly,
            const PlanePoint3i &point) noexcept
        {
            bool hasBoundary = false;
            for (const Plane3i &edge : poly.edgePlanes)
            {
                const int side = point.classify(edge);
                if (side > 0)
                {
                    return PolygonSurfaceLocation::Outside;
                }
                if (side == 0)
                {
                    hasBoundary = true;
                }
            }

            return hasBoundary ? PolygonSurfaceLocation::Boundary : PolygonSurfaceLocation::StrictInterior;
        }

        /**
         * @brief 在调用方已验证 polygon 的前提下判断点是否落在指定边段上。
         */
        inline bool isPointOnPolygonEdgeUnchecked(
            const PlanePoint3i &point,
            const Polygon256 &poly,
            std::size_t edgeIndex) noexcept
        {
            if (!point.hasUniqueIntersection())
            {
                return false;
            }

            const std::size_t edgeCount = poly.edgePlanes.size();
            if (edgeCount == 0 || edgeIndex >= edgeCount)
            {
                return false;
            }

            const std::size_t nextIndex = (edgeIndex + 1 == edgeCount) ? 0 : (edgeIndex + 1);
            const std::size_t prevIndex = (edgeIndex == 0) ? (edgeCount - 1) : (edgeIndex - 1);
            return point.classify(poly.plane) == 0 &&
                   point.classify(poly.edgePlanes[edgeIndex]) == 0 &&
                   point.classify(poly.edgePlanes[prevIndex]) != 1 &&
                   point.classify(poly.edgePlanes[nextIndex]) != 1;
        }

        /**
         * @brief 在调用方已验证 segment 与 polygon 的前提下判断线段是否接触 polygon 边界。
         */
        inline bool isSegmentTouchPolygonEdgeUnchecked(const Segment256 &seg, const Polygon256 &poly) noexcept
        {
            const PlanePoint3i startPoint = seg.getStartPoint();
            const PlanePoint3i endPoint = seg.getEndPoint();
            const std::size_t edgeCount = poly.edgePlanes.size();

            for (std::size_t edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
            {
                const std::size_t nextIndex = (edgeIndex + 1 == edgeCount) ? 0 : (edgeIndex + 1);
                const std::size_t prevIndex = (edgeIndex == 0) ? (edgeCount - 1) : (edgeIndex - 1);

                const Plane3i &edgePlane = poly.edgePlanes[edgeIndex];

                const PlanePoint3i edgeHit = intersect(seg.direction, edgePlane);
                if (edgeHit.hasUniqueIntersection())
                {
                    if (isPointOnSegment(edgeHit, seg) &&
                        isPointOnPolygonEdgeUnchecked(edgeHit, poly, edgeIndex))
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

                const PlanePoint3i edgeVertex0(poly.plane, edgePlane, poly.edgePlanes[prevIndex]);
                const PlanePoint3i edgeVertex1(poly.plane, poly.edgePlanes[nextIndex], edgePlane);
                if (isPointOnSegment(edgeVertex0, seg) || isPointOnSegment(edgeVertex1, seg))
                {
                    return true;
                }
                if (isPointOnPolygonEdgeUnchecked(startPoint, poly, edgeIndex) ||
                    isPointOnPolygonEdgeUnchecked(endPoint, poly, edgeIndex))
                {
                    return true;
                }
            }
            return false;
        }
    }

    inline bool isSegmentTouchPolygonEdge(const Segment256 &seg, const Polygon256 &poly) noexcept
    {
        if (!seg.isValid() || !poly.isValid())
        {
            return false;
        }

        return detail::isSegmentTouchPolygonEdgeUnchecked(seg, poly);
    }
}
