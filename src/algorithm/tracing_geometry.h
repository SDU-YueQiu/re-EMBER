/**
 * @file tracing_geometry.h
 * @brief 定义 WNV 追踪和路径诊断共享的几何谓词。
 */
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

    namespace detail
    {
        enum class PolygonSurfaceLocation
        {
            Outside,
            Boundary,
            StrictInterior
        };

        enum class PolygonBoundaryContactType
        {
            None,
            BoundaryPointHit,
            EndpointOnBoundary,
            EdgeOverlap
        };

        struct PolygonBoundaryContact
        {
            PolygonBoundaryContactType type = PolygonBoundaryContactType::None;
            std::vector<std::size_t> edgeIndices;
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

        inline void collectBoundaryEdgesAtPointUnchecked(
            const PlanePoint3i &point,
            const Polygon256 &poly,
            PolygonBoundaryContact &contact)
        {
            const std::size_t edgeCount = poly.edgePlanes.size();
            for (std::size_t edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
            {
                if (isPointOnPolygonEdgeUnchecked(point, poly, edgeIndex))
                {
                    bool duplicated = false;
                    for (const std::size_t existing : contact.edgeIndices)
                    {
                        if (existing == edgeIndex)
                        {
                            duplicated = true;
                            break;
                        }
                    }
                    if (!duplicated)
                    {
                        contact.edgeIndices.push_back(edgeIndex);
                    }
                }
            }
        }

        inline void appendUniqueBoundaryEdgeIndex(
            std::vector<std::size_t> &edgeIndices,
            std::size_t edgeIndex)
        {
            for (const std::size_t existing : edgeIndices)
            {
                if (existing == edgeIndex)
                {
                    return;
                }
            }

            edgeIndices.push_back(edgeIndex);
        }

        inline int boundaryContactPriority(PolygonBoundaryContactType type) noexcept
        {
            switch (type)
            {
            case PolygonBoundaryContactType::None:
                return 0;
            case PolygonBoundaryContactType::BoundaryPointHit:
                return 1;
            case PolygonBoundaryContactType::EndpointOnBoundary:
                return 2;
            case PolygonBoundaryContactType::EdgeOverlap:
                return 3;
            }

            return 0;
        }

        inline void mergeBoundaryContact(
            PolygonBoundaryContact &contact,
            PolygonBoundaryContactType type,
            std::size_t edgeIndex)
        {
            if (boundaryContactPriority(type) > boundaryContactPriority(contact.type))
            {
                contact.type = type;
            }

            appendUniqueBoundaryEdgeIndex(contact.edgeIndices, edgeIndex);
        }

        /**
         * @brief 在调用方已验证 segment 与 polygon 的前提下分类线段与 polygon 边界的接触方式。
         */
        inline PolygonBoundaryContact classifySegmentPolygonBoundaryContactUnchecked(
            const Segment256 &seg,
            const Polygon256 &poly) noexcept
        {
            PolygonBoundaryContact contact;
            const PlanePoint3i startPoint = seg.getStartPoint();
            const PlanePoint3i endPoint = seg.getEndPoint();
            const bool segmentInSupportPlane =
                startPoint.classify(poly.plane) == 0 &&
                endPoint.classify(poly.plane) == 0;

            if (segmentInSupportPlane)
            {
                const std::size_t edgeCount = poly.edgePlanes.size();
                for (std::size_t edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
                {
                    const std::size_t nextIndex = (edgeIndex + 1 == edgeCount) ? 0 : (edgeIndex + 1);
                    const std::size_t prevIndex = (edgeIndex == 0) ? (edgeCount - 1) : (edgeIndex - 1);
                    const Plane3i &edgePlane = poly.edgePlanes[edgeIndex];

                    const PlanePoint3i edgeHit = intersect(seg.direction, edgePlane);
                    if (edgeHit.hasUniqueIntersection() &&
                        isPointOnSegment(edgeHit, seg) &&
                        isPointOnPolygonEdgeUnchecked(edgeHit, poly, edgeIndex))
                    {
                        const bool endpointHit =
                            areSamePlanePoint(edgeHit, startPoint) ||
                            areSamePlanePoint(edgeHit, endPoint);
                        mergeBoundaryContact(
                            contact,
                            endpointHit
                                ? PolygonBoundaryContactType::EndpointOnBoundary
                                : PolygonBoundaryContactType::EdgeOverlap,
                            edgeIndex);
                        continue;
                    }

                    const bool startOnBoundary = isPointOnPolygonEdgeUnchecked(startPoint, poly, edgeIndex);
                    const bool endOnBoundary = isPointOnPolygonEdgeUnchecked(endPoint, poly, edgeIndex);
                    if (startOnBoundary || endOnBoundary)
                    {
                        mergeBoundaryContact(
                            contact,
                            PolygonBoundaryContactType::EndpointOnBoundary,
                            edgeIndex);
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
                        mergeBoundaryContact(contact, PolygonBoundaryContactType::EdgeOverlap, edgeIndex);
                    }
                }

                return contact;
            }

            const PlanePoint3i surfaceHit = intersect(seg.direction, poly.plane);
            if (!surfaceHit.hasUniqueIntersection() || !isPointOnSegment(surfaceHit, seg))
            {
                return contact;
            }

            if (classifyPolygonSurfacePointUnchecked(poly, surfaceHit) != PolygonSurfaceLocation::Boundary)
            {
                return contact;
            }

            collectBoundaryEdgesAtPointUnchecked(surfaceHit, poly, contact);
            if (contact.edgeIndices.empty())
            {
                return contact;
            }

            contact.type =
                (areSamePlanePoint(surfaceHit, startPoint) || areSamePlanePoint(surfaceHit, endPoint))
                    ? PolygonBoundaryContactType::EndpointOnBoundary
                    : PolygonBoundaryContactType::BoundaryPointHit;
            return contact;
        }

        /**
         * @brief 返回边界接触是否全部命中 subdivision 裁剪边。
         */
        inline bool areBoundaryContactEdgesSubdivisionClip(
            const PolygonBoundaryContact &contact,
            const Polygon256 &poly) noexcept
        {
            if (contact.edgeIndices.empty())
            {
                return false;
            }

            for (const std::size_t edgeIndex : contact.edgeIndices)
            {
                if (poly.edgeProvenance(edgeIndex) != PolygonEdgeProvenance::SubdivisionClip)
                {
                    return false;
                }
            }

            return true;
        }

        inline bool isSegmentTouchPolygonEdgeUnchecked(const Segment256 &seg, const Polygon256 &poly) noexcept
        {
            return classifySegmentPolygonBoundaryContactUnchecked(seg, poly).type !=
                   PolygonBoundaryContactType::None;
        }
    }

}
