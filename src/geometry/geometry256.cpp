/**
 * @file geometry256.cpp
 * @brief 实现精确 256 位线、线段和多边形几何操作。
 */
#include "geometry256.h"

#include <utility>

namespace ember
{
    namespace
    {
        Integer absInteger(const Integer &value) noexcept
        {
            return value < 0 ? -value : value;
        }

        bool canScalePlaneWithinInsetHeadroom(const Plane3i &plane, const Integer &scale) noexcept
        {
            if (scale <= 0)
            {
                return false;
            }

            const Integer coefficientLimit = Integer(1) << 70;
            return absInteger(plane.a) <= coefficientLimit / scale &&
                   absInteger(plane.b) <= coefficientLimit / scale &&
                   absInteger(plane.c) <= coefficientLimit / scale &&
                   absInteger(plane.d) <= coefficientLimit / scale;
        }
    }

    // 返回使用平面方程反转后的平面
    Plane3i flippedPlane(const Plane3i &plane) noexcept
    {
        return Plane3i(-plane.a, -plane.b, -plane.c, -plane.d);
    }

    // 保证多边形边平面法向向外
    void orientPolygonEdgesOutward(const Plane3i &supportPlane, std::vector<Plane3i> &edges) noexcept
    {
        const std::size_t n = edges.size();
        if (n < 3)
        {
            return;
        }

        // 计算多边形所有顶点（边平面的交点）
        std::vector<PlanePoint3i> vertices;
        vertices.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const PlanePoint3i vertex(supportPlane, edges[i], edges[prev]);
            if (!vertex.hasUniqueIntersection())
            {
                return;
            }
            vertices.push_back(vertex);
        }

        // 对每条边进行方向检查
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t next = (i + 1 == n) ? 0 : (i + 1);

            // 选择一个参考顶点（非当前边端点）
            std::size_t refIndex = n;
            for (std::size_t k = 0; k < n; ++k)
            {
                if (k != i && k != next)
                {
                    refIndex = k;
                    break;
                }
            }

            if (refIndex == n)
            {
                return;
            }

            // 检查参考顶点相对于当前边平面的位置
            const int interiorSide = vertices[refIndex].classify(edges[i]);
            if (interiorSide > 0)
            {
                // 如果参考顶点在正侧（外部），需要翻转边平面方向
                edges[i] = flippedPlane(edges[i]);
            }
        }
    }

    // 保证线段端点平面法向正确
    void orientSegmentBoundsOutward(Plane3i &startPlane, Plane3i &endPlane, const Line256 &directionLine) noexcept
    {
        if (!directionLine.isValid())
        {
            return;
        }

        // 待办：为点添加更多构造函数。
        const PlanePoint3i startPoint(directionLine.p1, directionLine.p2, startPlane);
        const PlanePoint3i endPoint(directionLine.p1, directionLine.p2, endPlane);
        if (!startPoint.hasUniqueIntersection() || !endPoint.hasUniqueIntersection())
        {
            return;
        }

        if (endPoint.classify(startPlane) > 0)
        {
            startPlane = flippedPlane(startPlane);
        }
        if (startPoint.classify(endPlane) > 0)
        {
            endPlane = flippedPlane(endPlane);
        }
    }

    Polygon256::Polygon256(const Plane3i &supportPlane, std::vector<Plane3i> edges)
        : plane(primitivePlane(supportPlane)), edgePlanes(std::move(edges))
    {
        for (Plane3i &edge : edgePlanes)
        {
            edge = primitivePlane(edge);
        }
        orientPolygonEdgesOutward(plane, edgePlanes);
    }

    Segment256::Segment256(const Plane3i &startPlane, const Plane3i &endPlane, const Line256 &directionLine) noexcept
        : start(primitivePlane(startPlane)), end(primitivePlane(endPlane)), direction(directionLine)
    {
        orientSegmentBoundsOutward(start, end, direction);
    }

    void Polygon256::addEdgePlane(const Plane3i &edge)
    {
        edgePlanes.push_back(primitivePlane(edge));
    }

    std::size_t Polygon256::edgeCount() const noexcept
    {
        return edgePlanes.size();
    }

    bool Polygon256::isValid() const noexcept
    {
        // >3边
        const std::size_t n = edgePlanes.size();
        if (n < 3)
        {
            return false;
        }

        // 边平面不与支撑平面平行
        for (const Plane3i &edge : edgePlanes)
        {
            if (arePlaneNormalsParallel(plane, edge))
            {
                return false;
            }
        }

        std::vector<PlanePoint3i> vertices;
        vertices.reserve(n);

        // 按约定：v_i = (plane, edge_i, edge_{i-1})。
        for (std::size_t i = 0; i < n; ++i)
        {
            // 顶点有效
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const PlanePoint3i v(plane, edgePlanes[i], edgePlanes[prev]);
            if (!v.hasUniqueIntersection())
            {
                return false;
            }

            vertices.push_back(v);
        }

        // 每个顶点都应在多边形内部或边界（用于过滤错误顺序）
        for (const PlanePoint3i &v : vertices)
        {
            if (!containsOrOnBoundary(v))
            {
                return false;
            }
        }

        // 对每条边: 仅相邻两个顶点在该边上，其余顶点严格在同一侧 => 顺序正确 + 严格凸
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t next = (i + 1 == n) ? 0 : (i + 1);
            const Plane3i &edge = edgePlanes[i];

            if (vertices[i].classify(edge) != 0 || vertices[next].classify(edge) != 0)
            {
                return false;
            }

            int refSide = 0;
            for (std::size_t j = 0; j < n; ++j)
            {
                if (j == i || j == next)
                {
                    continue;
                }

                const int side = vertices[j].classify(edge);
                if (side == 0)
                {
                    return false; // 非相邻顶点落在同一边上，非严格凸/顺序异常
                }

                if (refSide == 0)
                {
                    refSide = side;
                }
                else if (side != refSide)
                {
                    return false; // 同一条边两侧都有顶点，非凸或顺序错误
                }
            }

            if (refSide != -1)
            {
                return false; // 法向约定要求非邻接顶点都在边平面的负侧，即边平面法向向外
            }
        }

        return true;
    }

    int Polygon256::classify(const PlanePoint3i &point) const noexcept
    {
        if (!point.hasUniqueIntersection())
        {
            return -2;
        }

        const int planeSide = point.classify(plane);
        if (planeSide != 0)
        {
            return planeSide;
        }

        // `isValid()` 已要求边平面法向向外；支撑平面内只要落在任一边平面正侧就是外部。
        for (const Plane3i &edge : edgePlanes)
        {
            if (point.classify(edge) > 0)
            {
                return 2;
            }
        }

        return 0;
    }

    bool Polygon256::containsStrictly(const PlanePoint3i &point) const noexcept
    {
        if (!point.hasUniqueIntersection() || point.classify(plane) != 0)
        {
            return false;
        }

        for (const Plane3i &edge : edgePlanes)
        {
            if (point.classify(edge) >= 0)
            {
                return false;
            }
        }

        return true;
    }

    bool Polygon256::containsOrOnBoundary(const PlanePoint3i &point) const noexcept
    {
        return point.hasUniqueIntersection() && classify(point) == 0;
    }

    // 向内平移两条边构造内部点
    bool Polygon256::findStrictInteriorPoint(PlanePoint3i &outPoint) const noexcept
    {
        const std::size_t n = edgePlanes.size();
        if (n < 3)
        {
            return false;
        }

        std::vector<PlanePoint3i> vertices;
        vertices.reserve(n);
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const PlanePoint3i v(plane, edgePlanes[i], edgePlanes[prev]);
            if (!v.hasUniqueIntersection())
            {
                return false;
            }
            vertices.push_back(v);
        }

        auto findReferenceVertexForEdge = [&](std::size_t edgeIdx) -> std::size_t
        {
            const std::size_t next = (edgeIdx + 1 == n) ? 0 : (edgeIdx + 1);
            for (std::size_t k = 0; k < n; ++k)
            {
                if (k != edgeIdx && k != next)
                {
                    return k;
                }
            }
            return n;
        };

        auto buildInsetEdge = [&](const Plane3i &edge,
                                  int interiorSide,
                                  const PlanePoint3i &boundaryVertex,
                                  const PlanePoint3i &refVertex,
                                  Plane3i &outInset) -> bool
        {
            if (interiorSide == 0)
            {
                return false;
            }

            Integer scale = 1;
            for (int iter = 0; iter < 40; ++iter)
            {
                if (!canScalePlaneWithinInsetHeadroom(edge, scale))
                {
                    return false;
                }

                const Plane3i scaledEdge(edge.a * scale,
                                         edge.b * scale,
                                         edge.c * scale,
                                         edge.d * scale);

                Plane3i inset = scaledEdge;
                inset.d -= interiorSide;

                const int refSide = refVertex.classify(inset);
                const int boundarySide = boundaryVertex.classify(inset);
                if (refSide == interiorSide && boundarySide != 0 && boundarySide != interiorSide)
                {
                    outInset = inset;
                    return true;
                }

                scale += scale;
            }

            return false;
        };

        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);

            const std::size_t refIdxA = findReferenceVertexForEdge(i);
            const std::size_t refIdxB = findReferenceVertexForEdge(prev);
            if (refIdxA == n || refIdxB == n)
            {
                continue;
            }

            constexpr int interiorSideA = -1;
            constexpr int interiorSideB = -1;

            Plane3i insetA;
            Plane3i insetB;
            if (!buildInsetEdge(edgePlanes[i], interiorSideA, vertices[i], vertices[refIdxA], insetA))
            {
                continue;
            }
            if (!buildInsetEdge(edgePlanes[prev], interiorSideB, vertices[i], vertices[refIdxB], insetB))
            {
                continue;
            }

            if (!hasUniqueIntersection(plane, insetA, insetB))
            {
                continue;
            }

            const PlanePoint3i candidate(plane, insetA, insetB);
            if (!containsStrictly(candidate))
            {
                continue;
            }

            outPoint = candidate;
            return true;
        }

        return false;
    }

    bool areParallel(const Line256 &lhs, const Line256 &rhs) noexcept
    {
        const Vec3i crossVec = cross(lhs.directionVector(), rhs.directionVector());
        return isZero(crossVec.x) && isZero(crossVec.y) && isZero(crossVec.z);
    }

    bool intersectionSegmentPolygon(const Segment256 &seg, const Polygon256 &poly, PlanePoint3i &outPoint)
    {
        outPoint = intersect(seg.direction, poly.plane);

        if (outPoint.hasUniqueIntersection() == false || poly.containsOrOnBoundary(outPoint) == false)
            return false;

        // 点在线段内（含端点）
        if (outPoint.classify(seg.start) != 1 && outPoint.classify(seg.end) != 1)
            return true;

        return false;
    }
}

