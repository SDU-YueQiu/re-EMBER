#include "geometry256.h"

#include <utility>

namespace ember
{
    //返回使用平面方程反转后的平面
    Plane3i flippedPlane(const Plane3i &plane) noexcept
    {
        return Plane3i(-plane.a, -plane.b, -plane.c, -plane.d);
    }

    //保证多边形边平面法向向外
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

    //保证线段端点平面法向正确
    void orientSegmentBoundsOutward(Plane3i &startPlane, Plane3i &endPlane, const Line256 &directionLine) noexcept
    {
        if (!directionLine.isValid())
        {
            return;
        }

        //TODO：为点添加更多构造函数
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
        : plane(supportPlane), edgePlanes(std::move(edges))
    {
        orientPolygonEdgesOutward(plane, edgePlanes);
    }

    Segment256::Segment256(const Plane3i &startPlane, const Plane3i &endPlane, const Line256 &directionLine) noexcept
        : start(startPlane), end(endPlane), direction(directionLine)
    {
        orientSegmentBoundsOutward(start, end, direction);
    }

    // 该方法对法向不封闭
    void Polygon256::addEdgePlane(const Plane3i &edge)
    {
        edgePlanes.push_back(edge);
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

        // 按约定: v_i = (plane, edge_i, edge_{i-1})
        for (std::size_t i = 0; i < n; ++i)
        {
            // 顶点有效
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const PlanePoint3i v(plane, edgePlanes[i], edgePlanes[prev]);
            if (!v.hasUniqueIntersection())
            {
                return false;
            }

            // 顶点必须落在构造它的两条相邻边上
            // 数学上是不可能出现这种情况的，这里是防御性编程，先禁用掉
            // if (v.classify(edgePlanes[i]) != 0 || v.classify(edgePlanes[prev]) != 0)
            // {
            //     return false;
            // }

            vertices.push_back(v);
        }

        // 每个顶点都应在多边形内部或边界（用于过滤错误顺序）
        for (const PlanePoint3i &v : vertices)
        {
            if (classify(v) == 0)
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
        }

        return true;
    }

    int Polygon256::classify(const PlanePoint3i &point) const noexcept
    {
        if (!point.hasUniqueIntersection())
        {
            return 0;
        }

        if (point.classify(plane) != 0)
        {
            return 0;
        }

        // 要求边平面法向指向一致，但指向内部还是外部不要求
        bool hasPositive = false;
        bool hasNegative = false;
        for (const Plane3i &edge : edgePlanes)
        {
            const int side = point.classify(edge);
            if (side > 0)
            {
                hasPositive = true;
            }
            else if (side < 0)
            {
                hasNegative = true;
            }

            if (hasPositive && hasNegative)
            {
                return 0;
            }
        }

        if (hasPositive)
        {
            return 1;
        }
        if (hasNegative)
        {
            return -1;
        }
        // 多边形可能退化成一个点，此时先保证程序正常运行
        return 0;
    }

    bool Polygon256::containsStrictly(const PlanePoint3i &point) const noexcept
    {
        if (!point.hasUniqueIntersection() || point.classify(plane) != 0)
        {
            return false;
        }

        int sideRef = 0;
        for (const Plane3i &edge : edgePlanes)
        {
            const int side = point.classify(edge);
            if (side == 0)
            {
                return false;
            }

            if (sideRef == 0)
            {
                sideRef = side;
            }
            else if (side != sideRef)
            {
                return false;
            }
        }

        return sideRef != 0;
    }

    bool Polygon256::containsOrOnBoundary(const PlanePoint3i &point) const noexcept
    {
        return classify(point) != 0;
    }

    // 向内平移两条边构造内部点
    bool Polygon256::findStrictInteriorPoint(PlanePoint3i &outPoint) const noexcept
    {
        //TODO:论文中这里是求浮点重心然后近似，现在是向内平移顶点，
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

            //TODO:现在多边形边平面的约束是法向必须向外，这里现在是多余的
            const int interiorSideA = vertices[refIdxA].classify(edgePlanes[i]);
            const int interiorSideB = vertices[refIdxB].classify(edgePlanes[prev]);
            if (interiorSideA == 0 || interiorSideB == 0)
            {
                continue;
            }

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

    bool intersectionSegmentPolygon(Segment256 &seg, Polygon256 &poly, PlanePoint3i &outPoint)
    {
        outPoint = intersect(seg.direction, poly.plane);

        if (outPoint.hasUniqueIntersection() == false || poly.containsStrictly(outPoint) == false)
            return false;

        // 点在线段内
        if (outPoint.classify(seg.start) == -1 && outPoint.classify(seg.end) == -1)
            return true;
        return false;
    }
}