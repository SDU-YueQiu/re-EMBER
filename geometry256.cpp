#include "geometry256.h"

#include <utility>

namespace ember
{
    Polygon256::Polygon256(const Plane3i& supportPlane, std::vector<Plane3i> edges)
        : plane(supportPlane), edgePlanes(std::move(edges))
    {
    }

    void Polygon256::addEdgePlane(const Plane3i& edge)
    {
        edgePlanes.push_back(edge);
    }

    std::size_t Polygon256::edgeCount() const noexcept
    {
        return edgePlanes.size();
    }

    bool Polygon256::isValid() const noexcept
    {
        const std::size_t n = edgePlanes.size();
        if (n < 3)
        {
            return false;
        }

        for (const Plane3i& edge : edgePlanes)
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
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const PlanePoint3i v(plane, edgePlanes[i], edgePlanes[prev]);
            if (!v.hasUniqueIntersection())
            {
                return false;
            }

            // 顶点必须落在构造它的两条相邻边上
            if (v.classify(edgePlanes[i]) != 0 || v.classify(edgePlanes[prev]) != 0)
            {
                return false;
            }

            vertices.push_back(v);
        }

        // 每个顶点都应在多边形内部或边界（用于过滤错误顺序）
        for (const PlanePoint3i& v : vertices)
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
            const Plane3i& edge = edgePlanes[i];

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

    int Polygon256::classify(const PlanePoint3i& point) const noexcept
    {
        if (!point.hasUniqueIntersection())
        {
            return 0;
        }

        if (point.classify(plane) != 0)
        {
            return 0;
        }

        bool hasPositive = false;
        bool hasNegative = false;
        for (const Plane3i& edge : edgePlanes)
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
        return 0;
    }

    bool Polygon256::containsStrictly(const PlanePoint3i& point) const noexcept
    {
        if (!point.hasUniqueIntersection() || point.classify(plane) != 0)
        {
            return false;
        }

        int sideRef = 0;
        for (const Plane3i& edge : edgePlanes)
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

    bool Polygon256::containsOrOnBoundary(const PlanePoint3i& point) const noexcept
    {
        return classify(point) != 0;
    }

    bool Polygon256::findStrictInteriorPoint(PlanePoint3i& outPoint) const noexcept
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

        auto buildInsetEdge = [&](const Plane3i& edge,
                                  int interiorSide,
                                  const PlanePoint3i& boundaryVertex,
                                  const PlanePoint3i& refVertex,
                                  Plane3i& outInset) -> bool
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

    bool areParallel(const Line256& lhs, const Line256& rhs) noexcept
    {
        const Vec3i crossVec = cross(lhs.directionVector(), rhs.directionVector());
        return isZero(crossVec.x) && isZero(crossVec.y) && isZero(crossVec.z);
    }
}
