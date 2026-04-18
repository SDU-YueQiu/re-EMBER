#pragma once

#include "math/math256.h"
#include "math/plane_geometry256.h"
#include "math/winding_number.h"

#include <cstddef>
#include <vector>

namespace ember
{
    struct Line256
    {
        Plane3i p1, p2;

        constexpr Line256() noexcept = default;
        constexpr Line256(const Plane3i &first, const Plane3i &second) noexcept : p1(first), p2(second) {}

        constexpr Vec3i directionVector() const noexcept
        {
            return cross(p1.normal(), p2.normal());
        }

        constexpr bool isValid() const noexcept
        {
            const Vec3i d = directionVector();
            return !isZero(d.x) || !isZero(d.y) || !isZero(d.z);
        }

        constexpr bool contains(const HomPoint4i &x) const noexcept
        {
            return x.classify(p1) == 0 && x.classify(p2) == 0;
        }

        constexpr bool contains(const PlanePoint3i &x) const noexcept
        {
            return contains(x.x);
        }
    };

    // 法向要求：线段内为负，也即start和end平面法向相反向外
    struct Segment256
    {
        Plane3i start;
        Plane3i end;

        Line256 direction;

        Segment256() noexcept = default;
        Segment256(const Plane3i &startPlane, const Plane3i &endPlane) noexcept;
        Segment256(const Plane3i &startPlane, const Plane3i &endPlane, const Line256 &directionLine) noexcept;

        constexpr bool isValid() const noexcept
        {
            PlanePoint3i s = intersect(direction, start);
            PlanePoint3i e = intersect(direction, end);
            if (!s.hasUniqueIntersection() || !e.hasUniqueIntersection())
                return false;

            //注意此处的处理是把法向错误和s e共面一起判断
            if ((s.classify(end) != -1) || (e.classify(start) != -1))
                return false;

            return true;
        }
    };

    struct Polygon256
    {
        Plane3i plane; // 支撑平面

        // 裁剪要求边平面法向必须指向多边形外侧
        std::vector<Plane3i> edgePlanes;

        // 环绕数转换向量
        std::vector<int> WNTV;

        // 结果面状态，用于布尔指示函数，注意以数组下标确定网格
        WNV WNVF, WNVB;

        Polygon256() = default;
        Polygon256(const Plane3i &supportPlane, std::vector<Plane3i> edges);

        void addEdgePlane(const Plane3i &edge);
        std::size_t edgeCount() const noexcept;
        bool isValid() const noexcept;

        // 返回值0在外，在内时根据边平面法向指向不同返回值+-1（尽管边平面指向有约束向外，但该函数并不强制负号）
        int classify(const PlanePoint3i &point) const noexcept;

        bool containsStrictly(const PlanePoint3i &point) const noexcept;
        bool containsOrOnBoundary(const PlanePoint3i &point) const noexcept;
        bool findStrictInteriorPoint(PlanePoint3i &outPoint) const noexcept;
    };

    inline constexpr Line256 makeLine(const Plane3i &p1, const Plane3i &p2) noexcept
    {
        return Line256(p1, p2);
    }

    // 返回直线与平面的交点
    inline constexpr PlanePoint3i intersect(const Line256 &line, const Plane3i &plane) noexcept
    {
        return PlanePoint3i(line.p1, line.p2, plane);
    }

    bool areParallel(const Line256 &lhs, const Line256 &rhs) noexcept;

    // 求线段和多边形的交点，如果有交点返回true和outPoint
    bool intersectionSegmentPolygon(Segment256 &seg, Polygon256 &poly, PlanePoint3i &outPoint);
}
