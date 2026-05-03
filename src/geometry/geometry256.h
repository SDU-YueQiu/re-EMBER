#pragma once

#include "math/math256.h"
#include "geometry/plane_geometry256.h"
#include "math/winding_number_f.h"

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
        Segment256(const Plane3i &startPlane, const Plane3i &endPlane, const Line256 &directionLine) noexcept;

        PlanePoint3i getStartPoint() const noexcept
        {
            return PlanePoint3i(direction.p1, direction.p2, start);
        }

        PlanePoint3i getEndPoint() const noexcept
        {
            return PlanePoint3i(direction.p1, direction.p2, end);
        }

        constexpr bool isValid() const noexcept
        {
            PlanePoint3i s(direction.p1, direction.p2, start);
            PlanePoint3i e(direction.p1, direction.p2, end);
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

        Polygon256() = default;
        Polygon256(const Plane3i &supportPlane, std::vector<Plane3i> edges);

        // 该方法对法向不封闭
        void addEdgePlane(const Plane3i &edge);
        std::size_t edgeCount() const noexcept;

        bool isValid() const noexcept;

        // 临时返回约定：
        // -2: 输入点无唯一交点，属于数据错误
        // -1/+1: 点不在多边形内，且分别位于支撑平面负侧/正侧
        //  0: 点在多边形内或边界上
        //  2: 点在支撑平面上，但不在多边形内
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
    inline PlanePoint3i intersect(const Line256 &line, const Plane3i &plane) noexcept
    {
        return PlanePoint3i(line.p1, line.p2, plane);
    }

    bool areParallel(const Line256 &lhs, const Line256 &rhs) noexcept;

    /**
     * @brief 求线段和多边形的交点
     * 
     * 并不区分严格内部，交在边上也行，但只能有一个交点，有一段重叠的返回false
     * 
     * @param seg 输入线段
     * @param poly 输入多边形
     * @param[out] outPoint 输出交点坐标（如果有交点）
     * @return 如果有交点返回true，否则返回false
     * 
     * @todo 添加交在不同地方的不同返回状态
     */
    bool intersectionSegmentPolygon(const Segment256 &seg, const Polygon256 &poly, PlanePoint3i &outPoint);
}

