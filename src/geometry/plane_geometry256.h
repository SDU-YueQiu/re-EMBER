/**
 * @file plane_geometry256.h
 * @brief 定义精确整数平面和基于平面的交点 primitive。
 */
#pragma once

#include "math/math256.h"

namespace ember
{
/**
 * @brief 原地约分平面方程系数，保留同一个有向半空间。
 */
inline void reducePlaneCoefficients(Integer &a, Integer &b, Integer &c, Integer &d) noexcept
{
    const Integer divisor = gcdMagnitude(a, b, c, d);
    if (divisor <= 1)
        return;

    a = detail::divideExactByPositive(a, divisor);
    b = detail::divideExactByPositive(b, divisor);
    c = detail::divideExactByPositive(c, divisor);
    d = detail::divideExactByPositive(d, divisor);
}

struct Plane3i
{
    Integer a = 0;
    Integer b = 0;
    Integer c = 0;
    Integer d = 0;

    Plane3i() noexcept = default;
    Plane3i(const Integer &aVal, const Integer &bVal, const Integer &cVal, const Integer &dVal) noexcept
        : a(aVal), b(bVal), c(cVal), d(dVal)
    {
    }

    Vec3i normal() const noexcept
    {
        return Vec3i(a, b, c);
    }

    static Plane3i fromPointNormal(const Vec3i &point, const Vec3i &normal) noexcept
    {
        Integer a = normal.x;
        Integer b = normal.y;
        Integer c = normal.z;
        Integer d = -dot(point, normal);
        reducePlaneCoefficients(a, b, c, d);
        return Plane3i(a, b, c, d);
    }
};

/**
 * @brief 返回与输入几何等价的 primitive 平面方程。
 */
inline Plane3i primitivePlane(const Plane3i &plane) noexcept
{
    Integer a = plane.a;
    Integer b = plane.b;
    Integer c = plane.c;
    Integer d = plane.d;
    reducePlaneCoefficients(a, b, c, d);
    return Plane3i(a, b, c, d);
}

inline std::ostream &operator<<(std::ostream &os, const Plane3i &p)
{
    return os << "Plane3i(a=" << p.a
           << ", b=" << p.b
           << ", c=" << p.c
           << ", d=" << p.d << ")";
}

inline bool arePlaneNormalsParallel(const Plane3i &p, const Plane3i &q) noexcept
{
    if (p.a == q.a && p.b == q.b && p.c == q.c)
        return true;

    const auto normalCross = cross(p.normal(), q.normal());
    return isZero(normalCross.x) && isZero(normalCross.y) && isZero(normalCross.z);
}

inline bool areSamePlaneEquation(const Plane3i &lhs, const Plane3i &rhs) noexcept
{
    return lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c && lhs.d == rhs.d;
}

inline Integer normalDeterminant(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return determinant(p.normal(), q.normal(), r.normal());
}

inline bool hasUniqueIntersection(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return !isZero(normalDeterminant(p, q, r));
}

namespace detail
{
inline bool tryExtractUnitCoordinatePlaneForPoint(
    const Plane3i &plane,
    int &outAxis,
    Integer &outCoordinate) noexcept
{
    if (hasUnitMagnitude(plane.a) && isZero(plane.b) && isZero(plane.c))
    {
        outAxis = 0;
        outCoordinate = -plane.d / plane.a;
        return true;
    }
    if (isZero(plane.a) && hasUnitMagnitude(plane.b) && isZero(plane.c))
    {
        outAxis = 1;
        outCoordinate = -plane.d / plane.b;
        return true;
    }
    if (isZero(plane.a) && isZero(plane.b) && hasUnitMagnitude(plane.c))
    {
        outAxis = 2;
        outCoordinate = -plane.d / plane.c;
        return true;
    }

    return false;
}

inline bool tryBuildUnitCoordinateHomPoint(
    const Plane3i &p,
    const Plane3i &q,
    const Plane3i &r,
    HomPoint4i &outPoint) noexcept
{
    bool hasCoordinate[3] = {};
    Integer coordinates[3] = {};
    const Plane3i planes[3] = {p, q, r};

    for (const Plane3i &plane : planes)
    {
        int axis = 0;
        Integer coordinate = 0;
        if (!tryExtractUnitCoordinatePlaneForPoint(plane, axis, coordinate) || hasCoordinate[axis])
            return false;

        hasCoordinate[axis] = true;
        coordinates[axis] = coordinate;
    }

    if (!hasCoordinate[0] || !hasCoordinate[1] || !hasCoordinate[2])
        return false;

    outPoint = HomPoint4i(coordinates[0], coordinates[1], coordinates[2], 1);
    return true;
}
}

// 该函数一次性展开同一组三平面的四个齐次坐标行列式，避免重复计算 2x2 minor。
inline HomPoint4i intersectHomogeneousUnnormalized(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    const Integer bc = q.b * r.c - q.c * r.b;
    const Integer ac = q.a * r.c - q.c * r.a;
    const Integer ab = q.a * r.b - q.b * r.a;
    const Integer dc = q.d * r.c - q.c * r.d;
    const Integer db = q.d * r.b - q.b * r.d;
    const Integer da = q.d * r.a - q.a * r.d;

    return HomPoint4i(
               -p.d * bc + p.b * dc - p.c * db,
               -p.a * dc + p.d * ac + p.c * da,
               p.a * db - p.b * da - p.d * ab,
               p.a * bc - p.b * ac + p.c * ab);
}

// 该函数不做退化检查，调用前应使用 `hasUniqueIntersection()` 确认三平面有唯一交点。
inline HomPoint4i intersectHomogeneous(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return normalizedHomPointSign(intersectHomogeneousUnnormalized(p, q, r));
}

inline DotInteger HomPoint4i::dotPlane(const Plane3i &s) const noexcept
{
    return DotInteger(x) * DotInteger(s.a) +
           DotInteger(y) * DotInteger(s.b) +
           DotInteger(z) * DotInteger(s.c) +
           DotInteger(w) * DotInteger(s.d);
}

inline int HomPoint4i::classify(const Plane3i &s) const noexcept
{
    const DotInteger dot = dotPlane(s);
    return ((dot > 0) - (dot < 0)) * signum(w);
}

inline int classifyPointAgainstPlane(const HomPoint4i &x, const Plane3i &s) noexcept
{
    return x.classify(s);
}

struct PlanePoint3i
{
    Plane3i p;
    Plane3i q;
    Plane3i r;
    HomPoint4i x;

    // 仓库内默认把 PlanePoint3i 视为几何值对象；p/q/r 与缓存交点 x 必须保持一致。
    PlanePoint3i(const Plane3i &pVal, const Plane3i &qVal, const Plane3i &rVal) noexcept
        : p(pVal), q(qVal), r(rVal)
    {
        if (!detail::tryBuildUnitCoordinateHomPoint(pVal, qVal, rVal, x))
            x = intersectHomogeneous(pVal, qVal, rVal);
    }

    /**
     * @brief 使用调用方已知的三平面交点构造点值。
     *
     * @note 该入口只用于内部 trusted 路径；调用方必须保证 `xVal`
     *       是 `pVal/qVal/rVal` 的等价齐次交点。
     */
    PlanePoint3i(const Plane3i &pVal, const Plane3i &qVal, const Plane3i &rVal, const HomPoint4i &xVal) noexcept
        : p(pVal), q(qVal), r(rVal), x(xVal)
    {

    }

    PlanePoint3i() noexcept = default;

    bool hasUniqueIntersection() const noexcept
    {
        return !isZero(x.w);
    }

    int classify(const Plane3i &s) const noexcept
    {
        return classifyPointAgainstPlane(x, s);
    }
};

inline std::ostream &operator<<(std::ostream &os, const PlanePoint3i &pt)
{
    return os << "PlanePoint3i(p=" << pt.p
           << ", q=" << pt.q
           << ", r=" << pt.r
           << ", x=(" << pt.x.x
           << ", " << pt.x.y
           << ", " << pt.x.z
           << ", " << pt.x.w
           << "))";
}
}

