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

    a /= divisor;
    b /= divisor;
    c /= divisor;
    d /= divisor;
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
    const auto normalCross = cross(p.normal(), q.normal());
    return isZero(normalCross.x) && isZero(normalCross.y) && isZero(normalCross.z);
}

inline Integer normalDeterminant(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return determinant(p.normal(), q.normal(), r.normal());
}

inline bool hasUniqueIntersection(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return !isZero(normalDeterminant(p, q, r));
}

// 该函数不做退化检查，调用前应使用 `hasUniqueIntersection()` 确认三平面有唯一交点。
inline HomPoint4i intersectHomogeneous(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    const Integer x = determinant3x3(-p.d, p.b, p.c, -q.d, q.b, q.c, -r.d, r.b, r.c);
    const Integer y = determinant3x3(p.a, -p.d, p.c, q.a, -q.d, q.c, r.a, -r.d, r.c);
    const Integer z = determinant3x3(p.a, p.b, -p.d, q.a, q.b, -q.d, r.a, r.b, -r.d);
    const Integer w = determinant3x3(p.a, p.b, p.c, q.a, q.b, q.c, r.a, r.b, r.c);
    return primitiveHomPoint(HomPoint4i(x, y, z, w));
}

inline Integer HomPoint4i::dotPlane(const Plane3i &s) const noexcept
{
    return x * s.a + y * s.b + z * s.c + w * s.d;
}

inline int HomPoint4i::classify(const Plane3i &s) const noexcept
{
    return signum(dotPlane(s)) * signum(w);
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
        : p(pVal), q(qVal), r(rVal), x(intersectHomogeneous(pVal, qVal, rVal))
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

