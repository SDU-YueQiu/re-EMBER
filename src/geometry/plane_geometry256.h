#pragma once

#include "math/math256.h"

namespace ember
{
    struct Plane3i
    {
        Integer a = 0;
        Integer b = 0;
        Integer c = 0;
        Integer d = 0;

        constexpr Plane3i() noexcept = default;
        constexpr Plane3i(const Integer &aVal, const Integer &bVal, const Integer &cVal, const Integer &dVal) noexcept
            : a(aVal), b(bVal), c(cVal), d(dVal)
        {
        }

        constexpr Vec3i normal() const noexcept
        {
            return Vec3i(a, b, c);
        }

        static constexpr Plane3i fromPointNormal(const Vec3i &point, const Vec3i &normal) noexcept
        {
            return Plane3i(normal.x, normal.y, normal.z, -dot(point, normal));
        }
    };

    inline std::ostream &operator<<(std::ostream &os, const Plane3i &p)
    {
        return os << "Plane3i(a=" << p.a
                  << ", b=" << p.b
                  << ", c=" << p.c
                  << ", d=" << p.d << ")";
    }

    struct HomPoint4i
    {
        Integer x = 0;
        Integer y = 0;
        Integer z = 0;
        Integer w = 0;

        constexpr HomPoint4i() noexcept = default;
        constexpr HomPoint4i(const Integer &xVal, const Integer &yVal, const Integer &zVal, const Integer &wVal) noexcept
            : x(xVal), y(yVal), z(zVal), w(wVal)
        {
        }

        constexpr Integer dotPlane(const Plane3i &s) const noexcept
        {
            return x * s.a + y * s.b + z * s.c + w * s.d;
        }

        constexpr int classify(const Plane3i &s) const noexcept
        {
            return signum(dotPlane(s)) * signum(w);
        }

        //该函数为精确相等
        constexpr bool hasSameComponents(const HomPoint4i &rhs) const noexcept
        {
            return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
        }
    };

    inline constexpr bool areSameHomPoint(const HomPoint4i &lhs, const HomPoint4i &rhs) noexcept
    {
        if (lhs.hasSameComponents(rhs))
        {
            return true;
        }
        if (!isZero(lhs.w) && !isZero(rhs.w))
        {
            return lhs.x * rhs.w == rhs.x * lhs.w && lhs.y * rhs.w == rhs.y * lhs.w && lhs.z * rhs.w == rhs.z * lhs.w;
        }

        if (!isZero(lhs.z) && !isZero(rhs.z))
        {
            return lhs.x * rhs.z == rhs.x * lhs.z && lhs.y * rhs.z == rhs.y * lhs.z && lhs.w * rhs.z == rhs.w * lhs.z;
        }

        if (!isZero(lhs.y) && !isZero(rhs.y))
        {
            return lhs.x * rhs.y == rhs.x * lhs.y && lhs.z * rhs.y == rhs.z * lhs.y && lhs.w * rhs.y == rhs.w * lhs.y;
        }

        if (!isZero(lhs.x) && !isZero(rhs.x))
        {
            return lhs.y * rhs.x == rhs.y * lhs.x && lhs.z * rhs.x == rhs.z * lhs.x && lhs.w * rhs.x == rhs.w * lhs.x;
        }

        return false;
    }

    inline constexpr Integer determinant4x4(
        const Plane3i &row1,
        const Plane3i &row2,
        const Plane3i &row3,
        const Plane3i &row4) noexcept
    {
        return determinant4x4(row1.a, row1.b, row1.c, row1.d,
                              row2.a, row2.b, row2.c, row2.d,
                              row3.a, row3.b, row3.c, row3.d,
                              row4.a, row4.b, row4.c, row4.d);
    }

    inline constexpr bool arePlaneNormalsParallel(const Plane3i &p, const Plane3i &q) noexcept
    {
        const auto normalCross = cross(p.normal(), q.normal());
        return isZero(normalCross.x) && isZero(normalCross.y) && isZero(normalCross.z);
    }

    inline constexpr Integer normalDeterminant(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
    {
        return determinant(p.normal(), q.normal(), r.normal());
    }

    inline constexpr bool hasUniqueIntersection(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
    {
        return !isZero(normalDeterminant(p, q, r));
    }

    //该函数没有进行退化检查，注意使用hasUniqueIntersection确保不退化
    inline constexpr HomPoint4i intersectHomogeneous(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
    {
        const Integer x = determinant3x3(-p.d, p.b, p.c, -q.d, q.b, q.c, -r.d, r.b, r.c);
        const Integer y = determinant3x3(p.a, -p.d, p.c, q.a, -q.d, q.c, r.a, -r.d, r.c);
        const Integer z = determinant3x3(p.a, p.b, -p.d, q.a, q.b, -q.d, r.a, r.b, -r.d);
        const Integer w = determinant3x3(p.a, p.b, p.c, q.a, q.b, q.c, r.a, r.b, r.c);
        return HomPoint4i(x, y, z, w);
    }

    inline constexpr int classifyPointAgainstPlane(const HomPoint4i &x, const Plane3i &s) noexcept
    {
        return x.classify(s);
    }

    inline constexpr int classifyByDeterminants(
        const Plane3i &p,
        const Plane3i &q,
        const Plane3i &r,
        const Plane3i &s) noexcept
    {
        return signum(determinant4x4(p, q, r, s)) * signum(normalDeterminant(p, q, r));
    }

    struct PlanePoint3i
    {
        Plane3i p;
        Plane3i q;
        Plane3i r;
        HomPoint4i x;

        constexpr PlanePoint3i(const Plane3i &pVal, const Plane3i &qVal, const Plane3i &rVal) noexcept
            : p(pVal), q(qVal), r(rVal), x(intersectHomogeneous(pVal, qVal, rVal))
        {
        }

        constexpr bool hasUniqueIntersection() const noexcept
        {
            return ember::hasUniqueIntersection(p, q, r);
        }

        constexpr int classify(const Plane3i &s) const noexcept
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
