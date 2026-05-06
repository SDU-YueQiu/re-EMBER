/**
 * @file plane_geometry256.h
 * @brief 定义精确整数平面和齐次平面点 primitive。
 */
#pragma once

#include "math/math256.h"

namespace ember
{
    /**
     * @brief 返回整数绝对值，用于几何系数的幅值归一化。
     */
    inline Integer absMagnitude(Integer value) noexcept
    {
        return value < 0 ? -value : value;
    }

    /**
     * @brief 计算两个整数幅值的最大公约数。
     */
    inline Integer gcdMagnitude(Integer lhs, Integer rhs) noexcept
    {
        lhs = absMagnitude(lhs);
        rhs = absMagnitude(rhs);
        while (!isZero(rhs))
        {
            const Integer remainder = lhs % rhs;
            lhs = rhs;
            rhs = remainder;
        }
        return lhs;
    }

    /**
     * @brief 计算四个整数幅值的最大公约数。
     */
    inline Integer gcdMagnitude(const Integer &a, const Integer &b, const Integer &c, const Integer &d) noexcept
    {
        return gcdMagnitude(gcdMagnitude(a, b), gcdMagnitude(c, d));
    }

    /**
     * @brief 原地约分平面方程系数，保留同一个有向半空间。
     */
    inline void reducePlaneCoefficients(Integer &a, Integer &b, Integer &c, Integer &d) noexcept
    {
        const Integer divisor = gcdMagnitude(a, b, c, d);
        if (divisor <= 1)
        {
            return;
        }

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

    struct HomPoint4i
    {
        Integer x = 0;
        Integer y = 0;
        Integer z = 0;
        Integer w = 0;

        HomPoint4i() noexcept = default;
        HomPoint4i(const Integer &xVal, const Integer &yVal, const Integer &zVal, const Integer &wVal) noexcept
            : x(xVal), y(yVal), z(zVal), w(wVal)
        {
        }

        Integer dotPlane(const Plane3i &s) const noexcept
        {
            return x * s.a + y * s.b + z * s.c + w * s.d;
        }

        int classify(const Plane3i &s) const noexcept
        {
            return signum(dotPlane(s)) * signum(w);
        }

        // 该函数比较齐次坐标四个分量是否完全一致。
        bool hasSameComponents(const HomPoint4i &rhs) const noexcept
        {
            return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
        }
    };

    /**
     * @brief 返回与输入几何等价的 primitive 齐次点。
     */
    inline HomPoint4i primitiveHomPoint(const HomPoint4i &point) noexcept
    {
        Integer x = point.x;
        Integer y = point.y;
        Integer z = point.z;
        Integer w = point.w;
        const Integer divisor = gcdMagnitude(x, y, z, w);
        if (divisor > 1)
        {
            x /= divisor;
            y /= divisor;
            z /= divisor;
            w /= divisor;
        }
        if (w < 0 ||
            (isZero(w) && (z < 0 ||
                           (isZero(z) && (y < 0 ||
                                          (isZero(y) && x < 0))))))
        {
            x = -x;
            y = -y;
            z = -z;
            w = -w;
        }
        return HomPoint4i(x, y, z, w);
    }

    /**
     * @brief 使用当前固定宽度整数运算比较齐次点。
     *
     * @warning 这不是任意齐次点的通用安全相等谓词。
     * 交叉相乘可能超过 256 位预算；仅在调用方已知操作数来自有界构造时使用。
     */
    inline bool areSameHomPoint(const HomPoint4i &lhs, const HomPoint4i &rhs) noexcept
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

    inline Integer determinant4x4(
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

    inline int classifyPointAgainstPlane(const HomPoint4i &x, const Plane3i &s) noexcept
    {
        return x.classify(s);
    }

    inline int classifyByDeterminants(
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

        PlanePoint3i(const Plane3i &pVal, const Plane3i &qVal, const Plane3i &rVal) noexcept
            : p(pVal), q(qVal), r(rVal), x(intersectHomogeneous(pVal, qVal, rVal))
        {
        }

        PlanePoint3i() noexcept = default;

        bool hasUniqueIntersection() const noexcept
        {
            return ember::hasUniqueIntersection(p, q, r);
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

