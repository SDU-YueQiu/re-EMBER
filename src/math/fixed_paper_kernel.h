/**
 * @file fixed_paper_kernel.h
 * @brief 基于自定义定长整数后端的论文几何 primitive 草案。
 */
#pragma once

#include "math/fixed_int256.h"

#include <cstdint>

namespace ember::fixed
{
struct Plane3
{
    FixedInt256 a;
    FixedInt256 b;
    FixedInt256 c;
    FixedInt256 d;
};

struct HomPoint4
{
    FixedInt256 x;
    FixedInt256 y;
    FixedInt256 z;
    FixedInt256 w;
};

inline int signum(const FixedInt256 &value) noexcept
{
    if (value.isZero())
        return 0;
    return value.isNegative() ? -1 : 1;
}

inline Plane3 planeFromInt64(std::int64_t a, std::int64_t b, std::int64_t c, std::int64_t d) noexcept
{
    return Plane3{FixedInt256(a), FixedInt256(b), FixedInt256(c), FixedInt256(d)};
}

inline bool intersect3PlanesChecked(
    const Plane3 &p,
    const Plane3 &q,
    const Plane3 &r,
    HomPoint4 &out) noexcept
{
    FixedInt256 detBcd;
    FixedInt256 detAcd;
    FixedInt256 detAbd;
    FixedInt256 detAbc;

    if (!determinant3x3Checked(
            p.b, p.c, p.d,
            q.b, q.c, q.d,
            r.b, r.c, r.d,
            detBcd))
        return false;
    if (!determinant3x3Checked(
            p.a, p.c, p.d,
            q.a, q.c, q.d,
            r.a, r.c, r.d,
            detAcd))
        return false;
    if (!determinant3x3Checked(
            p.a, p.b, p.d,
            q.a, q.b, q.d,
            r.a, r.b, r.d,
            detAbd))
        return false;
    if (!determinant3x3Checked(
            p.a, p.b, p.c,
            q.a, q.b, q.c,
            r.a, r.b, r.c,
            detAbc))
        return false;

    if (!negateChecked(detBcd, out.x))
        return false;
    out.y = detAcd;
    if (!negateChecked(detAbd, out.z))
        return false;
    out.w = detAbc;
    return true;
}

inline bool classifyVertexChecked(
    const HomPoint4 &point,
    const Plane3 &plane,
    int &outSign) noexcept
{
    FixedInt256 dot;
    if (!dot4Checked(
            point.x,
            point.y,
            point.z,
            point.w,
            plane.a,
            plane.b,
            plane.c,
            plane.d,
            dot))
        return false;

    outSign = signum(dot) * signum(point.w);
    return true;
}
}
