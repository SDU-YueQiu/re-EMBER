/**
 * @file int256_checked.h
 * @brief 提供 `Integer` 固定宽度运算的窄范围检查辅助函数。
 */
#pragma once

#include "geometry/plane_geometry256.h"

#include <limits>

namespace ember
{
namespace detail
{
inline const Integer &integerMaxMagnitude() noexcept
{
    static const Integer value = ((Integer(1) << 254) - 1) * 2 + 1;
    return value;
}

inline const Integer &integerMinValue() noexcept
{
    static const Integer value = -integerMaxMagnitude() - 1;
    return value;
}

inline bool canMultiplyIntegerWithinRange(const Integer &lhs, const Integer &rhs) noexcept
{
    if (isZero(lhs) || isZero(rhs))
        return true;

    return absMagnitude(lhs) <= divInteger(integerMaxMagnitude(), absMagnitude(rhs));
}

inline bool tryMultiplyInteger(const Integer &lhs, const Integer &rhs, Integer &out) noexcept
{
    if (!canMultiplyIntegerWithinRange(lhs, rhs))
        return false;

    out = lhs * rhs;
    return true;
}

inline bool tryAddInteger(const Integer &lhs, const Integer &rhs, Integer &out) noexcept
{
    if (rhs > 0 && lhs > integerMaxMagnitude() - rhs)
        return false;
    if (rhs < 0 && lhs < integerMinValue() - rhs)
        return false;

    out = lhs + rhs;
    return true;
}

inline bool tryScalePlaneByPositiveInteger(
    const Plane3i &plane,
    const Integer &scale,
    Plane3i &outPlane) noexcept
{
    if (scale <= 0)
        return false;

    Integer a;
    Integer b;
    Integer c;
    Integer d;
    if (!tryMultiplyInteger(plane.a, scale, a) ||
            !tryMultiplyInteger(plane.b, scale, b) ||
            !tryMultiplyInteger(plane.c, scale, c) ||
            !tryMultiplyInteger(plane.d, scale, d))
        return false;

    outPlane = Plane3i(a, b, c, d);
    return true;
}

inline bool tryOffsetPlaneD(const Plane3i &plane, const Integer &delta, Plane3i &outPlane) noexcept
{
    Integer d;
    if (!tryAddInteger(plane.d, delta, d))
        return false;

    outPlane = Plane3i(plane.a, plane.b, plane.c, d);
    return true;
}
}
}
