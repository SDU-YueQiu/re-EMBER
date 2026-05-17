/**
 * @file math256.h
 * @brief 定义固定宽度整数向量、齐次点和行列式数学工具。
 */
#pragma once

#include "core/perf_tracing.h"

#include <boost/multiprecision/cpp_int.hpp>

#include <ostream>
#include <string>

namespace ember
{
using Integer = boost::multiprecision::int256_t;
using WideInteger = boost::multiprecision::int512_t;
struct Plane3i;

inline std::string integerToString(const Integer& value)
{
    return value.convert_to<std::string>();
}

inline long double integerToLongDouble(const Integer& value)
{
    return value.convert_to<long double>();
}

inline int signum(const Integer& value) noexcept
{
    return (value > 0) - (value < 0);
}

inline bool isZero(const Integer& value) noexcept
{
    return value == 0;
}

/**
 * @brief 返回整数绝对值，用于几何系数的幅值归一化。
 */
inline Integer absMagnitude(Integer value) noexcept
{
    return value < 0 ? -value : value;
}

/**
 * @brief 判断整数幅值是否为 1。
 */
inline bool hasUnitMagnitude(const Integer& value) noexcept
{
    return value == 1 || value == -1;
}

/**
 * @brief 计算两个整数幅值的最大公约数。
 */
inline Integer gcdMagnitude(Integer lhs, Integer rhs) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::gcdMagnitude");
    if (hasUnitMagnitude(lhs) || hasUnitMagnitude(rhs))
        return 1;

    lhs = absMagnitude(lhs);
    rhs = absMagnitude(rhs);
    if (isZero(lhs))
        return rhs;
    if (isZero(rhs) || lhs == rhs)
        return lhs;

    Integer result = 0;
    boost::multiprecision::backends::eval_gcd(result.backend(), lhs.backend(), rhs.backend());
    return result;
}

/**
 * @brief 计算四个整数幅值的最大公约数。
 */
inline Integer gcdMagnitude(const Integer& a, const Integer& b, const Integer& c, const Integer& d) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::gcdMagnitude4");
    if (hasUnitMagnitude(a) || hasUnitMagnitude(b) || hasUnitMagnitude(c) || hasUnitMagnitude(d))
        return 1;

    return gcdMagnitude(gcdMagnitude(a, b), gcdMagnitude(c, d));
}

inline Integer floorDiv(const Integer& a, const Integer& b) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::floorDiv");
    Integer num = a;
    Integer den = b;
    if (den < 0)
    {
        num = -num;
        den = -den;
    }

    if (num >= 0)
        return num / den;

    return (num - den + 1) / den;
}

inline Integer ceilDiv(const Integer& a, const Integer& b) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::ceilDiv");
    Integer num = a;
    Integer den = b;
    if (den < 0)
    {
        num = -num;
        den = -den;
    }

    if (num >= 0)
        return (num + den - 1) / den;

    return num / den;
}

/**
 * @brief 表示三维齐次点坐标。
 */
struct HomPoint4i
{
    Integer x = 0;
    Integer y = 0;
    Integer z = 0;
    Integer w = 0;

    HomPoint4i() noexcept = default;
    HomPoint4i(const Integer& xVal, const Integer& yVal, const Integer& zVal, const Integer& wVal) noexcept
        : x(xVal), y(yVal), z(zVal), w(wVal)
    {
    }

    Integer dotPlane(const Plane3i& s) const noexcept;

    int classify(const Plane3i& s) const noexcept;

    // 该函数比较齐次坐标四个分量是否完全一致。
    bool hasSameComponents(const HomPoint4i& rhs) const noexcept
    {
        return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
    }
};

/**
 * @brief 返回与输入几何等价的 primitive 齐次点。
 */
inline HomPoint4i primitiveHomPoint(const HomPoint4i& point) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::primitiveHomPoint");
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

inline bool isZeroHomPoint(const HomPoint4i& point) noexcept
{
    return isZero(point.x) && isZero(point.y) && isZero(point.z) && isZero(point.w);
}

inline WideInteger wideProduct(const Integer& lhs, const Integer& rhs) noexcept
{
    return WideInteger(lhs) * WideInteger(rhs);
}

/**
 * @brief 用 512 位中间乘积比较两个齐次点是否比例等价。
 *
 * @note 坐标分量为 int256_t，两个分量相乘需要最多 510 位；不能用 int256_t 交叉相乘。
 */
inline bool areSameHomPoint(const HomPoint4i& lhs, const HomPoint4i& rhs) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::areSameHomPoint");
    if (lhs.hasSameComponents(rhs))
        return true;

    const bool lhsZero = isZeroHomPoint(lhs);
    const bool rhsZero = isZeroHomPoint(rhs);
    if (lhsZero || rhsZero)
        return lhsZero && rhsZero;

    return wideProduct(lhs.x, rhs.y) == wideProduct(rhs.x, lhs.y) &&
           wideProduct(lhs.x, rhs.z) == wideProduct(rhs.x, lhs.z) &&
           wideProduct(lhs.x, rhs.w) == wideProduct(rhs.x, lhs.w) &&
           wideProduct(lhs.y, rhs.z) == wideProduct(rhs.y, lhs.z) &&
           wideProduct(lhs.y, rhs.w) == wideProduct(rhs.y, lhs.w) &&
           wideProduct(lhs.z, rhs.w) == wideProduct(rhs.z, lhs.w);
}

struct Vec3i
{
    Integer x = 0;
    Integer y = 0;
    Integer z = 0;

    Vec3i() noexcept = default;
    Vec3i(const Integer& xVal, const Integer& yVal, const Integer& zVal) noexcept : x(xVal), y(yVal), z(zVal) {}

    Vec3i& operator+=(const Vec3i& rhs) noexcept
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    Vec3i& operator-=(const Vec3i& rhs) noexcept
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    Vec3i& operator*=(const Integer& k) noexcept
    {
        x *= k;
        y *= k;
        z *= k;
        return *this;
    }

    Vec3i& operator/=(const Integer& k) noexcept
    {
        x /= k;
        y /= k;
        z /= k;
        return *this;
    }

    Vec3i operator+() const noexcept {
        return *this;
    }
    Vec3i operator-() const noexcept {
        return Vec3i(-x, -y, -z);
    }

    bool operator==(const Vec3i& rhs) const noexcept {
        return x == rhs.x && y == rhs.y && z == rhs.z;
    }
    bool operator!=(const Vec3i& rhs) const noexcept {
        return !(*this == rhs);
    }

};

inline Vec3i operator+(Vec3i lhs, const Vec3i& rhs) noexcept
{
    lhs += rhs;
    return lhs;
}

inline Vec3i operator-(Vec3i lhs, const Vec3i& rhs) noexcept
{
    lhs -= rhs;
    return lhs;
}

inline Vec3i operator*(Vec3i v, const Integer& k) noexcept
{
    v *= k;
    return v;
}

inline Vec3i operator*(const Integer& k, Vec3i v) noexcept
{
    v *= k;
    return v;
}

inline Vec3i operator/(Vec3i v, const Integer& k) noexcept
{
    v /= k;
    return v;
}

inline Integer dot(const Vec3i& a, const Vec3i& b) noexcept
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline Vec3i cross(const Vec3i& a, const Vec3i& b) noexcept
{
    return Vec3i(
               a.y * b.z - a.z * b.y,
               a.z * b.x - a.x * b.z,
               a.x * b.y - a.y * b.x);
}

inline Integer determinant3x3(
    const Integer& a11,
    const Integer& a12,
    const Integer& a13,
    const Integer& a21,
    const Integer& a22,
    const Integer& a23,
    const Integer& a31,
    const Integer& a32,
    const Integer& a33) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::determinant3x3");
    const bool zero11 = isZero(a11);
    const bool zero12 = isZero(a12);
    const bool zero13 = isZero(a13);
    if (zero11 || zero12 || zero13)
    {
        Integer value = 0;
        if (!zero11)
            value += a11 * (a22 * a33 - a23 * a32);
        if (!zero12)
            value -= a12 * (a21 * a33 - a23 * a31);
        if (!zero13)
            value += a13 * (a21 * a32 - a22 * a31);
        return value;
    }

    return a11 * (a22 * a33 - a23 * a32)
           - a12 * (a21 * a33 - a23 * a31)
           + a13 * (a21 * a32 - a22 * a31);
}

inline Integer determinant(const Vec3i& row1, const Vec3i& row2, const Vec3i& row3) noexcept
{
    return determinant3x3(
               row1.x,
               row1.y,
               row1.z,
               row2.x,
               row2.y,
               row2.z,
               row3.x,
               row3.y,
               row3.z);
}

inline std::ostream& operator<<(std::ostream& os, const Vec3i& v)
{
    return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
}


}
