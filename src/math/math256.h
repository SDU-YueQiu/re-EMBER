/**
 * @file math256.h
 * @brief 定义固定宽度整数向量、齐次点和行列式数学工具。
 */
#pragma once

#include "core/perf_tracing.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <ostream>
#include <string>

namespace ember
{
using Integer = signed _BitInt(256);
using UnsignedInteger = unsigned _BitInt(256);
using WideInteger = signed _BitInt(512);
using DotInteger = signed _BitInt(512);
struct Plane3i;

inline UnsignedInteger unsignedMagnitude(Integer value) noexcept
{
    if (value >= 0)
        return static_cast<UnsignedInteger>(value);
    return ~static_cast<UnsignedInteger>(value) + UnsignedInteger(1);
}

inline std::string integerToString(const Integer& value)
{
    if (value == 0)
        return "0";

    UnsignedInteger magnitude = unsignedMagnitude(value);
    std::string digits;
    while (magnitude != 0)
    {
        const UnsignedInteger quotient = magnitude / 10u;
        const unsigned digit = static_cast<unsigned>(magnitude - quotient * 10u);
        digits.push_back(static_cast<char>('0' + digit));
        magnitude = quotient;
    }
    if (value < 0)
        digits.push_back('-');
    std::reverse(digits.begin(), digits.end());
    return digits;
}

inline long double integerToLongDouble(const Integer& value)
{
    UnsignedInteger magnitude = unsignedMagnitude(value);
    long double result = 0.0L;
    long double scale = 1.0L;
    constexpr UnsignedInteger mask = UnsignedInteger(0xffffffffull);
    while (magnitude != 0)
    {
        const std::uint64_t chunk = static_cast<std::uint64_t>(magnitude & mask);
        result += static_cast<long double>(chunk) * scale;
        magnitude >>= 32u;
        scale *= 4294967296.0L;
    }
    return value < 0 ? -result : result;
}

inline std::uint64_t integerLow64(const Integer& value) noexcept
{
    return static_cast<std::uint64_t>(static_cast<UnsignedInteger>(value));
}

inline std::ostream& operator<<(std::ostream& os, const Integer& value)
{
    return os << integerToString(value);
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

inline unsigned countTrailingZeroBits64(std::uint64_t value) noexcept
{
#if defined(__clang__) || defined(__GNUC__)
    return static_cast<unsigned>(__builtin_ctzll(value));
#else
    unsigned count = 0;
    while ((value & 1ull) == 0)
    {
        value >>= 1u;
        ++count;
    }
    return count;
#endif
}

inline unsigned countTrailingZeroBits(UnsignedInteger value) noexcept
{
    unsigned count = 0;
    constexpr UnsignedInteger lowMask = UnsignedInteger(0xffffffffffffffffull);
    while ((value & lowMask) == 0)
    {
        value >>= 64u;
        count += 64u;
    }
    return count + countTrailingZeroBits64(static_cast<std::uint64_t>(value));
}

/**
 * @brief 计算两个整数幅值的最大公约数。
 */
inline Integer gcdMagnitude(Integer lhs, Integer rhs) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::gcdMagnitude");
    if (hasUnitMagnitude(lhs) || hasUnitMagnitude(rhs))
        return 1;

    UnsignedInteger lhsMagnitude = unsignedMagnitude(lhs);
    UnsignedInteger rhsMagnitude = unsignedMagnitude(rhs);
    if (lhsMagnitude == 0)
        return static_cast<Integer>(rhsMagnitude);
    if (rhsMagnitude == 0 || lhsMagnitude == rhsMagnitude)
        return static_cast<Integer>(lhsMagnitude);

    const unsigned commonTrailingZeroBits = std::min(
        countTrailingZeroBits(lhsMagnitude),
        countTrailingZeroBits(rhsMagnitude));
    lhsMagnitude >>= countTrailingZeroBits(lhsMagnitude);

    do
    {
        rhsMagnitude >>= countTrailingZeroBits(rhsMagnitude);
        if (lhsMagnitude > rhsMagnitude)
            std::swap(lhsMagnitude, rhsMagnitude);
        rhsMagnitude -= lhsMagnitude;
    }
    while (rhsMagnitude != 0);

    return static_cast<Integer>(lhsMagnitude << commonTrailingZeroBits);
}

/**
 * @brief 计算四个整数幅值的最大公约数。
 */
inline Integer gcdMagnitude(const Integer& a, const Integer& b, const Integer& c, const Integer& d) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::gcdMagnitude4");
    if (hasUnitMagnitude(a) || hasUnitMagnitude(b) || hasUnitMagnitude(c) || hasUnitMagnitude(d))
        return 1;

    const bool aZero = isZero(a);
    const bool bZero = isZero(b);
    const bool cZero = isZero(c);
    const bool dZero = isZero(d);
    if (aZero && bZero && cZero)
        return absMagnitude(d);
    if (aZero && bZero && dZero)
        return absMagnitude(c);
    if (aZero && cZero && dZero)
        return absMagnitude(b);
    if (bZero && cZero && dZero)
        return absMagnitude(a);

    Integer result = 0;
    if (!aZero)
        result = absMagnitude(a);
    if (!bZero)
        result = isZero(result) ? absMagnitude(b) : gcdMagnitude(result, b);
    if (result == 1)
        return 1;
    if (!cZero)
        result = isZero(result) ? absMagnitude(c) : gcdMagnitude(result, c);
    if (result == 1)
        return 1;
    if (!dZero)
        result = isZero(result) ? absMagnitude(d) : gcdMagnitude(result, d);
    return result;
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
 * @brief 若 `numerator / denominator` 是整数商，则返回该商。
 */
inline bool tryExactDiv(const Integer& numerator, const Integer& denominator, Integer& outQuotient) noexcept
{
    if (isZero(denominator))
        return false;

    const Integer quotient = numerator / denominator;
    if (quotient * denominator != numerator)
        return false;

    outQuotient = quotient;
    return true;
}

inline void floorCeilDiv(const Integer& a, const Integer& b, Integer& outFloor, Integer& outCeil) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::floorCeilDiv");
    Integer num = a;
    Integer den = b;
    if (den < 0)
    {
        num = -num;
        den = -den;
    }

    const Integer quotient = num / den;
    if (quotient * den == num)
    {
        outFloor = quotient;
        outCeil = quotient;
        return;
    }

    if (num >= 0)
    {
        outFloor = quotient;
        outCeil = quotient + 1;
        return;
    }

    outFloor = quotient - 1;
    outCeil = quotient;
}

inline Integer floorDivByTwo(const Integer& value) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::floorDivByTwo");
    const Integer quotient = value / 2;
    if (value < 0 && quotient * 2 != value)
        return quotient - 1;
    return quotient;
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

    DotInteger dotPlane(const Plane3i& s) const noexcept;

    int classify(const Plane3i& s) const noexcept;

    // 该函数比较齐次坐标四个分量是否完全一致。
    bool hasSameComponents(const HomPoint4i& rhs) const noexcept
    {
        return x == rhs.x && y == rhs.y && z == rhs.z && w == rhs.w;
    }
};

/**
 * @brief 判断齐次点是否需要整体反号以满足 canonical 符号约定。
 */
inline bool needsHomPointSignFlip(const HomPoint4i& point) noexcept
{
    return point.w < 0 ||
           (isZero(point.w) && (point.z < 0 ||
                                (isZero(point.z) && (point.y < 0 ||
                                                     (isZero(point.y) && point.x < 0)))));
}

/**
 * @brief 返回满足 canonical 符号约定的等价齐次点。
 */
inline HomPoint4i normalizedHomPointSign(HomPoint4i point) noexcept
{
    if (needsHomPointSignFlip(point))
    {
        point.x = -point.x;
        point.y = -point.y;
        point.z = -point.z;
        point.w = -point.w;
    }
    return point;
}

/**
 * @brief 返回与输入几何等价的 primitive 齐次点。
 */
inline HomPoint4i primitiveHomPoint(const HomPoint4i& point) noexcept
{
    REEMBER_PROFILE_MATH_ZONE("math256::primitiveHomPoint");
    if (hasUnitMagnitude(point.w))
        return normalizedHomPointSign(point);

    HomPoint4i normalized = normalizedHomPointSign(point);
    Integer x = normalized.x;
    Integer y = normalized.y;
    Integer z = normalized.z;
    Integer w = normalized.w;
    Integer quotientX;
    Integer quotientY;
    Integer quotientZ;
    if (tryExactDiv(x, w, quotientX) &&
            tryExactDiv(y, w, quotientY) &&
            tryExactDiv(z, w, quotientZ))
    {
        return HomPoint4i(quotientX, quotientY, quotientZ, 1);
    }

    const Integer divisor = gcdMagnitude(x, y, z, w);
    if (divisor > 1)
    {
        x /= divisor;
        y /= divisor;
        z /= divisor;
        w /= divisor;
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
    if (&lhs == &rhs)
        return true;
    if (lhs.hasSameComponents(rhs))
        return true;

    const bool lhsZero = isZeroHomPoint(lhs);
    const bool rhsZero = isZeroHomPoint(rhs);
    if (lhsZero || rhsZero)
        return lhsZero && rhsZero;

    if (!isZero(lhs.x))
    {
        return wideProduct(lhs.y, rhs.x) == wideProduct(rhs.y, lhs.x) &&
               wideProduct(lhs.z, rhs.x) == wideProduct(rhs.z, lhs.x) &&
               wideProduct(lhs.w, rhs.x) == wideProduct(rhs.w, lhs.x);
    }
    if (!isZero(lhs.y))
    {
        return wideProduct(lhs.x, rhs.y) == wideProduct(rhs.x, lhs.y) &&
               wideProduct(lhs.z, rhs.y) == wideProduct(rhs.z, lhs.y) &&
               wideProduct(lhs.w, rhs.y) == wideProduct(rhs.w, lhs.y);
    }
    if (!isZero(lhs.z))
    {
        return wideProduct(lhs.x, rhs.z) == wideProduct(rhs.x, lhs.z) &&
               wideProduct(lhs.y, rhs.z) == wideProduct(rhs.y, lhs.z) &&
               wideProduct(lhs.w, rhs.z) == wideProduct(rhs.w, lhs.z);
    }

    return wideProduct(lhs.x, rhs.w) == wideProduct(rhs.x, lhs.w) &&
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
