/**
 * @file math256.h
 * @brief 定义固定宽度整数向量、齐次点和行列式数学工具。
 */
#pragma once

#include "core/perf_tracing.h"

#include <algorithm>
#include <array>
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

namespace detail
{
using UInt128 = unsigned _BitInt(128);

struct DivRemUnsignedInteger
{
    UnsignedInteger quotient = 0;
    UnsignedInteger remainder = 0;
};

inline std::uint64_t limb64(UnsignedInteger value, unsigned index) noexcept
{
    return static_cast<std::uint64_t>(value >> (index * 64u));
}

inline UnsignedInteger composeUnsignedInteger(const std::array<std::uint64_t, 4>& limbs) noexcept
{
    return UnsignedInteger(limbs[0]) |
           (UnsignedInteger(limbs[1]) << 64u) |
           (UnsignedInteger(limbs[2]) << 128u) |
           (UnsignedInteger(limbs[3]) << 192u);
}

inline unsigned countLeadingZeroBits64(std::uint64_t value) noexcept
{
#if defined(__clang__) || defined(__GNUC__)
    return static_cast<unsigned>(__builtin_clzll(value));
#else
    unsigned count = 0;
    for (std::uint64_t bit = std::uint64_t(1) << 63u; (value & bit) == 0; bit >>= 1u)
        ++count;
    return count;
#endif
}

inline unsigned usedLimbCount(const std::array<std::uint64_t, 4>& limbs) noexcept
{
    for (unsigned count = 4; count > 0; --count)
    {
        if (limbs[count - 1] != 0)
            return count;
    }
    return 0;
}

inline bool lessUnsignedLimbs(
    const std::array<std::uint64_t, 4>& lhs,
    const std::array<std::uint64_t, 4>& rhs) noexcept
{
    for (unsigned i = 4; i > 0; --i)
    {
        const unsigned index = i - 1;
        if (lhs[index] != rhs[index])
            return lhs[index] < rhs[index];
    }
    return false;
}

inline std::array<std::uint64_t, 4> decomposeUnsignedInteger(UnsignedInteger value) noexcept
{
    return {limb64(value, 0), limb64(value, 1), limb64(value, 2), limb64(value, 3)};
}

inline std::uint64_t divRemU128ByU64(
    std::uint64_t high,
    std::uint64_t low,
    std::uint64_t denominator,
    std::uint64_t& outRemainder) noexcept
{
    UInt128 remainder = high;
    std::uint64_t quotient = 0;
    for (unsigned i = 64; i > 0; --i)
    {
        const unsigned bit = i - 1u;
        remainder = (remainder << 1u) | UInt128((low >> bit) & 1u);
        if (remainder >= denominator)
        {
            remainder -= denominator;
            quotient |= std::uint64_t(1) << bit;
        }
    }
    outRemainder = static_cast<std::uint64_t>(remainder);
    return quotient;
}

inline DivRemUnsignedInteger divRemUnsignedByU64(UnsignedInteger numerator, std::uint64_t denominator) noexcept
{
    std::array<std::uint64_t, 4> quotient{};
    std::uint64_t remainder = 0;
    for (unsigned i = 4; i > 0; --i)
    {
        const unsigned index = i - 1;
        quotient[index] = divRemU128ByU64(remainder, limb64(numerator, index), denominator, remainder);
    }
    return {composeUnsignedInteger(quotient), UnsignedInteger(remainder)};
}

inline DivRemUnsignedInteger divRemUnsignedKnuth(
    const std::array<std::uint64_t, 4>& numerator,
    const std::array<std::uint64_t, 4>& denominator,
    unsigned numeratorCount,
    unsigned denominatorCount) noexcept
{
    const unsigned quotientCount = numeratorCount - denominatorCount + 1u;
    const unsigned shift = countLeadingZeroBits64(denominator[denominatorCount - 1]);
    std::array<std::uint64_t, 4> normalizedDenominator{};
    std::array<std::uint64_t, 5> normalizedNumerator{};

    if (shift == 0)
    {
        for (unsigned i = 0; i < denominatorCount; ++i)
            normalizedDenominator[i] = denominator[i];
        for (unsigned i = 0; i < numeratorCount; ++i)
            normalizedNumerator[i] = numerator[i];
    }
    else
    {
        std::uint64_t carry = 0;
        for (unsigned i = 0; i < denominatorCount; ++i)
        {
            normalizedDenominator[i] = (denominator[i] << shift) | carry;
            carry = denominator[i] >> (64u - shift);
        }

        carry = 0;
        for (unsigned i = 0; i < numeratorCount; ++i)
        {
            normalizedNumerator[i] = (numerator[i] << shift) | carry;
            carry = numerator[i] >> (64u - shift);
        }
        normalizedNumerator[numeratorCount] = carry;
    }

    std::array<std::uint64_t, 4> quotient{};
    for (unsigned jStep = quotientCount; jStep > 0; --jStep)
    {
        const unsigned j = jStep - 1u;
        const std::uint64_t topHigh = normalizedNumerator[j + denominatorCount];
        const std::uint64_t topLow = normalizedNumerator[j + denominatorCount - 1u];
        UInt128 rhat = 0;
        std::uint64_t qhat = 0;
        if (topHigh >= normalizedDenominator[denominatorCount - 1u])
        {
            qhat = std::numeric_limits<std::uint64_t>::max();
            rhat = UInt128(topLow) + normalizedDenominator[denominatorCount - 1u];
        }
        else
        {
            std::uint64_t rhatLow = 0;
            qhat = divRemU128ByU64(
                topHigh,
                topLow,
                normalizedDenominator[denominatorCount - 1u],
                rhatLow);
            rhat = rhatLow;
        }
        if (denominatorCount > 1)
        {
            while (true)
            {
                const UInt128 lhs = UInt128(qhat) * normalizedDenominator[denominatorCount - 2u];
                if (rhat > UInt128(std::numeric_limits<std::uint64_t>::max()))
                    break;
                const UInt128 rhs =
                    (rhat << 64u) | UInt128(normalizedNumerator[j + denominatorCount - 2u]);
                if (lhs <= rhs)
                    break;
                --qhat;
                rhat += normalizedDenominator[denominatorCount - 1u];
            }
        }

        UInt128 borrowCarry = 0;
        for (unsigned i = 0; i < denominatorCount; ++i)
        {
            const UInt128 product = UInt128(qhat) * normalizedDenominator[i] + borrowCarry;
            const std::uint64_t productLow = static_cast<std::uint64_t>(product);
            borrowCarry = product >> 64u;

            const std::uint64_t before = normalizedNumerator[j + i];
            const std::uint64_t next = before - productLow;
            const std::uint64_t borrow = next > before ? 1u : 0u;
            normalizedNumerator[j + i] = next;
            borrowCarry += borrow;
        }

        bool negative = UInt128(normalizedNumerator[j + denominatorCount]) < borrowCarry;
        normalizedNumerator[j + denominatorCount] =
            static_cast<std::uint64_t>(UInt128(normalizedNumerator[j + denominatorCount]) - borrowCarry);

        if (negative)
        {
            --qhat;
            UInt128 carry = 0;
            for (unsigned i = 0; i < denominatorCount; ++i)
            {
                const UInt128 sum =
                    UInt128(normalizedNumerator[j + i]) + normalizedDenominator[i] + carry;
                normalizedNumerator[j + i] = static_cast<std::uint64_t>(sum);
                carry = sum >> 64u;
            }
            normalizedNumerator[j + denominatorCount] =
                static_cast<std::uint64_t>(
                    UInt128(normalizedNumerator[j + denominatorCount]) + carry);
        }

        quotient[j] = qhat;
    }

    std::array<std::uint64_t, 4> remainder{};
    if (shift == 0)
    {
        for (unsigned i = 0; i < denominatorCount; ++i)
            remainder[i] = normalizedNumerator[i];
    }
    else
    {
        for (unsigned i = 0; i < denominatorCount; ++i)
        {
            const std::uint64_t high = i + 1u < normalizedNumerator.size()
                ? normalizedNumerator[i + 1u]
                : 0u;
            remainder[i] = (normalizedNumerator[i] >> shift) | (high << (64u - shift));
        }
    }

    return {composeUnsignedInteger(quotient), composeUnsignedInteger(remainder)};
}

inline DivRemUnsignedInteger divRemUnsignedInteger(
    UnsignedInteger numerator,
    UnsignedInteger denominator) noexcept
{
    const std::array<std::uint64_t, 4> numeratorLimbs = decomposeUnsignedInteger(numerator);
    const std::array<std::uint64_t, 4> denominatorLimbs = decomposeUnsignedInteger(denominator);
    const unsigned denominatorCount = usedLimbCount(denominatorLimbs);
    if (denominatorCount == 0)
        return {};

    const unsigned numeratorCount = usedLimbCount(numeratorLimbs);
    if (numeratorCount == 0 || lessUnsignedLimbs(numeratorLimbs, denominatorLimbs))
        return {0, numerator};
    if (denominatorCount == 1)
        return divRemUnsignedByU64(numerator, denominatorLimbs[0]);

    return divRemUnsignedKnuth(numeratorLimbs, denominatorLimbs, numeratorCount, denominatorCount);
}

inline DivRemUnsignedInteger divRemMagnitude(Integer numerator, Integer denominator) noexcept
{
    const UnsignedInteger numeratorMagnitude = unsignedMagnitude(numerator);
    const UnsignedInteger denominatorMagnitude = unsignedMagnitude(denominator);
    return divRemUnsignedInteger(numeratorMagnitude, denominatorMagnitude);
}

inline Integer applySignedQuotient(
    UnsignedInteger quotientMagnitude,
    bool negative) noexcept
{
    const Integer quotient = static_cast<Integer>(quotientMagnitude);
    return negative && quotient != 0 ? -quotient : quotient;
}

inline Integer applySignedRemainder(
    UnsignedInteger remainderMagnitude,
    bool negative) noexcept
{
    const Integer remainder = static_cast<Integer>(remainderMagnitude);
    return negative && remainder != 0 ? -remainder : remainder;
}
}

inline Integer divInteger(Integer numerator, Integer denominator) noexcept
{
    const detail::DivRemUnsignedInteger divRem = detail::divRemMagnitude(numerator, denominator);
    return detail::applySignedQuotient(divRem.quotient, (numerator < 0) != (denominator < 0));
}

inline Integer remInteger(Integer numerator, Integer denominator) noexcept
{
    const detail::DivRemUnsignedInteger divRem = detail::divRemMagnitude(numerator, denominator);
    return detail::applySignedRemainder(divRem.remainder, numerator < 0);
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
        return divInteger(num, den);

    return divInteger(num - den + 1, den);
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
        return divInteger(num + den - 1, den);

    return divInteger(num, den);
}

/**
 * @brief 若 `numerator / denominator` 是整数商，则返回该商。
 */
inline bool tryExactDiv(const Integer& numerator, const Integer& denominator, Integer& outQuotient) noexcept
{
    if (isZero(denominator))
        return false;

    const detail::DivRemUnsignedInteger divRem = detail::divRemMagnitude(numerator, denominator);
    if (divRem.remainder != 0)
        return false;

    outQuotient = detail::applySignedQuotient(
        divRem.quotient,
        (numerator < 0) != (denominator < 0));
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

    const detail::DivRemUnsignedInteger divRem = detail::divRemMagnitude(num, den);
    const Integer quotient = detail::applySignedQuotient(divRem.quotient, num < 0);
    if (divRem.remainder == 0)
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
    return value >> 1;
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
        x = divInteger(x, divisor);
        y = divInteger(y, divisor);
        z = divInteger(z, divisor);
        w = divInteger(w, divisor);
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
        x = divInteger(x, k);
        y = divInteger(y, k);
        z = divInteger(z, k);
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
