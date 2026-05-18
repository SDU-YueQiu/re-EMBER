/**
 * @file fixed_int256.h
 * @brief 实验性的 256 位定长整数后端，用于论文 kernel 图元逐步替换验证。
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace ember::fixed
{
class FixedInt256
{
public:
    FixedInt256() noexcept = default;

    explicit FixedInt256(std::int64_t value) noexcept
    {
        limbs_[0] = static_cast<std::uint64_t>(value);
        const std::uint64_t fill = value < 0 ? ~std::uint64_t(0) : std::uint64_t(0);
        limbs_[1] = fill;
        limbs_[2] = fill;
        limbs_[3] = fill;
    }

    static FixedInt256 fromRawLimbs(std::array<std::uint64_t, 4> limbs) noexcept
    {
        FixedInt256 value;
        value.limbs_ = limbs;
        return value;
    }

    std::uint64_t limb(std::size_t index) const noexcept
    {
        return limbs_[index];
    }

    bool isNegative() const noexcept
    {
        return (limbs_[3] & (std::uint64_t(1) << 63)) != 0;
    }

    bool isZero() const noexcept
    {
        return limbs_[0] == 0 && limbs_[1] == 0 && limbs_[2] == 0 && limbs_[3] == 0;
    }

    friend bool operator==(const FixedInt256 &lhs, const FixedInt256 &rhs) noexcept
    {
        return lhs.limbs_ == rhs.limbs_;
    }

    friend bool operator!=(const FixedInt256 &lhs, const FixedInt256 &rhs) noexcept
    {
        return !(lhs == rhs);
    }

    friend bool operator<(const FixedInt256 &lhs, const FixedInt256 &rhs) noexcept
    {
        const bool lhsNegative = lhs.isNegative();
        const bool rhsNegative = rhs.isNegative();
        if (lhsNegative != rhsNegative)
            return lhsNegative;

        for (std::size_t i = 4; i-- > 0;)
        {
            if (lhs.limbs_[i] != rhs.limbs_[i])
                return lhs.limbs_[i] < rhs.limbs_[i];
        }
        return false;
    }

    friend bool operator>(const FixedInt256 &lhs, const FixedInt256 &rhs) noexcept
    {
        return rhs < lhs;
    }

    friend bool operator<=(const FixedInt256 &lhs, const FixedInt256 &rhs) noexcept
    {
        return !(rhs < lhs);
    }

    friend bool operator>=(const FixedInt256 &lhs, const FixedInt256 &rhs) noexcept
    {
        return !(lhs < rhs);
    }

private:
    std::array<std::uint64_t, 4> limbs_{};
};

inline std::uint64_t addCarry(std::uint64_t lhs, std::uint64_t rhs, std::uint64_t &out) noexcept
{
    out = lhs + rhs;
    return out < lhs ? 1u : 0u;
}

inline std::uint64_t addCarry3(
    std::uint64_t lhs,
    std::uint64_t rhs,
    std::uint64_t carry,
    std::uint64_t &out) noexcept
{
    std::uint64_t partial = 0;
    const std::uint64_t carry0 = addCarry(lhs, rhs, partial);
    const std::uint64_t carry1 = addCarry(partial, carry, out);
    return carry0 + carry1;
}

inline FixedInt256 negateUnchecked(const FixedInt256 &value) noexcept
{
    std::array<std::uint64_t, 4> limbs = {
        ~value.limb(0),
        ~value.limb(1),
        ~value.limb(2),
        ~value.limb(3)};

    std::uint64_t carry = 1;
    for (std::uint64_t &limb : limbs)
    {
        const std::uint64_t previous = limb;
        limb += carry;
        carry = carry != 0 && limb <= previous ? 1u : 0u;
    }
    return FixedInt256::fromRawLimbs(limbs);
}

inline bool addChecked(const FixedInt256 &lhs, const FixedInt256 &rhs, FixedInt256 &out) noexcept
{
    std::array<std::uint64_t, 4> limbs{};
    std::uint64_t carry = 0;
    for (std::size_t i = 0; i < limbs.size(); ++i)
        carry = addCarry3(lhs.limb(i), rhs.limb(i), carry, limbs[i]);

    out = FixedInt256::fromRawLimbs(limbs);
    const bool sameInputSign = lhs.isNegative() == rhs.isNegative();
    return !(sameInputSign && out.isNegative() != lhs.isNegative());
}

inline bool negateChecked(const FixedInt256 &value, FixedInt256 &out) noexcept
{
    static constexpr std::array<std::uint64_t, 4> kMinRaw = {
        0,
        0,
        0,
        std::uint64_t(1) << 63};
    if (value == FixedInt256::fromRawLimbs(kMinRaw))
        return false;

    out = negateUnchecked(value);
    return true;
}

inline bool subChecked(const FixedInt256 &lhs, const FixedInt256 &rhs, FixedInt256 &out) noexcept
{
    const FixedInt256 negatedRhs = negateUnchecked(rhs);
    std::array<std::uint64_t, 4> limbs{};
    std::uint64_t carry = 0;
    for (std::size_t i = 0; i < limbs.size(); ++i)
        carry = addCarry3(lhs.limb(i), negatedRhs.limb(i), carry, limbs[i]);

    out = FixedInt256::fromRawLimbs(limbs);
    const bool differentInputSign = lhs.isNegative() != rhs.isNegative();
    return !(differentInputSign && out.isNegative() != lhs.isNegative());
}

inline void mul64x64(
    std::uint64_t lhs,
    std::uint64_t rhs,
    std::uint64_t &lo,
    std::uint64_t &hi) noexcept
{
    static constexpr std::uint64_t kMask32 = 0xffffffffull;
    const std::uint64_t lhsLo = lhs & kMask32;
    const std::uint64_t lhsHi = lhs >> 32;
    const std::uint64_t rhsLo = rhs & kMask32;
    const std::uint64_t rhsHi = rhs >> 32;

    const std::uint64_t p0 = lhsLo * rhsLo;
    const std::uint64_t p1 = lhsLo * rhsHi;
    const std::uint64_t p2 = lhsHi * rhsLo;
    const std::uint64_t p3 = lhsHi * rhsHi;

    const std::uint64_t middle = (p0 >> 32) + (p1 & kMask32) + (p2 & kMask32);
    lo = (p0 & kMask32) | (middle << 32);
    hi = p3 + (p1 >> 32) + (p2 >> 32) + (middle >> 32);
}

inline void addProductWord(std::array<std::uint64_t, 8> &accum, std::size_t index, std::uint64_t value) noexcept
{
    while (value != 0 && index < accum.size())
    {
        const std::uint64_t previous = accum[index];
        accum[index] += value;
        value = accum[index] < previous ? 1u : 0u;
        ++index;
    }
}

inline std::array<std::uint64_t, 4> unsignedMagnitude(const FixedInt256 &value) noexcept
{
    FixedInt256 magnitude = value.isNegative() ? negateUnchecked(value) : value;
    return {magnitude.limb(0), magnitude.limb(1), magnitude.limb(2), magnitude.limb(3)};
}

inline bool isMagnitudeZero(const std::array<std::uint64_t, 4> &magnitude) noexcept
{
    return magnitude[0] == 0 && magnitude[1] == 0 && magnitude[2] == 0 && magnitude[3] == 0;
}

inline bool magnitudeFitsSignedPositive(const std::array<std::uint64_t, 8> &magnitude) noexcept
{
    return magnitude[7] == 0 &&
           magnitude[6] == 0 &&
           magnitude[5] == 0 &&
           magnitude[4] == 0 &&
           magnitude[3] <= 0x7fffffffffffffffull;
}

inline bool magnitudeFitsSignedNegative(const std::array<std::uint64_t, 8> &magnitude) noexcept
{
    if (magnitude[7] != 0 || magnitude[6] != 0 || magnitude[5] != 0 || magnitude[4] != 0)
        return false;
    if (magnitude[3] < 0x8000000000000000ull)
        return true;
    if (magnitude[3] > 0x8000000000000000ull)
        return false;
    return magnitude[2] == 0 && magnitude[1] == 0 && magnitude[0] == 0;
}

inline bool multiplyChecked(const FixedInt256 &lhs, const FixedInt256 &rhs, FixedInt256 &out) noexcept
{
    const bool negative = lhs.isNegative() != rhs.isNegative();
    const std::array<std::uint64_t, 4> lhsMag = unsignedMagnitude(lhs);
    const std::array<std::uint64_t, 4> rhsMag = unsignedMagnitude(rhs);

    std::array<std::uint64_t, 8> product{};
    for (std::size_t i = 0; i < lhsMag.size(); ++i)
    {
        for (std::size_t j = 0; j < rhsMag.size(); ++j)
        {
            std::uint64_t lo = 0;
            std::uint64_t hi = 0;
            mul64x64(lhsMag[i], rhsMag[j], lo, hi);
            addProductWord(product, i + j, lo);
            addProductWord(product, i + j + 1, hi);
        }
    }

    if (negative)
    {
        if (!magnitudeFitsSignedNegative(product))
            return false;
    }
    else
    {
        if (!magnitudeFitsSignedPositive(product))
            return false;
    }

    std::array<std::uint64_t, 4> raw = {product[0], product[1], product[2], product[3]};
    out = FixedInt256::fromRawLimbs(raw);
    if (negative && !isMagnitudeZero(raw))
        out = negateUnchecked(out);
    return true;
}

inline bool multiplyAddChecked(
    const FixedInt256 &lhs,
    const FixedInt256 &rhs,
    FixedInt256 &accum) noexcept
{
    FixedInt256 product;
    if (!multiplyChecked(lhs, rhs, product))
        return false;
    return addChecked(accum, product, accum);
}

inline bool multiplySubChecked(
    const FixedInt256 &lhs,
    const FixedInt256 &rhs,
    FixedInt256 &accum) noexcept
{
    FixedInt256 product;
    if (!multiplyChecked(lhs, rhs, product))
        return false;
    return subChecked(accum, product, accum);
}

inline bool dot4Checked(
    const FixedInt256 &x,
    const FixedInt256 &y,
    const FixedInt256 &z,
    const FixedInt256 &w,
    const FixedInt256 &a,
    const FixedInt256 &b,
    const FixedInt256 &c,
    const FixedInt256 &d,
    FixedInt256 &out) noexcept
{
    out = FixedInt256();
    return multiplyAddChecked(x, a, out) &&
           multiplyAddChecked(y, b, out) &&
           multiplyAddChecked(z, c, out) &&
           multiplyAddChecked(w, d, out);
}

inline bool determinant3x3Checked(
    const FixedInt256 &a11,
    const FixedInt256 &a12,
    const FixedInt256 &a13,
    const FixedInt256 &a21,
    const FixedInt256 &a22,
    const FixedInt256 &a23,
    const FixedInt256 &a31,
    const FixedInt256 &a32,
    const FixedInt256 &a33,
    FixedInt256 &out) noexcept
{
    FixedInt256 minor0;
    FixedInt256 minor1;
    FixedInt256 minor2;
    FixedInt256 term;

    if (!multiplyChecked(a22, a33, minor0) || !multiplySubChecked(a23, a32, minor0))
        return false;
    if (!multiplyChecked(a21, a33, minor1) || !multiplySubChecked(a23, a31, minor1))
        return false;
    if (!multiplyChecked(a21, a32, minor2) || !multiplySubChecked(a22, a31, minor2))
        return false;

    if (!multiplyChecked(a11, minor0, out))
        return false;
    if (!multiplyChecked(a12, minor1, term) || !subChecked(out, term, out))
        return false;
    return multiplyChecked(a13, minor2, term) && addChecked(out, term, out);
}
}
