/**
 * @file math256.h
 * @brief 定义固定宽度整数向量和行列式数学工具。
 */
#pragma once

#include <slimcpplib/long_int.h>
#include <slimcpplib/long_io.h>

#include <ostream>

namespace ember
{
    using Integer = slim::int256_t;

    inline int signum(const Integer& value) noexcept
    {
        return (value > 0) - (value < 0);
    }

    inline bool isZero(const Integer& value) noexcept
    {
        return value == 0;
    }

    inline bool isPositive(const Integer& value) noexcept
    {
        return value > 0;
    }

    inline Integer floorDiv(const Integer& a, const Integer& b) noexcept
    {
        Integer num = a;
        Integer den = b;
        if (den < 0)
        {
            num = -num;
            den = -den;
        }

        if (num >= 0)
        {
            return num / den;
        }

        return (num - den + 1) / den;
    }

    inline Integer ceilDiv(const Integer& a, const Integer& b) noexcept
    {
        Integer num = a;
        Integer den = b;
        if (den < 0)
        {
            num = -num;
            den = -den;
        }

        if (num >= 0)
        {
            return (num + den - 1) / den;
        }

        return num / den;
    }

    struct Vec2i
    {
        Integer x = 0;
        Integer y = 0;

        Vec2i() noexcept = default;
        Vec2i(const Integer& xVal, const Integer& yVal) noexcept : x(xVal), y(yVal) {}

        Vec2i& operator+=(const Vec2i& rhs) noexcept
        {
            x += rhs.x;
            y += rhs.y;
            return *this;
        }

        Vec2i& operator-=(const Vec2i& rhs) noexcept
        {
            x -= rhs.x;
            y -= rhs.y;
            return *this;
        }

        Vec2i& operator*=(const Integer& k) noexcept
        {
            x *= k;
            y *= k;
            return *this;
        }

        Vec2i& operator/=(const Integer& k) noexcept
        {
            x /= k;
            y /= k;
            return *this;
        }

        Vec2i operator+() const noexcept { return *this; }
        Vec2i operator-() const noexcept { return Vec2i(-x, -y); }

        bool operator==(const Vec2i& rhs) const noexcept { return x == rhs.x && y == rhs.y; }
        bool operator!=(const Vec2i& rhs) const noexcept { return !(*this == rhs); }

        Integer lengthSquared() const noexcept
        {
            return x * x + y * y;
        }
    };

    inline Vec2i operator+(Vec2i lhs, const Vec2i& rhs) noexcept
    {
        lhs += rhs;
        return lhs;
    }

    inline Vec2i operator-(Vec2i lhs, const Vec2i& rhs) noexcept
    {
        lhs -= rhs;
        return lhs;
    }

    inline Vec2i operator*(Vec2i v, const Integer& k) noexcept
    {
        v *= k;
        return v;
    }

    inline Vec2i operator*(const Integer& k, Vec2i v) noexcept
    {
        v *= k;
        return v;
    }

    inline Vec2i operator/(Vec2i v, const Integer& k) noexcept
    {
        v /= k;
        return v;
    }

    inline Integer dot(const Vec2i& a, const Vec2i& b) noexcept
    {
        return a.x * b.x + a.y * b.y;
    }

    inline Integer cross(const Vec2i& a, const Vec2i& b) noexcept
    {
        return a.x * b.y - a.y * b.x;
    }

    inline Integer determinant2x2(
        const Integer& a11,
        const Integer& a12,
        const Integer& a21,
        const Integer& a22) noexcept
    {
        return a11 * a22 - a12 * a21;
    }

    inline Integer determinant(const Vec2i& row1, const Vec2i& row2) noexcept
    {
        return determinant2x2(row1.x, row1.y, row2.x, row2.y);
    }

    inline Integer orient2d(const Vec2i& a, const Vec2i& b, const Vec2i& c) noexcept
    {
        return cross(b - a, c - a);
    }

    inline int orient2dSign(const Vec2i& a, const Vec2i& b, const Vec2i& c) noexcept
    {
        return signum(orient2d(a, b, c));
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

        Vec3i operator+() const noexcept { return *this; }
        Vec3i operator-() const noexcept { return Vec3i(-x, -y, -z); }

        bool operator==(const Vec3i& rhs) const noexcept { return x == rhs.x && y == rhs.y && z == rhs.z; }
        bool operator!=(const Vec3i& rhs) const noexcept { return !(*this == rhs); }

        Integer lengthSquared() const noexcept
        {
            return x * x + y * y + z * z;
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

    inline Integer scalarTriple(const Vec3i& a, const Vec3i& b, const Vec3i& c) noexcept
    {
        return dot(a, cross(b, c));
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
        return a11 * (a22 * a33 - a23 * a32)
            - a12 * (a21 * a33 - a23 * a31)
            + a13 * (a21 * a32 - a22 * a31);
    }

    inline Integer determinant4x4(
        const Integer& a11,
        const Integer& a12,
        const Integer& a13,
        const Integer& a14,
        const Integer& a21,
        const Integer& a22,
        const Integer& a23,
        const Integer& a24,
        const Integer& a31,
        const Integer& a32,
        const Integer& a33,
        const Integer& a34,
        const Integer& a41,
        const Integer& a42,
        const Integer& a43,
        const Integer& a44) noexcept
    {
        return a11 * determinant3x3(a22, a23, a24, a32, a33, a34, a42, a43, a44)
            - a12 * determinant3x3(a21, a23, a24, a31, a33, a34, a41, a43, a44)
            + a13 * determinant3x3(a21, a22, a24, a31, a32, a34, a41, a42, a44)
            - a14 * determinant3x3(a21, a22, a23, a31, a32, a33, a41, a42, a43);
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

    inline int orient3dSign(const Vec3i& a, const Vec3i& b, const Vec3i& c) noexcept
    {
        return signum(scalarTriple(a, b, c));
    }

    inline std::ostream& operator<<(std::ostream& os, const Vec2i& v)
    {
        return os << '(' << v.x << ", " << v.y << ')';
    }

    inline std::ostream& operator<<(std::ostream& os, const Vec3i& v)
    {
        return os << '(' << v.x << ", " << v.y << ", " << v.z << ')';
    }


}
