/**
 * @file paper_kernel.h
 * @brief 定义论文固定宽度几何 kernel 的受控 primitive 入口。
 */
#pragma once

#include "geometry/plane_geometry256.h"

#include <boost/multiprecision/cpp_int.hpp>

#include <array>

namespace ember::paper
{
/**
 * @brief 论文 kernel 的输入坐标上界：带符号 26 bit 整数。
 */
inline constexpr int kInputCoordinateBits = 26;

/**
 * @brief 论文 kernel 的核心中间结果预算：固定 256 bit 整数。
 */
inline constexpr int kCoreArithmeticBits = 256;

/**
 * @brief 表示一个核心 primitive 是否满足论文闭包契约。
 */
struct KernelContractResult
{
    bool ok = true;
    const char *reason = "";
};

/**
 * @brief 判断整数是否落在 signed 26 bit 输入坐标范围内。
 */
inline bool isWithinInputCoordinateBound(const Integer &value) noexcept
{
    const Integer bound = Integer(1) << (kInputCoordinateBits - 1);
    return value >= -bound && value < bound;
}

/**
 * @brief 判断三维整数顶点是否满足论文输入量化上界。
 */
inline bool isWithinInputCoordinateBound(const Vec3i &point) noexcept
{
    return isWithinInputCoordinateBound(point.x) &&
           isWithinInputCoordinateBound(point.y) &&
           isWithinInputCoordinateBound(point.z);
}

/**
 * @brief 用整数点和整数法向构造平面，等价于论文的 plane_from_points/point-normal 入口。
 *
 * @note 该入口只负责受控构造和 primitive 化，不允许从齐次输出点反推新平面。
 */
inline Plane3i planeFromPointNormal(const Vec3i &point, const Vec3i &normal) noexcept
{
    return Plane3i::fromPointNormal(point, normal);
}

/**
 * @brief 构造轴对齐 AABB 平面，供细分盒和轴向路径使用。
 */
inline Plane3i axisAlignedPlane(SplitAxis3i axis, const Integer &coordinate, int positiveSideNormal) noexcept
{
    const Integer normal = positiveSideNormal < 0 ? Integer(-1) : Integer(1);
    switch (axis)
    {
    case SplitAxis3i::X:
        return Plane3i(normal, 0, 0, -normal * coordinate);
    case SplitAxis3i::Y:
        return Plane3i(0, normal, 0, -normal * coordinate);
    case SplitAxis3i::Z:
        return Plane3i(0, 0, normal, -normal * coordinate);
    }
    return Plane3i();
}

/**
 * @brief 判断两个平面法向是否平行，对应论文 are_planes_parallel。
 */
inline bool arePlanesParallel(const Plane3i &lhs, const Plane3i &rhs) noexcept
{
    return arePlaneNormalsParallel(lhs, rhs);
}

/**
 * @brief 构造三平面的齐次交点，对应论文 intersect_3_planes。
 */
inline HomPoint4i intersect3Planes(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return intersectHomogeneous(p, q, r);
}

/**
 * @brief 用未约分齐次点构造三平面交点，供热路径避免额外 gcd。
 */
inline HomPoint4i intersect3PlanesUnnormalized(const Plane3i &p, const Plane3i &q, const Plane3i &r) noexcept
{
    return intersectHomogeneousUnnormalized(p, q, r);
}

/**
 * @brief 对齐次点做平面分类，对应论文 classify_vertex。
 */
inline int classifyVertex(const HomPoint4i &point, const Plane3i &plane) noexcept
{
    return classifyPointAgainstPlane(point, plane);
}

/**
 * @brief 对整数顶点做平面分类，即 signed_distance 的符号版本。
 */
inline int classifyIntegerVertex(const Vec3i &point, const Plane3i &plane) noexcept
{
    return signum(point.x * plane.a + point.y * plane.b + point.z * plane.c + plane.d);
}

/**
 * @brief 检查三平面交点再分类是否与宽整数 4x4 行列式符号一致。
 *
 * @note 宽整数只作为 oracle 使用，不允许核心算法依赖该结果分支。
 */
inline KernelContractResult verifyClassifyVertexWithOracle(
    const Plane3i &p,
    const Plane3i &q,
    const Plane3i &r,
    const Plane3i &s)
{
    using boost::multiprecision::cpp_int;

    const HomPoint4i x = intersect3PlanesUnnormalized(p, q, r);
    const int kernelSign = classifyVertex(x, s);

    const std::array<std::array<cpp_int, 4>, 4> m = {{
        {cpp_int(p.a), cpp_int(p.b), cpp_int(p.c), cpp_int(p.d)},
        {cpp_int(q.a), cpp_int(q.b), cpp_int(q.c), cpp_int(q.d)},
        {cpp_int(r.a), cpp_int(r.b), cpp_int(r.c), cpp_int(r.d)},
        {cpp_int(s.a), cpp_int(s.b), cpp_int(s.c), cpp_int(s.d)}
    }};

    const cpp_int det =
        m[0][0] * (m[1][1] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                   m[1][2] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) +
                   m[1][3] * (m[2][1] * m[3][2] - m[2][2] * m[3][1])) -
        m[0][1] * (m[1][0] * (m[2][2] * m[3][3] - m[2][3] * m[3][2]) -
                   m[1][2] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                   m[1][3] * (m[2][0] * m[3][2] - m[2][2] * m[3][0])) +
        m[0][2] * (m[1][0] * (m[2][1] * m[3][3] - m[2][3] * m[3][1]) -
                   m[1][1] * (m[2][0] * m[3][3] - m[2][3] * m[3][0]) +
                   m[1][3] * (m[2][0] * m[3][1] - m[2][1] * m[3][0])) -
        m[0][3] * (m[1][0] * (m[2][1] * m[3][2] - m[2][2] * m[3][1]) -
                   m[1][1] * (m[2][0] * m[3][2] - m[2][2] * m[3][0]) +
                   m[1][2] * (m[2][0] * m[3][1] - m[2][1] * m[3][0]));

    const int oracleSign = (det > 0) - (det < 0);
    if (kernelSign != oracleSign)
        return {false, "classify_vertex sign disagrees with cpp_int determinant oracle"};

    return {true, ""};
}
}
