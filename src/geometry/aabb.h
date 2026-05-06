/**
 * @file aabb.h
 * @brief 定义整数 AABB 辅助类型和中点切分操作。
 */
#pragma once

#include "core/perf_tracing.h"
#include "geometry/geometry256.h"

#include <array>
#include <vector>

namespace ember
{
    /**
     * @brief 细分阶段使用的整数轴对齐包围盒。
     */
    struct AABB3i
    {
        Integer xMin = 0;
        Integer xMax = 0;
        Integer yMin = 0;
        Integer yMax = 0;
        Integer zMin = 0;
        Integer zMax = 0;
        bool valid = false;
    };

    /**
     * @brief AABB 中点切分选择的坐标轴。
     */
    enum class SplitAxis3i
    {
        X,
        Y,
        Z
    };

    /**
     * @brief 单次 AABB 中点切分的结果。
     */
    struct AABBSplit3i
    {
        AABB3i left;
        AABB3i right;
        Plane3i splitPlane;
        SplitAxis3i axis = SplitAxis3i::X;
        Integer coordinate = 0;
        bool valid = false;
    };

    namespace detail
    {
        inline Integer clampInteger(const Integer &value, const Integer &lower, const Integer &upper) noexcept
        {
            if (value < lower)
            {
                return lower;
            }
            if (value > upper)
            {
                return upper;
            }
            return value;
        }

        inline Integer axisSpan(const AABB3i &box, SplitAxis3i axis) noexcept
        {
            switch (axis)
            {
            case SplitAxis3i::X:
                return box.xMax - box.xMin;
            case SplitAxis3i::Y:
                return box.yMax - box.yMin;
            case SplitAxis3i::Z:
                return box.zMax - box.zMin;
            }
            return 0;
        }

        inline int axisOrderKey(SplitAxis3i axis) noexcept
        {
            switch (axis)
            {
            case SplitAxis3i::X:
                return 0;
            case SplitAxis3i::Y:
                return 1;
            case SplitAxis3i::Z:
                return 2;
            }
            return 0;
        }

        inline bool areSamePlaneEquation(const Plane3i &lhs, const Plane3i &rhs) noexcept
        {
            return lhs.a == rhs.a && lhs.b == rhs.b && lhs.c == rhs.c && lhs.d == rhs.d;
        }

        inline bool tryExtractExactIntegerPoint(
            const PlanePoint3i &point,
            Integer &x,
            Integer &y,
            Integer &z) noexcept
        {
            if (!point.hasUniqueIntersection() || isZero(point.x.w))
            {
                return false;
            }

            const Integer xFloor = floorDiv(point.x.x, point.x.w);
            const Integer xCeil = ceilDiv(point.x.x, point.x.w);
            const Integer yFloor = floorDiv(point.x.y, point.x.w);
            const Integer yCeil = ceilDiv(point.x.y, point.x.w);
            const Integer zFloor = floorDiv(point.x.z, point.x.w);
            const Integer zCeil = ceilDiv(point.x.z, point.x.w);

            if (xFloor != xCeil || yFloor != yCeil || zFloor != zCeil)
            {
                return false;
            }

            x = xFloor;
            y = yFloor;
            z = zFloor;
            return true;
        }

        inline bool chooseLongestSplittableAxis(const AABB3i &box, SplitAxis3i &outAxis) noexcept
        {
            const Integer one = 1;
            SplitAxis3i bestAxis = SplitAxis3i::X;
            Integer bestSpan = -1;

            for (const SplitAxis3i axis : {SplitAxis3i::X, SplitAxis3i::Y, SplitAxis3i::Z})
            {
                const Integer span = axisSpan(box, axis);
                if (span <= one)
                {
                    continue;
                }

                if (span > bestSpan)
                {
                    bestSpan = span;
                    bestAxis = axis;
                }
            }

            if (bestSpan <= one)
            {
                return false;
            }

            outAxis = bestAxis;
            return true;
        }
    }

    inline bool isValidAABB(const AABB3i &box) noexcept
    {
        return box.valid && box.xMin <= box.xMax && box.yMin <= box.yMax && box.zMin <= box.zMax;
    }

    inline PlanePoint3i makeIntegerPoint(const Integer &x, const Integer &y, const Integer &z) noexcept
    {
        return PlanePoint3i(
            Plane3i(1, 0, 0, -x),
            Plane3i(0, 1, 0, -y),
            Plane3i(0, 0, 1, -z));
    }

    inline PlanePoint3i getAABBCornerPoint(const AABB3i &box, bool useXMax, bool useYMax, bool useZMax) noexcept
    {
        if (!isValidAABB(box))
        {
            return PlanePoint3i();
        }

        return makeIntegerPoint(
            useXMax ? box.xMax : box.xMin,
            useYMax ? box.yMax : box.yMin,
            useZMax ? box.zMax : box.zMin);
    }

    inline std::array<Plane3i, 6> makeAABBPlanes(const AABB3i &box) noexcept
    {
        return {
            Plane3i(-1, 0, 0, box.xMin),
            Plane3i(1, 0, 0, -box.xMax),
            Plane3i(0, -1, 0, box.yMin),
            Plane3i(0, 1, 0, -box.yMax),
            Plane3i(0, 0, -1, box.zMin),
            Plane3i(0, 0, 1, -box.zMax)};
    }

    inline AABB3i computeAABB(const std::vector<Polygon256> &polygons, const Integer &margin = 1)
    {
        REEMBER_PROFILE_ZONE("computeAABB");

        AABB3i box;
        bool initialized = false;

        for (const Polygon256 &poly : polygons)
        {
            const std::vector<PlanePoint3i> &cachedVertices = poly.vertices();
            for (const PlanePoint3i &vertex : cachedVertices)
            {
                if (!vertex.hasUniqueIntersection() || isZero(vertex.x.w))
                {
                    continue;
                }

                const Integer fx = floorDiv(vertex.x.x, vertex.x.w);
                const Integer cx = ceilDiv(vertex.x.x, vertex.x.w);
                const Integer fy = floorDiv(vertex.x.y, vertex.x.w);
                const Integer cy = ceilDiv(vertex.x.y, vertex.x.w);
                const Integer fz = floorDiv(vertex.x.z, vertex.x.w);
                const Integer cz = ceilDiv(vertex.x.z, vertex.x.w);

                if (!initialized)
                {
                    box.xMin = fx;
                    box.xMax = cx;
                    box.yMin = fy;
                    box.yMax = cy;
                    box.zMin = fz;
                    box.zMax = cz;
                    initialized = true;
                }
                else
                {
                    if (fx < box.xMin) box.xMin = fx;
                    if (cx > box.xMax) box.xMax = cx;
                    if (fy < box.yMin) box.yMin = fy;
                    if (cy > box.yMax) box.yMax = cy;
                    if (fz < box.zMin) box.zMin = fz;
                    if (cz > box.zMax) box.zMax = cz;
                }
            }
        }

        if (!initialized)
        {
            return box;
        }

        box.xMin -= margin;
        box.xMax += margin;
        box.yMin -= margin;
        box.yMax += margin;
        box.zMin -= margin;
        box.zMax += margin;
        box.valid = true;
        return box;
    }

    inline bool isPointInsideOrOnAABB(const PlanePoint3i &point, const AABB3i &box) noexcept
    {
        if (!point.hasUniqueIntersection() || !isValidAABB(box))
        {
            return false;
        }

        const auto planes = makeAABBPlanes(box);
        for (const Plane3i &plane : planes)
        {
            if (point.classify(plane) > 0)
            {
                return false;
            }
        }
        return true;
    }

    inline bool hasSplittableAxis(const AABB3i &box) noexcept
    {
        SplitAxis3i axis = SplitAxis3i::X;
        return isValidAABB(box) && detail::chooseLongestSplittableAxis(box, axis);
    }

    /**
     * @brief 按指定轴和整数坐标切分 AABB。
     *
     * @param[in] box 待切分的 AABB。
     * @param[in] axis 切分轴。
     * @param[in] coordinate 切分坐标，必须严格位于 `box` 对应轴范围内部。
     * @param[out] outSplit 成功时写入左右子 AABB 和切分平面。
     * @return 切分合法并成功构造时返回 `true`。
     */
    inline bool splitAABBAtCoordinate(
        const AABB3i &box,
        SplitAxis3i axis,
        const Integer &coordinate,
        AABBSplit3i &outSplit)
    {
        outSplit = AABBSplit3i();
        if (!isValidAABB(box))
        {
            return false;
        }

        AABB3i left = box;
        AABB3i right = box;
        Plane3i splitPlane;

        switch (axis)
        {
        case SplitAxis3i::X:
            if (coordinate <= box.xMin || coordinate >= box.xMax) return false;
            left.xMax = coordinate;
            right.xMin = coordinate;
            splitPlane = Plane3i(1, 0, 0, -coordinate);
            break;
        case SplitAxis3i::Y:
            if (coordinate <= box.yMin || coordinate >= box.yMax) return false;
            left.yMax = coordinate;
            right.yMin = coordinate;
            splitPlane = Plane3i(0, 1, 0, -coordinate);
            break;
        case SplitAxis3i::Z:
            if (coordinate <= box.zMin || coordinate >= box.zMax) return false;
            left.zMax = coordinate;
            right.zMin = coordinate;
            splitPlane = Plane3i(0, 0, 1, -coordinate);
            break;
        }

        outSplit.left = left;
        outSplit.right = right;
        outSplit.splitPlane = splitPlane;
        outSplit.axis = axis;
        outSplit.coordinate = coordinate;
        outSplit.valid = true;
        return true;
    }

    inline bool splitAABBAtMidpoint(const AABB3i &box, AABBSplit3i &outSplit)
    {
        outSplit = AABBSplit3i();
        if (!isValidAABB(box))
        {
            return false;
        }

        SplitAxis3i axis = SplitAxis3i::X;
        if (!detail::chooseLongestSplittableAxis(box, axis))
        {
            return false;
        }

        const Integer two = 2;
        Integer coordinate = 0;
        switch (axis)
        {
        case SplitAxis3i::X:
            coordinate = floorDiv(box.xMin + box.xMax, two);
            break;
        case SplitAxis3i::Y:
            coordinate = floorDiv(box.yMin + box.yMax, two);
            break;
        case SplitAxis3i::Z:
            coordinate = floorDiv(box.zMin + box.zMax, two);
            break;
        }

        return splitAABBAtCoordinate(box, axis, coordinate, outSplit);
    }

    inline PlanePoint3i projectPointToAABB(const PlanePoint3i &point, const AABB3i &box)
    {
        if (!point.hasUniqueIntersection() || !isValidAABB(box))
        {
            return PlanePoint3i();
        }

        const Integer px = floorDiv(point.x.x, point.x.w);
        const Integer py = floorDiv(point.x.y, point.x.w);
        const Integer pz = floorDiv(point.x.z, point.x.w);

        return makeIntegerPoint(
            detail::clampInteger(px, box.xMin, box.xMax),
            detail::clampInteger(py, box.yMin, box.yMax),
            detail::clampInteger(pz, box.zMin, box.zMax));
    }
}
