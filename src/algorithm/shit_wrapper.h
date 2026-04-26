#pragma once

// TODO：本文件是临时抽象层，存放所有新添加的重构前难以分类的几何处理等辅助函数，用于阻断前面的实现屎山外溢

#include "geometry/clipping.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace ember
{
    /**
     * @brief 读取多边形的一个顶点。
     *
     * @param[in] poly 输入凸多边形。
     * @param[in] index 顶点下标，对应当前边平面与前一条边平面的交点。
     * @return 当下标有效时返回对应顶点；否则返回无唯一交点的默认点。
     */
    inline PlanePoint3i getPolygonVertex(const Polygon256 &poly, std::size_t index) noexcept
    {
        const std::size_t n = poly.edgePlanes.size();
        if (n == 0 || index >= n)
        {
            return PlanePoint3i();
        }

        const std::size_t prev = (index == 0) ? (n - 1) : (index - 1);
        return PlanePoint3i(poly.plane, poly.edgePlanes[index], poly.edgePlanes[prev]);
    }

    /**
     * @brief 判断两个 `PlanePoint3i` 是否表示同一个齐次点。
     *
     * @param[in] lhs 左侧点。
     * @param[in] rhs 右侧点。
     * @return 当两点均有唯一交点且齐次坐标相等时返回 `true`。
     */
    inline constexpr bool areSamePlanePoint(const PlanePoint3i &lhs, const PlanePoint3i &rhs) noexcept
    {
        if (!lhs.hasUniqueIntersection() || !rhs.hasUniqueIntersection())
        {
            return false;
        }

        return areSameHomPoint(lhs.x, rhs.x);
    }

    /**
     * @brief 判断点是否在线段上，端点也算在线段上。
     *
     * @param[in] point 查询点。
     * @param[in] seg 查询线段。
     * @return 当点在线段支撑线上且位于两个端点平面之间时返回 `true`。
     */
    inline constexpr bool isPointOnSegment(const PlanePoint3i &point, const Segment256 &seg) noexcept
    {
        if (!point.hasUniqueIntersection() || !seg.direction.contains(point))
        {
            return false;
        }

        const int pcs = point.classify(seg.start);
        const int pec = point.classify(seg.end);
        return (pcs != 1) && (pec != 1);
    }

    /**
     * @brief 判断线段是否接触多边形顶点。
     *
     * @param[in] seg 查询线段。
     * @param[in] poly 查询多边形。
     * @return 当线段经过任意多边形顶点时返回 `true`。
     */
    inline bool isSegmentTouchPolygonVertex(const Segment256 &seg, const Polygon256 &poly) noexcept
    {
        if (!seg.isValid() || !poly.isValid())
        {
            return false;
        }

        const std::size_t n = poly.edgePlanes.size();
        for (std::size_t i = 0; i < n; ++i)
        {
            const PlanePoint3i vertex = getPolygonVertex(poly, i);
            if (vertex.hasUniqueIntersection() && isPointOnSegment(vertex, seg))
            {
                return true;
            }
        }

        return false;
    }

    /**
     * @brief 判断线段是否接触多边形边界。
     *
     * @param[in] seg 查询线段。
     * @param[in] poly 查询多边形。
     * @return 当线段与任意边相交、共线重叠或触碰端点时返回 `true`。
     */
    inline bool isSegmentTouchPolygonEdge(const Segment256 &seg, const Polygon256 &poly) noexcept
    {
        if (!seg.isValid() || !poly.isValid())
        {
            return false;
        }

        const PlanePoint3i startPoint = seg.getStartPoint();
        const PlanePoint3i endPoint = seg.getEndPoint();

        const std::size_t n = poly.edgePlanes.size();
        for (std::size_t i = 0; i < n; ++i)
        {
            const Plane3i &edgePlane = poly.edgePlanes[i];
            const auto &nextEdgePlane = poly.edgePlanes[(i + 1) % n];
            const auto &prevEdgePlane = poly.edgePlanes[(i - 1 + n) % n];

            const Segment256 edge(prevEdgePlane, nextEdgePlane, {poly.plane, edgePlane});

            const PlanePoint3i edgeVertex0 = edge.getStartPoint();
            const PlanePoint3i edgeVertex1 = edge.getEndPoint();

            const PlanePoint3i edgeHit = intersect(seg.direction, edgePlane);
            if (edgeHit.hasUniqueIntersection())
            {
                if (isPointOnSegment(edgeHit, seg) && isPointOnSegment(edgeHit, edge))
                {
                    return true;
                }
                continue;
            }

            // 唯一求交失败时，只额外处理“线段与该边共线”的情况。
            if (startPoint.classify(poly.plane) != 0 || endPoint.classify(poly.plane) != 0 ||
                startPoint.classify(edgePlane) != 0 || endPoint.classify(edgePlane) != 0)
            {
                continue;
            }

            if (isPointOnSegment(edgeVertex0, seg) || isPointOnSegment(edgeVertex1, seg))
            {
                return true;
            }
            if (isPointOnSegment(startPoint, edge) || isPointOnSegment(endPoint, edge))
            {
                return true;
            }
        }

        return false;
    }

    /**
     * @brief 轴对齐整数包围盒。
     *
     * 六个边界均按闭区间存储，且内部点满足：
     * `xMin <= x <= xMax`、`yMin <= y <= yMax`、`zMin <= z <= zMax`。
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
     * @brief AABB 中点切分所选择的轴。
     */
    enum class SplitAxis3i
    {
        X, ///< 沿 x 轴切分。
        Y, ///< 沿 y 轴切分。
        Z  ///< 沿 z 轴切分。
    };

    /**
     * @brief AABB 一次中点切分的结果。
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

    /**
     * @brief subdivision 阶段的局部参考点传播路径候选。
     */
    struct AABBPathCandidate
    {
        PlanePoint3i targetPoint;
        std::vector<Segment256> path;
    };

    namespace detail
    {
        /**
         * @brief 将整数截断到闭区间 `[lower, upper]` 内。
         */
        inline constexpr Integer clampInteger(const Integer &value, const Integer &lower, const Integer &upper) noexcept
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

        /**
         * @brief 读取 AABB 在指定坐标轴上的跨度。
         */
        inline constexpr Integer axisSpan(const AABB3i &box, SplitAxis3i axis) noexcept
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

        /**
         * @brief 为 `SplitAxis3i` 提供稳定的排序编号。
         */
        inline constexpr int axisOrderKey(SplitAxis3i axis) noexcept
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

        /**
         * @brief 尝试从 `PlanePoint3i` 恢复精确整数坐标。
         *
         * @return 仅当三个坐标都能无歧义恢复为整数时返回 `true`。
         */
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

        /**
         * @brief 构造一条单轴变化的轴对齐线段。
         *
         * @return 仅当两端点恰好只在一个坐标轴上不同且线段合法时返回 `true`。
         */
        inline bool buildAxisAlignedSegment(
            const PlanePoint3i &startPoint,
            const PlanePoint3i &endPoint,
            Segment256 &outSegment)
        {
            Integer x0, y0, z0;
            Integer x1, y1, z1;
            if (!tryExtractExactIntegerPoint(startPoint, x0, y0, z0) ||
                !tryExtractExactIntegerPoint(endPoint, x1, y1, z1))
            {
                return false;
            }

            const bool diffX = (x0 != x1);
            const bool diffY = (y0 != y1);
            const bool diffZ = (z0 != z1);
            const int diffCount = static_cast<int>(diffX) + static_cast<int>(diffY) + static_cast<int>(diffZ);
            if (diffCount != 1)
            {
                return false;
            }

            if (diffX)
            {
                outSegment = Segment256(
                    Plane3i(1, 0, 0, -x0),
                    Plane3i(1, 0, 0, -x1),
                    makeLine(Plane3i(0, 1, 0, -y0), Plane3i(0, 0, 1, -z0)));
                return outSegment.isValid();
            }

            if (diffY)
            {
                outSegment = Segment256(
                    Plane3i(0, 1, 0, -y0),
                    Plane3i(0, 1, 0, -y1),
                    makeLine(Plane3i(1, 0, 0, -x0), Plane3i(0, 0, 1, -z0)));
                return outSegment.isValid();
            }

            outSegment = Segment256(
                Plane3i(0, 0, 1, -z0),
                Plane3i(0, 0, 1, -z1),
                makeLine(Plane3i(1, 0, 0, -x0), Plane3i(0, 1, 0, -y0)));
            return outSegment.isValid();
        }

        /**
         * @brief 按给定轴顺序构造一条 Manhattan 风格的角点路径。
         */
        inline bool buildAxisAlignedCornerPath(
            const PlanePoint3i &startPoint,
            const PlanePoint3i &targetPoint,
            const std::vector<SplitAxis3i> &axisOrder,
            std::vector<Segment256> &outPath)
        {
            Integer currentX, currentY, currentZ;
            Integer targetX, targetY, targetZ;
            if (!tryExtractExactIntegerPoint(startPoint, currentX, currentY, currentZ) ||
                !tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ))
            {
                return false;
            }

            outPath.clear();
            PlanePoint3i currentPoint = startPoint;
            for (const SplitAxis3i axis : axisOrder)
            {
                Integer nextX = currentX;
                Integer nextY = currentY;
                Integer nextZ = currentZ;

                switch (axis)
                {
                case SplitAxis3i::X:
                    nextX = targetX;
                    break;
                case SplitAxis3i::Y:
                    nextY = targetY;
                    break;
                case SplitAxis3i::Z:
                    nextZ = targetZ;
                    break;
                }

                if (nextX == currentX && nextY == currentY && nextZ == currentZ)
                {
                    continue;
                }

                const PlanePoint3i nextPoint = makeIntegerPoint(nextX, nextY, nextZ);
                Segment256 segment;
                if (!buildAxisAlignedSegment(currentPoint, nextPoint, segment))
                {
                    outPath.clear();
                    return false;
                }

                outPath.push_back(std::move(segment));
                currentPoint = nextPoint;
                currentX = nextX;
                currentY = nextY;
                currentZ = nextZ;
            }

            return areSamePlanePoint(currentPoint, targetPoint);
        }

        /**
         * @brief 用切分平面把凸多边形裁到某个半空间内。
         *
         * @param[in] keepNonPositive 为 `true` 时保留 `<= 0` 一侧，否则保留 `>= 0` 一侧。
         */
        inline bool clipPolygonToHalfSpace(const Polygon256 &source, const Plane3i &clipPlane, bool keepNonPositive, Polygon256 &outPolygon)
        {
            if (!source.isValid())
            {
                return false;
            }

            const std::size_t n = source.edgeCount();
            bool hasPositive = false;
            bool hasNegative = false;
            bool hasZero = false;

            for (std::size_t i = 0; i < n; ++i)
            {
                const PlanePoint3i vertex = getPolygonVertex(source, i);
                const int side = vertex.classify(clipPlane);
                if (side > 0)
                {
                    hasPositive = true;
                }
                else if (side < 0)
                {
                    hasNegative = true;
                }
                else
                {
                    hasZero = true;
                }
            }

            if (keepNonPositive)
            {
                if (!hasPositive)
                {
                    outPolygon = source;
                    return true;
                }

                if (!hasNegative)
                {
                    return false;
                }
            }
            else
            {
                if (!hasNegative)
                {
                    if (!hasPositive && hasZero)
                    {
                        return false;
                    }

                    outPolygon = source;
                    return true;
                }

                if (!hasPositive)
                {
                    return false;
                }
            }

            Polygon256 frontClipped;
            Polygon256 backClipped;
            if (!clipLeafGeometryByPlane(source, clipPlane, frontClipped, backClipped))
            {
                return false;
            }

            outPolygon = keepNonPositive ? backClipped : frontClipped;
            return outPolygon.isValid();
        }

        /**
         * @brief 选择当前 AABB 中跨度最大的可继续切分轴。
         */
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

    /**
     * @brief 判断 AABB 是否为有效闭区间盒。
     *
     * @param[in] box 输入包围盒。
     * @return 当 `box` 已初始化且六个边界满足最小值不大于最大值时返回 `true`。
     */
    inline constexpr bool isValidAABB(const AABB3i &box) noexcept
    {
        return box.valid &&
               box.xMin <= box.xMax &&
               box.yMin <= box.yMax &&
               box.zMin <= box.zMax;
    }

    /**
     * @brief 构造一个整数笛卡尔点对应的 `PlanePoint3i`。
     *
     * @param[in] x 点的 x 坐标。
     * @param[in] y 点的 y 坐标。
     * @param[in] z 点的 z 坐标。
     * @return 由三个轴对齐平面交出的唯一整数点。
     */
    inline constexpr PlanePoint3i makeIntegerPoint(const Integer &x, const Integer &y, const Integer &z) noexcept
    {
        return PlanePoint3i(
            Plane3i(1, 0, 0, -x),
            Plane3i(0, 1, 0, -y),
            Plane3i(0, 0, 1, -z));
    }

    /**
     * @brief 读取 AABB 的一个角点。
     *
     * @param[in] box 输入包围盒。
     * @param[in] useXMax 为 `true` 时取 `xMax`，否则取 `xMin`。
     * @param[in] useYMax 为 `true` 时取 `yMax`，否则取 `yMin`。
     * @param[in] useZMax 为 `true` 时取 `zMax`，否则取 `zMin`。
     * @return 对应角点；若 `box` 无效则返回默认点。
     */
    inline constexpr PlanePoint3i getAABBCornerPoint(const AABB3i &box, bool useXMax, bool useYMax, bool useZMax) noexcept
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

    /**
     * @brief 将 AABB 转成六个朝向内部为非正侧的平面。
     *
     * @param[in] box 输入包围盒。
     * @return 顺序为 `xmin/xmax/ymin/ymax/zmin/zmax` 的六个平面。
     */
    inline constexpr std::array<Plane3i, 6> makeAABBPlanes(const AABB3i &box) noexcept
    {
        return {
            Plane3i(-1, 0, 0, box.xMin),
            Plane3i(1, 0, 0, -box.xMax),
            Plane3i(0, -1, 0, box.yMin),
            Plane3i(0, 1, 0, -box.yMax),
            Plane3i(0, 0, -1, box.zMin),
            Plane3i(0, 0, 1, -box.zMax)};
    }

    /**
     * @brief 计算 polygon soup 的整体 AABB。
     *
     * @param[in] polygons 输入多边形集合。
     * @param[in] margin 在各方向额外扩出的整数边距，默认值为 `1`。
     * @return 若能从有效顶点恢复边界则返回有效 AABB，否则返回 `valid == false` 的结果。
     */
    inline AABB3i computeAABB(const std::vector<Polygon256> &polygons, const Integer &margin = 1)
    {
        AABB3i box;
        bool initialized = false;

        for (const auto &poly : polygons)
        {
            const std::size_t n = poly.edgePlanes.size();
            for (std::size_t i = 0; i < n; ++i)
            {
                const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
                const HomPoint4i v = intersectHomogeneous(poly.plane, poly.edgePlanes[i], poly.edgePlanes[prev]);

                if (isZero(v.w))
                {
                    continue;
                }

                const Integer fx = floorDiv(v.x, v.w);
                const Integer cx = ceilDiv(v.x, v.w);
                const Integer fy = floorDiv(v.y, v.w);
                const Integer cy = ceilDiv(v.y, v.w);
                const Integer fz = floorDiv(v.z, v.w);
                const Integer cz = ceilDiv(v.z, v.w);

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
                    if (fx < box.xMin)
                    {
                        box.xMin = fx;
                    }
                    if (cx > box.xMax)
                    {
                        box.xMax = cx;
                    }
                    if (fy < box.yMin)
                    {
                        box.yMin = fy;
                    }
                    if (cy > box.yMax)
                    {
                        box.yMax = cy;
                    }
                    if (fz < box.zMin)
                    {
                        box.zMin = fz;
                    }
                    if (cz > box.zMax)
                    {
                        box.zMax = cz;
                    }
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

    /**
     * @brief 兼容旧接口，返回整体 AABB 的六个包围平面。
     *
     * @param[in] polygons 输入多边形集合。
     * @return 当 AABB 有效时返回 6 个轴对齐平面，否则返回空数组。
     */
    inline std::vector<Plane3i> computeAABBPlanes(const std::vector<Polygon256> &polygons)
    {
        const AABB3i box = computeAABB(polygons);
        if (!isValidAABB(box))
        {
            return {};
        }

        const auto planes = makeAABBPlanes(box);
        return std::vector<Plane3i>(planes.begin(), planes.end());
    }

    /**
     * @brief 判断点是否在 AABB 内部或边界上。
     *
     * @param[in] point 查询点。
     * @param[in] box 查询包围盒。
     * @return 当点对六个边界平面的分类都不为正时返回 `true`。
     */
    inline constexpr bool isPointInsideOrOnAABB(const PlanePoint3i &point, const AABB3i &box) noexcept
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

    /**
     * @brief 判断 AABB 是否还存在可继续二分的轴。
     *
     * @param[in] box 输入包围盒。
     * @return 当至少一条轴向跨度大于 `1` 时返回 `true`。
     */
    inline bool hasSplittableAxis(const AABB3i &box) noexcept
    {
        SplitAxis3i axis = SplitAxis3i::X;
        return isValidAABB(box) && detail::chooseLongestSplittableAxis(box, axis);
    }

    /**
     * @brief 按最长可切分轴的整数中点切分 AABB。
     *
     * @param[in] box 输入包围盒。
     * @param[out] outSplit 输出切分结果。
     * @return 当存在跨度大于 `1` 的轴并成功构造左右子盒时返回 `true`。
     * @note 左盒保留切分平面本身，右盒同样保留该平面，符合论文中“平面上的面归左侧”的约定。
     */
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
        AABB3i left = box;
        AABB3i right = box;
        Plane3i splitPlane;
        Integer coordinate = 0;

        switch (axis)
        {
        case SplitAxis3i::X:
            coordinate = floorDiv(box.xMin + box.xMax, two);
            if (coordinate <= box.xMin || coordinate >= box.xMax)
            {
                return false;
            }
            left.xMax = coordinate;
            right.xMin = coordinate;
            splitPlane = Plane3i(1, 0, 0, -coordinate);
            break;
        case SplitAxis3i::Y:
            coordinate = floorDiv(box.yMin + box.yMax, two);
            if (coordinate <= box.yMin || coordinate >= box.yMax)
            {
                return false;
            }
            left.yMax = coordinate;
            right.yMin = coordinate;
            splitPlane = Plane3i(0, 1, 0, -coordinate);
            break;
        case SplitAxis3i::Z:
            coordinate = floorDiv(box.zMin + box.zMax, two);
            if (coordinate <= box.zMin || coordinate >= box.zMax)
            {
                return false;
            }
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

    /**
     * @brief 将点逐坐标投影到目标 AABB 内。
     *
     * @param[in] point 需要投影的点。
     * @param[in] box 目标包围盒。
     * @return 若输入有效，则返回逐坐标 clamp 后的整数点。
     * @note subdivision 中会优先尝试以该投影点为目标构造单段传播路径。
     */
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

    /**
     * @brief 将一个凸多边形裁到 AABB 内部。
     *
     * @param[in] polygon 输入多边形。
     * @param[in] box 目标包围盒。
     * @param[out] outPolygon 裁剪后的多边形。
     * @return 当裁剪结果仍为非空多边形时返回 `true`。
     */
    inline bool clipPolygonToAABB(const Polygon256 &polygon, const AABB3i &box, Polygon256 &outPolygon)
    {
        if (!polygon.isValid() || !isValidAABB(box))
        {
            return false;
        }

        Polygon256 current = polygon;
        const auto planes = makeAABBPlanes(box);
        for (const Plane3i &plane : planes)
        {
            Polygon256 clipped;
            if (!detail::clipPolygonToHalfSpace(current, plane, true, clipped))
            {
                return false;
            }
            current = clipped;
        }

        outPolygon = current;
        return outPolygon.isValid();
    }

    /**
     * @brief 枚举从当前参考点到目标子 AABB 的轴对齐传播路径候选。
     *
     * @param[in] startPoint 已知 WNV 的当前参考点，要求为精确整数点。
     * @param[in] targetBox 目标子 AABB。
     * @return 结果按“投影目标优先、离投影点更近优先、路径段数更少优先”排序。
     * @note subdivision 阶段的路径构造只使用 AABB 面和切分平面来定义轴对齐线段，
     *       不使用通用的换平面路径构造。
     */
    inline std::vector<AABBPathCandidate> enumerateAABBPathCandidates(const PlanePoint3i &startPoint, const AABB3i &targetBox)
    {
        std::vector<AABBPathCandidate> candidates;
        if (!startPoint.hasUniqueIntersection() || !isValidAABB(targetBox))
        {
            return candidates;
        }

        const PlanePoint3i preferredTarget = projectPointToAABB(startPoint, targetBox);

        Integer startX, startY, startZ;
        Integer preferredX, preferredY, preferredZ;
        if (!detail::tryExtractExactIntegerPoint(startPoint, startX, startY, startZ) ||
            !detail::tryExtractExactIntegerPoint(preferredTarget, preferredX, preferredY, preferredZ))
        {
            return candidates;
        }

        std::vector<Integer> xChoices;
        std::vector<Integer> yChoices;
        std::vector<Integer> zChoices;

        xChoices.push_back(preferredX);
        yChoices.push_back(preferredY);
        zChoices.push_back(preferredZ);

        for (const Integer &delta : {Integer(1), Integer(2)})
        {
            if (preferredX - delta >= targetBox.xMin)
            {
                xChoices.push_back(preferredX - delta);
            }
            if (preferredX + delta <= targetBox.xMax)
            {
                xChoices.push_back(preferredX + delta);
            }
            if (preferredY - delta >= targetBox.yMin)
            {
                yChoices.push_back(preferredY - delta);
            }
            if (preferredY + delta <= targetBox.yMax)
            {
                yChoices.push_back(preferredY + delta);
            }
            if (preferredZ - delta >= targetBox.zMin)
            {
                zChoices.push_back(preferredZ - delta);
            }
            if (preferredZ + delta <= targetBox.zMax)
            {
                zChoices.push_back(preferredZ + delta);
            }
        }

        xChoices.push_back(targetBox.xMin);
        xChoices.push_back(targetBox.xMax);
        yChoices.push_back(targetBox.yMin);
        yChoices.push_back(targetBox.yMax);
        zChoices.push_back(targetBox.zMin);
        zChoices.push_back(targetBox.zMax);

        std::sort(xChoices.begin(), xChoices.end());
        xChoices.erase(std::unique(xChoices.begin(), xChoices.end()), xChoices.end());
        std::sort(yChoices.begin(), yChoices.end());
        yChoices.erase(std::unique(yChoices.begin(), yChoices.end()), yChoices.end());
        std::sort(zChoices.begin(), zChoices.end());
        zChoices.erase(std::unique(zChoices.begin(), zChoices.end()), zChoices.end());

        std::vector<PlanePoint3i> targetPoints;
        for (const Integer &xChoice : xChoices)
        {
            for (const Integer &yChoice : yChoices)
            {
                for (const Integer &zChoice : zChoices)
                {
                    const PlanePoint3i targetPoint = makeIntegerPoint(xChoice, yChoice, zChoice);

                    bool duplicated = false;
                    for (const PlanePoint3i &existing : targetPoints)
                    {
                        if (areSamePlanePoint(existing, targetPoint))
                        {
                            duplicated = true;
                            break;
                        }
                    }

                    if (!duplicated)
                    {
                        targetPoints.push_back(targetPoint);
                    }
                }
            }
        }

        for (const PlanePoint3i &targetPoint : targetPoints)
        {
            Integer targetX, targetY, targetZ;
            detail::tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ);

            std::vector<SplitAxis3i> changedAxes;
            changedAxes.reserve(3);
            if (startX != targetX)
            {
                changedAxes.push_back(SplitAxis3i::X);
            }
            if (startY != targetY)
            {
                changedAxes.push_back(SplitAxis3i::Y);
            }
            if (startZ != targetZ)
            {
                changedAxes.push_back(SplitAxis3i::Z);
            }

            std::sort(
                changedAxes.begin(),
                changedAxes.end(),
                [](SplitAxis3i lhs, SplitAxis3i rhs)
                {
                    return detail::axisOrderKey(lhs) < detail::axisOrderKey(rhs);
                });

            if (changedAxes.empty())
            {
                candidates.push_back({targetPoint, {}});
                continue;
            }

            do
            {
                std::vector<Segment256> path;
                if (detail::buildAxisAlignedCornerPath(startPoint, targetPoint, changedAxes, path))
                {
                    candidates.push_back({targetPoint, std::move(path)});
                }
            } while (std::next_permutation(
                changedAxes.begin(),
                changedAxes.end(),
                [](SplitAxis3i lhs, SplitAxis3i rhs)
                {
                    return detail::axisOrderKey(lhs) < detail::axisOrderKey(rhs);
                }));
        }

        std::stable_sort(
            candidates.begin(),
            candidates.end(),
            [preferredX, preferredY, preferredZ](const AABBPathCandidate &lhs, const AABBPathCandidate &rhs)
            {
                Integer lhsX, lhsY, lhsZ;
                Integer rhsX, rhsY, rhsZ;
                detail::tryExtractExactIntegerPoint(lhs.targetPoint, lhsX, lhsY, lhsZ);
                detail::tryExtractExactIntegerPoint(rhs.targetPoint, rhsX, rhsY, rhsZ);

                const bool lhsPreferred = (lhsX == preferredX && lhsY == preferredY && lhsZ == preferredZ);
                const bool rhsPreferred = (rhsX == preferredX && rhsY == preferredY && rhsZ == preferredZ);
                if (lhsPreferred != rhsPreferred)
                {
                    return lhsPreferred;
                }

                const Integer lhsDx = lhsX - preferredX;
                const Integer lhsDy = lhsY - preferredY;
                const Integer lhsDz = lhsZ - preferredZ;
                const Integer rhsDx = rhsX - preferredX;
                const Integer rhsDy = rhsY - preferredY;
                const Integer rhsDz = rhsZ - preferredZ;

                const Integer lhsDistance =
                    (lhsDx < 0 ? -lhsDx : lhsDx) +
                    (lhsDy < 0 ? -lhsDy : lhsDy) +
                    (lhsDz < 0 ? -lhsDz : lhsDz);
                const Integer rhsDistance =
                    (rhsDx < 0 ? -rhsDx : rhsDx) +
                    (rhsDy < 0 ? -rhsDy : rhsDy) +
                    (rhsDz < 0 ? -rhsDz : rhsDz);
                if (lhsDistance != rhsDistance)
                {
                    return lhsDistance < rhsDistance;
                }

                return lhs.path.size() < rhs.path.size();
            });

        return candidates;
    }

    /**
     * @brief 将多边形集合逐个裁到 AABB 内部。
     *
     * @param[in] polygons 输入多边形集合。
     * @param[in] box 目标包围盒。
     * @return 仅包含裁后仍为非空多边形的结果集合。
     */
    inline std::vector<Polygon256> clipPolygonsToAABB(const std::vector<Polygon256> &polygons, const AABB3i &box)
    {
        std::vector<Polygon256> outPolygons;
        outPolygons.reserve(polygons.size());

        for (const Polygon256 &polygon : polygons)
        {
            Polygon256 clipped;
            if (clipPolygonToAABB(polygon, box, clipped))
            {
                outPolygons.push_back(std::move(clipped));
            }
        }

        return outPolygons;
    }
}
