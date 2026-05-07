/**
 * @file path_candidate_details.h
 * @brief 提供 AABB 与 leaf 分类路径候选的内部辅助实现。
 */
#pragma once

#include "algorithm/tracing_geometry.h"
#include "core/perf_tracing.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <utility>
#include <vector>

namespace ember
{
namespace detail
{
inline Plane3i makeCoordinatePlaneFromPoint(const PlanePoint3i &point, SplitAxis3i axis) noexcept;
inline bool buildPlaneReplacementSegment(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &endPoint,
    int replacedPlaneIndex,
    Segment256 &outSegment);
inline bool buildAnyBoundedPlaneReplacementBridgePath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    std::vector<Segment256> &outPath);
inline bool buildAxisAlignedSegment(
    const Integer &x0,
    const Integer &y0,
    const Integer &z0,
    const Integer &x1,
    const Integer &y1,
    const Integer &z1,
    Segment256 &outSegment)
{
    const bool diffX = (x0 != x1);
    const bool diffY = (y0 != y1);
    const bool diffZ = (z0 != z1);
    const int diffCount = static_cast<int>(diffX) + static_cast<int>(diffY) + static_cast<int>(diffZ);
    if (diffCount != 1)
        return false;

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

inline bool isIntegerPointInsideOrOnAABB(
    const Integer &x,
    const Integer &y,
    const Integer &z,
    const AABB3i &box) noexcept
{
    return isValidAABB(box) &&
           x >= box.xMin && x <= box.xMax &&
           y >= box.yMin && y <= box.yMax &&
           z >= box.zMin && z <= box.zMax;
}

inline bool buildAxisAlignedCornerPathFromIntegers(
    Integer currentX,
    Integer currentY,
    Integer currentZ,
    const Integer &targetX,
    const Integer &targetY,
    const Integer &targetZ,
    const std::array<SplitAxis3i, 3> &axisOrder,
    std::size_t axisCount,
    const AABB3i *box,
    std::vector<Segment256> &outPath)
{
    if (box != nullptr &&
            (!isIntegerPointInsideOrOnAABB(currentX, currentY, currentZ, *box) ||
             !isIntegerPointInsideOrOnAABB(targetX, targetY, targetZ, *box)))
        return false;

    outPath.clear();
    for (std::size_t axisIndex = 0; axisIndex < axisCount; ++axisIndex)
    {
        const SplitAxis3i axis = axisOrder[axisIndex];
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
            continue;

        if (box != nullptr && !isIntegerPointInsideOrOnAABB(nextX, nextY, nextZ, *box))
        {
            outPath.clear();
            return false;
        }

        Segment256 segment;
        if (!buildAxisAlignedSegment(currentX, currentY, currentZ, nextX, nextY, nextZ, segment))
        {
            outPath.clear();
            return false;
        }

        outPath.push_back(std::move(segment));
        currentX = nextX;
        currentY = nextY;
        currentZ = nextZ;
    }

    return currentX == targetX && currentY == targetY && currentZ == targetZ;
}

/**
 * @brief 按给定轴顺序构造一条曼哈顿风格的角点路径。
 */
inline bool buildAxisAlignedCornerPath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const std::array<SplitAxis3i, 3> &axisOrder,
    std::size_t axisCount,
    std::vector<Segment256> &outPath)
{
    Integer currentX, currentY, currentZ;
    Integer targetX, targetY, targetZ;
    if (!tryExtractExactIntegerPoint(startPoint, currentX, currentY, currentZ) ||
            !tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ))
        return false;

    return buildAxisAlignedCornerPathFromIntegers(
               currentX,
               currentY,
               currentZ,
               targetX,
               targetY,
               targetZ,
               axisOrder,
               axisCount,
               nullptr,
               outPath);
}

inline bool buildAxisAlignedCornerPath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::array<SplitAxis3i, 3> &axisOrder,
    std::size_t axisCount,
    std::vector<Segment256> &outPath)
{
    Integer currentX, currentY, currentZ;
    Integer targetX, targetY, targetZ;
    if (!tryExtractExactIntegerPoint(startPoint, currentX, currentY, currentZ) ||
            !tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ))
        return false;

    return buildAxisAlignedCornerPathFromIntegers(
               currentX,
               currentY,
               currentZ,
               targetX,
               targetY,
               targetZ,
               axisOrder,
               axisCount,
               &box,
               outPath);
}

inline bool buildAxisAlignedCornerPath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const std::vector<SplitAxis3i> &axisOrder,
    std::vector<Segment256> &outPath)
{
    std::array<SplitAxis3i, 3> axisOrderArray = {};
    const std::size_t axisCount = std::min<std::size_t>(axisOrder.size(), axisOrderArray.size());
    for (std::size_t axisIndex = 0; axisIndex < axisCount; ++axisIndex)
        axisOrderArray[axisIndex] = axisOrder[axisIndex];

    return buildAxisAlignedCornerPath(startPoint, targetPoint, axisOrderArray, axisCount, outPath);
}

/**
 * @brief 按给定轴顺序，从多边形包围盒中心附近构造轴对齐探测线上的严格内部点。
 *
 * @param[in] polygon 待分类的 leaf polygon。
 * @param[in] axis 作为探测线方向的坐标轴。
 * @param[in] coord0 第一条正交轴的整数坐标。
 * @param[in] coord1 第二条正交轴的整数坐标。
 * @param[out] outPoint 当成功时写入严格内部点。
 * @return 当探测线与多边形支撑平面唯一相交且交点严格落在内部时返回 `true`。
 */
inline bool buildAxisProbeInteriorPoint(
    const Polygon256 &polygon,
    SplitAxis3i axis,
    const Plane3i &plane0,
    const Plane3i &plane1,
    PlanePoint3i &outPoint)
{
    Line256 probeLine;
    switch (axis)
    {
    case SplitAxis3i::X:
        probeLine = makeLine(plane0, plane1);
        break;
    case SplitAxis3i::Y:
        probeLine = makeLine(plane0, plane1);
        break;
    case SplitAxis3i::Z:
        probeLine = makeLine(plane0, plane1);
        break;
    }

    const PlanePoint3i candidate = intersect(probeLine, polygon.plane);
    if (!candidate.hasUniqueIntersection() || !polygon.containsStrictly(candidate))
        return false;

    outPoint = candidate;
    return true;
}

/**
 * @brief 向候选列表追加一个不重复的严格内部点。
 */
inline bool appendUniqueStrictInteriorPoint(
    std::vector<PlanePoint3i> &candidates,
    const Polygon256 &polygon,
    const PlanePoint3i &candidate)
{
    if (!polygon.containsStrictly(candidate))
        return false;

    for (const PlanePoint3i &existing : candidates)
    {
        if (areSamePlanePoint(existing, candidate))
            return false;
    }

    candidates.push_back(candidate);
    return true;
}

/**
 * @brief 对有理坐标执行确定性的最近整数舍入。
 */
inline Integer roundDivNearest(Integer numerator, Integer denominator) noexcept
{
    if (denominator < 0)
    {
        numerator = -numerator;
        denominator = -denominator;
    }
    if (isZero(denominator))
        return 0;

    const Integer half = denominator / 2;
    if (numerator >= 0)
        return (numerator + half) / denominator;

    return -((-numerator + half) / denominator);
}

inline Integer absInteger(const Integer &value) noexcept
{
    return value < 0 ? -value : value;
}

inline bool canScalePlaneWithinInsetHeadroom(const Plane3i &plane, const Integer &scale) noexcept
{
    if (scale <= 0)
        return false;

    const Integer coefficientLimit = Integer(1) << 70;
    return absInteger(plane.a) <= coefficientLimit / scale &&
           absInteger(plane.b) <= coefficientLimit / scale &&
           absInteger(plane.c) <= coefficientLimit / scale &&
           absInteger(plane.d) <= coefficientLimit / scale;
}

inline bool isZeroPlaneEquation(const Plane3i &plane) noexcept
{
    return isZero(plane.a) && isZero(plane.b) && isZero(plane.c) && isZero(plane.d);
}

inline Plane3i subtractPlaneEquation(const Plane3i &lhs, const Plane3i &rhs) noexcept
{
    return primitivePlane(Plane3i(
                              lhs.a - rhs.a,
                              lhs.b - rhs.b,
                              lhs.c - rhs.c,
                              lhs.d - rhs.d));
}

inline PlanePoint3i makePointFromHomogeneousCoordinates(const HomPoint4i &point) noexcept
{
    if (isZero(point.w))
        return PlanePoint3i();

    return PlanePoint3i(
               primitivePlane(Plane3i(point.w, 0, 0, -point.x)),
               primitivePlane(Plane3i(0, point.w, 0, -point.y)),
               primitivePlane(Plane3i(0, 0, point.w, -point.z)));
}

/**
 * @brief 以顶点坐标均值的整数舍入值近似论文中的浮点重心猜测。
 */
inline bool buildRoundedCentroidPoint(const Polygon256 &polygon, PlanePoint3i &outPoint)
{
    const std::vector<PlanePoint3i> &cachedVertices = polygon.vertices();
    const std::size_t n = cachedVertices.size();
    if (n < 3)
        return false;

    Integer xSum = 0;
    Integer ySum = 0;
    Integer zSum = 0;
    for (const PlanePoint3i &vertex : cachedVertices)
    {
        if (!vertex.hasUniqueIntersection() || isZero(vertex.x.w))
            return false;

        xSum += roundDivNearest(vertex.x.x, vertex.x.w);
        ySum += roundDivNearest(vertex.x.y, vertex.x.w);
        zSum += roundDivNearest(vertex.x.z, vertex.x.w);
    }

    const Integer count = Integer(static_cast<int>(n));
    outPoint = makeIntegerPoint(
                   roundDivNearest(xSum, count),
                   roundDivNearest(ySum, count),
                   roundDivNearest(zSum, count));
    return outPoint.hasUniqueIntersection();
}

/**
 * @brief 按论文第一启发式追加“重心舍入点 + 最不平行轴探测线”的内部点。
 */
inline void appendCentroidProbeCandidates(const Polygon256 &polygon, std::vector<PlanePoint3i> &candidates)
{
    PlanePoint3i centroid;
    if (!buildRoundedCentroidPoint(polygon, centroid))
        return;

    const Plane3i centroidXPlane = makeCoordinatePlaneFromPoint(centroid, SplitAxis3i::X);
    const Plane3i centroidYPlane = makeCoordinatePlaneFromPoint(centroid, SplitAxis3i::Y);
    const Plane3i centroidZPlane = makeCoordinatePlaneFromPoint(centroid, SplitAxis3i::Z);

    const Integer absNx = polygon.plane.a < 0 ? -polygon.plane.a : polygon.plane.a;
    const Integer absNy = polygon.plane.b < 0 ? -polygon.plane.b : polygon.plane.b;
    const Integer absNz = polygon.plane.c < 0 ? -polygon.plane.c : polygon.plane.c;

    std::vector<std::pair<Integer, SplitAxis3i>> axisPriority = {
        {absNx, SplitAxis3i::X},
        {absNy, SplitAxis3i::Y},
        {absNz, SplitAxis3i::Z}
    };
    std::stable_sort(
        axisPriority.begin(),
        axisPriority.end(),
        [](const auto &lhs, const auto &rhs)
    {
        return lhs.first > rhs.first;
    });

    for (const auto &[_, axis] : axisPriority)
    {
        PlanePoint3i candidate;
        switch (axis)
        {
        case SplitAxis3i::X:
            if (buildAxisProbeInteriorPoint(polygon, axis, centroidYPlane, centroidZPlane, candidate))
                appendUniqueStrictInteriorPoint(candidates, polygon, candidate);
            break;
        case SplitAxis3i::Y:
            if (buildAxisProbeInteriorPoint(polygon, axis, centroidXPlane, centroidZPlane, candidate))
                appendUniqueStrictInteriorPoint(candidates, polygon, candidate);
            break;
        case SplitAxis3i::Z:
            if (buildAxisProbeInteriorPoint(polygon, axis, centroidXPlane, centroidYPlane, candidate))
                appendUniqueStrictInteriorPoint(candidates, polygon, candidate);
            break;
        }
    }
}

/**
 * @brief 按相邻边平面向内偏移的论文兜底策略构造内部点候选。
 */
inline bool collectFinitePolygonVertices(
    const Polygon256 &polygon,
    std::vector<PlanePoint3i> &vertices)
{
    const std::vector<PlanePoint3i> &cachedVertices = polygon.vertices();
    const std::size_t edgeCount = cachedVertices.size();
    vertices.clear();
    vertices.reserve(edgeCount);
    for (const PlanePoint3i &vertex : cachedVertices)
    {
        if (!vertex.hasUniqueIntersection())
            return false;

        vertices.push_back(vertex);
    }

    return true;
}

inline std::size_t findInsetReferenceVertex(std::size_t edgeIndex, std::size_t edgeCount) noexcept
{
    const std::size_t next = (edgeIndex + 1 == edgeCount) ? 0 : (edgeIndex + 1);
    for (std::size_t vertexIndex = 0; vertexIndex < edgeCount; ++vertexIndex)
    {
        if (vertexIndex != edgeIndex && vertexIndex != next)
            return vertexIndex;
    }

    return edgeCount;
}

inline bool buildScaledInsetEdgePlanes(
    const Polygon256 &polygon,
    std::size_t edgeIndex,
    std::size_t prevEdgeIndex,
    const Integer &scale,
    Plane3i &scaledA,
    Plane3i &scaledB) noexcept
{
    if (!canScalePlaneWithinInsetHeadroom(polygon.edgePlanes[edgeIndex], scale) ||
            !canScalePlaneWithinInsetHeadroom(polygon.edgePlanes[prevEdgeIndex], scale))
        return false;

    scaledA = Plane3i(
                  polygon.edgePlanes[edgeIndex].a * scale,
                  polygon.edgePlanes[edgeIndex].b * scale,
                  polygon.edgePlanes[edgeIndex].c * scale,
                  polygon.edgePlanes[edgeIndex].d * scale);
    scaledB = Plane3i(
                  polygon.edgePlanes[prevEdgeIndex].a * scale,
                  polygon.edgePlanes[prevEdgeIndex].b * scale,
                  polygon.edgePlanes[prevEdgeIndex].c * scale,
                  polygon.edgePlanes[prevEdgeIndex].d * scale);
    return true;
}

inline void appendInsetCandidatesForEdge(
    const Polygon256 &polygon,
    const std::vector<PlanePoint3i> &vertices,
    std::size_t edgeIndex,
    const Integer &scale,
    const std::array<Integer, 4> &offsets,
    std::vector<PlanePoint3i> &candidates,
    std::size_t maxTotalCandidates,
    bool &anyEdgeWithinHeadroom)
{
    if (candidates.size() >= maxTotalCandidates)
        return;

    const std::size_t edgeCount = polygon.edgePlanes.size();
    const std::size_t prevEdgeIndex = (edgeIndex == 0) ? (edgeCount - 1) : (edgeIndex - 1);
    const std::size_t refIdxA = findInsetReferenceVertex(edgeIndex, edgeCount);
    const std::size_t refIdxB = findInsetReferenceVertex(prevEdgeIndex, edgeCount);
    if (refIdxA == edgeCount || refIdxB == edgeCount)
        return;

    Plane3i scaledA;
    Plane3i scaledB;
    if (!buildScaledInsetEdgePlanes(polygon, edgeIndex, prevEdgeIndex, scale, scaledA, scaledB))
        return;

    anyEdgeWithinHeadroom = true;
    int interiorSideA = -1;
    int interiorSideB = -1;
    for (const Integer &offsetA : offsets)
    {
        if (candidates.size() >= maxTotalCandidates)
            break;

        Plane3i insetA = scaledA;
        insetA.d -= Integer(interiorSideA) * offsetA;
        if (vertices[refIdxA].classify(insetA) != interiorSideA)
            continue;

        for (const Integer &offsetB : offsets)
        {
            if (candidates.size() >= maxTotalCandidates)
                break;

            Plane3i insetB = scaledB;
            insetB.d -= Integer(interiorSideB) * offsetB;
            if (vertices[refIdxB].classify(insetB) != interiorSideB ||
                    !hasUniqueIntersection(polygon.plane, insetA, insetB))
                continue;

            const PlanePoint3i candidate(polygon.plane, insetA, insetB);
            appendUniqueStrictInteriorPoint(candidates, polygon, candidate);
        }
    }
}

inline void appendInsetInteriorPointCandidates(const Polygon256 &polygon, std::vector<PlanePoint3i> &candidates)
{
    std::size_t maxTotalCandidates = 32;
    const std::size_t n = polygon.edgePlanes.size();
    if (n < 3)
        return;

    std::vector<PlanePoint3i> vertices;
    if (!collectFinitePolygonVertices(polygon, vertices))
        return;

    const std::array<Integer, 4> offsets = {
        Integer(1),
        Integer(2),
        Integer(4),
        Integer(8)
    };

    Integer scale = 1;
    for (int scaleIter = 0; scaleIter < 40 && candidates.size() < maxTotalCandidates; ++scaleIter)
    {
        bool anyEdgeWithinHeadroom = false;
        for (std::size_t i = 0; i < n; ++i)
        {
            appendInsetCandidatesForEdge(
                polygon,
                vertices,
                i,
                scale,
                offsets,
                candidates,
                maxTotalCandidates,
                anyEdgeWithinHeadroom);
        }

        if (!anyEdgeWithinHeadroom)
            break;
        scale += scale;
    }
}

/**
 * @brief 用所有顶点的齐次坐标正权重求和，构造一个精确严格内部点。
 *
 * 有效严格凸多边形的每条边至少有一个非邻接顶点在负侧。所有有限顶点
 * 以正权重相加后仍在支撑平面内，并且对每条边的符号严格为负。
 */
inline void appendHomogeneousVertexAverageInteriorPointCandidate(
    const Polygon256 &polygon,
    std::vector<PlanePoint3i> &candidates)
{
    const std::vector<PlanePoint3i> &cachedVertices = polygon.vertices();
    const std::size_t n = cachedVertices.size();
    if (n < 3)
        return;

    HomPoint4i average;
    for (const PlanePoint3i &vertex : cachedVertices)
    {
        if (!vertex.hasUniqueIntersection() || vertex.x.w <= 0)
            return;

        average.x += vertex.x.x;
        average.y += vertex.x.y;
        average.z += vertex.x.z;
        average.w += vertex.x.w;
        average = primitiveHomPoint(average);
    }

    const PlanePoint3i candidate = makePointFromHomogeneousCoordinates(average);
    appendUniqueStrictInteriorPoint(candidates, polygon, candidate);
}

/**
 * @brief 枚举若干组边平面函数值相等的精确内部点候选。
 *
 * 对极薄或远离原点的片段，整数 `+1` inset 可能因 headroom 限制仍过粗。
 * 这里不放大原平面系数，而是在支撑平面内求 `e_i = e_j = e_k` 的点，
 * 再统一交给 `containsStrictly()` 过滤，避免把边界点当作分类目标。
 */
inline void appendEqualizedEdgeInteriorPointCandidates(
    const Polygon256 &polygon,
    std::vector<PlanePoint3i> &candidates)
{
    std::size_t maxTotalCandidates = 64;
    const std::size_t n = polygon.edgePlanes.size();
    if (n < 3)
        return;

    for (std::size_t i = 0; i + 2 < n && candidates.size() < maxTotalCandidates; ++i)
    {
        for (std::size_t j = i + 1; j + 1 < n && candidates.size() < maxTotalCandidates; ++j)
        {
            for (std::size_t k = j + 1; k < n && candidates.size() < maxTotalCandidates; ++k)
            {
                const Plane3i firstEqualizer =
                    subtractPlaneEquation(polygon.edgePlanes[i], polygon.edgePlanes[j]);
                const Plane3i secondEqualizer =
                    subtractPlaneEquation(polygon.edgePlanes[j], polygon.edgePlanes[k]);
                if (isZeroPlaneEquation(firstEqualizer) || isZeroPlaneEquation(secondEqualizer))
                    continue;
                if (!hasUniqueIntersection(polygon.plane, firstEqualizer, secondEqualizer))
                    continue;

                const PlanePoint3i candidate(polygon.plane, firstEqualizer, secondEqualizer);
                appendUniqueStrictInteriorPoint(candidates, polygon, candidate);
            }
        }
    }
}

/**
 * @brief 用三个平面恢复一个 `PlanePoint3i`。
 */
inline PlanePoint3i makePointFromPlanes(const std::array<Plane3i, 3> &planes) noexcept
{
    return PlanePoint3i(planes[0], planes[1], planes[2]);
}

/**
 * @brief 为一个齐次点的指定坐标构造轴对齐坐标平面。
 *
 * 若该点某坐标为 `p / w`，则返回 `w * x - p = 0` 一类平面。
 */
inline Plane3i makeCoordinatePlaneFromPoint(const PlanePoint3i &point, SplitAxis3i axis) noexcept
{
    if (!point.hasUniqueIntersection() || isZero(point.x.w))
        return Plane3i();

    switch (axis)
    {
    case SplitAxis3i::X:
        return primitivePlane(Plane3i(point.x.w, 0, 0, -point.x.x));
    case SplitAxis3i::Y:
        return primitivePlane(Plane3i(0, point.x.w, 0, -point.x.y));
    case SplitAxis3i::Z:
        return primitivePlane(Plane3i(0, 0, point.x.w, -point.x.z));
    }

    return Plane3i();
}

/**
 * @brief 按轴对齐坐标平面构造一段单轴变化的路径线段。
 */
inline bool buildAxisAlignedSegmentFromCoordinatePlanes(
    const std::array<Plane3i, 3> &startCoordinatePlanes,
    const std::array<Plane3i, 3> &endCoordinatePlanes,
    SplitAxis3i changedAxis,
    Segment256 &outSegment)
{
    std::array<int, 2> fixedIndices = {1, 2};
    if (changedAxis == SplitAxis3i::Y)
    {
        fixedIndices = {0, 2};
    }
    else if (changedAxis == SplitAxis3i::Z)
    {
        fixedIndices = {0, 1};
    }

    const Line256 direction(
        startCoordinatePlanes[fixedIndices[0]],
        startCoordinatePlanes[fixedIndices[1]]);
    if (!direction.isValid())
        return false;

    const int changedIndex = axisOrderKey(changedAxis);
    outSegment = Segment256(
                     startCoordinatePlanes[changedIndex],
                     endCoordinatePlanes[changedIndex],
                     direction);
    return outSegment.isValid();
}

/**
 * @brief 按坐标轴顺序构造 1 到 3 段轴对齐路径。
 */
inline bool buildAxisAlignedCoordinatePath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::vector<SplitAxis3i> &axisOrder,
    std::vector<Segment256> &outPath)
{
    REEMBER_PROFILE_ZONE("buildAxisAlignedCoordinatePath");

    std::array<Plane3i, 3> currentCoordinatePlanes = {
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::X),
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Y),
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Z)
    };
    const std::array<Plane3i, 3> targetCoordinatePlanes = {
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::X),
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Y),
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Z)
    };

    PlanePoint3i currentPoint = startPoint;
    outPath.clear();
    for (const SplitAxis3i axis : axisOrder)
    {
        REEMBER_PROFILE_ZONE("buildAxisAlignedCoordinatePath::axisStep");
        const int axisIndex = axisOrderKey(axis);
        const std::array<Plane3i, 3> startCoordinatePlanes = currentCoordinatePlanes;
        currentCoordinatePlanes[axisIndex] = targetCoordinatePlanes[axisIndex];

        const PlanePoint3i nextPoint = makePointFromPlanes(currentCoordinatePlanes);
        if (!nextPoint.hasUniqueIntersection() || !isPointInsideOrOnAABB(nextPoint, box))
        {
            outPath.clear();
            return false;
        }

        Segment256 segment;
        if (!buildAxisAlignedSegmentFromCoordinatePlanes(startCoordinatePlanes, currentCoordinatePlanes, axis, segment))
        {
            outPath.clear();
            return false;
        }

        outPath.push_back(std::move(segment));
        currentPoint = nextPoint;
    }

    return areSamePlanePoint(currentPoint, targetPoint);
}

/**
 * @brief 构造不受某个 AABB 约束的轴对齐齐次坐标路径。
 */
inline bool buildAxisAlignedFreeCoordinatePath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const std::array<SplitAxis3i, 3> &axisOrder,
    std::size_t axisCount,
    std::vector<Segment256> &outPath)
{
    std::array<Plane3i, 3> currentCoordinatePlanes = {
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::X),
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Y),
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Z)
    };
    const std::array<Plane3i, 3> targetCoordinatePlanes = {
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::X),
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Y),
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Z)
    };

    PlanePoint3i currentPoint = startPoint;
    outPath.clear();
    for (std::size_t axisIndex = 0; axisIndex < axisCount; ++axisIndex)
    {
        const SplitAxis3i axis = axisOrder[axisIndex];
        const int coordinateIndex = axisOrderKey(axis);
        if (areSamePlaneEquation(currentCoordinatePlanes[coordinateIndex], targetCoordinatePlanes[coordinateIndex]))
            continue;

        const std::array<Plane3i, 3> startCoordinatePlanes = currentCoordinatePlanes;
        currentCoordinatePlanes[coordinateIndex] = targetCoordinatePlanes[coordinateIndex];

        const PlanePoint3i nextPoint = makePointFromPlanes(currentCoordinatePlanes);
        if (!nextPoint.hasUniqueIntersection())
        {
            outPath.clear();
            return false;
        }

        Segment256 segment;
        if (!buildAxisAlignedSegmentFromCoordinatePlanes(startCoordinatePlanes, currentCoordinatePlanes, axis, segment))
        {
            outPath.clear();
            return false;
        }

        outPath.push_back(std::move(segment));
        currentPoint = nextPoint;
    }

    return areSamePlanePoint(currentPoint, targetPoint);
}

inline bool buildAxisAlignedFreeCoordinatePath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const std::vector<SplitAxis3i> &axisOrder,
    std::vector<Segment256> &outPath)
{
    std::array<SplitAxis3i, 3> axisOrderArray = {};
    const std::size_t axisCount = std::min<std::size_t>(axisOrder.size(), axisOrderArray.size());
    for (std::size_t axisIndex = 0; axisIndex < axisCount; ++axisIndex)
        axisOrderArray[axisIndex] = axisOrder[axisIndex];

    return buildAxisAlignedFreeCoordinatePath(startPoint, targetPoint, axisOrderArray, axisCount, outPath);
}

/**
 * @brief 为 AABB 内两个点构造一条轴对齐桥接路径。
 */
inline bool buildAxisAlignedBridgePath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    std::vector<Segment256> &outPath)
{
    outPath.clear();
    if (areSamePlanePoint(startPoint, targetPoint))
        return true;
    if (!isPointInsideOrOnAABB(startPoint, box) || !isPointInsideOrOnAABB(targetPoint, box))
        return false;

    Integer startX, startY, startZ;
    Integer targetX, targetY, targetZ;
    if (!tryExtractExactIntegerPoint(startPoint, startX, startY, startZ) ||
            !tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ))
        return false;

    std::array<SplitAxis3i, 3> changedAxes = {};
    std::size_t axisCount = 0;
    if (startX != targetX)
        changedAxes[axisCount++] = SplitAxis3i::X;
    if (startY != targetY)
        changedAxes[axisCount++] = SplitAxis3i::Y;
    if (startZ != targetZ)
        changedAxes[axisCount++] = SplitAxis3i::Z;

    if (axisCount == 0)
        return false;

    std::sort(
        changedAxes.begin(),
        changedAxes.begin() + static_cast<std::ptrdiff_t>(axisCount),
        [](SplitAxis3i lhs, SplitAxis3i rhs)
    {
        return axisOrderKey(lhs) < axisOrderKey(rhs);
    });

    std::array<SplitAxis3i, 3> axisOrder = changedAxes;
    do
    {
        std::vector<Segment256> bridge;
        if (buildAxisAlignedCornerPath(startPoint, targetPoint, box, axisOrder, axisCount, bridge))
        {
            outPath = std::move(bridge);
            return true;
        }
    } while (std::next_permutation(
                 axisOrder.begin(),
                 axisOrder.begin() + static_cast<std::ptrdiff_t>(axisCount),
                 [](SplitAxis3i lhs, SplitAxis3i rhs)
{
    return axisOrderKey(lhs) < axisOrderKey(rhs);
    }));

    return false;
}

/**
 * @brief 追加一条桥接路径，保持输出路径连续。
 */
inline bool appendBridgePath(
    std::vector<Segment256> &outPath,
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box)
{
    if (areSamePlanePoint(startPoint, targetPoint))
        return true;

    std::vector<Segment256> bridge;
    if (buildAxisAlignedBridgePath(startPoint, targetPoint, box, bridge) ||
            buildAnyBoundedPlaneReplacementBridgePath(startPoint, targetPoint, box, bridge))
    {
        outPath.insert(outPath.end(), std::make_move_iterator(bridge.begin()), std::make_move_iterator(bridge.end()));
        return true;
    }

    return false;
}

inline Plane3i makePlaneThroughPointWithNormal(const PlanePoint3i &point, const Vec3i &normal) noexcept
{
    if (!point.hasUniqueIntersection() || isZero(point.x.w))
        return Plane3i();

    const Integer d =
        -(normal.x * point.x.x + normal.y * point.x.y + normal.z * point.x.z);
    return primitivePlane(Plane3i(
                              normal.x * point.x.w,
                              normal.y * point.x.w,
                              normal.z * point.x.w,
                              d));
}

inline bool makeNormalLineThroughPoint(
    const PlanePoint3i &point,
    const Plane3i &surfacePlane,
    Line256 &outLine) noexcept
{
    const Vec3i normal = surfacePlane.normal();
    if (isZero(normal.x) && isZero(normal.y) && isZero(normal.z))
        return false;

    Vec3i tangent0;
    Vec3i tangent1;
    if (!isZero(normal.x) || !isZero(normal.y))
    {
        tangent0 = Vec3i(normal.y, -normal.x, 0);
        tangent1 = Vec3i(
                       normal.x * normal.z,
                       normal.y * normal.z,
                       -(normal.x * normal.x + normal.y * normal.y));
    }
    else
    {
        tangent0 = Vec3i(1, 0, 0);
        tangent1 = Vec3i(0, 1, 0);
    }

    const Plane3i linePlane0 = makePlaneThroughPointWithNormal(point, tangent0);
    const Plane3i linePlane1 = makePlaneThroughPointWithNormal(point, tangent1);
    const Line256 line(linePlane0, linePlane1);
    if (!line.isValid())
        return false;

    outLine = line;
    return true;
}

inline bool buildNormalApproachPath(
    const PlanePoint3i &referencePoint,
    const PlanePoint3i &targetPoint,
    const Plane3i &surfacePlane,
    const AABB3i &box,
    const Integer &signedPlaneOffset,
    std::vector<Segment256> &outPath)
{
    outPath.clear();

    Line256 approachLine;
    if (isZero(signedPlaneOffset) ||
            !makeNormalLineThroughPoint(targetPoint, surfacePlane, approachLine))
        return false;

    const Plane3i startPlane(
        surfacePlane.a,
        surfacePlane.b,
        surfacePlane.c,
        surfacePlane.d - signedPlaneOffset);
    Segment256 approachSegment(startPlane, surfacePlane, approachLine);
    if (!approachSegment.isValid() ||
            !areSamePlanePoint(approachSegment.getEndPointRef(), targetPoint))
        return false;

    const PlanePoint3i &approachStart = approachSegment.getStartPointRef();
    if (!approachStart.hasUniqueIntersection() ||
            !isPointInsideOrOnAABB(approachStart, box) ||
            areSamePlanePoint(approachStart, targetPoint))
        return false;

    std::vector<Segment256> path;
    if (!appendBridgePath(path, referencePoint, approachStart, box))
        return false;

    path.push_back(std::move(approachSegment));
    if (!areSamePlanePoint(path.front().getStartPointRef(), referencePoint) ||
            !areSamePlanePoint(path.back().getEndPointRef(), targetPoint))
        return false;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        if (!areSamePlanePoint(path[i - 1].getEndPointRef(), path[i].getStartPointRef()))
            return false;
    }

    outPath = std::move(path);
    return true;
}

inline bool buildNormalApproachPathViaBridgePoint(
    const PlanePoint3i &referencePoint,
    const PlanePoint3i &bridgePoint,
    const PlanePoint3i &targetPoint,
    const Plane3i &surfacePlane,
    const AABB3i &box,
    const Integer &signedPlaneOffset,
    std::vector<Segment256> &outPath)
{
    outPath.clear();
    if (!bridgePoint.hasUniqueIntersection() ||
            !isPointInsideOrOnAABB(bridgePoint, box) ||
            areSamePlanePoint(referencePoint, bridgePoint))
        return false;

    Line256 approachLine;
    if (isZero(signedPlaneOffset) ||
            !makeNormalLineThroughPoint(targetPoint, surfacePlane, approachLine))
        return false;

    const Plane3i startPlane(
        surfacePlane.a,
        surfacePlane.b,
        surfacePlane.c,
        surfacePlane.d - signedPlaneOffset);
    Segment256 approachSegment(startPlane, surfacePlane, approachLine);
    if (!approachSegment.isValid() ||
            !areSamePlanePoint(approachSegment.getEndPointRef(), targetPoint))
        return false;

    const PlanePoint3i &approachStart = approachSegment.getStartPointRef();
    if (!approachStart.hasUniqueIntersection() ||
            !isPointInsideOrOnAABB(approachStart, box) ||
            areSamePlanePoint(approachStart, targetPoint))
        return false;

    std::vector<Segment256> path;
    if (!appendBridgePath(path, referencePoint, bridgePoint, box) ||
            !appendBridgePath(path, bridgePoint, approachStart, box))
        return false;

    path.push_back(std::move(approachSegment));
    if (!areSamePlanePoint(path.front().getStartPointRef(), referencePoint) ||
            !areSamePlanePoint(path.back().getEndPointRef(), targetPoint))
        return false;
    for (std::size_t i = 1; i < path.size(); ++i)
    {
        if (!areSamePlanePoint(path[i - 1].getEndPointRef(), path[i].getStartPointRef()))
            return false;
    }

    outPath = std::move(path);
    return true;
}

inline bool chooseStrictInteriorAABBCoordinate(
    const Integer &minValue,
    const Integer &maxValue,
    Integer preferred,
    Integer &outValue) noexcept
{
    if (maxValue - minValue <= 1)
        return false;

    if (preferred <= minValue)
        preferred = minValue + 1;
    if (preferred >= maxValue)
        preferred = maxValue - 1;
    if (preferred <= minValue || preferred >= maxValue)
        return false;

    outValue = preferred;
    return true;
}

inline void appendUniqueAABBInteriorPoint(
    std::vector<PlanePoint3i> &points,
    const AABB3i &box,
    const Integer &x,
    const Integer &y,
    const Integer &z)
{
    if (x <= box.xMin || x >= box.xMax ||
            y <= box.yMin || y >= box.yMax ||
            z <= box.zMin || z >= box.zMax)
        return;

    const PlanePoint3i point = makeIntegerPoint(x, y, z);
    for (const PlanePoint3i &existing : points)
    {
        if (areSamePlanePoint(existing, point))
            return;
    }

    points.push_back(point);
}

inline std::vector<PlanePoint3i> enumerateAABBInteriorBridgePoints(
    const PlanePoint3i &referencePoint,
    const AABB3i &box)
{
    std::vector<PlanePoint3i> points;
    if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        return points;

    Integer referenceX;
    Integer referenceY;
    Integer referenceZ;
    if (tryExtractExactIntegerPoint(referencePoint, referenceX, referenceY, referenceZ))
    {
        Integer x;
        Integer y;
        Integer z;
        if (chooseStrictInteriorAABBCoordinate(box.xMin, box.xMax, referenceX, x) &&
                chooseStrictInteriorAABBCoordinate(box.yMin, box.yMax, referenceY, y) &&
                chooseStrictInteriorAABBCoordinate(box.zMin, box.zMax, referenceZ, z))
            appendUniqueAABBInteriorPoint(points, box, x, y, z);
    }

    const Integer centerX = floorDiv(box.xMin + box.xMax, Integer(2));
    const Integer centerY = floorDiv(box.yMin + box.yMax, Integer(2));
    const Integer centerZ = floorDiv(box.zMin + box.zMax, Integer(2));
    Integer x;
    Integer y;
    Integer z;
    if (chooseStrictInteriorAABBCoordinate(box.xMin, box.xMax, centerX, x) &&
            chooseStrictInteriorAABBCoordinate(box.yMin, box.yMax, centerY, y) &&
            chooseStrictInteriorAABBCoordinate(box.zMin, box.zMax, centerZ, z))
        appendUniqueAABBInteriorPoint(points, box, x, y, z);

    const std::array<Integer, 2> xInsets = {box.xMin + 1, box.xMax - 1};
    const std::array<Integer, 2> yInsets = {box.yMin + 1, box.yMax - 1};
    const std::array<Integer, 2> zInsets = {box.zMin + 1, box.zMax - 1};
    for (const Integer &xInset : xInsets)
    {
        for (const Integer &yInset : yInsets)
        {
            for (const Integer &zInset : zInsets)
                appendUniqueAABBInteriorPoint(points, box, xInset, yInset, zInset);
        }
    }

    return points;
}

/**
 * @brief 将单条线段裁剪到 AABB 内部。
 */
inline bool clipSegmentAgainstAABBPlane(
    Segment256 &segment,
    const Plane3i &clipPlane)
{
    const PlanePoint3i &startPoint = segment.getStartPointRef();
    const PlanePoint3i &endPoint = segment.getEndPointRef();
    const int startSide = startPoint.classify(clipPlane);
    const int endSide = endPoint.classify(clipPlane);

    if (startSide > 0 && endSide > 0)
        return false;
    if (startSide <= 0 && endSide <= 0)
        return true;

    const PlanePoint3i hitPoint = intersect(segment.direction, clipPlane);
    if (!hitPoint.hasUniqueIntersection() || !isPointOnSegment(hitPoint, segment))
        return false;

    segment = (startSide > 0)
              ? Segment256(clipPlane, segment.end, segment.direction)
              : Segment256(segment.start, clipPlane, segment.direction);
    return segment.isValid();
}

inline bool isSegmentInsideOrOnAABB(const Segment256 &segment, const AABB3i &box)
{
    return isPointInsideOrOnAABB(segment.getStartPointRef(), box) &&
           isPointInsideOrOnAABB(segment.getEndPointRef(), box);
}

inline bool clipSegmentToAABB(const Segment256 &segment, const AABB3i &box, Segment256 &outSegment)
{
    if (!segment.isValid() || !isValidAABB(box))
        return false;

    Segment256 current = segment;
    const auto planes = makeAABBPlanes(box);
    for (const Plane3i &clipPlane : planes)
    {
        if (!clipSegmentAgainstAABBPlane(current, clipPlane))
            return false;
    }

    if (!isSegmentInsideOrOnAABB(current, box))
        return false;

    outSegment = current;
    return true;
}

/**
 * @brief 裁剪换平面路径，并在 AABB 内用轴对齐桥接段重接断点。
 */
inline bool clipAndBridgePathToAABB(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::vector<Segment256> &rawPath,
    std::vector<Segment256> &outPath)
{
    outPath.clear();
    if (!isPointInsideOrOnAABB(startPoint, box) || !isPointInsideOrOnAABB(targetPoint, box))
        return false;

    PlanePoint3i currentPoint = startPoint;
    for (const Segment256 &segment : rawPath)
    {
        Segment256 clipped;
        if (!clipSegmentToAABB(segment, box, clipped))
            continue;

        const PlanePoint3i &clippedStart = clipped.getStartPointRef();
        const PlanePoint3i &clippedEnd = clipped.getEndPointRef();
        if (!appendBridgePath(outPath, currentPoint, clippedStart, box))
        {
            outPath.clear();
            return false;
        }

        if (!areSamePlanePoint(clippedStart, clippedEnd))
            outPath.push_back(clipped);
        currentPoint = clippedEnd;
    }

    if (!appendBridgePath(outPath, currentPoint, targetPoint, box))
    {
        outPath.clear();
        return false;
    }

    if (outPath.empty())
        return false;

    if (!areSamePlanePoint(outPath.front().getStartPointRef(), startPoint) ||
            !areSamePlanePoint(outPath.back().getEndPointRef(), targetPoint))
    {
        outPath.clear();
        return false;
    }
    for (std::size_t i = 1; i < outPath.size(); ++i)
    {
        if (!areSamePlanePoint(outPath[i - 1].getEndPointRef(), outPath[i].getStartPointRef()))
        {
            outPath.clear();
            return false;
        }
    }

    return true;
}

/**
 * @brief 构造两个共享两张平面的点之间的一段路径线段。
 *
 * @param[in] startPoint 路径起点。
 * @param[in] endPoint 路径终点。
 * @param[in] replacedPlaneIndex 本段中被替换的那张定义平面索引。
 * @param[out] outSegment 成功时写入线段。
 * @return 当共享平面能够定义唯一支撑线，且线段端点与输入一致时返回 `true`。
 */
inline bool buildPlaneReplacementSegment(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &endPoint,
    int replacedPlaneIndex,
    Segment256 &outSegment)
{
    const Plane3i startPlanes[3] = {startPoint.p, startPoint.q, startPoint.r};
    const Plane3i endPlanes[3] = {endPoint.p, endPoint.q, endPoint.r};

    int sharedIndices[2];
    int cursor = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (i == replacedPlaneIndex)
            continue;
        sharedIndices[cursor++] = i;
    }

    const Line256 direction = makeLine(
                                  startPlanes[sharedIndices[0]],
                                  startPlanes[sharedIndices[1]]);
    if (!direction.isValid())
        return false;

    outSegment = Segment256(
                     startPlanes[replacedPlaneIndex],
                     endPlanes[replacedPlaneIndex],
                     direction);

    return outSegment.isValid() &&
           areSamePlanePoint(outSegment.getStartPointRef(), startPoint) &&
           areSamePlanePoint(outSegment.getEndPointRef(), endPoint);
}

/**
 * @brief 按给定顺序构造完全留在 AABB 内的平面替换桥接路径。
 */
inline bool buildBoundedPlaneReplacementPathForOrder(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::array<int, 3> &planeReplacementOrder,
    std::size_t planeReplacementCount,
    std::vector<Segment256> &outPath)
{
    REEMBER_PROFILE_ZONE("buildBoundedPlaneReplacementPathForOrder");

    outPath.clear();
    if (!isPointInsideOrOnAABB(startPoint, box) || !isPointInsideOrOnAABB(targetPoint, box))
        return false;

    std::array<Plane3i, 3> currentPlanes = {startPoint.p, startPoint.q, startPoint.r};
    const Plane3i targetPlanes[3] = {targetPoint.p, targetPoint.q, targetPoint.r};
    PlanePoint3i currentPoint = startPoint;

    for (std::size_t orderIndex = 0; orderIndex < planeReplacementCount; ++orderIndex)
    {
        REEMBER_PROFILE_ZONE("buildBoundedPlaneReplacementPathForOrder::step");
        const int replacedIndex = planeReplacementOrder[orderIndex];
        currentPlanes[replacedIndex] = targetPlanes[replacedIndex];
        const PlanePoint3i nextPoint = makePointFromPlanes(currentPlanes);
        if (!nextPoint.hasUniqueIntersection() || !isPointInsideOrOnAABB(nextPoint, box))
        {
            outPath.clear();
            return false;
        }

        Segment256 segment;
        if (!buildPlaneReplacementSegment(currentPoint, nextPoint, replacedIndex, segment))
        {
            outPath.clear();
            return false;
        }

        outPath.push_back(std::move(segment));
        currentPoint = nextPoint;
    }

    if (!areSamePlanePoint(currentPoint, targetPoint))
    {
        outPath.clear();
        return false;
    }

    return true;
}

inline bool buildBoundedPlaneReplacementPathForOrder(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::vector<int> &planeReplacementOrder,
    std::vector<Segment256> &outPath)
{
    std::array<int, 3> planeReplacementOrderArray = {};
    const std::size_t planeReplacementCount =
        std::min<std::size_t>(planeReplacementOrder.size(), planeReplacementOrderArray.size());
    for (std::size_t orderIndex = 0; orderIndex < planeReplacementCount; ++orderIndex)
        planeReplacementOrderArray[orderIndex] = planeReplacementOrder[orderIndex];

    return buildBoundedPlaneReplacementPathForOrder(
               startPoint,
               targetPoint,
               box,
               planeReplacementOrderArray,
               planeReplacementCount,
               outPath);
}

/**
 * @brief 在所有定义平面替换顺序中寻找一条 AABB 内桥接路径。
 */
inline bool buildAnyBoundedPlaneReplacementBridgePath(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    std::vector<Segment256> &outPath)
{
    outPath.clear();
    if (areSamePlanePoint(startPoint, targetPoint))
        return true;

    std::array<int, 3> changedPlaneIndices = {};
    std::size_t changedPlaneCount = 0;
    if (!areSamePlaneEquation(startPoint.p, targetPoint.p))
        changedPlaneIndices[changedPlaneCount++] = 0;
    if (!areSamePlaneEquation(startPoint.q, targetPoint.q))
        changedPlaneIndices[changedPlaneCount++] = 1;
    if (!areSamePlaneEquation(startPoint.r, targetPoint.r))
        changedPlaneIndices[changedPlaneCount++] = 2;

    if (changedPlaneCount == 0)
        return false;

    std::sort(
        changedPlaneIndices.begin(),
        changedPlaneIndices.begin() + static_cast<std::ptrdiff_t>(changedPlaneCount));
    std::array<int, 3> planeReplacementOrder = changedPlaneIndices;
    do
    {
        std::vector<Segment256> path;
        if (buildBoundedPlaneReplacementPathForOrder(
                    startPoint,
                    targetPoint,
                    box,
                    planeReplacementOrder,
                    changedPlaneCount,
                    path))
        {
            outPath = std::move(path);
            return true;
        }
    } while (std::next_permutation(
                 planeReplacementOrder.begin(),
                 planeReplacementOrder.begin() + static_cast<std::ptrdiff_t>(changedPlaneCount)));

    return false;
}

/**
 * @brief 按给定平面替换顺序构造从参考点到目标点的 1 到 3 段路径。
 *
 * @param[in] refPoint 已知 WNV 的局部参考点。
 * @param[in] targetPoint 严格内部的目标分类点。
 * @param[in] box 当前叶子子问题的 AABB。
 * @param[in] planeReplacementOrder 依次替换的定义平面索引。
 * @param[out] outPath 成功时写入完整候选路径。
 * @return 当所有中间点都落在 AABB 内，且每段线段都能精确定义时返回 `true`。
 */
inline bool buildPlaneReplacementPath(
    const PlanePoint3i &refPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::array<int, 3> &planeReplacementOrder,
    std::size_t planeReplacementCount,
    std::vector<Segment256> &outPath)
{
    if (buildBoundedPlaneReplacementPathForOrder(
                refPoint,
                targetPoint,
                box,
                planeReplacementOrder,
                planeReplacementCount,
                outPath))
        return true;

    std::array<Plane3i, 3> currentPlanes = {refPoint.p, refPoint.q, refPoint.r};
    const Plane3i targetPlanes[3] = {targetPoint.p, targetPoint.q, targetPoint.r};
    PlanePoint3i currentPoint = refPoint;
    std::vector<Segment256> rawPath;

    rawPath.clear();
    for (std::size_t orderIndex = 0; orderIndex < planeReplacementCount; ++orderIndex)
    {
        const int replacedIndex = planeReplacementOrder[orderIndex];
        currentPlanes[replacedIndex] = targetPlanes[replacedIndex];
        const PlanePoint3i nextPoint = makePointFromPlanes(currentPlanes);
        if (!nextPoint.hasUniqueIntersection())
            return false;

        Segment256 segment;
        if (!buildPlaneReplacementSegment(currentPoint, nextPoint, replacedIndex, segment))
            return false;

        rawPath.push_back(std::move(segment));
        currentPoint = nextPoint;
    }

    if (!areSamePlanePoint(currentPoint, targetPoint))
        return false;

    return clipAndBridgePathToAABB(refPoint, targetPoint, box, rawPath, outPath);
}

inline bool buildPlaneReplacementPath(
    const PlanePoint3i &refPoint,
    const PlanePoint3i &targetPoint,
    const AABB3i &box,
    const std::vector<int> &planeReplacementOrder,
    std::vector<Segment256> &outPath)
{
    std::array<int, 3> planeReplacementOrderArray = {};
    const std::size_t planeReplacementCount =
        std::min<std::size_t>(planeReplacementOrder.size(), planeReplacementOrderArray.size());
    for (std::size_t orderIndex = 0; orderIndex < planeReplacementCount; ++orderIndex)
        planeReplacementOrderArray[orderIndex] = planeReplacementOrder[orderIndex];

    return buildPlaneReplacementPath(
               refPoint,
               targetPoint,
               box,
               planeReplacementOrderArray,
               planeReplacementCount,
               outPath);
}

/**
 * @brief 生成优先尝试的低成本叶子分类点候选。
 *
 * @pre `polygon` 已由阶段入口完成 `isValid()` 校验。
 */
inline std::vector<PlanePoint3i> enumerateLeafClassificationPrimaryPointCandidatesUnchecked(const Polygon256 &polygon)
{
    std::vector<PlanePoint3i> candidates;
    appendCentroidProbeCandidates(polygon, candidates);

    PlanePoint3i fallbackPoint;
    if (polygon.findStrictInteriorPoint(fallbackPoint))
        appendUniqueStrictInteriorPoint(candidates, polygon, fallbackPoint);

    return candidates;
}

/**
 * @brief 在已验证多边形上生成严格内部分类点候选。
 *
 * @pre `polygon` 已由阶段入口完成 `isValid()` 校验。
 */
inline std::vector<PlanePoint3i> enumerateLeafClassificationPointCandidatesUnchecked(const Polygon256 &polygon)
{
    std::vector<PlanePoint3i> candidates = enumerateLeafClassificationPrimaryPointCandidatesUnchecked(polygon);
    appendInsetInteriorPointCandidates(polygon, candidates);
    if (candidates.empty())
        appendHomogeneousVertexAverageInteriorPointCandidate(polygon, candidates);
    if (candidates.empty())
        appendEqualizedEdgeInteriorPointCandidates(polygon, candidates);

    return candidates;
}

}
}
