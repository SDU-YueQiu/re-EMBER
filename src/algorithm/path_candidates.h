/**
 * @file path_candidates.h
 * @brief 声明路径追踪候选的公开类型与枚举入口。
 */
#pragma once

#include "algorithm/path_candidate_details.h"
#include "core/perf_tracing.h"

#include <algorithm>
#include <array>
#include <iterator>
#include <utility>
#include <vector>

namespace ember
{
struct AABBPathCandidate
{
    PlanePoint3i targetPoint;
    std::vector<Segment256> path;
};

struct LeafClassificationPathCandidate
{
    PlanePoint3i targetPoint;
    std::vector<Segment256> path;
};

namespace detail
{
enum class AABBPathBuildMode
{
    Empty,
    Corner,
    FreeCoordinate
};

struct AABBPathCandidateSeed
{
    PlanePoint3i targetPoint;
    std::array<SplitAxis3i, 3> axisOrder{};
    std::size_t axisCount = 0;
    AABBPathBuildMode buildMode = AABBPathBuildMode::Empty;
};

inline bool axisOrderLess(SplitAxis3i lhs, SplitAxis3i rhs) noexcept
{
    return axisOrderKey(lhs) < axisOrderKey(rhs);
}

inline void appendUniqueIntegerChoice(std::vector<Integer> &choices, const Integer &value)
{
    if (std::find(choices.begin(), choices.end(), value) == choices.end())
        choices.push_back(value);
}

inline bool appendUniqueIntegerTarget(
    std::vector<std::array<Integer, 3>> &targets,
    const Integer &x,
    const Integer &y,
    const Integer &z)
{
    const std::array<Integer, 3> target = {x, y, z};
    if (std::find(targets.begin(), targets.end(), target) != targets.end())
        return false;

    targets.push_back(target);
    return true;
}

template <typename SeedVisitor>
inline bool emitAABBPathCandidateSeed(
    const PlanePoint3i &targetPoint,
    const std::array<SplitAxis3i, 3> &axisOrder,
    std::size_t axisCount,
    AABBPathBuildMode buildMode,
    std::size_t &emitted,
    SeedVisitor &visitor)
{
    const AABBPathCandidateSeed seed{targetPoint, axisOrder, axisCount, buildMode};
    ++emitted;
    return visitor(seed);
}

template <typename SeedVisitor>
inline bool visitIntegerAABBPathTargetSeeds(
    const PlanePoint3i &targetPoint,
    const Integer &startX,
    const Integer &startY,
    const Integer &startZ,
    const Integer &targetX,
    const Integer &targetY,
    const Integer &targetZ,
    std::size_t &emitted,
    SeedVisitor &visitor)
{
    std::array<SplitAxis3i, 3> changedAxes = {};
    std::size_t axisCount = 0;
    if (startX != targetX)
        changedAxes[axisCount++] = SplitAxis3i::X;
    if (startY != targetY)
        changedAxes[axisCount++] = SplitAxis3i::Y;
    if (startZ != targetZ)
        changedAxes[axisCount++] = SplitAxis3i::Z;

    if (axisCount == 0)
    {
        return emitAABBPathCandidateSeed(
                   targetPoint,
                   changedAxes,
                   0,
                   AABBPathBuildMode::Empty,
                   emitted,
                   visitor);
    }

    std::sort(
        changedAxes.begin(),
        changedAxes.begin() + static_cast<std::ptrdiff_t>(axisCount),
        axisOrderLess);
    do
    {
        if (!emitAABBPathCandidateSeed(
                    targetPoint,
                    changedAxes,
                    axisCount,
                    AABBPathBuildMode::Corner,
                    emitted,
                    visitor))
            return false;
    } while (std::next_permutation(
                 changedAxes.begin(),
                 changedAxes.begin() + static_cast<std::ptrdiff_t>(axisCount),
                 axisOrderLess));

    return true;
}

template <typename SeedVisitor>
inline bool visitFreeCoordinateAABBPathTargetSeeds(
    const PlanePoint3i &startPoint,
    const PlanePoint3i &targetPoint,
    std::size_t &emitted,
    SeedVisitor &visitor)
{
    if (!targetPoint.hasUniqueIntersection())
        return true;

    const std::array<Plane3i, 3> startCoordinatePlanes = {
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::X),
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Y),
        makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Z)
    };
    const std::array<Plane3i, 3> targetCoordinatePlanes = {
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::X),
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Y),
        makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Z)
    };

    std::array<SplitAxis3i, 3> changedAxes = {};
    std::size_t axisCount = 0;
    if (!areSamePlaneEquation(startCoordinatePlanes[0], targetCoordinatePlanes[0]))
        changedAxes[axisCount++] = SplitAxis3i::X;
    if (!areSamePlaneEquation(startCoordinatePlanes[1], targetCoordinatePlanes[1]))
        changedAxes[axisCount++] = SplitAxis3i::Y;
    if (!areSamePlaneEquation(startCoordinatePlanes[2], targetCoordinatePlanes[2]))
        changedAxes[axisCount++] = SplitAxis3i::Z;

    if (axisCount == 0)
    {
        return emitAABBPathCandidateSeed(
                   targetPoint,
                   changedAxes,
                   0,
                   AABBPathBuildMode::Empty,
                   emitted,
                   visitor);
    }

    std::sort(
        changedAxes.begin(),
        changedAxes.begin() + static_cast<std::ptrdiff_t>(axisCount),
        axisOrderLess);
    do
    {
        if (!emitAABBPathCandidateSeed(
                    targetPoint,
                    changedAxes,
                    axisCount,
                    AABBPathBuildMode::FreeCoordinate,
                    emitted,
                    visitor))
            return false;
    } while (std::next_permutation(
                 changedAxes.begin(),
                 changedAxes.begin() + static_cast<std::ptrdiff_t>(axisCount),
                 axisOrderLess));

    return true;
}

inline bool buildAABBPathFromSeed(
    const PlanePoint3i &startPoint,
    const AABBPathCandidateSeed &seed,
    std::vector<Segment256> &outPath)
{
    switch (seed.buildMode)
    {
    case AABBPathBuildMode::Empty:
        outPath.clear();
        return areSamePlanePoint(startPoint, seed.targetPoint);
    case AABBPathBuildMode::Corner:
        return buildAxisAlignedCornerPath(startPoint, seed.targetPoint, seed.axisOrder, seed.axisCount, outPath);
    case AABBPathBuildMode::FreeCoordinate:
        return buildAxisAlignedFreeCoordinatePath(startPoint, seed.targetPoint, seed.axisOrder, seed.axisCount, outPath);
    }

    outPath.clear();
    return false;
}

template <typename SeedVisitor>
inline std::size_t visitFastAABBPathCandidateSeeds(
    const PlanePoint3i &startPoint,
    const AABB3i &targetBox,
    SeedVisitor &&visitor)
{
    std::size_t emitted = 0;
    if (!startPoint.hasUniqueIntersection() || !isValidAABB(targetBox))
        return emitted;

    const PlanePoint3i preferredTarget = projectPointToAABB(startPoint, targetBox);
    if (areSamePlanePoint(preferredTarget, startPoint))
    {
        const std::array<SplitAxis3i, 3> emptyAxes = {};
        if (!emitAABBPathCandidateSeed(
                    startPoint,
                    emptyAxes,
                    0,
                    AABBPathBuildMode::Empty,
                    emitted,
                    visitor))
            return emitted;
    }

    Integer startX, startY, startZ;
    Integer preferredX, preferredY, preferredZ;
    if (!tryExtractExactIntegerPoint(startPoint, startX, startY, startZ) ||
            !tryExtractExactIntegerPoint(preferredTarget, preferredX, preferredY, preferredZ))
        return emitted;

    Integer strictX;
    Integer strictY;
    Integer strictZ;
    if (!chooseStrictInteriorAABBCoordinate(targetBox.xMin, targetBox.xMax, preferredX, strictX) ||
            !chooseStrictInteriorAABBCoordinate(targetBox.yMin, targetBox.yMax, preferredY, strictY) ||
            !chooseStrictInteriorAABBCoordinate(targetBox.zMin, targetBox.zMax, preferredZ, strictZ))
        return emitted;

    std::vector<std::array<Integer, 3>> emittedTargets;
    emittedTargets.reserve(20);
    auto emitUniqueIntegerTarget =
        [&](const Integer &x, const Integer &y, const Integer &z) -> bool
    {
        if (!appendUniqueIntegerTarget(emittedTargets, x, y, z))
        {
            return true;
        }

        return visitIntegerAABBPathTargetSeeds(
            makeIntegerPoint(x, y, z),
            startX,
            startY,
            startZ,
            x,
            y,
            z,
            emitted,
            visitor);
    };

    if (!emitUniqueIntegerTarget(strictX, strictY, strictZ))
    {
        return emitted;
    }

    for (const Integer &delta : {
                Integer(1), Integer(-1), Integer(2), Integer(-2)
            })
    {
        if (strictX + delta > targetBox.xMin && strictX + delta < targetBox.xMax)
        {
            if (!emitUniqueIntegerTarget(strictX + delta, strictY, strictZ))
                return emitted;
        }
        if (strictY + delta > targetBox.yMin && strictY + delta < targetBox.yMax)
        {
            if (!emitUniqueIntegerTarget(strictX, strictY + delta, strictZ))
                return emitted;
        }
        if (strictZ + delta > targetBox.zMin && strictZ + delta < targetBox.zMax)
        {
            if (!emitUniqueIntegerTarget(strictX, strictY, strictZ + delta))
                return emitted;
        }
    }

    const Integer centerX = floorDiv(targetBox.xMin + targetBox.xMax, Integer(2));
    const Integer centerY = floorDiv(targetBox.yMin + targetBox.yMax, Integer(2));
    const Integer centerZ = floorDiv(targetBox.zMin + targetBox.zMax, Integer(2));
    Integer strictCenterX;
    Integer strictCenterY;
    Integer strictCenterZ;
    if (chooseStrictInteriorAABBCoordinate(targetBox.xMin, targetBox.xMax, centerX, strictCenterX) &&
            chooseStrictInteriorAABBCoordinate(targetBox.yMin, targetBox.yMax, centerY, strictCenterY) &&
            chooseStrictInteriorAABBCoordinate(targetBox.zMin, targetBox.zMax, centerZ, strictCenterZ))
    {
        if (!emitUniqueIntegerTarget(strictCenterX, strictCenterY, strictCenterZ))
            return emitted;
    }

    std::vector<Integer> xInsets;
    std::vector<Integer> yInsets;
    std::vector<Integer> zInsets;
    xInsets.reserve(2);
    yInsets.reserve(2);
    zInsets.reserve(2);
    appendUniqueIntegerChoice(xInsets, targetBox.xMin + Integer(1));
    appendUniqueIntegerChoice(xInsets, targetBox.xMax - Integer(1));
    appendUniqueIntegerChoice(yInsets, targetBox.yMin + Integer(1));
    appendUniqueIntegerChoice(yInsets, targetBox.yMax - Integer(1));
    appendUniqueIntegerChoice(zInsets, targetBox.zMin + Integer(1));
    appendUniqueIntegerChoice(zInsets, targetBox.zMax - Integer(1));
    for (const Integer &xInset : xInsets)
    {
        if (xInset <= targetBox.xMin || xInset >= targetBox.xMax)
            continue;
        for (const Integer &yInset : yInsets)
        {
            if (yInset <= targetBox.yMin || yInset >= targetBox.yMax)
                continue;
            for (const Integer &zInset : zInsets)
            {
                if (zInset <= targetBox.zMin || zInset >= targetBox.zMax)
                    continue;
                if (!emitUniqueIntegerTarget(xInset, yInset, zInset))
                    return emitted;
            }
        }
    }

    return emitted;
}

template <typename SeedVisitor>
inline std::size_t visitExhaustiveAABBPathCandidateSeeds(
    const PlanePoint3i &startPoint,
    const AABB3i &targetBox,
    SeedVisitor &&visitor)
{
    std::size_t emitted = 0;
    if (!startPoint.hasUniqueIntersection() || !isValidAABB(targetBox))
        return emitted;

    const PlanePoint3i preferredTarget = projectPointToAABB(startPoint, targetBox);

    Integer startX, startY, startZ;
    Integer preferredX, preferredY, preferredZ;
    if (!tryExtractExactIntegerPoint(startPoint, startX, startY, startZ) ||
            !tryExtractExactIntegerPoint(preferredTarget, preferredX, preferredY, preferredZ))
    {
        const PlanePoint3i centerPoint(
            Plane3i(2, 0, 0, -(targetBox.xMin + targetBox.xMax)),
            Plane3i(0, 2, 0, -(targetBox.yMin + targetBox.yMax)),
            Plane3i(0, 0, 2, -(targetBox.zMin + targetBox.zMax)));
        if (!visitFreeCoordinateAABBPathTargetSeeds(startPoint, preferredTarget, emitted, visitor))
            return emitted;
        if (!areSamePlanePoint(centerPoint, preferredTarget) &&
                !visitFreeCoordinateAABBPathTargetSeeds(startPoint, centerPoint, emitted, visitor))
            return emitted;
        return emitted;
    }

    std::vector<Integer> xChoices;
    std::vector<Integer> yChoices;
    std::vector<Integer> zChoices;
    xChoices.reserve(7);
    yChoices.reserve(7);
    zChoices.reserve(7);

    appendUniqueIntegerChoice(xChoices, preferredX);
    appendUniqueIntegerChoice(yChoices, preferredY);
    appendUniqueIntegerChoice(zChoices, preferredZ);
    for (const Integer &delta : {
                Integer(-1), Integer(1), Integer(-2), Integer(2)
            })
    {
        if (preferredX + delta >= targetBox.xMin && preferredX + delta <= targetBox.xMax)
            appendUniqueIntegerChoice(xChoices, preferredX + delta);
        if (preferredY + delta >= targetBox.yMin && preferredY + delta <= targetBox.yMax)
            appendUniqueIntegerChoice(yChoices, preferredY + delta);
        if (preferredZ + delta >= targetBox.zMin && preferredZ + delta <= targetBox.zMax)
            appendUniqueIntegerChoice(zChoices, preferredZ + delta);
    }
    appendUniqueIntegerChoice(xChoices, targetBox.xMin);
    appendUniqueIntegerChoice(xChoices, targetBox.xMax);
    appendUniqueIntegerChoice(yChoices, targetBox.yMin);
    appendUniqueIntegerChoice(yChoices, targetBox.yMax);
    appendUniqueIntegerChoice(zChoices, targetBox.zMin);
    appendUniqueIntegerChoice(zChoices, targetBox.zMax);

    struct IntegerAABBTarget
    {
        Integer x;
        Integer y;
        Integer z;
        bool interior = false;
        bool preferred = false;
        Integer distance = 0;
        std::size_t axisCount = 0;
    };

    std::vector<IntegerAABBTarget> targets;
    targets.reserve(xChoices.size() * yChoices.size() * zChoices.size());
    for (const Integer &xChoice : xChoices)
    {
        for (const Integer &yChoice : yChoices)
        {
            for (const Integer &zChoice : zChoices)
            {
                IntegerAABBTarget target;
                target.x = xChoice;
                target.y = yChoice;
                target.z = zChoice;
                target.interior =
                    xChoice > targetBox.xMin && xChoice < targetBox.xMax &&
                    yChoice > targetBox.yMin && yChoice < targetBox.yMax &&
                    zChoice > targetBox.zMin && zChoice < targetBox.zMax;
                target.preferred =
                    xChoice == preferredX &&
                    yChoice == preferredY &&
                    zChoice == preferredZ;
                target.distance =
                    absInteger(xChoice - preferredX) +
                    absInteger(yChoice - preferredY) +
                    absInteger(zChoice - preferredZ);
                target.axisCount =
                    static_cast<std::size_t>(startX != xChoice) +
                    static_cast<std::size_t>(startY != yChoice) +
                    static_cast<std::size_t>(startZ != zChoice);
                targets.push_back(std::move(target));
            }
        }
    }

    std::sort(
        targets.begin(),
        targets.end(),
        [](const IntegerAABBTarget &lhs, const IntegerAABBTarget &rhs)
    {
        if (lhs.interior != rhs.interior)
            return lhs.interior;
        if (lhs.preferred != rhs.preferred)
            return lhs.preferred;
        if (lhs.distance != rhs.distance)
            return lhs.distance < rhs.distance;
        if (lhs.axisCount != rhs.axisCount)
            return lhs.axisCount < rhs.axisCount;
        if (lhs.x != rhs.x)
            return lhs.x < rhs.x;
        if (lhs.y != rhs.y)
            return lhs.y < rhs.y;
        return lhs.z < rhs.z;
    });

    for (const IntegerAABBTarget &target : targets)
    {
        if (!visitIntegerAABBPathTargetSeeds(
                    makeIntegerPoint(target.x, target.y, target.z),
                    startX,
                    startY,
                    startZ,
                    target.x,
                    target.y,
                    target.z,
                    emitted,
                    visitor))
            return emitted;
    }

    const PlanePoint3i centerPoint(
        Plane3i(2, 0, 0, -(targetBox.xMin + targetBox.xMax)),
        Plane3i(0, 2, 0, -(targetBox.yMin + targetBox.yMax)),
        Plane3i(0, 0, 2, -(targetBox.zMin + targetBox.zMax)));
    if (!areSamePlanePoint(centerPoint, preferredTarget) &&
            !visitFreeCoordinateAABBPathTargetSeeds(startPoint, centerPoint, emitted, visitor))
        return emitted;

    return emitted;
}
}

/**
 * @brief 枚举以目标片段支撑平面法向作为最后一段的分类路径候选。
 *
 * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
 * @param[in] referencePoint 当前叶子子问题的局部参考点。
 * @param[in] targetPoints 已生成的叶子多边形严格内部点。
 * @param[in] surfacePlane 待分类片段的支撑平面。
 * @param[in] box 当前叶子子问题的 AABB。
 * @param[in,out] visitor 候选路径访问器。
 * @return 实际枚举出的候选路径数量。
 */
template <typename CandidateVisitor>
inline std::size_t enumerateLeafClassificationNormalApproachCandidatesFromPoints(
    const PlanePoint3i &referencePoint,
    const std::vector<PlanePoint3i> &targetPoints,
    const Plane3i &surfacePlane,
    const AABB3i &box,
    CandidateVisitor &&visitor)
{
    REEMBER_PROFILE_ZONE("enumerateLeafClassificationNormalApproachCandidatesFromPoints");

    std::size_t emitted = 0;
    if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        return emitted;

    std::array<int, 10> offsets = {1, -1, 2, -2, 4, -4, 8, -8, 16, -16};
    const std::vector<PlanePoint3i> bridgePoints =
        detail::enumerateAABBInteriorBridgePoints(referencePoint, box);
    for (const PlanePoint3i &targetPoint : targetPoints)
    {
        for (const int offset : offsets)
        {
            std::vector<Segment256> path;
            if (!detail::buildNormalApproachPath(
                        referencePoint,
                        targetPoint,
                        surfacePlane,
                        box,
                        Integer(offset),
                        path))
                continue;

            ++emitted;
            if (!visitor(LeafClassificationPathCandidate{targetPoint, std::move(path)}))
            {
                return emitted;
            }

            for (const PlanePoint3i &bridgePoint : bridgePoints)
            {
                std::vector<Segment256> detourPath;
                if (!detail::buildNormalApproachPathViaBridgePoint(
                            referencePoint,
                            bridgePoint,
                            targetPoint,
                            surfacePlane,
                            box,
                            Integer(offset),
                            detourPath))
                    continue;

                ++emitted;
                if (!visitor(LeafClassificationPathCandidate{targetPoint, std::move(detourPath)}))
                {
                    return emitted;
                }
            }
        }
    }

    return emitted;
}

/**
 * @brief 枚举快速分类路径候选，并逐个交给调用方处理。
 *
 * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
 * @param[in] referencePoint 当前叶子子问题的局部参考点。
 * @param[in] targetPoints 已生成的叶子多边形严格内部点。
 * @param[in] box 当前叶子子问题的 AABB。
 * @param[in,out] visitor 候选路径访问器。
 * @return 实际枚举出的候选路径数量。
 *
 * @pre `targetPoints` 已由叶子分类点生成阶段保证为严格内部点。
 */
template <typename CandidateVisitor>
inline std::size_t enumerateLeafClassificationFastPathCandidatesFromPoints(
    const PlanePoint3i &referencePoint,
    const std::vector<PlanePoint3i> &targetPoints,
    const AABB3i &box,
    CandidateVisitor &&visitor)
{
    REEMBER_PROFILE_ZONE("enumerateLeafClassificationFastPathCandidatesFromPoints");

    std::size_t emitted = 0;
    if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        return emitted;

    Integer referenceX = 0;
    Integer referenceY = 0;
    Integer referenceZ = 0;
    const bool hasExactReferencePoint =
        detail::tryExtractExactIntegerPoint(referencePoint, referenceX, referenceY, referenceZ);
    for (const PlanePoint3i &targetPoint : targetPoints)
    {
        auto emitCandidate =
            [&](std::vector<Segment256> path) -> bool
        {
            ++emitted;
            return visitor(LeafClassificationPathCandidate{targetPoint, std::move(path)});
        };

        Integer targetX, targetY, targetZ;
        if (hasExactReferencePoint &&
                detail::tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ))
        {
            std::array<SplitAxis3i, 3> changedAxes = {};
            std::size_t axisCount = 0;
            if (referenceX != targetX)
                changedAxes[axisCount++] = SplitAxis3i::X;
            if (referenceY != targetY)
                changedAxes[axisCount++] = SplitAxis3i::Y;
            if (referenceZ != targetZ)
                changedAxes[axisCount++] = SplitAxis3i::Z;

            std::sort(
                changedAxes.begin(),
                changedAxes.begin() + static_cast<std::ptrdiff_t>(axisCount),
                [](SplitAxis3i lhs, SplitAxis3i rhs)
            {
                return detail::axisOrderKey(lhs) < detail::axisOrderKey(rhs);
            });

            if (axisCount == 0)
            {
                if (!emitCandidate({}))
                {
                    return emitted;
                }
            }
            else
            {
                std::array<SplitAxis3i, 3> axisOrder = changedAxes;
                do
                {
                    std::vector<Segment256> path;
                    if (detail::buildAxisAlignedCornerPath(referencePoint, targetPoint, box, axisOrder, axisCount, path))
                    {
                        if (!emitCandidate(std::move(path)))
                            return emitted;
                    }
                } while (std::next_permutation(
                             axisOrder.begin(),
                             axisOrder.begin() + static_cast<std::ptrdiff_t>(axisCount),
                             [](SplitAxis3i lhs, SplitAxis3i rhs)
            {
                return detail::axisOrderKey(lhs) < detail::axisOrderKey(rhs);
                }));
            }
        }

        std::array<int, 3> changedPlaneIndices = {};
        std::size_t changedPlaneCount = 0;
        if (!detail::areSamePlaneEquation(referencePoint.p, targetPoint.p))
            changedPlaneIndices[changedPlaneCount++] = 0;
        if (!detail::areSamePlaneEquation(referencePoint.q, targetPoint.q))
            changedPlaneIndices[changedPlaneCount++] = 1;
        if (!detail::areSamePlaneEquation(referencePoint.r, targetPoint.r))
            changedPlaneIndices[changedPlaneCount++] = 2;

        if (changedPlaneCount == 0)
            continue;

        std::sort(
            changedPlaneIndices.begin(),
            changedPlaneIndices.begin() + static_cast<std::ptrdiff_t>(changedPlaneCount));
        std::array<int, 3> planeReplacementOrder = changedPlaneIndices;
        do
        {
            std::vector<Segment256> path;
            if (detail::buildPlaneReplacementPath(
                        referencePoint,
                        targetPoint,
                        box,
                        planeReplacementOrder,
                        changedPlaneCount,
                        path))
            {
                if (!emitCandidate(std::move(path)))
                    return emitted;
            }
        } while (std::next_permutation(
                     planeReplacementOrder.begin(),
                     planeReplacementOrder.begin() + static_cast<std::ptrdiff_t>(changedPlaneCount)));
    }

    return emitted;
}

/**
 * @brief 枚举兜底分类路径候选，并逐个交给调用方处理。
 *
 * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
 * @param[in] referencePoint 当前叶子子问题的局部参考点。
 * @param[in] targetPoints 已生成的叶子多边形严格内部点。
 * @param[in] box 当前叶子子问题的 AABB。
 * @param[in,out] visitor 候选路径访问器。
 * @return 实际枚举出的候选路径数量。
 *
 * @pre 快速候选已全部失败或没有产生候选。
 */
template <typename CandidateVisitor>
inline std::size_t enumerateLeafClassificationFallbackPathCandidatesFromPoints(
    const PlanePoint3i &referencePoint,
    const std::vector<PlanePoint3i> &targetPoints,
    const AABB3i &box,
    CandidateVisitor &&visitor)
{
    REEMBER_PROFILE_ZONE("enumerateLeafClassificationFallbackPathCandidatesFromPoints");

    std::size_t emitted = 0;
    if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        return emitted;

    for (const PlanePoint3i &targetPoint : targetPoints)
    {
        const std::array<Plane3i, 3> targetPlanes = {targetPoint.p, targetPoint.q, targetPoint.r};
        std::array<int, 3> targetPlaneOrder = {0, 1, 2};
        do
        {
            if (targetPlaneOrder[0] == 0 && targetPlaneOrder[1] == 1 && targetPlaneOrder[2] == 2)
                continue;

            const PlanePoint3i permutedTargetPoint = detail::makePointFromPlanes({
                targetPlanes[targetPlaneOrder[0]],
                targetPlanes[targetPlaneOrder[1]],
                targetPlanes[targetPlaneOrder[2]]});
            if (!permutedTargetPoint.hasUniqueIntersection() ||
                    !areSamePlanePoint(permutedTargetPoint, targetPoint))
                continue;

            std::vector<int> changedPlaneIndices;
            if (!detail::areSamePlaneEquation(referencePoint.p, permutedTargetPoint.p))
                changedPlaneIndices.push_back(0);
            if (!detail::areSamePlaneEquation(referencePoint.q, permutedTargetPoint.q))
                changedPlaneIndices.push_back(1);
            if (!detail::areSamePlaneEquation(referencePoint.r, permutedTargetPoint.r))
                changedPlaneIndices.push_back(2);
            if (changedPlaneIndices.empty())
                continue;

            std::sort(changedPlaneIndices.begin(), changedPlaneIndices.end());
            do
            {
                std::vector<Segment256> path;
                if (detail::buildPlaneReplacementPath(referencePoint, permutedTargetPoint, box, changedPlaneIndices, path))
                {
                    ++emitted;
                    if (!visitor(LeafClassificationPathCandidate{targetPoint, std::move(path)}))
                    {
                        return emitted;
                    }
                }
            } while (std::next_permutation(changedPlaneIndices.begin(), changedPlaneIndices.end()));
        } while (std::next_permutation(targetPlaneOrder.begin(), targetPlaneOrder.end()));
    }

    return emitted;
}

/**
 * @brief 先桥接到 AABB 严格内部点，再枚举叶子分类路径候选。
 *
 * 这些候选保留原有干净穿越判定，只在常规轴对齐路径贴着网格或切分边界失效后提供
 * 少量确定性绕行路径。
 *
 * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
 * @param[in] referencePoint 当前叶子子问题的局部参考点。
 * @param[in] targetPoints 已生成的叶子多边形严格内部点。
 * @param[in] box 当前叶子子问题的 AABB。
 * @param[in,out] visitor 候选路径访问器。
 * @return 实际枚举出的候选路径数量。
 */
template <typename CandidateVisitor>
inline std::size_t enumerateLeafClassificationInteriorBridgeCandidatesFromPoints(
    const PlanePoint3i &referencePoint,
    const std::vector<PlanePoint3i> &targetPoints,
    const AABB3i &box,
    CandidateVisitor &&visitor)
{
    REEMBER_PROFILE_ZONE("enumerateLeafClassificationInteriorBridgeCandidatesFromPoints");

    std::size_t emitted = 0;
    if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        return emitted;

    const std::vector<PlanePoint3i> bridgePoints =
        detail::enumerateAABBInteriorBridgePoints(referencePoint, box);
    for (const PlanePoint3i &bridgePoint : bridgePoints)
    {
        std::vector<LeafClassificationPathCandidate> prefixCandidates;
        enumerateLeafClassificationFastPathCandidatesFromPoints(
            referencePoint,
            std::vector<PlanePoint3i> {bridgePoint},
            box,
            [&prefixCandidates](LeafClassificationPathCandidate candidate)
        {
            prefixCandidates.push_back(std::move(candidate));
            return true;
        });
        enumerateLeafClassificationFallbackPathCandidatesFromPoints(
            referencePoint,
            std::vector<PlanePoint3i> {bridgePoint},
            box,
            [&prefixCandidates](LeafClassificationPathCandidate candidate)
        {
            prefixCandidates.push_back(std::move(candidate));
            return true;
        });

        if (prefixCandidates.empty())
            continue;

        bool keepGoing = true;
        for (const LeafClassificationPathCandidate &prefixCandidate : prefixCandidates)
        {
            const auto emitWithPrefix =
                [&](LeafClassificationPathCandidate candidate) -> bool
            {
                std::vector<Segment256> path;
                path.reserve(prefixCandidate.path.size() + candidate.path.size());
                path.insert(path.end(), prefixCandidate.path.begin(), prefixCandidate.path.end());
                path.insert(
                    path.end(),
                    std::make_move_iterator(candidate.path.begin()),
                    std::make_move_iterator(candidate.path.end()));

                ++emitted;
                return visitor(LeafClassificationPathCandidate{candidate.targetPoint, std::move(path)});
            };

            enumerateLeafClassificationFastPathCandidatesFromPoints(
                bridgePoint,
                targetPoints,
                box,
                [&](LeafClassificationPathCandidate candidate)
            {
                keepGoing = emitWithPrefix(std::move(candidate));
                return keepGoing;
            });

            if (!keepGoing)
                return emitted;

            enumerateLeafClassificationFallbackPathCandidatesFromPoints(
                bridgePoint,
                targetPoints,
                box,
                [&](LeafClassificationPathCandidate candidate)
            {
                keepGoing = emitWithPrefix(std::move(candidate));
                return keepGoing;
            });

            if (!keepGoing)
                return emitted;
        }
    }

    return emitted;
}

/**
 * @brief 从已生成的内部点收集快速分类路径候选。
 */
inline std::vector<LeafClassificationPathCandidate> enumerateLeafClassificationFastPathCandidatesFromPoints(
    const PlanePoint3i &referencePoint,
    const std::vector<PlanePoint3i> &targetPoints,
    const AABB3i &box)
{
    std::vector<LeafClassificationPathCandidate> candidates;
    enumerateLeafClassificationFastPathCandidatesFromPoints(
        referencePoint,
        targetPoints,
        box,
        [&candidates](LeafClassificationPathCandidate candidate)
    {
        candidates.push_back(std::move(candidate));
        return true;
    });
    return candidates;
}

/**
 * @brief 从已生成的内部点收集兜底分类路径候选。
 */
inline std::vector<LeafClassificationPathCandidate> enumerateLeafClassificationFallbackPathCandidatesFromPoints(
    const PlanePoint3i &referencePoint,
    const std::vector<PlanePoint3i> &targetPoints,
    const AABB3i &box)
{
    std::vector<LeafClassificationPathCandidate> candidates;
    enumerateLeafClassificationFallbackPathCandidatesFromPoints(
        referencePoint,
        targetPoints,
        box,
        [&candidates](LeafClassificationPathCandidate candidate)
    {
        candidates.push_back(std::move(candidate));
        return true;
    });
    return candidates;
}

/**
 * @brief 枚举从局部参考点到叶子多边形内部点的全部分类路径候选。
 *
 * @param[in] referencePoint 当前叶子子问题的局部参考点。
 * @param[in] polygon 待分类的叶子多边形。
 * @param[in] box 当前叶子子问题的 AABB。
 * @return 快速候选在前，兜底候选在后。
 *
 * @note 热路径应直接使用基于已生成点的快速/兜底接口，避免成功路径预先构造全量兜底候选。
 */
inline std::vector<LeafClassificationPathCandidate> enumerateLeafClassificationPathCandidates(
    const PlanePoint3i &referencePoint,
    const Polygon256 &polygon,
    const AABB3i &box)
{
    std::vector<LeafClassificationPathCandidate> candidates;
    if (!referencePoint.hasUniqueIntersection() || !polygon.isValid() || !isValidAABB(box))
        return candidates;

    const std::vector<PlanePoint3i> targetPoints = detail::enumerateLeafClassificationPointCandidatesUnchecked(polygon);
    candidates = enumerateLeafClassificationFastPathCandidatesFromPoints(referencePoint, targetPoints, box);
    std::vector<LeafClassificationPathCandidate> fallbackCandidates =
        enumerateLeafClassificationFallbackPathCandidatesFromPoints(referencePoint, targetPoints, box);
    candidates.insert(
        candidates.end(),
        std::make_move_iterator(fallbackCandidates.begin()),
        std::make_move_iterator(fallbackCandidates.end()));
    return candidates;
}
}
