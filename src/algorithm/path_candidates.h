/**
 * @file path_candidates.h
 * @brief Declares public candidate types and enumeration entry points for tracing paths.
 */
#pragma once

#include "algorithm/path_candidate_details.h"

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


    /**
     * @brief 枚举以目标 fragment 支撑平面法向作为最后一段的分类路径候选。
     *
     * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
     * @param[in] referencePoint 当前叶子子问题的局部参考点。
     * @param[in] targetPoints 已生成的 leaf polygon 严格内部点。
     * @param[in] surfacePlane 待分类 fragment 的支撑平面。
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
        std::size_t emitted = 0;
        if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        {
            return emitted;
        }

        constexpr std::array<int, 10> offsets = {1, -1, 2, -2, 4, -4, 8, -8, 16, -16};
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
                {
                    continue;
                }

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
                    {
                        continue;
                    }

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

    inline std::vector<AABBPathCandidate> enumerateAABBPathCandidates(const PlanePoint3i &startPoint, const AABB3i &targetBox)
    {
        std::vector<AABBPathCandidate> candidates;
        if (!startPoint.hasUniqueIntersection() || !isValidAABB(targetBox))
        {
            return candidates;
        }

        const PlanePoint3i preferredTarget = projectPointToAABB(startPoint, targetBox);

        auto appendCoordinatePathCandidates =
            [&candidates, &startPoint](const PlanePoint3i &targetPoint)
        {
            if (!targetPoint.hasUniqueIntersection())
            {
                return;
            }

            const std::array<Plane3i, 3> startCoordinatePlanes = {
                detail::makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::X),
                detail::makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Y),
                detail::makeCoordinatePlaneFromPoint(startPoint, SplitAxis3i::Z)};
            const std::array<Plane3i, 3> targetCoordinatePlanes = {
                detail::makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::X),
                detail::makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Y),
                detail::makeCoordinatePlaneFromPoint(targetPoint, SplitAxis3i::Z)};

            std::vector<SplitAxis3i> changedAxes;
            changedAxes.reserve(3);
            if (!detail::areSamePlaneEquation(startCoordinatePlanes[0], targetCoordinatePlanes[0]))
            {
                changedAxes.push_back(SplitAxis3i::X);
            }
            if (!detail::areSamePlaneEquation(startCoordinatePlanes[1], targetCoordinatePlanes[1]))
            {
                changedAxes.push_back(SplitAxis3i::Y);
            }
            if (!detail::areSamePlaneEquation(startCoordinatePlanes[2], targetCoordinatePlanes[2]))
            {
                changedAxes.push_back(SplitAxis3i::Z);
            }

            if (changedAxes.empty())
            {
                candidates.push_back({targetPoint, {}});
                return;
            }

            std::sort(
                changedAxes.begin(),
                changedAxes.end(),
                [](SplitAxis3i lhs, SplitAxis3i rhs)
                {
                    return detail::axisOrderKey(lhs) < detail::axisOrderKey(rhs);
                });

            do
            {
                std::vector<Segment256> path;
                if (detail::buildAxisAlignedFreeCoordinatePath(startPoint, targetPoint, changedAxes, path))
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
        };

        auto appendInteriorCenterCandidates =
            [&appendCoordinatePathCandidates, &targetBox]()
        {
            const PlanePoint3i centerPoint(
                Plane3i(2, 0, 0, -(targetBox.xMin + targetBox.xMax)),
                Plane3i(0, 2, 0, -(targetBox.yMin + targetBox.yMax)),
                Plane3i(0, 0, 2, -(targetBox.zMin + targetBox.zMax)));
            appendCoordinatePathCandidates(centerPoint);
        };

        Integer startX, startY, startZ;
        Integer preferredX, preferredY, preferredZ;
        if (!detail::tryExtractExactIntegerPoint(startPoint, startX, startY, startZ) ||
            !detail::tryExtractExactIntegerPoint(preferredTarget, preferredX, preferredY, preferredZ))
        {
            appendCoordinatePathCandidates(preferredTarget);
            appendInteriorCenterCandidates();
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

        appendInteriorCenterCandidates();
        return candidates;
    }

    /**
     * @brief 为 leaf polygon 生成严格内部分类点候选。
     *
     * @param[in] polygon 待分类的 leaf polygon。
     * @return 先返回论文中的重心舍入探测线命中点，再返回确定性向内偏移 fallback 点。
     */
    inline std::vector<PlanePoint3i> enumerateLeafClassificationPointCandidates(const Polygon256 &polygon)
    {
        if (!polygon.isValid())
        {
            return {};
        }

        return detail::enumerateLeafClassificationPointCandidatesUnchecked(polygon);
    }

    /**
     * @brief 枚举快速分类路径候选，并逐个交给调用方处理。
     *
     * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
     * @param[in] referencePoint 当前叶子子问题的局部参考点。
     * @param[in] targetPoints 已生成的 leaf polygon 严格内部点。
     * @param[in] box 当前叶子子问题的 AABB。
     * @param[in,out] visitor 候选路径访问器。
     * @return 实际枚举出的候选路径数量。
     *
     * @pre `targetPoints` 已由 leaf 分类点生成阶段保证为严格内部点。
     */
    template <typename CandidateVisitor>
    inline std::size_t enumerateLeafClassificationFastPathCandidatesFromPoints(
        const PlanePoint3i &referencePoint,
        const std::vector<PlanePoint3i> &targetPoints,
        const AABB3i &box,
        CandidateVisitor &&visitor)
    {
        std::size_t emitted = 0;
        if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        {
            return emitted;
        }

        for (const PlanePoint3i &targetPoint : targetPoints)
        {
            auto emitCandidate =
                [&](std::vector<Segment256> path) -> bool
            {
                ++emitted;
                return visitor(LeafClassificationPathCandidate{targetPoint, std::move(path)});
            };

            Integer referenceX, referenceY, referenceZ;
            Integer targetX, targetY, targetZ;
            if (detail::tryExtractExactIntegerPoint(referencePoint, referenceX, referenceY, referenceZ) &&
                detail::tryExtractExactIntegerPoint(targetPoint, targetX, targetY, targetZ))
            {
                std::vector<SplitAxis3i> changedAxes;
                if (referenceX != targetX)
                {
                    changedAxes.push_back(SplitAxis3i::X);
                }
                if (referenceY != targetY)
                {
                    changedAxes.push_back(SplitAxis3i::Y);
                }
                if (referenceZ != targetZ)
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
                    if (!emitCandidate({}))
                    {
                        return emitted;
                    }
                }
                else
                {
                    std::vector<SplitAxis3i> axisOrder = changedAxes;
                    do
                    {
                        std::vector<Segment256> path;
                        if (detail::buildAxisAlignedCoordinatePath(referencePoint, targetPoint, box, axisOrder, path))
                        {
                            if (!emitCandidate(std::move(path)))
                            {
                                return emitted;
                            }
                        }
                    } while (std::next_permutation(
                        axisOrder.begin(),
                        axisOrder.end(),
                        [](SplitAxis3i lhs, SplitAxis3i rhs)
                        {
                            return detail::axisOrderKey(lhs) < detail::axisOrderKey(rhs);
                        }));
                }
            }

            std::vector<int> changedPlaneIndices;
            if (!detail::areSamePlaneEquation(referencePoint.p, targetPoint.p))
            {
                changedPlaneIndices.push_back(0);
            }
            if (!detail::areSamePlaneEquation(referencePoint.q, targetPoint.q))
            {
                changedPlaneIndices.push_back(1);
            }
            if (!detail::areSamePlaneEquation(referencePoint.r, targetPoint.r))
            {
                changedPlaneIndices.push_back(2);
            }

            if (changedPlaneIndices.empty())
            {
                continue;
            }

            std::sort(changedPlaneIndices.begin(), changedPlaneIndices.end());
            do
            {
                std::vector<Segment256> path;
                if (detail::buildPlaneReplacementPath(referencePoint, targetPoint, box, changedPlaneIndices, path))
                {
                    if (!emitCandidate(std::move(path)))
                    {
                        return emitted;
                    }
                }
            } while (std::next_permutation(changedPlaneIndices.begin(), changedPlaneIndices.end()));
        }

        return emitted;
    }

    /**
     * @brief 枚举兜底分类路径候选，并逐个交给调用方处理。
     *
     * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
     * @param[in] referencePoint 当前叶子子问题的局部参考点。
     * @param[in] targetPoints 已生成的 leaf polygon 严格内部点。
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
        std::size_t emitted = 0;
        if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        {
            return emitted;
        }

        for (const PlanePoint3i &targetPoint : targetPoints)
        {
            const std::array<Plane3i, 3> targetPlanes = {targetPoint.p, targetPoint.q, targetPoint.r};
            std::array<int, 3> targetPlaneOrder = {0, 1, 2};
            do
            {
                if (targetPlaneOrder[0] == 0 && targetPlaneOrder[1] == 1 && targetPlaneOrder[2] == 2)
                {
                    continue;
                }

                const PlanePoint3i permutedTargetPoint = detail::makePointFromPlanes({
                    targetPlanes[targetPlaneOrder[0]],
                    targetPlanes[targetPlaneOrder[1]],
                    targetPlanes[targetPlaneOrder[2]]});
                if (!permutedTargetPoint.hasUniqueIntersection() ||
                    !areSamePlanePoint(permutedTargetPoint, targetPoint))
                {
                    continue;
                }

                std::vector<int> changedPlaneIndices;
                if (!detail::areSamePlaneEquation(referencePoint.p, permutedTargetPoint.p))
                {
                    changedPlaneIndices.push_back(0);
                }
                if (!detail::areSamePlaneEquation(referencePoint.q, permutedTargetPoint.q))
                {
                    changedPlaneIndices.push_back(1);
                }
                if (!detail::areSamePlaneEquation(referencePoint.r, permutedTargetPoint.r))
                {
                    changedPlaneIndices.push_back(2);
                }
                if (changedPlaneIndices.empty())
                {
                    continue;
                }

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
     * @brief 先桥接到 AABB 严格内部点，再枚举 leaf 分类路径候选。
     *
     * 这些候选保留原有 clean-crossing 判定，只在常规轴对齐路径贴着 cell/split 边界失效后提供
     * 少量确定性绕行路径。
     *
     * @tparam CandidateVisitor 接收 `LeafClassificationPathCandidate` 的回调；返回 `false` 表示停止枚举。
     * @param[in] referencePoint 当前叶子子问题的局部参考点。
     * @param[in] targetPoints 已生成的 leaf polygon 严格内部点。
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
        std::size_t emitted = 0;
        if (!referencePoint.hasUniqueIntersection() || !isValidAABB(box))
        {
            return emitted;
        }

        const std::vector<PlanePoint3i> bridgePoints =
            detail::enumerateAABBInteriorBridgePoints(referencePoint, box);
        for (const PlanePoint3i &bridgePoint : bridgePoints)
        {
            std::vector<LeafClassificationPathCandidate> prefixCandidates;
            enumerateLeafClassificationFastPathCandidatesFromPoints(
                referencePoint,
                std::vector<PlanePoint3i>{bridgePoint},
                box,
                [&prefixCandidates](LeafClassificationPathCandidate candidate)
                {
                    prefixCandidates.push_back(std::move(candidate));
                    return true;
                });
            enumerateLeafClassificationFallbackPathCandidatesFromPoints(
                referencePoint,
                std::vector<PlanePoint3i>{bridgePoint},
                box,
                [&prefixCandidates](LeafClassificationPathCandidate candidate)
                {
                    prefixCandidates.push_back(std::move(candidate));
                    return true;
                });

            if (prefixCandidates.empty())
            {
                continue;
            }

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
                {
                    return emitted;
                }

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
                {
                    return emitted;
                }
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
     * @brief 枚举从局部参考点到 leaf polygon 内部点的全部分类路径候选。
     *
     * @param[in] referencePoint 当前叶子子问题的局部参考点。
     * @param[in] polygon 待分类的 leaf polygon。
     * @param[in] box 当前叶子子问题的 AABB。
     * @return 快速候选在前，兜底候选在后。
     *
     * @note 热路径应直接使用 from-points 的快速/兜底接口，避免成功路径预先构造全量兜底候选。
     */
    inline std::vector<LeafClassificationPathCandidate> enumerateLeafClassificationPathCandidates(
        const PlanePoint3i &referencePoint,
        const Polygon256 &polygon,
        const AABB3i &box)
    {
        std::vector<LeafClassificationPathCandidate> candidates;
        if (!referencePoint.hasUniqueIntersection() || !polygon.isValid() || !isValidAABB(box))
        {
            return candidates;
        }

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
