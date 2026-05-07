/**
 * @file subdivision_solver.cpp
 * @brief 实现递归空间细分与子节点参考点传播。
 */
#include "core/subdivision_solver.h"

#include "algorithm/path_candidates.h"
#include "algorithm/WNV_tracing.h"
#include "core/perf_tracing.h"
#include "core/solver_shared.h"
#include "geometry/polygon_ops.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ember
{
namespace
{
std::string formatAABB(const AABB3i &box)
{
    std::ostringstream message;
    message << "{valid=" << box.valid
            << ", x=[" << box.xMin << ", " << box.xMax
            << "], y=[" << box.yMin << ", " << box.yMax
            << "], z=[" << box.zMin << ", " << box.zMax
            << "]}";
    return message.str();
}

bool isPointOnAnyPolygonSupportPlane(
    const PlanePoint3i &point,
    const std::vector<Polygon256> &polygons) noexcept
{
    for (const Polygon256 &polygon : polygons)
    {
        if (point.classify(polygon.plane) == 0)
            return true;
    }

    return false;
}

bool isPointOnAnyPolygonSurface(
    const PlanePoint3i &point,
    const std::vector<Polygon256> &polygons) noexcept
{
    for (const Polygon256 &polygon : polygons)
    {
        if (polygon.classify(point) == 0)
            return true;
    }

    return false;
}

Integer axisMinimum(const AABB3i &box, SplitAxis3i axis) noexcept
{
    switch (axis)
    {
    case SplitAxis3i::X:
        return box.xMin;
    case SplitAxis3i::Y:
        return box.yMin;
    case SplitAxis3i::Z:
        return box.zMin;
    }

    return 0;
}

Integer axisMaximum(const AABB3i &box, SplitAxis3i axis) noexcept
{
    switch (axis)
    {
    case SplitAxis3i::X:
        return box.xMax;
    case SplitAxis3i::Y:
        return box.yMax;
    case SplitAxis3i::Z:
        return box.zMax;
    }

    return 0;
}

Integer axisMidpoint(const AABB3i &box, SplitAxis3i axis) noexcept
{
    return floorDiv(axisMinimum(box, axis) + axisMaximum(box, axis), Integer(2));
}

struct BinaryWnvDomain
{
    bool canBeZero = false;
    bool canBeNonZero = false;
};

BoolStatus evaluateBinaryIndicatorState(BoolOp op, bool lhsNonZero, bool rhsNonZero) noexcept
{
    switch (op)
    {
    case BoolOp::Union:
        return (lhsNonZero || rhsNonZero) ? IN : OUT;
    case BoolOp::Intersection:
        return (lhsNonZero && rhsNonZero) ? IN : OUT;
    case BoolOp::Difference:
        return (lhsNonZero && !rhsNonZero) ? IN : OUT;
    }

    return OUT;
}

BinaryWnvDomain computeBinaryWnvDomain(
    const WNV &referenceWnv,
    const std::vector<Polygon256> &polygons,
    std::size_t dimension) noexcept
{
    BinaryWnvDomain domain;
    for (const Polygon256 &polygon : polygons)
    {
        if (dimension < polygon.WNTV.size() && polygon.WNTV[dimension] != 0)
        {
            domain.canBeZero = true;
            domain.canBeNonZero = true;
            return domain;
        }
    }

    const bool isNonZero =
        dimension < referenceWnv.size() &&
        referenceWnv[dimension] != 0;
    domain.canBeZero = !isNonZero;
    domain.canBeNonZero = isNonZero;
    return domain;
}

// 二元布尔只依赖前两维 WNV；这里用零/非零可达域保守判断 indicator 是否恒定。
bool tryEvaluateConstantBinaryIndicator(
    BoolOp op,
    const WNV &referenceWnv,
    const std::vector<Polygon256> &polygons,
    BoolStatus &constantStatus) noexcept
{
    if (referenceWnv.size() < detail::kBinaryWnvDimension)
        return false;

    const BinaryWnvDomain lhsDomain =
        computeBinaryWnvDomain(referenceWnv, polygons, detail::kLhsOperandIndex);
    const BinaryWnvDomain rhsDomain =
        computeBinaryWnvDomain(referenceWnv, polygons, detail::kRhsOperandIndex);

    bool hasStatus = false;
    for (const bool lhsNonZero : {
                false, true
            })
    {
        if ((lhsNonZero && !lhsDomain.canBeNonZero) || (!lhsNonZero && !lhsDomain.canBeZero))
            continue;

        for (const bool rhsNonZero : {
                    false, true
                })
        {
            if ((rhsNonZero && !rhsDomain.canBeNonZero) || (!rhsNonZero && !rhsDomain.canBeZero))
                continue;

            const BoolStatus status = evaluateBinaryIndicatorState(op, lhsNonZero, rhsNonZero);
            if (!hasStatus)
            {
                constantStatus = status;
                hasStatus = true;
                continue;
            }

            if (status != constantStatus)
                return false;
        }
    }

    return hasStatus;
}

bool appendPolygonBoundsToAABB(AABB3i &box, const Polygon256 &polygon)
{
    bool hasVertex = false;
    const std::vector<PlanePoint3i> &cachedVertices = polygon.vertices();
    for (const PlanePoint3i &vertex : cachedVertices)
    {
        if (!vertex.hasUniqueIntersection() || isZero(vertex.x.w))
            continue;

        const Integer fx = floorDiv(vertex.x.x, vertex.x.w);
        const Integer cx = ceilDiv(vertex.x.x, vertex.x.w);
        const Integer fy = floorDiv(vertex.x.y, vertex.x.w);
        const Integer cy = ceilDiv(vertex.x.y, vertex.x.w);
        const Integer fz = floorDiv(vertex.x.z, vertex.x.w);
        const Integer cz = ceilDiv(vertex.x.z, vertex.x.w);

        if (!box.valid)
        {
            box.xMin = fx;
            box.xMax = cx;
            box.yMin = fy;
            box.yMax = cy;
            box.zMin = fz;
            box.zMax = cz;
            box.valid = true;
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

        hasVertex = true;
    }

    return hasVertex;
}

void appendAABBToAABB(AABB3i &box, const AABB3i &source) noexcept
{
    if (!isValidAABB(source))
        return;

    if (!box.valid)
    {
        box = source;
        return;
    }

    if (source.xMin < box.xMin) box.xMin = source.xMin;
    if (source.xMax > box.xMax) box.xMax = source.xMax;
    if (source.yMin < box.yMin) box.yMin = source.yMin;
    if (source.yMax > box.yMax) box.yMax = source.yMax;
    if (source.zMin < box.zMin) box.zMin = source.zMin;
    if (source.zMax > box.zMax) box.zMax = source.zMax;
}

struct WntvSubdivisionGroup
{
    WNV wntv;
    AABB3i box;
    Integer centerCount = 0;
    Integer sumCenterX = 0;
    Integer sumCenterY = 0;
    Integer sumCenterZ = 0;
    Integer centerX = 0;
    Integer centerY = 0;
    Integer centerZ = 0;
    std::size_t polygonCount = 0;
};

Integer groupCenterCoordinate(const WntvSubdivisionGroup &group, SplitAxis3i axis) noexcept
{
    switch (axis)
    {
    case SplitAxis3i::X:
        return group.centerX;
    case SplitAxis3i::Y:
        return group.centerY;
    case SplitAxis3i::Z:
        return group.centerZ;
    }

    return 0;
}

struct PolygonCenterSplitStats
{
    bool valid = false;
    Integer count = 0;
    Integer sumX = 0;
    Integer sumY = 0;
    Integer sumZ = 0;
    Integer minX = 0;
    Integer maxX = 0;
    Integer minY = 0;
    Integer maxY = 0;
    Integer minZ = 0;
    Integer maxZ = 0;
};

struct SubdivisionSplitStats
{
    std::vector<WntvSubdivisionGroup> wntvGroups;
    PolygonCenterSplitStats centerStats;
};

void appendCenterCoordinate(
    const Integer &coordinate,
    Integer &minCoordinate,
    Integer &maxCoordinate,
    bool initialize)
{
    if (initialize)
    {
        minCoordinate = coordinate;
        maxCoordinate = coordinate;
        return;
    }

    if (coordinate < minCoordinate) minCoordinate = coordinate;
    if (coordinate > maxCoordinate) maxCoordinate = coordinate;
}

bool buildSubdivisionSplitStats(
    const std::vector<Polygon256> &polygons,
    SubdivisionSplitStats &stats)
{
    REEMBER_PROFILE_ZONE("buildSubdivisionSplitStats");

    stats.wntvGroups.clear();
    stats.centerStats = PolygonCenterSplitStats();
    stats.wntvGroups.reserve(polygons.size());
    for (const Polygon256 &polygon : polygons)
    {
        auto it = std::find_if(
                      stats.wntvGroups.begin(),
                      stats.wntvGroups.end(),
                      [&polygon](const WntvSubdivisionGroup &group)
        {
            return group.wntv == polygon.WNTV;
        });
        if (it == stats.wntvGroups.end())
        {
            WntvSubdivisionGroup group;
            group.wntv = polygon.WNTV;
            stats.wntvGroups.push_back(std::move(group));
            it = stats.wntvGroups.end() - 1;
        }

        AABB3i polygonBox;
        if (!appendPolygonBoundsToAABB(polygonBox, polygon) || !isValidAABB(polygonBox))
            return false;

        const Integer centerX = axisMidpoint(polygonBox, SplitAxis3i::X);
        const Integer centerY = axisMidpoint(polygonBox, SplitAxis3i::Y);
        const Integer centerZ = axisMidpoint(polygonBox, SplitAxis3i::Z);

        appendAABBToAABB(it->box, polygonBox);
        it->sumCenterX += centerX;
        it->sumCenterY += centerY;
        it->sumCenterZ += centerZ;
        ++it->centerCount;
        ++it->polygonCount;

        const bool initialize = !stats.centerStats.valid;
        appendCenterCoordinate(centerX, stats.centerStats.minX, stats.centerStats.maxX, initialize);
        appendCenterCoordinate(centerY, stats.centerStats.minY, stats.centerStats.maxY, initialize);
        appendCenterCoordinate(centerZ, stats.centerStats.minZ, stats.centerStats.maxZ, initialize);
        stats.centerStats.sumX += centerX;
        stats.centerStats.sumY += centerY;
        stats.centerStats.sumZ += centerZ;
        ++stats.centerStats.count;
        stats.centerStats.valid = true;
    }

    for (WntvSubdivisionGroup &group : stats.wntvGroups)
    {
        if (!isValidAABB(group.box) || group.polygonCount == 0 || group.centerCount <= 0)
            return false;

        group.centerX = floorDiv(group.sumCenterX, group.centerCount);
        group.centerY = floorDiv(group.sumCenterY, group.centerCount);
        group.centerZ = floorDiv(group.sumCenterZ, group.centerCount);
    }

    return stats.centerStats.valid && stats.centerStats.count > 0;
}

bool considerWntvSeparationCandidate(
    const AABB3i &box,
    SplitAxis3i axis,
    const Integer &centerCoordinate,
    const AABB3i &otherBox,
    bool &hasBestCandidate,
    Integer &bestDistance,
    AABBSplit3i &bestSplit)
{
    Integer coordinate = 0;
    Integer distance = 0;
    const Integer otherMin = axisMinimum(otherBox, axis);
    const Integer otherMax = axisMaximum(otherBox, axis);

    if (centerCoordinate < otherMin)
    {
        coordinate = otherMin - 1;
        distance = otherMin - centerCoordinate;
    }
    else if (centerCoordinate > otherMax)
    {
        coordinate = otherMax + 1;
        distance = centerCoordinate - otherMax;
    }
    else
        return false;

    AABBSplit3i candidate;
    if (!splitAABBAtCoordinate(box, axis, coordinate, candidate))
        return false;

    if (!hasBestCandidate || distance > bestDistance)
    {
        hasBestCandidate = true;
        bestDistance = distance;
        bestSplit = candidate;
    }

    return true;
}

// 论文 4.5.3 策略：优先选能把某个 WNTV 类整体隔到单侧的轴向切分面。
bool chooseWntvAwareSplit(
    const std::vector<WntvSubdivisionGroup> &groups,
    const AABB3i &box,
    AABBSplit3i &outSplit)
{
    REEMBER_PROFILE_ZONE("chooseWntvAwareSplit");

    if (groups.size() < 2u)
        return false;

    bool hasCandidate = false;
    Integer bestDistance = 0;
    AABBSplit3i bestSplit;
    for (std::size_t centerGroupIndex = 0; centerGroupIndex < groups.size(); ++centerGroupIndex)
    {
        const WntvSubdivisionGroup &centerGroup = groups[centerGroupIndex];
        for (std::size_t otherGroupIndex = 0; otherGroupIndex < groups.size(); ++otherGroupIndex)
        {
            if (centerGroupIndex == otherGroupIndex)
                continue;

            const WntvSubdivisionGroup &otherGroup = groups[otherGroupIndex];
            for (const SplitAxis3i axis : {
                        SplitAxis3i::X, SplitAxis3i::Y, SplitAxis3i::Z
                    })
            {
                considerWntvSeparationCandidate(
                    box,
                    axis,
                    groupCenterCoordinate(centerGroup, axis),
                    otherGroup.box,
                    hasCandidate,
                    bestDistance,
                    bestSplit);
            }
        }
    }

    if (!hasCandidate)
        return false;

    outSplit = bestSplit;
    return true;
}

Integer centerRange(const PolygonCenterSplitStats &stats, SplitAxis3i axis) noexcept
{
    switch (axis)
    {
    case SplitAxis3i::X:
        return stats.maxX - stats.minX;
    case SplitAxis3i::Y:
        return stats.maxY - stats.minY;
    case SplitAxis3i::Z:
        return stats.maxZ - stats.minZ;
    }

    return 0;
}

Integer averageCenterCoordinate(const PolygonCenterSplitStats &stats, SplitAxis3i axis) noexcept
{
    switch (axis)
    {
    case SplitAxis3i::X:
        return floorDiv(stats.sumX, stats.count);
    case SplitAxis3i::Y:
        return floorDiv(stats.sumY, stats.count);
    case SplitAxis3i::Z:
        return floorDiv(stats.sumZ, stats.count);
    }

    return 0;
}

enum class SubdivisionSplitStrategy
{
    WntvAware,
    CenterRange,
    Midpoint
};

void recordSplitStrategyMetrics(
    BoolSolveMetrics &metrics,
    SubdivisionSplitStrategy splitStrategy) noexcept
{
    switch (splitStrategy)
    {
    case SubdivisionSplitStrategy::WntvAware:
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::splitStrategyWntvAware");
        ++metrics.wntvAwareSplitCount;
        break;
    }
    case SubdivisionSplitStrategy::CenterRange:
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::splitStrategyCenterRange");
        ++metrics.centerRangeSplitCount;
        break;
    }
    case SubdivisionSplitStrategy::Midpoint:
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::splitStrategyMidpoint");
        ++metrics.midpointSplitCount;
        break;
    }
    }
}

// 无 WNTV 分离候选时，用多边形中心范围近似最大方差轴，再按平均中心切分。
bool chooseCenterRangeSplit(
    const PolygonCenterSplitStats &stats,
    const AABB3i &box,
    AABBSplit3i &outSplit)
{
    REEMBER_PROFILE_ZONE("chooseCenterRangeSplit");

    if (!stats.valid || stats.count <= 0)
        return false;

    bool hasCandidate = false;
    Integer bestRange = 0;
    AABBSplit3i bestSplit;
    for (const SplitAxis3i axis : {
                SplitAxis3i::X, SplitAxis3i::Y, SplitAxis3i::Z
            })
    {
        const Integer range = centerRange(stats, axis);
        if (range <= 1)
            continue;

        AABBSplit3i candidate;
        if (!splitAABBAtCoordinate(box, axis, averageCenterCoordinate(stats, axis), candidate))
            continue;

        if (!hasCandidate || range > bestRange)
        {
            hasCandidate = true;
            bestRange = range;
            bestSplit = candidate;
        }
    }

    if (!hasCandidate)
        return false;

    outSplit = bestSplit;
    return true;
}

bool chooseSubdivisionSplit(
    const std::vector<Polygon256> &polygons,
    const AABB3i &box,
    AABBSplit3i &outSplit,
    SubdivisionSplitStrategy &outStrategy)
{
    REEMBER_PROFILE_ZONE("chooseSubdivisionSplit");

    SubdivisionSplitStats splitStats;
    const bool hasSplitStats = buildSubdivisionSplitStats(polygons, splitStats);

    if (hasSplitStats && chooseWntvAwareSplit(splitStats.wntvGroups, box, outSplit))
    {
        outStrategy = SubdivisionSplitStrategy::WntvAware;
        return true;
    }
    if (hasSplitStats && chooseCenterRangeSplit(splitStats.centerStats, box, outSplit))
    {
        outStrategy = SubdivisionSplitStrategy::CenterRange;
        return true;
    }

    if (splitAABBAtMidpoint(box, outSplit))
    {
        outStrategy = SubdivisionSplitStrategy::Midpoint;
        return true;
    }

    return false;
}

bool appendSplitChildPolygons(
    const Polygon256 &polygon,
    const Plane3i &splitPlane,
    std::vector<Polygon256> &leftPolygons,
    std::vector<Polygon256> &rightPolygons)
{
    REEMBER_PROFILE_ZONE("appendSplitChildPolygons");

    bool hasPositive = false;
    bool hasNegative = false;
    const std::size_t edgeCount = polygon.edgeCount();
    std::vector<int> vertexSides;
    vertexSides.resize(edgeCount);
    for (std::size_t edgeIndex = 0; edgeIndex < edgeCount; ++edgeIndex)
    {
        const PlanePoint3i &vertex = getPolygonVertex(polygon, edgeIndex);
        const int side = vertex.classify(splitPlane);
        vertexSides[edgeIndex] = side;
        if (side > 0)
            hasPositive = true;
        else if (side < 0)
            hasNegative = true;
    }

    if (!hasPositive)
    {
        leftPolygons.push_back(polygon);
        return true;
    }
    if (!hasNegative)
    {
        rightPolygons.push_back(polygon);
        return true;
    }

    Polygon256 frontClipped;
    Polygon256 backClipped;
    if (!detail::clipLeafGeometryByPlaneTrustedWithSides(
                polygon,
                splitPlane,
                vertexSides,
                frontClipped,
                backClipped,
                PolygonEdgeProvenance::SubdivisionClip))
        return false;

    leftPolygons.push_back(std::move(backClipped));
    rightPolygons.push_back(std::move(frontClipped));
    return true;
}

bool isPointStrictlyInsideAABB(const PlanePoint3i &point, const AABB3i &box) noexcept
{
    Integer x;
    Integer y;
    Integer z;
    return detail::tryExtractExactIntegerPoint(point, x, y, z) &&
           x > box.xMin && x < box.xMax &&
           y > box.yMin && y < box.yMax &&
           z > box.zMin && z < box.zMax;
}

struct ChildReferenceSearchState
{
    SubdivisionRefState &outReference;
    std::vector<Segment256> candidatePath;
    bool hardFailure = false;
};

enum class ChildReferenceCandidatePhase
{
    Fast,
    Exhaustive
};

bool tryTraceChildReferenceCandidate(
    std::size_t depth,
    const PlanePoint3i &sourcePoint,
    bool sourceReferenceIsStrictInterior,
    const refPoint &sourceRef,
    const std::vector<Polygon256> &nodePolygons,
    const std::vector<Polygon256> &childPolygons,
    ChildReferenceCandidatePhase candidatePhase,
    const detail::AABBPathCandidateSeed &candidateSeed,
    BoolSolveMetrics &solveMetrics,
    ChildReferenceSearchState &searchState)
{
    if (candidatePhase == ChildReferenceCandidatePhase::Fast)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::childReferenceFastCandidate");
        ++solveMetrics.childReferenceFastCandidateCount;
    }
    else
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::childReferenceExhaustiveCandidate");
        ++solveMetrics.childReferenceExhaustiveCandidateCount;
    }

    ++solveMetrics.childReferenceCandidateCount;

    if (!sourceReferenceIsStrictInterior &&
            candidateSeed.buildMode == detail::AABBPathBuildMode::Empty &&
            areSamePlanePoint(candidateSeed.targetPoint, sourcePoint) &&
            isPointOnAnyPolygonSupportPlane(candidateSeed.targetPoint, childPolygons))
        return true;

    if (isPointOnAnyPolygonSurface(candidateSeed.targetPoint, childPolygons))
        return true;

    if (!detail::buildAABBPathFromSeed(sourcePoint, candidateSeed, searchState.candidatePath))
        return true;

    WNV propagatedWNV;
    traceStatus status = FAIL;
    {
        if (candidatePhase == ChildReferenceCandidatePhase::Fast)
        {
            REEMBER_PROFILE_ZONE("SubdivisionSolver::childReferenceFastCandidateTrace");
            ++solveMetrics.childReferenceFastCandidateTriedCount;
        }
        else
        {
            REEMBER_PROFILE_ZONE("SubdivisionSolver::childReferenceExhaustiveCandidateTrace");
            ++solveMetrics.childReferenceExhaustiveCandidateTriedCount;
        }

        ++solveMetrics.childReferenceCandidateTriedCount;
        status = detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                     sourceRef,
                     searchState.candidatePath,
                     nodePolygons,
                     propagatedWNV);
    }
    if (status == SUCCESS)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::childReferenceTraceSuccess");
        ++solveMetrics.childReferenceTraceCount;
        searchState.outReference.point = candidateSeed.targetPoint;
        searchState.outReference.wnv = std::move(propagatedWNV);
        return false;
    }

    if (status != PATH_INVALID)
    {
        searchState.hardFailure = true;
        return false;
    }

    return true;
}
}

SubdivisionSolver::SubdivisionSolver(
    BoolOp op,
    std::size_t leafPolygonThreshold,
    const std::vector<Polygon256> &polygons,
    const AABB3i &rootAABB,
    BoolOperandAssumptions lhsAssumptions,
    BoolOperandAssumptions rhsAssumptions)
    : op_(op),
      leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold),
      lhsAssumptions_(lhsAssumptions),
      rhsAssumptions_(rhsAssumptions),
      aabb_(rootAABB),
      polygons_(polygons)
{
}

SubdivisionSolver::SubdivisionSolver(
    BoolOp op,
    std::size_t leafPolygonThreshold,
    std::size_t depth,
    std::vector<Polygon256> polygons,
    const AABB3i &aabb,
    SubdivisionRefState reference,
    BoolOperandAssumptions lhsAssumptions,
    BoolOperandAssumptions rhsAssumptions)
    : op_(op),
      leafPolygonThreshold_(leafPolygonThreshold == 0 ? 1 : leafPolygonThreshold),
      lhsAssumptions_(lhsAssumptions),
      rhsAssumptions_(rhsAssumptions),
      depth_(depth),
      aabb_(aabb),
      reference_(std::move(reference)),
      polygons_(std::move(polygons))
{
}

void SubdivisionSolver::solve()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::solve");

    const AABB3i rootAABB = aabb_;
    resetSolveState();
    aabb_ = rootAABB;
    solveMetrics_.inputPolygonCount = polygons_.size();
    if (polygons_.empty())
    {
        discarded_ = true;
        solved_ = true;
        return;
    }

    if (!isValidAABB(aabb_))
    {
        std::ostringstream message;
        message << "SubdivisionSolver received an invalid root AABB root_aabb="
                << formatAABB(aabb_)
                << ".";
        throw std::runtime_error(message.str());
    }

    initializeRootReference();
    solveRecursive();
    leafSummaries_.clear();
    collectLeafSummaries(leafSummaries_);
    BoolSolveMetrics aggregatedMetrics;
    aggregatedMetrics.inputPolygonCount = polygons_.size();
    aggregatedMetrics.resultFragmentCount = resultFragments_.size();
    collectSolveMetrics(aggregatedMetrics);
    solveMetrics_ = aggregatedMetrics;
}

bool SubdivisionSolver::isDiscarded() const noexcept
{
    return discarded_;
}

const std::vector<Polygon256> &SubdivisionSolver::resultFragments() const noexcept
{
    return resultFragments_;
}

const std::vector<BoolLeafSummary> &SubdivisionSolver::leafSummaries() const noexcept
{
    return leafSummaries_;
}

const BoolSolveMetrics &SubdivisionSolver::solveMetrics() const noexcept
{
    return solveMetrics_;
}

void SubdivisionSolver::resetSolveState() noexcept
{
    depth_ = 0;
    isLeaf_ = true;
    discarded_ = false;
    solved_ = false;
    splitPlane_ = Plane3i();
    aabb_ = AABB3i();
    reference_ = SubdivisionRefState();
    leafFragments_.clear();
    classifiedFragments_.clear();
    resultFragments_.clear();
    leafSummaries_.clear();
    solveMetrics_ = BoolSolveMetrics();
    leftChild_.reset();
    rightChild_.reset();
}

void SubdivisionSolver::initializeRootReference()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::initializeRootReference");

    reference_.point = getAABBCornerPoint(aabb_, false, false, false);
    reference_.wnv.assign(detail::kBinaryWnvDimension, 0);
}

void SubdivisionSolver::finishCurrentNodeAsLeaf()
{
    isLeaf_ = true;
    solveLeafArrangement();
    classifyLeafFragmentsAndCollectResults();
    solved_ = true;
}

bool SubdivisionSolver::tryDiscardInvalidOrEmptyNode()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::tryDiscardInvalidOrEmptyNode");

    if (!polygons_.empty() && isValidAABB(aabb_))
        return false;

    ++solveMetrics_.invalidOrEmptyDiscardCount;
    discarded_ = true;
    isLeaf_ = true;
    solved_ = true;
    return true;
}

bool SubdivisionSolver::tryDiscardConstantIndicatorNode()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::tryDiscardConstantIndicatorNode");

    BoolStatus earlyOutStatus = OUT;
    if (!shouldDiscardSubproblemEarly(earlyOutStatus))
        return false;

    ++solveMetrics_.constantDiscardCount;
    discarded_ = true;
    isLeaf_ = true;
    solved_ = true;
    return true;
}

bool SubdivisionSolver::tryFinishStoppedSubdivisionNode()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::tryFinishStoppedSubdivisionNode");

    if (!shouldStopSubdivision())
        return false;

    const bool stoppedByLeafThreshold = polygons_.size() <= leafPolygonThreshold_;
    if (stoppedByLeafThreshold)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::stopByLeafThreshold");
        ++solveMetrics_.leafThresholdStopCount;
    }
    else
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::stopByAabbNotSplittable");
        ++solveMetrics_.aabbNotSplittableStopCount;
    }

    finishCurrentNodeAsLeaf();
    return true;
}

void SubdivisionSolver::mergeSolvedChildren()
{
    resultFragments_.clear();
    if (leftChild_ && !leftChild_->discarded_)
    {
        resultFragments_.insert(
            resultFragments_.end(),
            leftChild_->resultFragments_.begin(),
            leftChild_->resultFragments_.end());
    }
    if (rightChild_ && !rightChild_->discarded_)
    {
        resultFragments_.insert(
            resultFragments_.end(),
            rightChild_->resultFragments_.begin(),
            rightChild_->resultFragments_.end());
    }

    discarded_ =
        (!leftChild_ || leftChild_->discarded_) &&
        (!rightChild_ || rightChild_->discarded_);
    solved_ = true;
}

// 递归推进 subdivision；到达叶子后立即执行局部 BSP 和 WNV 分类。
void SubdivisionSolver::solveRecursive()
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::solveRecursive");

    if (tryDiscardInvalidOrEmptyNode() || tryDiscardConstantIndicatorNode())
        return;

    if (trySolveSingleOperandAssumptionLeaf())
    {
        solved_ = true;
        return;
    }

    if (tryFinishStoppedSubdivisionNode())
        return;

    AABBSplit3i split;
    SubdivisionSplitStrategy splitStrategy = SubdivisionSplitStrategy::Midpoint;
    if (!chooseSubdivisionSplit(polygons_, aabb_, split, splitStrategy))
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::stopBySplitFailure");
        ++solveMetrics_.splitFailureStopCount;
        finishCurrentNodeAsLeaf();
        return;
    }

    recordSplitStrategyMetrics(solveMetrics_, splitStrategy);

    splitPlane_ = split.splitPlane;
    if (!createChildrenFromSplit(split))
    {
        std::ostringstream message;
        message << "Failed to create child subdivision references depth=" << depth_
                << " polygon_count=" << polygons_.size()
                << ".";
        throw std::runtime_error(message.str());
    }

    isLeaf_ = false;

    if (leftChild_)
        leftChild_->solveRecursive();
    if (rightChild_)
        rightChild_->solveRecursive();

    mergeSolvedChildren();
}

// 叶子阈值和 AABB 可切分性共同决定当前节点是否停止递归。
bool SubdivisionSolver::shouldStopSubdivision() const noexcept
{
    return polygons_.size() <= leafPolygonThreshold_ || !hasSplittableAxis(aabb_);
}

bool SubdivisionSolver::shouldDiscardSubproblemEarly(BoolStatus &constantStatus) const noexcept
{
    return tryEvaluateConstantBinaryIndicator(op_, reference_.wnv, polygons_, constantStatus);
}

bool SubdivisionSolver::tryGetSingleOperandAssumptions(BoolOperandAssumptions &outAssumptions) const noexcept
{
    if (polygons_.empty())
        return false;

    const WNV &wntv = polygons_.front().WNTV;
    if (!detail::isCanonicalBinaryOperandWNTV(wntv))
        return false;

    for (const Polygon256 &polygon : polygons_)
    {
        if (polygon.WNTV != wntv)
            return false;
    }

    if (detail::isLhsOperandWNTV(wntv))
    {
        outAssumptions = lhsAssumptions_;
        return true;
    }

    outAssumptions = rhsAssumptions_;
    return true;
}

// 裁剪当前多边形集合到左右子 AABB，并为每个非空子问题建立参考状态。
bool SubdivisionSolver::createChildrenFromSplit(const AABBSplit3i &split)
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::createChildrenFromSplit");

    std::vector<Polygon256> leftPolygons;
    std::vector<Polygon256> rightPolygons;
    if (!buildSplitChildPolygonSoups(split.splitPlane, leftPolygons, rightPolygons))
        return false;

    if (leftPolygons.empty() && rightPolygons.empty())
        return false;

    SubdivisionRefState leftReference;
    SubdivisionRefState rightReference;
    buildChildReferenceStates(split, leftPolygons, rightPolygons, leftReference, rightReference);

    if (!leftPolygons.empty() && shouldCreateChildNode("left", leftPolygons, leftReference))
        createChildSolver(leftChild_, split.left, std::move(leftPolygons), std::move(leftReference));

    if (!rightPolygons.empty() && shouldCreateChildNode("right", rightPolygons, rightReference))
        createChildSolver(rightChild_, split.right, std::move(rightPolygons), std::move(rightReference));

    return true;
}

bool SubdivisionSolver::buildSplitChildPolygonSoups(
    const Plane3i &splitPlane,
    std::vector<Polygon256> &leftPolygons,
    std::vector<Polygon256> &rightPolygons) const
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::buildSplitChildPolygonSoups");

    leftPolygons.reserve(polygons_.size());
    rightPolygons.reserve(polygons_.size());

    for (const Polygon256 &polygon : polygons_)
    {
        if (!appendSplitChildPolygons(polygon, splitPlane, leftPolygons, rightPolygons))
            return false;
    }

    return true;
}

void SubdivisionSolver::buildChildReferenceStates(
    const AABBSplit3i &split,
    const std::vector<Polygon256> &leftPolygons,
    const std::vector<Polygon256> &rightPolygons,
    SubdivisionRefState &leftReference,
    SubdivisionRefState &rightReference)
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::buildChildReferenceStates");

    if (!leftPolygons.empty() && !makeChildReference(split.left, leftPolygons, leftReference))
    {
        std::ostringstream message;
        message << "Failed to propagate left child reference depth=" << depth_
                << " candidate_polygons=" << leftPolygons.size()
                << ".";
        throw std::runtime_error(message.str());
    }
    if (!rightPolygons.empty() && !makeChildReference(split.right, rightPolygons, rightReference))
    {
        std::ostringstream message;
        message << "Failed to propagate right child reference depth=" << depth_
                << " candidate_polygons=" << rightPolygons.size()
                << ".";
        throw std::runtime_error(message.str());
    }
}

bool SubdivisionSolver::shouldCreateChildNode(
    const char *side,
    const std::vector<Polygon256> &childPolygons,
    const SubdivisionRefState &childReference)
{
    BoolStatus constantStatus = OUT;
    if (!tryEvaluateConstantBinaryIndicator(op_, childReference.wnv, childPolygons, constantStatus))
        return true;

    REEMBER_PROFILE_ZONE("SubdivisionSolver::discardChildConstantIndicator");
    ++solveMetrics_.constantDiscardCount;
    ++solveMetrics_.childConstantDiscardCount;
    return false;
}

void SubdivisionSolver::createChildSolver(
    std::unique_ptr<SubdivisionSolver> &child,
    const AABB3i &childBox,
    std::vector<Polygon256> childPolygons,
    SubdivisionRefState childReference)
{
    child.reset(new SubdivisionSolver(
                    op_,
                    leafPolygonThreshold_,
                    depth_ + 1,
                    std::move(childPolygons),
                    childBox,
                    std::move(childReference),
                    lhsAssumptions_,
                    rhsAssumptions_));
}

// 优先复用仍在子 AABB 内且不落在表面上的参考点，否则枚举 AABB 路径传播 WNV。
bool SubdivisionSolver::makeChildReference(
    const AABB3i &childBox,
    const std::vector<Polygon256> &childPolygons,
    SubdivisionRefState &outReference)
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::makeChildReference");

    if (tryReuseChildReference(childBox, childPolygons, outReference))
        return true;

    const bool sourceReferenceIsStrictInterior = isPointStrictlyInsideAABB(reference_.point, childBox);
    const refPoint sourceRef(reference_.point, reference_.wnv);
    outReference = SubdivisionRefState();
    ChildReferenceSearchState searchState{outReference};
    searchState.candidatePath.reserve(3);
    auto processCandidateSeed =
        [this,
         sourceReferenceIsStrictInterior,
         &sourceRef,
         &childPolygons,
         &searchState](ChildReferenceCandidatePhase candidatePhase, const detail::AABBPathCandidateSeed &candidateSeed)
    {
        return tryTraceChildReferenceCandidate(
                   depth_,
                   reference_.point,
                   sourceReferenceIsStrictInterior,
                   sourceRef,
                   polygons_,
                   childPolygons,
                   candidatePhase,
                   candidateSeed,
                   solveMetrics_,
                   searchState);
    };

    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::makeChildReferenceFastCandidates");
        detail::visitFastAABBPathCandidateSeeds(
            reference_.point,
            childBox,
            [&processCandidateSeed](const detail::AABBPathCandidateSeed &candidateSeed)
        {
            return processCandidateSeed(ChildReferenceCandidatePhase::Fast, candidateSeed);
        });
    }
    if (outReference.point.hasUniqueIntersection())
        return true;
    if (!searchState.hardFailure)
    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::makeChildReferenceExhaustiveCandidates");
        detail::visitExhaustiveAABBPathCandidateSeeds(
            reference_.point,
            childBox,
            [&processCandidateSeed](const detail::AABBPathCandidateSeed &candidateSeed)
        {
            return processCandidateSeed(ChildReferenceCandidatePhase::Exhaustive, candidateSeed);
        });
    }
    if (outReference.point.hasUniqueIntersection())
        return true;

    outReference = SubdivisionRefState();
    return false;
}

bool SubdivisionSolver::tryReuseChildReference(
    const AABB3i &childBox,
    const std::vector<Polygon256> &childPolygons,
    SubdivisionRefState &outReference)
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::tryReuseChildReference");

    if (!isPointStrictlyInsideAABB(reference_.point, childBox) ||
            isPointOnAnyPolygonSurface(reference_.point, childPolygons))
        return false;

    {
        REEMBER_PROFILE_ZONE("SubdivisionSolver::reuseChildReference");
        ++solveMetrics_.childReferenceReuseCount;
        outReference = reference_;
    }
    return true;
}

void SubdivisionSolver::collectLeafSummaries(std::vector<BoolLeafSummary> &outSummaries) const
{
    if (discarded_)
        return;

    if (isLeaf_)
    {
        outSummaries.push_back(BoolLeafSummary{depth_, polygons_.size(), aabb_, discarded_});
        return;
    }

    if (leftChild_)
        leftChild_->collectLeafSummaries(outSummaries);
    if (rightChild_)
        rightChild_->collectLeafSummaries(outSummaries);
}

void SubdivisionSolver::collectSolveMetrics(BoolSolveMetrics &outMetrics) const
{
    REEMBER_PROFILE_ZONE("SubdivisionSolver::collectSolveMetrics");

    outMetrics.constantDiscardCount += solveMetrics_.constantDiscardCount;
    outMetrics.invalidOrEmptyDiscardCount += solveMetrics_.invalidOrEmptyDiscardCount;
    outMetrics.childConstantDiscardCount += solveMetrics_.childConstantDiscardCount;
    outMetrics.leafThresholdStopCount += solveMetrics_.leafThresholdStopCount;
    outMetrics.aabbNotSplittableStopCount += solveMetrics_.aabbNotSplittableStopCount;
    outMetrics.splitFailureStopCount += solveMetrics_.splitFailureStopCount;
    outMetrics.wntvAwareSplitCount += solveMetrics_.wntvAwareSplitCount;
    outMetrics.centerRangeSplitCount += solveMetrics_.centerRangeSplitCount;
    outMetrics.midpointSplitCount += solveMetrics_.midpointSplitCount;
    outMetrics.childReferenceReuseCount += solveMetrics_.childReferenceReuseCount;
    outMetrics.childReferenceTraceCount += solveMetrics_.childReferenceTraceCount;
    outMetrics.childReferenceCandidateCount += solveMetrics_.childReferenceCandidateCount;
    outMetrics.childReferenceFastCandidateCount += solveMetrics_.childReferenceFastCandidateCount;
    outMetrics.childReferenceExhaustiveCandidateCount += solveMetrics_.childReferenceExhaustiveCandidateCount;
    outMetrics.childReferenceCandidateTriedCount += solveMetrics_.childReferenceCandidateTriedCount;
    outMetrics.childReferenceFastCandidateTriedCount += solveMetrics_.childReferenceFastCandidateTriedCount;
    outMetrics.childReferenceExhaustiveCandidateTriedCount += solveMetrics_.childReferenceExhaustiveCandidateTriedCount;
    outMetrics.singleOperandAssumptionStopCount += solveMetrics_.singleOperandAssumptionStopCount;
    outMetrics.singleOperandAssumptionFallbackCount += solveMetrics_.singleOperandAssumptionFallbackCount;
    outMetrics.singleOperandLeafBspSkipCount += solveMetrics_.singleOperandLeafBspSkipCount;
    outMetrics.singleOperandClassificationReuseCount += solveMetrics_.singleOperandClassificationReuseCount;
    outMetrics.leafBspBuildCount += solveMetrics_.leafBspBuildCount;
    outMetrics.leafClassificationPointCandidateCount += solveMetrics_.leafClassificationPointCandidateCount;
    outMetrics.leafClassificationPrimaryPointCandidateCount += solveMetrics_.leafClassificationPrimaryPointCandidateCount;
    outMetrics.leafClassificationExpandedPointCandidateCount += solveMetrics_.leafClassificationExpandedPointCandidateCount;
    outMetrics.leafClassificationTraceAttemptCount += solveMetrics_.leafClassificationTraceAttemptCount;
    outMetrics.leafClassificationFastCandidateCount += solveMetrics_.leafClassificationFastCandidateCount;
    outMetrics.leafClassificationFallbackCandidateCount += solveMetrics_.leafClassificationFallbackCandidateCount;
    outMetrics.leafClassificationNormalCandidateCount += solveMetrics_.leafClassificationNormalCandidateCount;
    outMetrics.leafClassificationInteriorBridgeCandidateCount += solveMetrics_.leafClassificationInteriorBridgeCandidateCount;
    ++outMetrics.nodeCount;
    outMetrics.totalPolygonCount += polygons_.size();
    outMetrics.maxDepth = std::max(outMetrics.maxDepth, depth_);
    if (discarded_)
        ++outMetrics.discardedNodeCount;

    if (isLeaf_)
    {
        ++outMetrics.leafNodeCount;
        outMetrics.leafFragmentCount += leafFragments_.size();
        outMetrics.classifiedFragmentCount += classifiedFragments_.size();
        return;
    }

    ++outMetrics.internalNodeCount;
    if (leftChild_)
        leftChild_->collectSolveMetrics(outMetrics);
    if (rightChild_)
        rightChild_->collectSolveMetrics(outMetrics);
}
}
