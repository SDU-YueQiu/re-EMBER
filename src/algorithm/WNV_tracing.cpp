/**
 * @file WNV_tracing.cpp
 * @brief 实现沿已验证路径传播 WNV 的流程。
 */
#include "WNV_tracing.h"
#include "algorithm/tracing_geometry.h"
#include "core/bool_problem.h"
#include "core/perf_tracing.h"

#include <utility>
#include <vector>

namespace ember
{
namespace
{
constexpr int kUnknownPolygonStartSide = 2;

enum class BoundaryPolicy
{
    RejectAll,
    AllowSubdivisionClipCrossing
};

enum class TracePathInvalidReason
{
    StartPointOnBoundary,
    EndPointOnBoundary,
    EndpointOnBoundaryContact,
    EdgeOverlap,
    NonStrictIntersection,
    BoundaryContactWithoutIntersection
};

enum class BoundaryHitRejectedKind
{
    RegularEdge,
    SubdivisionClipEdge,
    MixedEdge,
    Unknown
};

struct PathAABBPrecheck
{
    AABB3i pathBox;
    AABB3i singleSegmentBox;
    std::vector<AABB3i> segmentBoxes;
    std::size_t segmentCount = 0;
    bool valid = false;

    const AABB3i &segmentBox(std::size_t index) const noexcept
    {
        return segmentBoxes.empty() ? singleSegmentBox : segmentBoxes[index];
    }
};

bool buildPathAABBPrecheck(const Path &path, PathAABBPrecheck &outPrecheck)
{
    outPrecheck = PathAABBPrecheck();
    const bool useVector = path.size() > 1;
    if (useVector)
        outPrecheck.segmentBoxes.reserve(path.size());

    for (const Segment256 &segment : path)
    {
        AABB3i segmentBox;
        if (!buildPointPairAABB(
                    segment.getStartPointRef(),
                    segment.getEndPointRef(),
                    segmentBox))
        {
            outPrecheck = PathAABBPrecheck();
            return false;
        }

        mergeAABB(outPrecheck.pathBox, segmentBox);
        if (useVector)
            outPrecheck.segmentBoxes.push_back(std::move(segmentBox));
        else
            outPrecheck.singleSegmentBox = std::move(segmentBox);
        ++outPrecheck.segmentCount;
    }

    outPrecheck.valid =
        isValidAABB(outPrecheck.pathBox) &&
        outPrecheck.segmentCount == path.size();
    if (!outPrecheck.valid)
        outPrecheck = PathAABBPrecheck();

    return outPrecheck.valid;
}

bool canSkipPolygonForPathAABB(const Polygon256 &polygon, const AABB3i &pathBox) noexcept
{
    if (!isValidAABB(pathBox))
        return false;

    const AABB3i &polygonBox = polygon.aabb();
    if (!isValidAABB(polygonBox))
        return false;

    return !doAABBsOverlap(pathBox, polygonBox) ||
           !doesPlaneIntersectAABB(polygon.plane, pathBox);
}

bool isSegmentRelevantToPolygon(
    const Segment256 &seg,
    const Polygon256 &poly,
    const AABB3i *knownSegmentBox) noexcept
{
    return knownSegmentBox != nullptr
           ? detail::isSegmentRelevantToPolygonByAABB(*knownSegmentBox, poly)
           : detail::isSegmentRelevantToPolygonByAABB(seg, poly);
}

PlanePoint3i intersectLinePlaneUnnormalized(const Line256 &line, const Plane3i &plane) noexcept
{
    // 追踪热路径只需要交点相对多边形边平面的符号，直接保留 intersect_3_planes
    // 的未规范化齐次输出，避免把论文核心 primitive 退回 canonical 点构造。
    return PlanePoint3i(
               line.p1,
               line.p2,
               plane,
               intersectHomogeneousUnnormalized(line.p1, line.p2, plane));
}

void recordTracePathInvalid(
    BoolSolveMetrics *solveMetrics,
    TracePathInvalidReason reason) noexcept
{
    if (solveMetrics == nullptr)
        return;

    switch (reason)
    {
    case TracePathInvalidReason::StartPointOnBoundary:
        ++solveMetrics->tracePathStartPointOnBoundaryCount;
        break;
    case TracePathInvalidReason::EndPointOnBoundary:
        ++solveMetrics->tracePathEndPointOnBoundaryCount;
        break;
    case TracePathInvalidReason::EndpointOnBoundaryContact:
        ++solveMetrics->tracePathEndpointOnBoundaryContactCount;
        break;
    case TracePathInvalidReason::EdgeOverlap:
        ++solveMetrics->tracePathEdgeOverlapCount;
        break;
    case TracePathInvalidReason::NonStrictIntersection:
        ++solveMetrics->tracePathNonStrictIntersectionCount;
        break;
    case TracePathInvalidReason::BoundaryContactWithoutIntersection:
        ++solveMetrics->tracePathBoundaryContactWithoutIntersectionCount;
        break;
    }
}

BoundaryHitRejectedKind classifyBoundaryHitRejectedKind(
    const detail::PolygonBoundaryContact &boundaryContact,
    const Polygon256 &poly) noexcept
{
    if (boundaryContact.edgeIndices.empty())
        return BoundaryHitRejectedKind::Unknown;

    bool hasRegularEdge = false;
    bool hasSubdivisionClipEdge = false;
    for (const std::size_t edgeIndex : boundaryContact.edgeIndices)
    {
        if (poly.edgeProvenance(edgeIndex) == PolygonEdgeProvenance::SubdivisionClip)
            hasSubdivisionClipEdge = true;
        else
            hasRegularEdge = true;
    }

    if (hasRegularEdge && hasSubdivisionClipEdge)
        return BoundaryHitRejectedKind::MixedEdge;
    if (hasRegularEdge)
        return BoundaryHitRejectedKind::RegularEdge;
    if (hasSubdivisionClipEdge)
        return BoundaryHitRejectedKind::SubdivisionClipEdge;
    return BoundaryHitRejectedKind::Unknown;
}

void recordBoundaryHitRejected(
    BoolSolveMetrics *solveMetrics,
    const detail::PolygonBoundaryContact &boundaryContact,
    const Polygon256 &poly) noexcept
{
    if (solveMetrics == nullptr)
        return;

    switch (classifyBoundaryHitRejectedKind(boundaryContact, poly))
    {
    case BoundaryHitRejectedKind::RegularEdge:
        ++solveMetrics->tracePathBoundaryHitRejectedRegularEdgeCount;
        break;
    case BoundaryHitRejectedKind::SubdivisionClipEdge:
        ++solveMetrics->tracePathBoundaryHitRejectedSubdivisionClipEdgeCount;
        break;
    case BoundaryHitRejectedKind::MixedEdge:
        ++solveMetrics->tracePathBoundaryHitRejectedMixedEdgeCount;
        break;
    case BoundaryHitRejectedKind::Unknown:
        ++solveMetrics->tracePathBoundaryHitRejectedUnknownCount;
        break;
    }
}

void recordBoundaryHitAllowed(BoolSolveMetrics *solveMetrics) noexcept
{
    if (solveMetrics != nullptr)
        ++solveMetrics->tracePathBoundaryHitAllowedSubdivisionClipEdgeCount;
}

void addScaledWNTV(WNV &accumulator, const WNV &delta, int scale)
{
    for (std::size_t i = 0; i < accumulator.size(); ++i)
        accumulator[i] += scale * delta[i];
}

bool isSameStrictSide(int lhs, int rhs) noexcept
{
    return lhs == rhs && (lhs == -1 || lhs == 1);
}

bool isOppositeStrictSide(int lhs, int rhs) noexcept
{
    return lhs == -rhs && (lhs == -1 || lhs == 1);
}

bool canTreatSubdivisionClipBoundaryHitAsCrossing(
    BoundaryPolicy policy,
    int pcs,
    int pce,
    const detail::PolygonBoundaryContact &boundaryContact,
    const Polygon256 &poly) noexcept
{
    return policy == BoundaryPolicy::AllowSubdivisionClipCrossing &&
           boundaryContact.type == detail::PolygonBoundaryContactType::BoundaryPointHit &&
           isOppositeStrictSide(pcs, pce) &&
           detail::areBoundaryContactEdgesSubdivisionClip(boundaryContact, poly);
}

traceStatus tracePathWNVImpl(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV,
    bool validatePolygons,
    bool validatePath,
    BoundaryPolicy boundaryPolicy,
    BoolSolveMetrics *solveMetrics)
{
    REEMBER_PROFILE_ZONE("tracePathWNVImpl");

    if (validatePath)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVImpl::validatePath");
        for (const auto &seg : path)
        {
            if (!seg.isValid())
                return INPUT_INVALID;
        }
    }

    if (validatePolygons)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVImpl::validatePolygons");
        for (const auto &poly : polygons)
        {
            if (!poly.isValid() || poly.WNTV.size() != refpoint.wnv.size())
                return INPUT_INVALID;
        }
    }

    if (path.empty())
    {
        targetWNV = refpoint.wnv;
        return SUCCESS;
    }

    PathAABBPrecheck pathAABB;
    const bool hasPathBox = buildPathAABBPrecheck(path, pathAABB);

    const PlanePoint3i &pathStartPoint = path.front().getStartPointRef();
    if (validatePath && !areSamePlanePoint(pathStartPoint, refpoint.point))
        return INPUT_INVALID;

    if (validatePath)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVImpl::validateContinuity");
        for (std::size_t i = 1; i < path.size(); ++i)
        {
            if (!areSamePlanePoint(path[i - 1].getEndPointRef(), path[i].getStartPointRef()))
                return INPUT_INVALID;
        }
    }

    WNV propagatedWNV = refpoint.wnv;

    for (const Polygon256 &poly : polygons)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVImpl::polygon");
        if (hasPathBox && canSkipPolygonForPathAABB(poly, pathAABB.pathBox))
            continue;

        int pcs = 0;
        {
            REEMBER_PROFILE_ZONE("tracePathWNVImpl::classifyStartPoint");
            pcs = poly.classify(pathStartPoint);
        }
        if (pcs == 0)
        {
            recordTracePathInvalid(solveMetrics, TracePathInvalidReason::StartPointOnBoundary);
            return PATH_INVALID;
        }

        for (std::size_t segmentIndex = 0; segmentIndex < path.size(); ++segmentIndex)
        {
            const Segment256 &seg = path[segmentIndex];
            const AABB3i *segmentBox =
                hasPathBox ? &pathAABB.segmentBox(segmentIndex) : nullptr;
            const PlanePoint3i &endPoint = seg.getEndPointRef();
            int pce = 0;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVImpl::classifyEndPoint");
                pce = poly.classify(endPoint);
            }

            if (pce == 0)
            {
                recordTracePathInvalid(solveMetrics, TracePathInvalidReason::EndPointOnBoundary);
                return PATH_INVALID;
            }
            if (isSameStrictSide(pcs, pce))
            {
                pcs = pce;
                continue;
            }

            if (!isSegmentRelevantToPolygon(seg, poly, segmentBox))
            {
                pcs = pce;
                continue;
            }

            detail::PolygonBoundaryContact boundaryContact;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVImpl::boundaryContact");
                boundaryContact =
                    detail::classifySegmentPolygonBoundaryContactUnchecked(seg, poly, segmentBox);
            }
            if (boundaryContact.type == detail::PolygonBoundaryContactType::EndpointOnBoundary)
            {
                recordTracePathInvalid(solveMetrics, TracePathInvalidReason::EndpointOnBoundaryContact);
                return PATH_INVALID;
            }
            if (boundaryContact.type == detail::PolygonBoundaryContactType::EdgeOverlap)
            {
                recordTracePathInvalid(solveMetrics, TracePathInvalidReason::EdgeOverlap);
                return PATH_INVALID;
            }

            PlanePoint3i intersectPoint;
            bool hasIntersection = false;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVImpl::intersectionSegmentPolygon");
                hasIntersection = intersectionSegmentPolygon(seg, poly, intersectPoint);
            }
            if (hasIntersection)
            {
                detail::PolygonSurfaceLocation hitLocation = detail::PolygonSurfaceLocation::Outside;
                {
                    REEMBER_PROFILE_ZONE("tracePathWNVImpl::classifySurfaceHit");
                    hitLocation =
                        detail::classifyPolygonSurfacePointUnchecked(poly, intersectPoint);
                }
                if (hitLocation == detail::PolygonSurfaceLocation::Boundary)
                {
                    if (!canTreatSubdivisionClipBoundaryHitAsCrossing(
                                boundaryPolicy,
                                pcs,
                                pce,
                                boundaryContact,
                                poly))
                    {
                        recordBoundaryHitRejected(solveMetrics, boundaryContact, poly);
                        return PATH_INVALID;
                    }
                    recordBoundaryHitAllowed(solveMetrics);
                }
                else if (hitLocation != detail::PolygonSurfaceLocation::StrictInterior)
                {
                    recordTracePathInvalid(solveMetrics, TracePathInvalidReason::NonStrictIntersection);
                    return PATH_INVALID;
                }

                const int sigma = (pcs - pce) / 2;
                addScaledWNTV(propagatedWNV, poly.WNTV, sigma);
            }
            else if (boundaryContact.type != detail::PolygonBoundaryContactType::None)
            {
                recordTracePathInvalid(
                    solveMetrics,
                    TracePathInvalidReason::BoundaryContactWithoutIntersection);
                return PATH_INVALID;
            }

            pcs = pce;
        }
    }

    targetWNV = std::move(propagatedWNV);
    return SUCCESS;
}

traceStatus tracePathWNVToSurfacePointImpl(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV,
    bool validatePolygons,
    bool validatePath,
    std::vector<int> *polygonStartSideCache,
    BoolSolveMetrics *solveMetrics)
{
    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl");

    if (path.empty())
        return INPUT_INVALID;
    if (polygonStartSideCache != nullptr && polygonStartSideCache->size() != polygons.size())
        return INPUT_INVALID;

    if (validatePath)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::validatePath");
        for (const auto &seg : path)
        {
            if (!seg.isValid())
                return INPUT_INVALID;
        }
    }

    if (validatePolygons)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::validatePolygons");
        for (const auto &poly : polygons)
        {
            if (!poly.isValid() || poly.WNTV.size() != refpoint.wnv.size())
                return INPUT_INVALID;
        }
    }

    const PlanePoint3i &pathStartPoint = path.front().getStartPointRef();
    if (validatePath)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::validateContinuity");
        if (!areSamePlanePoint(pathStartPoint, refpoint.point))
            return INPUT_INVALID;
        for (std::size_t i = 1; i < path.size(); ++i)
        {
            if (!areSamePlanePoint(path[i - 1].getEndPointRef(), path[i].getStartPointRef()))
                return INPUT_INVALID;
        }
    }

    const PlanePoint3i &targetPoint = path.back().getEndPointRef();
    if (!targetPoint.hasUniqueIntersection() || targetPoint.classify(referencePlane) != 0)
        return INPUT_INVALID;

    PathAABBPrecheck pathAABB;
    const bool hasPathBox = buildPathAABBPrecheck(path, pathAABB);

    WNV surfaceWNV = refpoint.wnv;
    WNV surfaceDelta(refpoint.wnv.size(), 0);

    for (std::size_t polygonIndex = 0; polygonIndex < polygons.size(); ++polygonIndex)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::polygon");
        const Polygon256 &poly = polygons[polygonIndex];
        if (hasPathBox && canSkipPolygonForPathAABB(poly, pathAABB.pathBox))
            continue;

        int previousEndSide = 0;
        bool hasPreviousEndSide = false;
        for (std::size_t segmentIndex = 0; segmentIndex < path.size(); ++segmentIndex)
        {
            const Segment256 &seg = path[segmentIndex];
            const AABB3i *segmentBox =
                hasPathBox ? &pathAABB.segmentBox(segmentIndex) : nullptr;
            const PlanePoint3i &endPoint = seg.getEndPointRef();
            const bool isLastSegment = (segmentIndex + 1 == path.size());
            const bool segmentRelevant = isSegmentRelevantToPolygon(seg, poly, segmentBox);
            if (!segmentRelevant)
            {
                hasPreviousEndSide = false;
                continue;
            }

            int pcs = 0;
            if (hasPreviousEndSide)
                pcs = previousEndSide;
            else if (segmentIndex == 0 && polygonStartSideCache != nullptr)
            {
                int &cachedSide = (*polygonStartSideCache)[polygonIndex];
                if (cachedSide == kUnknownPolygonStartSide)
                {
                    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::classifyStartPoint");
                    cachedSide = poly.classify(pathStartPoint);
                }
                pcs = cachedSide;
            }
            else
            {
                REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::classifyStartPoint");
                pcs = poly.classify(seg.getStartPointRef());
            }
            if (pcs == 0)
            {
                recordTracePathInvalid(solveMetrics, TracePathInvalidReason::StartPointOnBoundary);
                return PATH_INVALID;
            }

            int pce = 0;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::classifyEndPoint");
                pce = poly.classify(endPoint);
            }

            if (!isLastSegment && pce == 0)
            {
                recordTracePathInvalid(solveMetrics, TracePathInvalidReason::EndPointOnBoundary);
                return PATH_INVALID;
            }
            if (isSameStrictSide(pcs, pce))
            {
                previousEndSide = pce;
                hasPreviousEndSide = true;
                continue;
            }

            PlanePoint3i intersectPoint;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::intersectSupportPlane");
                intersectPoint = intersectLinePlaneUnnormalized(seg.direction, poly.plane);
            }
            if (intersectPoint.hasUniqueIntersection())
            {
                detail::PolygonSurfaceLocation hitLocation = detail::PolygonSurfaceLocation::Outside;
                {
                    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::classifySurfaceHit");
                    hitLocation =
                        detail::classifyPolygonSurfacePointUnchecked(poly, intersectPoint);
                }
                if (hitLocation == detail::PolygonSurfaceLocation::Boundary)
                {
                    const detail::PolygonBoundaryContact boundaryContact =
                        detail::classifySegmentPolygonBoundaryContactUnchecked(seg, poly, segmentBox);
                    if (!canTreatSubdivisionClipBoundaryHitAsCrossing(
                                BoundaryPolicy::AllowSubdivisionClipCrossing,
                                pcs,
                                pce,
                                boundaryContact,
                                poly))
                    {
                        recordBoundaryHitRejected(solveMetrics, boundaryContact, poly);
                        return PATH_INVALID;
                    }

                    recordBoundaryHitAllowed(solveMetrics);
                    if (isLastSegment && pce == 0)
                        addScaledWNTV(surfaceDelta, poly.WNTV, pcs);
                    else
                    {
                        const int sigma = (pcs - pce) / 2;
                        addScaledWNTV(surfaceWNV, poly.WNTV, sigma);
                    }
                    previousEndSide = pce;
                    hasPreviousEndSide = true;
                    continue;
                }
                if (hitLocation == detail::PolygonSurfaceLocation::StrictInterior)
                {
                    if (isLastSegment && pce == 0)
                        addScaledWNTV(surfaceDelta, poly.WNTV, pcs);
                    else
                    {
                        const int sigma = (pcs - pce) / 2;
                        addScaledWNTV(surfaceWNV, poly.WNTV, sigma);
                    }
                }
                previousEndSide = pce;
                hasPreviousEndSide = true;
                continue;
            }

            const bool segmentTouchesEdge = segmentBox != nullptr
                ? detail::isSegmentTouchPolygonEdgeUnchecked(seg, poly, *segmentBox)
                : detail::isSegmentTouchPolygonEdgeUnchecked(seg, poly);
            if (segmentTouchesEdge)
            {
                recordTracePathInvalid(
                    solveMetrics,
                    TracePathInvalidReason::BoundaryContactWithoutIntersection);
                return PATH_INVALID;
            }

            previousEndSide = pce;
            hasPreviousEndSide = true;
        }
    }

    const PlanePoint3i &lastStartPoint = path.back().getStartPointRef();
    const int referenceHitSide = lastStartPoint.classify(referencePlane);
    if (referenceHitSide == 0)
        return PATH_INVALID;

    if (referenceHitSide > 0)
    {
        frontWNV = surfaceWNV;
        backWNV = surfaceWNV;
        addScaledWNTV(backWNV, surfaceDelta, 1);
        return SUCCESS;
    }

    frontWNV = surfaceWNV;
    addScaledWNTV(frontWNV, surfaceDelta, 1);
    backWNV = surfaceWNV;
    return SUCCESS;
}
}

traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV)
{
    REEMBER_PROFILE_ZONE("tracePathWNV");

    return tracePathWNVImpl(
        refpoint,
        path,
        polygons,
        targetWNV,
        true,
        true,
        BoundaryPolicy::AllowSubdivisionClipCrossing,
        nullptr);
}

traceStatus detail::tracePathWNVTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV,
    BoolSolveMetrics *solveMetrics)
{
    REEMBER_PROFILE_ZONE("tracePathWNVTrusted");

    return tracePathWNVImpl(
        refpoint,
        path,
        polygons,
        targetWNV,
        false,
        false,
        BoundaryPolicy::AllowSubdivisionClipCrossing,
        solveMetrics);
}

traceStatus detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV,
    BoolSolveMetrics *solveMetrics)
{
    REEMBER_PROFILE_ZONE("tracePathWNVAllowSubdivisionClipCrossingTrusted");

    return tracePathWNVImpl(
               refpoint,
               path,
               polygons,
               targetWNV,
               false,
               false,
               BoundaryPolicy::AllowSubdivisionClipCrossing,
               solveMetrics);
}

traceStatus tracePathWNVToSurfacePoint(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV)
{
    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePoint");

    return tracePathWNVToSurfacePointImpl(
        refpoint,
        path,
        polygons,
        referencePlane,
        frontWNV,
        backWNV,
        true,
        true,
        nullptr,
        nullptr);
}

traceStatus detail::tracePathWNVToSurfacePointTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV,
    BoolSolveMetrics *solveMetrics)
{
    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointTrusted");

    return tracePathWNVToSurfacePointImpl(
        refpoint,
        path,
        polygons,
        referencePlane,
        frontWNV,
        backWNV,
        false,
        false,
        nullptr,
        solveMetrics);
}

traceStatus detail::tracePathWNVToSurfacePointTrustedWithStartSides(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    std::vector<int> &polygonStartSideCache,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV,
    BoolSolveMetrics *solveMetrics)
{
    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointTrustedWithStartSides");

    return tracePathWNVToSurfacePointImpl(
        refpoint,
        path,
        polygons,
        referencePlane,
        frontWNV,
        backWNV,
        false,
        false,
        &polygonStartSideCache,
        solveMetrics);
}
}

