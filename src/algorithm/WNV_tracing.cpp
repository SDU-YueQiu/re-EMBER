/**
 * @file WNV_tracing.cpp
 * @brief 实现沿已验证路径传播 WNV 的流程。
 */
#include "WNV_tracing.h"
#include "algorithm/tracing_geometry.h"
#include "core/perf_tracing.h"

#include <utility>

namespace ember
{
namespace
{
enum class BoundaryPolicy
{
    RejectAll,
    AllowSubdivisionClipCrossing
};

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
    BoundaryPolicy boundaryPolicy)
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

        int pcs = 0;
        {
            REEMBER_PROFILE_ZONE("tracePathWNVImpl::classifyStartPoint");
            pcs = poly.classify(pathStartPoint);
        }
        if (pcs == 0)
            return PATH_INVALID;

        for (const Segment256 &seg : path)
        {
            const PlanePoint3i &endPoint = seg.getEndPointRef();
            int pce = 0;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVImpl::classifyEndPoint");
                pce = poly.classify(endPoint);
            }

            if (pce == 0)
                return PATH_INVALID;
            if (isSameStrictSide(pcs, pce))
            {
                pcs = pce;
                continue;
            }

            if (!detail::isSegmentRelevantToPolygonByAABB(seg, poly))
            {
                pcs = pce;
                continue;
            }

            detail::PolygonBoundaryContact boundaryContact;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVImpl::boundaryContact");
                boundaryContact =
                    detail::classifySegmentPolygonBoundaryContactUnchecked(seg, poly);
            }
            if (boundaryContact.type == detail::PolygonBoundaryContactType::EndpointOnBoundary ||
                    boundaryContact.type == detail::PolygonBoundaryContactType::EdgeOverlap)
                return PATH_INVALID;

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
                        return PATH_INVALID;
                }
                else if (hitLocation != detail::PolygonSurfaceLocation::StrictInterior)
                    return PATH_INVALID;

                const int sigma = (pcs - pce) / 2;
                addScaledWNTV(propagatedWNV, poly.WNTV, sigma);
            }
            else if (boundaryContact.type != detail::PolygonBoundaryContactType::None)
                return PATH_INVALID;

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
    bool validatePath)
{
    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl");

    if (path.empty())
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

    WNV surfaceWNV = refpoint.wnv;
    WNV surfaceDelta(refpoint.wnv.size(), 0);

    for (const Polygon256 &poly : polygons)
    {
        REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::polygon");

        int pcs = 0;
        {
            REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::classifyStartPoint");
            pcs = poly.classify(pathStartPoint);
        }
        if (pcs == 0)
            return PATH_INVALID;

        for (std::size_t segmentIndex = 0; segmentIndex < path.size(); ++segmentIndex)
        {
            const Segment256 &seg = path[segmentIndex];
            const PlanePoint3i &endPoint = seg.getEndPointRef();
            int pce = 0;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::classifyEndPoint");
                pce = poly.classify(endPoint);
            }
            const bool isLastSegment = (segmentIndex + 1 == path.size());

            if (!isLastSegment && pce == 0)
                return PATH_INVALID;
            if (isSameStrictSide(pcs, pce))
            {
                pcs = pce;
                continue;
            }

            if (!detail::isSegmentRelevantToPolygonByAABB(seg, poly))
            {
                pcs = pce;
                continue;
            }

            PlanePoint3i intersectPoint;
            {
                REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointImpl::intersectSupportPlane");
                intersectPoint = intersect(seg.direction, poly.plane);
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
                    return PATH_INVALID;
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
                pcs = pce;
                continue;
            }

            if (detail::isSegmentTouchPolygonEdgeUnchecked(seg, poly))
                return PATH_INVALID;

            pcs = pce;
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

    return tracePathWNVImpl(refpoint, path, polygons, targetWNV, true, true, BoundaryPolicy::RejectAll);
}

traceStatus detail::tracePathWNVTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV)
{
    REEMBER_PROFILE_ZONE("tracePathWNVTrusted");

    return tracePathWNVImpl(refpoint, path, polygons, targetWNV, false, false, BoundaryPolicy::RejectAll);
}

traceStatus detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV)
{
    REEMBER_PROFILE_ZONE("tracePathWNVAllowSubdivisionClipCrossingTrusted");

    return tracePathWNVImpl(
               refpoint,
               path,
               polygons,
               targetWNV,
               false,
               false,
               BoundaryPolicy::AllowSubdivisionClipCrossing);
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

    return tracePathWNVToSurfacePointImpl(refpoint, path, polygons, referencePlane, frontWNV, backWNV, true, true);
}

traceStatus detail::tracePathWNVToSurfacePointTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV)
{
    REEMBER_PROFILE_ZONE("tracePathWNVToSurfacePointTrusted");

    return tracePathWNVToSurfacePointImpl(refpoint, path, polygons, referencePlane, frontWNV, backWNV, false, false);
}
}

