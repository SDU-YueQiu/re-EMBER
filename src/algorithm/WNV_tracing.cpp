#include "WNV_tracing.h"
#include "algorithm/tracing_geometry.h"

#include <utility>

namespace ember
{
    namespace
    {
        void addScaledWNTV(WNV &accumulator, const WNV &delta, int scale)
        {
            for (std::size_t i = 0; i < accumulator.size(); ++i)
            {
                accumulator[i] += scale * delta[i];
            }
        }

        bool isSameStrictSide(int lhs, int rhs) noexcept
        {
            return lhs == rhs && (lhs == -1 || lhs == 1);
        }

        traceStatus tracePathWNVImpl(
            const refPoint &refpoint,
            const Path &path,
            const std::vector<Polygon256> &polygons,
            WNV &targetWNV,
            bool validatePolygons,
            bool validatePath)
        {
            if (validatePath)
            {
                for (const auto &seg : path)
                {
                    if (!seg.isValid())
                    {
                        return INPUT_INVALID;
                    }
                }
            }

            if (validatePolygons)
            {
                for (const auto &poly : polygons)
                {
                    if (!poly.isValid() || poly.WNTV.size() != refpoint.wnv.size())
                    {
                        return INPUT_INVALID;
                    }
                }
            }

            if (path.empty())
            {
                targetWNV = refpoint.wnv;
                return SUCCESS;
            }

            if (validatePath && !areSamePlanePoint(path[0].getStartPoint(), refpoint.point))
            {
                return INPUT_INVALID;
            }

            if (validatePath)
            {
                for (std::size_t i = 1; i < path.size(); ++i)
                {
                    if (!areSamePlanePoint(path[i - 1].getEndPoint(), path[i].getStartPoint()))
                    {
                        return INPUT_INVALID;
                    }
                }
            }

            WNV propagatedWNV = refpoint.wnv;

            for (const Polygon256 &poly : polygons)
            {
                PlanePoint3i startPoint = path[0].getStartPoint();
                int pcs = poly.classify(startPoint);
                if (pcs == 0)
                {
                    return PATH_INVALID;
                }

                for (const Segment256 &seg : path)
                {
                    const PlanePoint3i endPoint = seg.getEndPoint();
                    const int pce = poly.classify(endPoint);

                    if (pce == 0)
                    {
                        return PATH_INVALID;
                    }
                    if (isSameStrictSide(pcs, pce))
                    {
                        pcs = pce;
                        continue;
                    }

                    if (detail::isSegmentTouchPolygonEdgeUnchecked(seg, poly))
                    {
                        return PATH_INVALID;
                    }

                    PlanePoint3i intersectPoint;
                    if (intersectionSegmentPolygon(seg, poly, intersectPoint))
                    {
                        const detail::PolygonSurfaceLocation hitLocation =
                            detail::classifyPolygonSurfacePointUnchecked(poly, intersectPoint);
                        if (hitLocation != detail::PolygonSurfaceLocation::StrictInterior)
                        {
                            return PATH_INVALID;
                        }

                        const int sigma = (pcs - pce) / 2;
                        addScaledWNTV(propagatedWNV, poly.WNTV, sigma);
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
            bool validatePath)
        {
            if (path.empty())
            {
                return INPUT_INVALID;
            }

            if (validatePath)
            {
                for (const auto &seg : path)
                {
                    if (!seg.isValid())
                    {
                        return INPUT_INVALID;
                    }
                }
            }

            if (validatePolygons)
            {
                for (const auto &poly : polygons)
                {
                    if (!poly.isValid() || poly.WNTV.size() != refpoint.wnv.size())
                    {
                        return INPUT_INVALID;
                    }
                }
            }

            if (validatePath && !areSamePlanePoint(path[0].getStartPoint(), refpoint.point))
            {
                return INPUT_INVALID;
            }

            if (validatePath)
            {
                for (std::size_t i = 1; i < path.size(); ++i)
                {
                    if (!areSamePlanePoint(path[i - 1].getEndPoint(), path[i].getStartPoint()))
                    {
                        return INPUT_INVALID;
                    }
                }
            }

            const PlanePoint3i targetPoint = path.back().getEndPoint();
            if (!targetPoint.hasUniqueIntersection() || targetPoint.classify(referencePlane) != 0)
            {
                return INPUT_INVALID;
            }

            WNV surfaceWNV = refpoint.wnv;
            WNV surfaceDelta(refpoint.wnv.size(), 0);

            for (const Polygon256 &poly : polygons)
            {
                PlanePoint3i startPoint = path[0].getStartPoint();
                int pcs = poly.classify(startPoint);
                if (pcs == 0)
                {
                    return PATH_INVALID;
                }

                for (std::size_t segmentIndex = 0; segmentIndex < path.size(); ++segmentIndex)
                {
                    const Segment256 &seg = path[segmentIndex];
                    const PlanePoint3i endPoint = seg.getEndPoint();
                    const int pce = poly.classify(endPoint);
                    const bool isLastSegment = (segmentIndex + 1 == path.size());

                    if (!isLastSegment && pce == 0)
                    {
                        return PATH_INVALID;
                    }
                    if (isSameStrictSide(pcs, pce))
                    {
                        pcs = pce;
                        continue;
                    }

                    const PlanePoint3i intersectPoint = intersect(seg.direction, poly.plane);
                    if (intersectPoint.hasUniqueIntersection())
                    {
                        const detail::PolygonSurfaceLocation hitLocation =
                            detail::classifyPolygonSurfacePointUnchecked(poly, intersectPoint);
                        if (hitLocation == detail::PolygonSurfaceLocation::Boundary)
                        {
                            return PATH_INVALID;
                        }
                        if (hitLocation == detail::PolygonSurfaceLocation::StrictInterior)
                        {
                            if (isLastSegment && pce == 0)
                            {
                                addScaledWNTV(surfaceDelta, poly.WNTV, pcs);
                            }
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
                    {
                        return PATH_INVALID;
                    }

                    pcs = pce;
                }
            }

            const PlanePoint3i lastStartPoint = path.back().getStartPoint();
            const int referenceHitSide = lastStartPoint.classify(referencePlane);
            if (referenceHitSide == 0)
            {
                return PATH_INVALID;
            }

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
        return tracePathWNVImpl(refpoint, path, polygons, targetWNV, true, true);
    }

    traceStatus detail::tracePathWNVTrusted(
        const refPoint &refpoint,
        const Path &path,
        const std::vector<Polygon256> &polygons,
        WNV &targetWNV)
    {
        return tracePathWNVImpl(refpoint, path, polygons, targetWNV, false, false);
    }

    traceStatus tracePathWNVToSurfacePoint(
        const refPoint &refpoint,
        const Path &path,
        const std::vector<Polygon256> &polygons,
        const Plane3i &referencePlane,
        WNV &frontWNV,
        WNV &backWNV)
    {
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
        return tracePathWNVToSurfacePointImpl(refpoint, path, polygons, referencePlane, frontWNV, backWNV, false, false);
    }
}

