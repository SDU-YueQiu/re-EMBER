#include "WNV_tracing.h"

namespace ember
{
    enum class PathPointStatus
    {
        OFF_POSITIVE,
        OFF_NEGATIVE,
        ON_POLYGON_STRICT,
        ON_BOUNDARY,
        ON_PLANE_OUTSIDE
    };

    PathPointStatus classifyPathPoint(const PlanePoint3i &point, const Polygon256 &poly) noexcept
    {
        const int planeSide = point.classify(poly.plane);
        if (planeSide > 0)
        {
            return PathPointStatus::OFF_POSITIVE;
        }
        if (planeSide < 0)
        {
            return PathPointStatus::OFF_NEGATIVE;
        }
        if (poly.containsStrictly(point))
        {
            return PathPointStatus::ON_POLYGON_STRICT;
        }
        if (poly.containsOrOnBoundary(point))
        {
            return PathPointStatus::ON_BOUNDARY;
        }
        return PathPointStatus::ON_PLANE_OUTSIDE;
    }

    int offPlaneSide(PathPointStatus status) noexcept
    {
        if (status == PathPointStatus::OFF_POSITIVE)
        {
            return 1;
        }
        if (status == PathPointStatus::OFF_NEGATIVE)
        {
            return -1;
        }
        return 0;
    }

    bool accumulateWNTV(WNV &targetWNV, const WNV &delta, int sigma) noexcept
    {
        if (targetWNV.size() != delta.size())
        {
            return false;
        }

        for (std::size_t i = 0; i < targetWNV.size(); ++i)
        {
            targetWNV[i] += sigma * delta[i];
        }

        return true;
    }

    traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV)
    {
        // 按 polygon 扫整条 path，把连续的面内段折叠成一次穿越事件再累加 WNTV。
        auto tmpwnv = refpoint.wnv;

        if (path.empty())
        {
            targetWNV = tmpwnv;
            return SUCCESS;
        }

        std::vector<PlanePoint3i> pathPoints;
        pathPoints.reserve(path.size() + 1);
        for (std::size_t i = 0; i < path.size(); ++i)
        {
            const auto &seg = path[i];
            if (!seg.isValid())
            {
                return INVALID_SEGMENT;
            }

            if (i == 0)
            {
                pathPoints.push_back(seg.getStartPoint());
            }
            pathPoints.push_back(seg.getEndPoint());
        }

        for (const auto &poly : polygons)
        {
            if (!poly.isValid())
            {
                return INVALID_POLYGON;
            }
            if (poly.WNTV.size() != tmpwnv.size())
            {
                return FAIL;
            }

            PathPointStatus prevStatus = classifyPathPoint(pathPoints.front(), poly);
            if (prevStatus == PathPointStatus::ON_BOUNDARY)
            {
                return BOUNDARY_HIT;
            }
            if (prevStatus == PathPointStatus::ON_POLYGON_STRICT)
            {
                return START_AT_POLYGON;
            }

            int pendingSide = offPlaneSide(prevStatus);
            bool insideRun = false;

            for (std::size_t i = 0; i < path.size(); ++i)
            {
                const auto &seg = path[i];
                const PathPointStatus currStatus = classifyPathPoint(pathPoints[i + 1], poly);

                if (currStatus == PathPointStatus::ON_BOUNDARY)
                {
                    return BOUNDARY_HIT;
                }

                if (currStatus == PathPointStatus::ON_POLYGON_STRICT)
                {
                    if (prevStatus == PathPointStatus::ON_PLANE_OUTSIDE)
                    {
                        return BOUNDARY_HIT;
                    }

                    const int prevSide = offPlaneSide(prevStatus);
                    if (prevSide != 0)
                    {
                        pendingSide = prevSide;
                        insideRun = true;
                    }
                    else if (prevStatus != PathPointStatus::ON_POLYGON_STRICT)
                    {
                        return FAIL;
                    }

                    prevStatus = currStatus;
                    continue;
                }

                if (currStatus == PathPointStatus::ON_PLANE_OUTSIDE)
                {
                    if (prevStatus == PathPointStatus::ON_POLYGON_STRICT)
                    {
                        return BOUNDARY_HIT;
                    }

                    prevStatus = currStatus;
                    continue;
                }

                const int currSide = offPlaneSide(currStatus);
                if (currSide == 0)
                {
                    return FAIL;
                }

                if (insideRun)
                {
                    const int sigma = (pendingSide - currSide) / 2;
                    if (sigma != 0 && !accumulateWNTV(tmpwnv, poly.WNTV, sigma))
                    {
                        return FAIL;
                    }
                    insideRun = false;
                }
                else
                {
                    const int prevSide = offPlaneSide(prevStatus);
                    if (prevSide != 0 && prevSide != currSide)
                    {
                        PlanePoint3i intersectPoint;
                        if (intersectionSegmentPolygon(seg, poly, intersectPoint))
                        {
                            if (!poly.containsStrictly(intersectPoint))
                            {
                                return BOUNDARY_HIT;
                            }

                            const int sigma = (prevSide - currSide) / 2;
                            if (!accumulateWNTV(tmpwnv, poly.WNTV, sigma))
                            {
                                return FAIL;
                            }
                        }
                    }
                }

                pendingSide = currSide;
                prevStatus = currStatus;
            }

            if (insideRun || prevStatus == PathPointStatus::ON_POLYGON_STRICT)
            {
                return END_AT_POLYGON;
            }
        }

        targetWNV = tmpwnv;
        return SUCCESS;
    }
}