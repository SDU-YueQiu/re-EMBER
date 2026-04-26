#include "WNV_tracing.h"
#include "shit_wrapper.h"

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
    }

    traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV)
    {
        // 实现为对path里的每条线段遍历所有多边形求交，然后累加wnvt到wnv上

        // 检查输入有效性
        for (const auto &seg : path)
        {
            if (!seg.isValid())
                return INPUT_INVALID;
        }
        for (const auto &poly : polygons)
        {
            if (!poly.isValid())
                return INPUT_INVALID;
            if (poly.WNTV.size() != refpoint.wnv.size())
                return INPUT_INVALID;
        }

        if (path.empty())
        {
            targetWNV = refpoint.wnv;
            return SUCCESS;
        }

        // 检查路径是否连续
        if (!areSamePlanePoint(path[0].getStartPoint(), refpoint.point))
            return INPUT_INVALID;

        for (int i = 1; i < path.size(); i++)
        {
            auto preEnd = path[i - 1].getEndPoint();
            auto curStart = path[i].getStartPoint();
            if (!areSamePlanePoint(preEnd, curStart))
                return INPUT_INVALID;
        }

        auto tmpwnv = refpoint.wnv;

        // 点位于表面上，或任何追踪线段位于多边形内部或接触边这三种情况路径无效
        for (int i = 0; i < polygons.size(); i++)
        {
            const auto &poly = polygons[i];

            PlanePoint3i startPoint = path[0].getStartPoint();
            PlanePoint3i endPoint;

            int pcs = poly.classify(startPoint); // start point classify polygon
            int pce;

            if (pcs == 0)
                return PATH_INVALID;

            for (int j = 0; j < path.size(); j++)
            {
                const auto &seg = path[j];
                endPoint = seg.getEndPoint();
                pce = poly.classify(endPoint);

                if (pce == 0)
                    return PATH_INVALID;
                if (isSegmentTouchPolygonEdge(seg, poly))
                    return PATH_INVALID;

                PlanePoint3i intersectPoint;

                // clean case
                if (intersectionSegmentPolygon(seg, poly, intersectPoint))
                {
                    // 点对面分类时已经包含了面的法向量
                    // 此时pcs和pce只能是+-1
                    int sigma = (pcs - pce) / 2;
                    for (int k = 0; k < tmpwnv.size(); ++k)
                        tmpwnv[k] += sigma * poly.WNTV[k];
                }

                pcs = pce;
            }
        }

        // 成功计算wnv
        targetWNV.resize(tmpwnv.size());
        targetWNV = tmpwnv;

        return SUCCESS;
    }

    traceStatus tracePathWNVToSurfacePoint(
        const refPoint &refpoint,
        const Path &path,
        const std::vector<Polygon256> &polygons,
        const Plane3i &referencePlane,
        WNV &frontWNV,
        WNV &backWNV)
    {
        if (path.empty())
        {
            return INPUT_INVALID;
        }

        for (const auto &seg : path)
        {
            if (!seg.isValid())
            {
                return INPUT_INVALID;
            }
        }

        for (const auto &poly : polygons)
        {
            if (!poly.isValid() || poly.WNTV.size() != refpoint.wnv.size())
            {
                return INPUT_INVALID;
            }
        }

        if (!areSamePlanePoint(path[0].getStartPoint(), refpoint.point))
        {
            return INPUT_INVALID;
        }

        for (std::size_t i = 1; i < path.size(); ++i)
        {
            if (!areSamePlanePoint(path[i - 1].getEndPoint(), path[i].getStartPoint()))
            {
                return INPUT_INVALID;
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
                if (isSegmentTouchPolygonEdge(seg, poly))
                {
                    return PATH_INVALID;
                }

                PlanePoint3i intersectPoint;
                if (intersectionSegmentPolygon(seg, poly, intersectPoint))
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

