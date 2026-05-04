/**
 * @file clipping.cpp
 * @brief 实现精确的多边形-平面相交与裁剪操作。
 */
#include "clipping.h"

#include "core/logging.h"

#include <sstream>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kIntersectionCarrierScope = "computePolygonIntersectionCarrier";

        bool tryBuildIntersectionCarrierFromCuts(
            const Polygon256& target,
            const Polygon256& incoming,
            const Plane3i& targetCut0,
            const Plane3i& targetCut1,
            const Plane3i& incomingCut0,
            const Plane3i& incomingCut1,
            detail::IntersectionCarrier& outCarrier)
        {
            const PlanePoint3i incomingPoint0(target.plane, incoming.plane, incomingCut0);
            const PlanePoint3i incomingPoint1(target.plane, incoming.plane, incomingCut1);
            if (!incomingPoint0.hasUniqueIntersection() || !incomingPoint1.hasUniqueIntersection())
            {
                emitLog(
                    LogLevel::Debug,
                    LogCategory::Geometry,
                    kIntersectionCarrierScope,
                    "Rejected polygon intersection carrier because the recovered line endpoints are not unique.");
                return false;
            }

            const int side00 = incomingPoint0.classify(targetCut0);
            const int side01 = incomingPoint0.classify(targetCut1);
            const int side10 = incomingPoint1.classify(targetCut0);
            const int side11 = incomingPoint1.classify(targetCut1);

            if (side00 >= 0 && side10 >= 0)
            {
                return false;
            }
            if (side01 >= 0 && side11 >= 0)
            {
                return false;
            }

            outCarrier.splitPlane = incoming.plane;
            outCarrier.v0 = targetCut0;
            outCarrier.v1 = targetCut1;

            if (side00 <= 0 && side10 <= 0)
            {
                if (side01 >= 0)
                {
                    outCarrier.v0 = incomingCut1;
                }
                else if (side11 >= 0)
                {
                    outCarrier.v0 = incomingCut0;
                }
                else
                {
                    outCarrier.v0 = incomingCut0;
                    outCarrier.v1 = incomingCut1;
                }
            }
            else if (side01 <= 0 && side11 <= 0)
            {
                if (side00 >= 0)
                {
                    outCarrier.v1 = incomingCut1;
                }
                else if (side10 >= 0)
                {
                    outCarrier.v1 = incomingCut0;
                }
                else
                {
                    emitLog(
                        LogLevel::Debug,
                        LogCategory::Geometry,
                        kIntersectionCarrierScope,
                        "Reached the unexpected side-assignment branch while trimming the overlap segment.");
                }
            }
            else if (!((side00 < 0) == (side11 < 0) && (side10 < 0) == (side01 < 0)))
            {
                emitLog(
                    LogLevel::Debug,
                    LogCategory::Geometry,
                    kIntersectionCarrierScope,
                    "Rejected polygon intersection carrier because the side-consistency check failed.");
                return false;
            }

            return true;
        }
    }

    bool computePolygonPlaneIntersection(
        const Polygon256& source,
        const Plane3i& target,
        Plane3i& p0,
        Plane3i& p1)
    {
        const std::size_t n = source.edgeCount();

        std::vector<int> sides;
        sides.resize(n);

        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t last = (i == 0) ? (n - 1) : (i - 1);
            const PlanePoint3i v(source.plane, source.edgePlanes[i], source.edgePlanes[last]);

            sides[i] = v.classify(target);
        }
        
        std::vector<Plane3i> intersectionCarriers;
        std::vector<PlanePoint3i> intersectionPoints;

        auto appendIntersectionCarrier = [&](const Plane3i& carrier) -> bool
        {
            const PlanePoint3i point(source.plane, target, carrier);
            if (!point.hasUniqueIntersection())
            {
                return true;
            }

            for (const PlanePoint3i& existing : intersectionPoints)
            {
                if (areSameHomPoint(existing.x, point.x))
                {
                    return true;
                }
            }

            if (intersectionCarriers.size() == 2u)
            {
                emitLog(
                    LogLevel::Debug,
                    LogCategory::Geometry,
                    "computePolygonPlaneIntersection",
                    "Rejected polygon-plane intersection because more than two intersections were detected.");
                return false;
            }

            intersectionCarriers.push_back(carrier);
            intersectionPoints.push_back(point);
            return true;
        };

        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t next = (i + 1 == n) ? 0 : (i + 1);
            const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
            const Plane3i& segmentEdge = source.edgePlanes[i];

            const int sSide = sides[i];// 起点侧别。
            const int eSide = sides[next];// 终点侧别。

            if (sSide == 0 && eSide == 0)
            {
                if (!appendIntersectionCarrier(source.edgePlanes[prev]) ||
                    !appendIntersectionCarrier(source.edgePlanes[next]))
                {
                    return false;
                }
                continue;
            }

            if (sSide == 0)//起点在裁剪平面上
            {
                const int prevSide = sides[prev];
                if ((prevSide < 0 && eSide > 0) || (prevSide > 0 && eSide < 0))
                {
                    if (!appendIntersectionCarrier(segmentEdge))
                    {
                        return false;
                    }
                }
                continue;
            }

            if (sSide + eSide == 0)//边两端点在裁剪平面两侧->裁剪平面与边相交
            {
                if (!appendIntersectionCarrier(segmentEdge))
                {
                    return false;
                }
            }

        }

        if (intersectionCarriers.size() != 2u ||
            areSameHomPoint(intersectionPoints[0].x, intersectionPoints[1].x))
        {
            return false;
        }

        p0 = intersectionCarriers[0];
        p1 = intersectionCarriers[1];
		return true;
    }

    bool computePolygonIntersectionCarrier(
        const Polygon256& target,
        const Polygon256& incoming,
        Plane3i& outSplitPlane,
        Plane3i& outV0,
        Plane3i& outV1)
    {
        if (!target.isValid() || !incoming.isValid())
        {
            emitLog(
                LogLevel::Debug,
                LogCategory::Geometry,
                "computePolygonIntersectionCarrier",
                "Rejected polygon intersection carrier because one polygon is invalid.");
            return false;
        }

        return detail::computePolygonIntersectionCarrierTrusted(target, incoming, outSplitPlane, outV0, outV1);
    }

    bool detail::computePolygonIntersectionCarrierTrusted(
        const Polygon256& target,
        const Polygon256& incoming,
        Plane3i& outSplitPlane,
        Plane3i& outV0,
        Plane3i& outV1)
    {
        //法向平行要么共面要么不相交
        if (arePlaneNormalsParallel(target.plane, incoming.plane))
        {
            return false;
        }

        Plane3i p0, p1;//源多边形上的交线边平面
        Plane3i q0, q1;//新输入多边形上的交线边平面
        if (!computePolygonPlaneIntersection(target, incoming.plane, p0, p1))
        {
            return false;
        }
        if (!computePolygonPlaneIntersection(incoming, target.plane, q0, q1))
        {
            return false;
        }

        detail::IntersectionCarrier carrier;
        if (!tryBuildIntersectionCarrierFromCuts(target, incoming, p0, p1, q0, q1, carrier))
        {
            return false;
        }

        outSplitPlane = carrier.splitPlane;
        outV0 = carrier.v0;
        outV1 = carrier.v1;
        return true;
    }

    bool detail::computeBidirectionalPolygonIntersectionCarriersTrusted(
        const Polygon256& lhs,
        const Polygon256& rhs,
        IntersectionCarrier& outLhsCarrier,
        IntersectionCarrier& outRhsCarrier)
    {
        if (arePlaneNormalsParallel(lhs.plane, rhs.plane))
        {
            return false;
        }

        Plane3i lhsCut0;
        Plane3i lhsCut1;
        Plane3i rhsCut0;
        Plane3i rhsCut1;
        if (!computePolygonPlaneIntersection(lhs, rhs.plane, lhsCut0, lhsCut1))
        {
            return false;
        }
        if (!computePolygonPlaneIntersection(rhs, lhs.plane, rhsCut0, rhsCut1))
        {
            return false;
        }

        return tryBuildIntersectionCarrierFromCuts(lhs, rhs, lhsCut0, lhsCut1, rhsCut0, rhsCut1, outLhsCarrier) &&
               tryBuildIntersectionCarrierFromCuts(rhs, lhs, rhsCut0, rhsCut1, lhsCut0, lhsCut1, outRhsCarrier);
    }

    bool clipLeafGeometryByPlane(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped)
    {
        if (!source.isValid())
        {
            emitLog(
                LogLevel::Debug,
                LogCategory::Geometry,
                "clipLeafGeometryByPlane",
                "Rejected leaf clipping because the source polygon is invalid.");
            return false;
        }

        return detail::clipLeafGeometryByPlaneTrusted(source, clipPlane, frontClipped, backClipped);
    }

    //按顶点分类裁剪
    bool detail::clipLeafGeometryByPlaneTrusted(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped)
    {
        if (arePlaneNormalsParallel(source.plane, clipPlane))
        {
            emitLog(
                LogLevel::Debug,
                LogCategory::Geometry,
                "clipLeafGeometryByPlane",
                "Rejected leaf clipping because the clip plane is parallel to the source polygon.");
            return false;
        }

		frontClipped = Polygon256();
		frontClipped.plane = source.plane;
		frontClipped.WNTV = source.WNTV;
		backClipped = Polygon256();
		backClipped.plane = source.plane;
		backClipped.WNTV = source.WNTV;

		const std::size_t n = source.edgeCount();
        const Plane3i oppositePlane(-clipPlane.a, -clipPlane.b, -clipPlane.c, -clipPlane.d);// 取反后表示裁剪平面的另一侧。

		std::vector<int> sides;
		sides.resize(n);

        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t last = (i == 0) ? (n - 1) : (i - 1);
			const PlanePoint3i v(source.plane, source.edgePlanes[i], source.edgePlanes[last]);

			sides[i] = v.classify(clipPlane);
        }

        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t next = (i + 1 == n) ? 0 : (i + 1);
            const Plane3i& segmentEdge = source.edgePlanes[i];

            const int sSide = sides[i];
            const int eSide = sides[next];
            const bool sInside = (sSide >= 0);
            const bool eInside = (eSide >= 0);

			if (sSide == 0 && eSide == 0)
            {
                emitLog(
                    LogLevel::Debug,
                    LogCategory::Geometry,
                    "clipLeafGeometryByPlane",
                    "Rejected leaf clipping because an entire edge lies on the clip plane.");
                frontClipped = Polygon256();
				backClipped = Polygon256();
                return false;
            }

            if (sInside && eInside)
            {
				frontClipped.addEdgePlane(segmentEdge);
                continue;
            }

            if (!sInside && !eInside)
            {
				backClipped.addEdgePlane(segmentEdge);
                continue;
            }

            if (sInside && !eInside)
            {
				if (sSide == 1) // (s,e) == (1, -1)
                {
                    frontClipped.addEdgePlane(segmentEdge);
                }
				frontClipped.addEdgePlane(oppositePlane);

                backClipped.addEdgePlane(segmentEdge);
                continue;
            }

            if (!sInside && eInside)
            {
                backClipped.addEdgePlane(segmentEdge);
				backClipped.addEdgePlane(clipPlane);
                if (eSide == 1) // (s,e) == (-1, 1)
                {
                    frontClipped.addEdgePlane(segmentEdge);
				}
            }
        }

        Polygon256 orientedFront(frontClipped.plane, std::move(frontClipped.edgePlanes));
        orientedFront.WNTV = std::move(frontClipped.WNTV);
        Polygon256 orientedBack(backClipped.plane, std::move(backClipped.edgePlanes));
        orientedBack.WNTV = std::move(backClipped.WNTV);

        const bool frontValid = orientedFront.isValid();
        const bool backValid = orientedBack.isValid();
		bool ret = backValid && frontValid;
        if (!ret) {
            std::ostringstream message;
            message << "Rejected leaf clipping because one clipped polygon is invalid"
                    << " source_plane=" << source.plane
                    << " clip_plane=" << clipPlane
                    << " source_edges=" << source.edgeCount()
                    << " front_edges=" << orientedFront.edgeCount()
                    << " back_edges=" << orientedBack.edgeCount()
                    << " front_valid=" << frontValid
                    << " back_valid=" << backValid
                    << " sides=[";
            for (std::size_t i = 0; i < sides.size(); ++i)
            {
                if (i != 0)
                {
                    message << ",";
                }
                message << sides[i];
            }
            message << "].";
            emitLog(
                LogLevel::Debug,
                LogCategory::Geometry,
                "clipLeafGeometryByPlane",
                message.str());
            frontClipped = Polygon256();
            backClipped = Polygon256();
            return false;
        }

        frontClipped = std::move(orientedFront);
        backClipped = std::move(orientedBack);
        return true;
    }
}

