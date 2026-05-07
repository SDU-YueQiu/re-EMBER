/**
 * @file clipping.cpp
 * @brief 实现精确的多边形-平面相交与裁剪操作。
 */
#include "clipping.h"

#include <sstream>
#include <utility>

namespace ember
{
namespace
{
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
        return false;

    const int side00 = incomingPoint0.classify(targetCut0);
    const int side01 = incomingPoint0.classify(targetCut1);
    const int side10 = incomingPoint1.classify(targetCut0);
    const int side11 = incomingPoint1.classify(targetCut1);

    if (side00 >= 0 && side10 >= 0)
        return false;
    if (side01 >= 0 && side11 >= 0)
        return false;

    outCarrier.splitPlane = incoming.plane;
    outCarrier.v0 = targetCut0;
    outCarrier.v1 = targetCut1;

    if (side00 <= 0 && side10 <= 0)
    {
        if (side01 >= 0)
            outCarrier.v0 = incomingCut1;
        else if (side11 >= 0)
            outCarrier.v0 = incomingCut0;
        else
        {
            outCarrier.v0 = incomingCut0;
            outCarrier.v1 = incomingCut1;
        }
    }
    else if (side01 <= 0 && side11 <= 0)
    {
        if (side00 >= 0)
            outCarrier.v1 = incomingCut1;
        else if (side10 >= 0)
            outCarrier.v1 = incomingCut0;
        else
        {
        }
    }
    else if (!((side00 < 0) == (side11 < 0) && (side10 < 0) == (side01 < 0)))
        return false;

    return true;
}
}

bool computePolygonPlaneIntersection(
    const Polygon256& source,
    const Plane3i& target,
    Plane3i& p0,
    Plane3i& p1)
{
    REEMBER_PROFILE_ZONE("computePolygonPlaneIntersection");

    const AABB3i &sourceBox = source.aabb();
    if (!isValidAABB(sourceBox) || !doesPlaneIntersectAABB(target, sourceBox))
        return false;

    const std::size_t n = source.edgeCount();

    std::vector<int> sides;
    sides.resize(n);
    {
        REEMBER_PROFILE_ZONE("computePolygonPlaneIntersection::classifyVertices");
        for (std::size_t i = 0; i < n; ++i)
        {
            const PlanePoint3i &v = source.vertex(i);
            sides[i] = v.classify(target);
        }
    }

    std::vector<Plane3i> intersectionCarriers;
    std::vector<PlanePoint3i> intersectionPoints;

    auto appendIntersectionCarrier = [&](const Plane3i& carrier) -> bool
    {
        const PlanePoint3i point(source.plane, target, carrier);
        if (!point.hasUniqueIntersection())
            return true;

        for (const PlanePoint3i& existing : intersectionPoints)
        {
            if (areSameHomPoint(existing.x, point.x))
                return true;
        }

        if (intersectionCarriers.size() == 2u)
            return false;

        intersectionCarriers.push_back(carrier);
        intersectionPoints.push_back(point);
        return true;
    };

    {
        REEMBER_PROFILE_ZONE("computePolygonPlaneIntersection::collectCarriers");
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
                    return false;
                continue;
            }

            if (sSide == 0)//起点在裁剪平面上
            {
                const int prevSide = sides[prev];
                if ((prevSide < 0 && eSide > 0) || (prevSide > 0 && eSide < 0))
                {
                    if (!appendIntersectionCarrier(segmentEdge))
                        return false;
                }
                continue;
            }

            if (sSide + eSide == 0)//边两端点在裁剪平面两侧->裁剪平面与边相交
            {
                if (!appendIntersectionCarrier(segmentEdge))
                    return false;
            }

        }
    }

    if (intersectionCarriers.size() != 2u ||
            areSameHomPoint(intersectionPoints[0].x, intersectionPoints[1].x))
        return false;

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
        return false;

    return detail::computePolygonIntersectionCarrierTrusted(target, incoming, outSplitPlane, outV0, outV1);
}

bool detail::computePolygonIntersectionCarrierTrusted(
    const Polygon256& target,
    const Polygon256& incoming,
    Plane3i& outSplitPlane,
    Plane3i& outV0,
    Plane3i& outV1)
{
    REEMBER_PROFILE_ZONE("computePolygonIntersectionCarrierTrusted");

    if (!doAABBsOverlap(target.aabb(), incoming.aabb()))
        return false;

    //法向平行要么共面要么不相交
    if (arePlaneNormalsParallel(target.plane, incoming.plane))
        return false;

    Plane3i p0, p1;//源多边形上的交线边平面
    Plane3i q0, q1;//新输入多边形上的交线边平面
    {
        REEMBER_PROFILE_ZONE("computePolygonIntersectionCarrierTrusted::planeIntersections");
        if (!computePolygonPlaneIntersection(target, incoming.plane, p0, p1))
            return false;
        if (!computePolygonPlaneIntersection(incoming, target.plane, q0, q1))
            return false;
    }

    detail::IntersectionCarrier carrier;
    {
        REEMBER_PROFILE_ZONE("computePolygonIntersectionCarrierTrusted::buildCarrier");
        if (!tryBuildIntersectionCarrierFromCuts(target, incoming, p0, p1, q0, q1, carrier))
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
    REEMBER_PROFILE_ZONE("computeBidirectionalPolygonIntersectionCarriersTrusted");

    if (!doAABBsOverlap(lhs.aabb(), rhs.aabb()))
        return false;

    if (arePlaneNormalsParallel(lhs.plane, rhs.plane))
        return false;

    Plane3i lhsCut0;
    Plane3i lhsCut1;
    Plane3i rhsCut0;
    Plane3i rhsCut1;
    {
        REEMBER_PROFILE_ZONE("computeBidirectionalPolygonIntersectionCarriersTrusted::planeIntersections");
        if (!computePolygonPlaneIntersection(lhs, rhs.plane, lhsCut0, lhsCut1))
            return false;
        if (!computePolygonPlaneIntersection(rhs, lhs.plane, rhsCut0, rhsCut1))
            return false;
    }

    {
        REEMBER_PROFILE_ZONE("computeBidirectionalPolygonIntersectionCarriersTrusted::buildCarriers");
        return tryBuildIntersectionCarrierFromCuts(lhs, rhs, lhsCut0, lhsCut1, rhsCut0, rhsCut1, outLhsCarrier) &&
               tryBuildIntersectionCarrierFromCuts(rhs, lhs, rhsCut0, rhsCut1, lhsCut0, lhsCut1, outRhsCarrier);
    }
}

bool clipLeafGeometryByPlane(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped)
{
    if (!source.isValid())
        return false;

    return detail::clipLeafGeometryByPlaneTrusted(source, clipPlane, frontClipped, backClipped);
}

//按顶点分类裁剪
bool detail::clipLeafGeometryByPlaneTrusted(
    const Polygon256& source,
    const Plane3i& clipPlane,
    Polygon256& frontClipped,
    Polygon256& backClipped,
    PolygonEdgeProvenance insertedEdgeProvenance)
{
    if (arePlaneNormalsParallel(source.plane, clipPlane))
        return false;

    std::vector<int> sides;
    sides.resize(source.edgeCount());
    for (std::size_t i = 0; i < source.edgeCount(); ++i)
    {
        const PlanePoint3i &v = source.vertex(i);
        sides[i] = v.classify(clipPlane);
    }

    return clipLeafGeometryByPlaneTrustedWithSides(
               source,
               clipPlane,
               sides,
               frontClipped,
               backClipped,
               insertedEdgeProvenance);
}

bool detail::clipLeafGeometryByPlaneTrustedWithSides(
    const Polygon256& source,
    const Plane3i& clipPlane,
    const std::vector<int>& vertexSides,
    Polygon256& frontClipped,
    Polygon256& backClipped,
    PolygonEdgeProvenance insertedEdgeProvenance)
{
    frontClipped = Polygon256();
    frontClipped.plane = source.plane;
    frontClipped.WNTV = source.WNTV;
    backClipped = Polygon256();
    backClipped.plane = source.plane;
    backClipped.WNTV = source.WNTV;

    const std::size_t n = source.edgeCount();
    if (vertexSides.size() != n)
        return false;

    const Plane3i oppositePlane(-clipPlane.a, -clipPlane.b, -clipPlane.c, -clipPlane.d);// 取反后表示裁剪平面的另一侧。

    for (std::size_t i = 0; i < n; ++i)
    {
        const std::size_t next = (i + 1 == n) ? 0 : (i + 1);
        const Plane3i& segmentEdge = source.edgePlanes[i];
        const PolygonEdgeProvenance segmentEdgeProvenance = source.edgeProvenance(i);

        const int sSide = vertexSides[i];
        const int eSide = vertexSides[next];
        const bool sInside = (sSide >= 0);
        const bool eInside = (eSide >= 0);

        if (sSide == 0 && eSide == 0)
        {
            frontClipped = Polygon256();
            backClipped = Polygon256();
            return false;
        }

        if (sInside && eInside)
        {
            frontClipped.addEdgePlane(segmentEdge, segmentEdgeProvenance);
            continue;
        }

        if (!sInside && !eInside)
        {
            backClipped.addEdgePlane(segmentEdge, segmentEdgeProvenance);
            continue;
        }

        if (sInside && !eInside)
        {
            if (sSide == 1) // (s,e) == (1, -1)
                frontClipped.addEdgePlane(segmentEdge, segmentEdgeProvenance);
            frontClipped.addEdgePlane(oppositePlane, insertedEdgeProvenance);

            backClipped.addEdgePlane(segmentEdge, segmentEdgeProvenance);
            continue;
        }

        if (!sInside && eInside)
        {
            backClipped.addEdgePlane(segmentEdge, segmentEdgeProvenance);
            backClipped.addEdgePlane(clipPlane, insertedEdgeProvenance);
            if (eSide == 1) // (s,e) == (-1, 1)
                frontClipped.addEdgePlane(segmentEdge, segmentEdgeProvenance);
        }
    }

    Polygon256 orientedFront(
        frontClipped.plane,
        std::move(frontClipped.edgePlanes),
        std::move(frontClipped.edgeProvenances));
    orientedFront.WNTV = std::move(frontClipped.WNTV);
    Polygon256 orientedBack(
        backClipped.plane,
        std::move(backClipped.edgePlanes),
        std::move(backClipped.edgeProvenances));
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
        for (std::size_t i = 0; i < vertexSides.size(); ++i)
        {
            if (i != 0)
                message << ",";
            message << vertexSides[i];
        }
        message << "].";
        frontClipped = Polygon256();
        backClipped = Polygon256();
        return false;
    }

    frontClipped = std::move(orientedFront);
    backClipped = std::move(orientedBack);
    return true;
}
}

