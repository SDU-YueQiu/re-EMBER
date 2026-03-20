#include "clipping.h"

namespace ember
{

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
        
		int intersectionCount = 0;
        for (std::size_t i = 0; i < n; ++i)
        {
            const std::size_t next = (i + 1 == n) ? 0 : (i + 1);
            const Plane3i& segmentEdge = source.edgePlanes[i];

            const int sSide = sides[i];
            const int eSide = sides[next];

            if (sSide == 0)
            {
				if (eSide == 0)
                {
                    return false;
                }
				++intersectionCount;
                if (intersectionCount == 1)
                {
                    p0 = segmentEdge;
                }
                else if (intersectionCount == 2)
                {
                    p1 = segmentEdge;
				}
                else
                {
					std::cout << "[computePolygonPlaneIntersection] intersectionCount > 2" << std::endl;
					return false;
                }
            }
            else if (sSide + eSide == 0)
            {
                ++intersectionCount;
                if (intersectionCount == 1)
                {
                    p0 = segmentEdge;
                }
                else if (intersectionCount == 2)
                {
                    p1 = segmentEdge;
                }
                else
                {
                    std::cout << "[computePolygonPlaneIntersection] intersectionCount > 2" << std::endl;
                    return false;
                }
            }

        }

		return intersectionCount == 2;
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
			std::cout << "[computePolygonIntersectionCarrier] !target.isValid() || !incoming.isValid()" << std::endl;
            return false;
        }

        if (arePlaneNormalsParallel(target.plane, incoming.plane))
        {
            return false;
        }

		Plane3i p0, p1;
		Plane3i q0, q1;
        if (!computePolygonPlaneIntersection(target, incoming.plane, p0, p1))
        {
            return false;
        }
        if (!computePolygonPlaneIntersection(incoming, target.plane, q0, q1))
        {
            return false;
        }

		PlanePoint3i vq0(target.plane, incoming.plane, q0);
		PlanePoint3i vq1(target.plane, incoming.plane, q1);
        if (!vq0.hasUniqueIntersection() || !vq1.hasUniqueIntersection())
        {
			std::cout << "[computePolygonIntersectionCarrier] !vq0.hasUniqueIntersection() || !vq1.hasUniqueIntersection()" << std::endl;
            return false;
        }

		int side00 = vq0.classify(p0);
		int side01 = vq0.classify(p1);
		int side10 = vq1.classify(p0);
		int side11 = vq1.classify(p1);

        if (side00 >= 0 && side10 >= 0) {
            return false;
        }

        if (side01 >= 0 && side11 >= 0) {
            return false;
        }

        outSplitPlane = incoming.plane;
        outV0 = p0;
        outV1 = p1;

        if (side00 <= 0 && side10 <= 0) {
            if (side01 >= 0) {
                outV0 = q1;
            }
            else if (side11 >= 0) {
				outV0 = q0;
            }
            else
            {
                outV0 = q0;
                outV1 = q1;
            }
        }
        else if (side01 <= 0 && side11 <= 0)
        {
            if (side00 >= 0) {
                outV1 = q1;
            }
            else if (side10 >= 0) {
                outV1 = q0;
            }
            else
            {
				std::cout << "[computePolygonIntersectionCarrier] side00 < 0 && side10 < 0 && side01 <= 0 && side11 <= 0 不可能到这啊" << std::endl;
			}
        }
        else {

            if (!((side00 < 0) == (side11 < 0) && (side10 < 0) == (side01 < 0)))
            {
                std::cout << "[computePolygonIntersectionCarrier] side consistency check failed" << std::endl;
                return false;
            }
        }

        return true;
    }



    bool clipLeafGeometryByPlane(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped)
    {
        if (!source.isValid() || arePlaneNormalsParallel(source.plane, clipPlane))
        {
			std::cout << "[clipLeafGeometryByPlane] !source.isValid() || arePlaneNormalsParallel(source.plane, clipPlane)" << std::endl;
            return false;
        }

		frontClipped = Polygon256();
		frontClipped.plane = source.plane;
		frontClipped.WNTV = source.WNTV;
		backClipped = Polygon256();
		backClipped.plane = source.plane;
		backClipped.WNTV = source.WNTV;

		const std::size_t n = source.edgeCount();
        const Plane3i oppositePlane(-clipPlane.a, -clipPlane.b, -clipPlane.c, -clipPlane.d);

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
            const bool sInside = sSide >= 0;
            const bool eInside = eSide >= 0;

			if (sSide == 0 && eSide == 0)
            {
				std::cout << "[clipLeafGeometryByPlane] sSide == 0 && eSide == 0" << std::endl;
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

		bool ret = backClipped.isValid() && frontClipped.isValid();
        if (!ret) {
			std::cout << "[clipLeafGeometryByPlane] !backClipped.isValid() || !frontClipped.isValid()" << std::endl;
        }
        return ret;
    }
}
