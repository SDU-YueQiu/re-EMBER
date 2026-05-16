/**
 * @file math256_tests.cpp
 * @brief 实现精确整数几何 primitive 的回归测试。
 */
#include "math256_tests.h"

#include <cassert>
#include <stdexcept>

#include "geometry/geometry256.h"
#include "math/math256.h"
#include "geometry/plane_geometry256.h"
#include "geometry/clipping.h"
#include "algorithm/bsp.h"

using ember::Integer;
using ember::Vec3i;

#undef assert
#define assert(expr)                                                                \
    do                                                                              \
    {                                                                               \
        if (!(expr))                                                                \
        {                                                                           \
            throw std::runtime_error("math256_tests assertion failed: " #expr);     \
        }                                                                           \
    } while (false)

void runMath256Tests()
{
    {
        assert(ember::absMagnitude(Integer(-7)) == Integer(7));
        assert(ember::gcdMagnitude(Integer(-18), Integer(24)) == Integer(6));
        assert(ember::gcdMagnitude(Integer(6), Integer(9), Integer(12), Integer(15)) == Integer(3));

        const ember::HomPoint4i primitivePoint = ember::primitiveHomPoint(ember::HomPoint4i(2, 4, 6, 2));
        assert(primitivePoint.x == Integer(1));
        assert(primitivePoint.y == Integer(2));
        assert(primitivePoint.z == Integer(3));
        assert(primitivePoint.w == Integer(1));
        assert(ember::areSameHomPoint(primitivePoint, ember::HomPoint4i(3, 6, 9, 3)));
    }

    {
        const Vec3i ex(1, 0, 0);
        const Vec3i ey(0, 1, 0);
        const Vec3i ez(0, 0, 1);

        assert(ember::dot(ex, ey) == Integer(0));
        assert(ember::dot(ex, ex) == Integer(1));
        assert(ember::cross(ex, ey) == ez);
        assert(ember::cross(ey, ex) == -ez);

        assert(ember::determinant3x3(1, 0, 0, 0, 1, 0, 0, 0, 1) == Integer(1));
        assert(ember::determinant(ex, ey, ez) == Integer(1));

        const Vec3i r1(2, 3, 1);
        const Vec3i r2(4, 1, -2);
        const Vec3i r3(-1, 5, 3);
        assert(ember::determinant(r1, r2, r3) == Integer(17));
    }

    {
        const Integer huge = (Integer(1) << 120);
        const Vec3i a(huge, huge + 1, huge - 1);
        const Vec3i b(2, -3, 5);

        const Integer d = ember::dot(a, b);
        assert(d > 0);
        assert(!ember::isZero(d));
        assert(ember::signum(d) > 0);
    }

    {
        const ember::Plane3i px = ember::Plane3i::fromPointNormal(Vec3i(1, 0, 0), Vec3i(1, 0, 0));
        const ember::Plane3i py = ember::Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, 1, 0));
        const ember::Plane3i pz = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, 0, 1));

        ember::Line256 line(px, py);
        assert(line.isValid());

        const ember::PlanePoint3i lineHit = ember::intersect(line, pz);
        assert(lineHit.hasUniqueIntersection());
        assert(line.contains(lineHit));
        const ember::PlanePoint3i degenerateLineHit(px, px, pz);
        assert(!degenerateLineHit.hasUniqueIntersection());

        const ember::Line256 otherLine(py, pz);
        assert(!ember::areParallel(line, otherLine));

        const ember::Line256 axisSegmentLine(px, py);
        const ember::Segment256 segment(
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 6), Vec3i(0, 0, 1)),
            axisSegmentLine);
        assert(segment.isValid());
        const ember::PlanePoint3i firstStartPoint = segment.getStartPointRef();
        const ember::PlanePoint3i firstEndPoint = segment.getEndPointRef();
        assert(ember::areSameHomPoint(segment.getStartPointRef().x, firstStartPoint.x));
        assert(ember::areSameHomPoint(segment.getEndPointRef().x, firstEndPoint.x));

        const ember::Plane3i sameEndpointPlane =
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, 0, 1));
        const ember::Segment256 zeroLengthSegment(sameEndpointPlane, sameEndpointPlane, axisSegmentLine);
        assert(!zeroLengthSegment.isValid());

        ember::Polygon256 poly(
            pz,
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, -1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(2, 0, 3), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 2, 3), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(-1, 0, 0))
        });

        assert(poly.isValid());
        assert(poly.edgeCount() == 4u);
        assert(poly.classify(lineHit) == 0);
        assert(poly.containsOrOnBoundary(lineHit));
        const ember::AABB3i &polyAABB = poly.aabb();
        assert(polyAABB.valid);
        assert(polyAABB.xMin == Integer(0));
        assert(polyAABB.xMax == Integer(2));
        assert(polyAABB.yMin == Integer(0));
        assert(polyAABB.yMax == Integer(2));
        assert(polyAABB.zMin == Integer(3));
        assert(polyAABB.zMax == Integer(3));
        poly.WNTV = {1, 0};

        {
            const ember::Plane3i leftEdgeCarrier =
                ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(1, 0, 0));
            ember::Plane3i p0;
            ember::Plane3i p1;
            assert(ember::computePolygonPlaneIntersection(poly, leftEdgeCarrier, p0, p1));

            const ember::PlanePoint3i hit0(poly.plane, leftEdgeCarrier, p0);
            const ember::PlanePoint3i hit1(poly.plane, leftEdgeCarrier, p1);
            assert(hit0.hasUniqueIntersection());
            assert(hit1.hasUniqueIntersection());
            assert(!ember::areSameHomPoint(hit0.x, hit1.x));
            assert(poly.containsOrOnBoundary(hit0));
            assert(poly.containsOrOnBoundary(hit1));

            const ember::Plane3i diagonalThroughVertices =
                ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(1, -1, 0));
            assert(ember::computePolygonPlaneIntersection(poly, diagonalThroughVertices, p0, p1));

            const ember::Plane3i vertexOnlyTouch =
                ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(1, 1, 0));
            assert(!ember::computePolygonPlaneIntersection(poly, vertexOnlyTouch, p0, p1));

            const ember::Plane3i aabbSeparatedPlane =
                ember::Plane3i::fromPointNormal(Vec3i(0, 5, 0), Vec3i(0, 1, 0));
            assert(!ember::computePolygonPlaneIntersection(poly, aabbSeparatedPlane, p0, p1));
        }

        {
            ember::Polygon256 incoming(
                px,
            std::vector<ember::Plane3i> {
                ember::Plane3i::fromPointNormal(Vec3i(1, 0, 2), Vec3i(0, 1, 0)),   // 平面 y = 0。
                ember::Plane3i::fromPointNormal(Vec3i(1, 0, 2), Vec3i(0, 0, 1)),   // 平面 z = 2。
                ember::Plane3i::fromPointNormal(Vec3i(1, 1, 2), Vec3i(0, -1, 0)),  // 平面 y = 2。
                ember::Plane3i::fromPointNormal(Vec3i(1, 0, 4), Vec3i(0, 0, -1))
            }); // 平面 z = 4。

            assert(incoming.isValid());

            ember::Plane3i splitPlane;
            ember::Plane3i v0;
            ember::Plane3i v1;
            assert(ember::computePolygonIntersectionCarrier(poly, incoming, splitPlane, v0, v1));
            assert(splitPlane.a == incoming.plane.a && splitPlane.b == incoming.plane.b && splitPlane.c == incoming.plane.c && splitPlane.d == incoming.plane.d);

            const ember::PlanePoint3i hit0(poly.plane, splitPlane, v0);
            const ember::PlanePoint3i hit1(poly.plane, splitPlane, v1);
            assert(hit0.hasUniqueIntersection());
            assert(hit1.hasUniqueIntersection());
            assert(!ember::areSameHomPoint(hit0.x, hit1.x));

            assert(poly.containsOrOnBoundary(hit0));
            assert(poly.containsOrOnBoundary(hit1));
            assert(incoming.containsOrOnBoundary(hit0));
            assert(incoming.containsOrOnBoundary(hit1));

        }

        ember::Plane3i splitter = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(-1, 1, 0));
        ember::Polygon256 frontGeometry;
        ember::Polygon256 backGeometry;
        assert(ember::clipLeafGeometryByPlane(poly, splitter, frontGeometry, backGeometry));

        assert(frontGeometry.isValid());
        assert(frontGeometry.edgeCount() == 3);
        assert(backGeometry.isValid());
        assert(backGeometry.edgeCount() == 3);

        ember::Polygon256 taggedFrontGeometry;
        ember::Polygon256 taggedBackGeometry;
        assert(ember::detail::clipLeafGeometryByPlaneTrusted(
                   poly,
                   splitter,
                   taggedFrontGeometry,
                   taggedBackGeometry,
                   ember::PolygonEdgeProvenance::SubdivisionClip));
        assert(taggedFrontGeometry.isValid());
        assert(taggedBackGeometry.isValid());
        assert(taggedFrontGeometry.WNTV == poly.WNTV);
        assert(taggedBackGeometry.WNTV == poly.WNTV);
        assert(taggedFrontGeometry.edgeProvenances.size() == taggedFrontGeometry.edgeCount());
        assert(taggedBackGeometry.edgeProvenances.size() == taggedBackGeometry.edgeCount());
        std::size_t taggedFrontClipEdges = 0;
        for (const auto provenance : taggedFrontGeometry.edgeProvenances)
        {
            if (provenance == ember::PolygonEdgeProvenance::SubdivisionClip)
                ++taggedFrontClipEdges;
        }
        std::size_t taggedBackClipEdges = 0;
        for (const auto provenance : taggedBackGeometry.edgeProvenances)
        {
            if (provenance == ember::PolygonEdgeProvenance::SubdivisionClip)
                ++taggedBackClipEdges;
        }
        assert(taggedFrontClipEdges == 1u);
        assert(taggedBackClipEdges == 1u);

        splitter = ember::Plane3i::fromPointNormal(Vec3i(1, 0, 3), Vec3i(-1, 1, 0));
        assert(ember::clipLeafGeometryByPlane(poly, splitter, frontGeometry, backGeometry));

        assert(frontGeometry.isValid());
        assert(frontGeometry.edgeCount() == 5);
        assert(backGeometry.isValid());
        assert(backGeometry.edgeCount() == 3);

        splitter = ember::Plane3i::fromPointNormal(Vec3i(2, 2, 3), Vec3i(2, -1, 0));
        assert(ember::clipLeafGeometryByPlane(poly, splitter, frontGeometry, backGeometry));

        assert(frontGeometry.isValid());
        assert(frontGeometry.edgeCount() == 3);
        assert(backGeometry.isValid());
        assert(backGeometry.edgeCount() == 4);

        splitter = ember::Plane3i::fromPointNormal(Vec3i(2, 2, 3), Vec3i(1, -2, 0));
        assert(ember::clipLeafGeometryByPlane(poly, splitter, frontGeometry, backGeometry));

        assert(frontGeometry.isValid());
        assert(frontGeometry.edgeCount() == 4);
        assert(backGeometry.isValid());
        assert(backGeometry.edgeCount() == 3);

        {
            const ember::Line256 axisLine(px, pz);
            const ember::Plane3i startWrong = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 1, 0));
            const ember::Plane3i endWrong = ember::Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, -1, 0));
            ember::Segment256 orientedSegment(startWrong, endWrong, axisLine);

            const ember::PlanePoint3i insidePoint(
                axisLine.p1,
                axisLine.p2,
                ember::Plane3i::fromPointNormal(Vec3i(0, 1, 0), Vec3i(0, 1, 0)));
            assert(insidePoint.hasUniqueIntersection());
            assert(insidePoint.classify(orientedSegment.start) < 0);
            assert(insidePoint.classify(orientedSegment.end) < 0);
        }
    }

    {
        const ember::Plane3i pz = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, 0, 1));
        const ember::Polygon256 autoOrientedSquare(
            pz,
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(2, 0, 3), Vec3i(-1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 2, 3), Vec3i(0, -1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(1, 0, 0))
        });

        assert(autoOrientedSquare.isValid());

        const ember::PlanePoint3i interior(
            ember::Plane3i::fromPointNormal(Vec3i(1, 0, 0), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 1, 0), Vec3i(0, 1, 0)),
            pz);
        assert(interior.hasUniqueIntersection());
        assert(autoOrientedSquare.classify(interior) == 0);
        for (const ember::Plane3i& edge : autoOrientedSquare.edgePlanes)
            assert(interior.classify(edge) < 0);

        ember::Polygon256 inwardSquare;
        inwardSquare.plane = pz;
        inwardSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, 1, 0)));
        inwardSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(2, 0, 3), Vec3i(-1, 0, 0)));
        inwardSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(0, 2, 3), Vec3i(0, -1, 0)));
        inwardSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(1, 0, 0)));
        inwardSquare.precomputeVertices();
        assert(!inwardSquare.isValid());

        ember::Polygon256 cachedSquare;
        cachedSquare.plane = pz;
        cachedSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, -1, 0)));
        cachedSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(2, 0, 3), Vec3i(1, 0, 0)));
        cachedSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(0, 2, 3), Vec3i(0, 1, 0)));
        cachedSquare.addEdgePlane(ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(-1, 0, 0)));
        cachedSquare.precomputeVertices();
        assert(cachedSquare.vertices().size() == 4u);
        for (const ember::PlanePoint3i &vertex : cachedSquare.vertices())
            assert(vertex.hasUniqueIntersection());
        assert(cachedSquare.isValid());
        const ember::AABB3i &cachedSquareAABB = cachedSquare.aabb();
        assert(cachedSquareAABB.valid);
        assert(cachedSquareAABB.xMin == Integer(0));
        assert(cachedSquareAABB.xMax == Integer(2));
        assert(cachedSquareAABB.yMin == Integer(0));
        assert(cachedSquareAABB.yMax == Integer(2));
        assert(cachedSquareAABB.zMin == Integer(3));
        assert(cachedSquareAABB.zMax == Integer(3));

        cachedSquare.edgePlanes[1] =
            ember::Plane3i::fromPointNormal(Vec3i(4, 0, 3), Vec3i(1, 0, 0));
        cachedSquare.precomputeVertices();
        const ember::AABB3i &expandedSquareAABB = cachedSquare.aabb();
        assert(expandedSquareAABB.valid);
        assert(expandedSquareAABB.xMin == Integer(0));
        assert(expandedSquareAABB.xMax == Integer(4));
        assert(expandedSquareAABB.yMin == Integer(0));
        assert(expandedSquareAABB.yMax == Integer(2));
        assert(expandedSquareAABB.zMin == Integer(3));
        assert(expandedSquareAABB.zMax == Integer(3));

        const ember::Polygon256 rationalTriangle(
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
            std::vector<ember::Plane3i> {
                ember::Plane3i(0, -1, 0, 0),
                ember::Plane3i(2, 2, 0, -1),
                ember::Plane3i(-1, 0, 0, 0)
            });
        assert(rationalTriangle.isValid());
        const ember::AABB3i &rationalTriangleAABB = rationalTriangle.aabb();
        assert(rationalTriangleAABB.valid);
        assert(rationalTriangleAABB.xMin == Integer(0));
        assert(rationalTriangleAABB.xMax == Integer(1));
        assert(rationalTriangleAABB.yMin == Integer(0));
        assert(rationalTriangleAABB.yMax == Integer(1));
        assert(rationalTriangleAABB.zMin == Integer(0));
        assert(rationalTriangleAABB.zMax == Integer(0));
    }

    {
        const ember::Polygon256 square1 = ember::Polygon256(
                                              ember::Plane3i::fromPointNormal(Vec3i(1, 1, 1), Vec3i(0, 0, -11)),
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(1, 0, 0), Vec3i(0, -1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(2, 1, 2), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 2, 1), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 4), Vec3i(-1, 0, 0))
        });

        const ember::Polygon256 square2 = ember::Polygon256(
                                              ember::Plane3i::fromPointNormal(Vec3i(1, 1, 0), Vec3i(0, 1, 0)),
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(-1, 0, 1), Vec3i(-1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, -100)),
            ember::Plane3i::fromPointNormal(Vec3i(1, 12, 14), Vec3i(12, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 2), Vec3i(0, 0, 10))
        });


        const ember::Polygon256 square3 = ember::Polygon256(
                                              ember::Plane3i::fromPointNormal(Vec3i(1, 1, 0), Vec3i(1, 0, 0)),
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(0, -1, 1), Vec3i(0, -1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, -100)),
            ember::Plane3i::fromPointNormal(Vec3i(12, 1, 14), Vec3i(0, 120, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 2), Vec3i(0, 0, 10))
        });


        assert(square1.isValid());
        assert(square2.isValid());
        assert(square3.isValid());

        ember::BSPTree tree;
        tree.setBasePolygon(square1);
        tree.insert(square2);
        tree.insert(square3);

        auto leafGeometries = tree.collectLeafGeometries();
        assert(leafGeometries.size() == 3);

        for (const auto& leaf : leafGeometries)
        {
            assert(leaf.isValid());
            assert(leaf.edgeCount() == 4);
            assert(leaf.containsOrOnBoundary(ember::PlanePoint3i(square1.plane, square2.plane, square3.plane)));
        }



    }

    {
        const ember::Plane3i px = ember::Plane3i::fromPointNormal(Vec3i(1, 0, 0), Vec3i(1, 0, 0));
        const ember::Plane3i py = ember::Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, 1, 0));
        const ember::Plane3i pz = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, 0, 1));

        assert(px.d == Integer(-1));
        assert(py.d == Integer(-2));
        assert(pz.d == Integer(-3));

        assert(!ember::arePlaneNormalsParallel(px, py));

        const ember::PlanePoint3i point(px, py, pz);
        assert(point.hasUniqueIntersection());
        assert(point.x.x == Integer(1));
        assert(point.x.y == Integer(2));
        assert(point.x.z == Integer(3));
        assert(point.x.w == Integer(1));

        const ember::Plane3i above = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 2), Vec3i(0, 0, 1));
        const ember::Plane3i below = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 4), Vec3i(0, 0, 1));
        const ember::Plane3i on = ember::Plane3i::fromPointNormal(Vec3i(1, 2, 3), Vec3i(1, 1, 1));

        assert(point.classify(above) > 0);
        assert(point.classify(below) < 0);
        assert(point.classify(on) == 0);

    }

    {
        const ember::Polygon256 baseSquare = ember::Polygon256(
                ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, -1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(4, 0, 0), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 4, 0), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(-1, 0, 0))
        });

        const ember::Polygon256 overlapSquare = ember::Polygon256(
                ember::Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(0, 0, 5)),
        std::vector<ember::Plane3i> {
            ember::Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(0, -1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(6, 0, 0), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(2, 4, 0), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(-1, 0, 0))
        });

        assert(baseSquare.isValid());
        assert(overlapSquare.isValid());

        const ember::PlanePoint3i leftInterior(
            ember::Plane3i::fromPointNormal(Vec3i(1, 0, 0), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 1, 0), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)));
        const ember::PlanePoint3i overlapInterior(
            ember::Plane3i::fromPointNormal(Vec3i(3, 0, 0), Vec3i(1, 0, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 1, 0), Vec3i(0, 1, 0)),
            ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)));

        ember::BSPTree highOrderTree;
        highOrderTree.setBasePolygon(baseSquare, 1);
        highOrderTree.insert(overlapSquare, 0);

        auto disabledLeaves = highOrderTree.collectLeafGeometries();
        assert(disabledLeaves.size() == 1);
        assert(disabledLeaves[0].containsOrOnBoundary(leftInterior));
        assert(!disabledLeaves[0].containsOrOnBoundary(overlapInterior));

        ember::BSPTree lowOrderTree;
        lowOrderTree.setBasePolygon(baseSquare, 0);
        lowOrderTree.insert(overlapSquare, 1);

        auto keptLeaves = lowOrderTree.collectLeafGeometries();
        assert(keptLeaves.size() == 2);

        bool foundLeftLeaf = false;
        bool foundOverlapLeaf = false;
        for (const auto& leaf : keptLeaves)
        {
            foundLeftLeaf = foundLeftLeaf || leaf.containsOrOnBoundary(leftInterior);
            foundOverlapLeaf = foundOverlapLeaf || leaf.containsOrOnBoundary(overlapInterior);
        }

        assert(foundLeftLeaf);
        assert(foundOverlapLeaf);
    }
}

