#include "math256_tests.h"

#include <cassert>

#include "geometry/geometry256.h"
#include "math/math256.h"
#include "math/plane_geometry256.h"
#include "algorithm/clipping.h"
#include "algorithm/bsp.h"

using ember::Integer;
using ember::Vec2i;
using ember::Vec3i;

void runMath256Tests()
{
	{
		const Vec2i a(3, 4);
		const Vec2i b(-2, 5);

		assert(a + b == Vec2i(1, 9));
		assert(a - b == Vec2i(5, -1));
		assert(2 * a == Vec2i(6, 8));
		assert((a * 3) / 3 == a);

		assert(ember::dot(a, b) == Integer(14));
		assert(ember::cross(a, b) == Integer(23));
		assert(a.lengthSquared() == Integer(25));

		const Vec2i p0(0, 0);
		const Vec2i p1(4, 0);
		const Vec2i p2(4, 3);
		const Vec2i p3(2, 0);

		assert(ember::orient2dSign(p0, p1, p2) > 0);
		assert(ember::orient2dSign(p0, p1, p3) == 0);
		assert(ember::orient2dSign(p2, p1, p0) < 0);

		assert(ember::determinant2x2(3, 4, -2, 5) == Integer(23));
		assert(ember::determinant(a, b) == Integer(23));
	}

	{
		const Vec3i ex(1, 0, 0);
		const Vec3i ey(0, 1, 0);
		const Vec3i ez(0, 0, 1);

		assert(ember::dot(ex, ey) == Integer(0));
		assert(ember::dot(ex, ex) == Integer(1));
		assert(ember::cross(ex, ey) == ez);
		assert(ember::cross(ey, ex) == -ez);

		const Vec3i v(3, -4, 12);
		assert(v.lengthSquared() == Integer(169));

		assert(ember::scalarTriple(ex, ey, ez) == Integer(1));
		assert(ember::orient3dSign(ex, ey, ez) > 0);
		assert(ember::orient3dSign(ex, ez, ey) < 0);

		const Vec3i cop1(2, 2, 2);
		const Vec3i cop2(4, 4, 4);
		assert(ember::orient3dSign(ex, cop1, cop2) == 0);

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
		assert(ember::isPositive(d));
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

		const ember::Line256 otherLine(py, pz);
		assert(!ember::areParallel(line, otherLine));

		ember::Segment256 segment(px, py);
		assert(!segment.isDegenerate());

		ember::Polygon256 poly(
			pz,
			std::vector<ember::Plane3i>{
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(0, -1, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(2, 0, 3), Vec3i(1, 0, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 2, 3), Vec3i(0, 1, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(-1, 0, 0))
				});

		assert(poly.isValid());
		assert(poly.edgeCount() == 4u);
		assert(poly.classify(lineHit) < 0);
		assert(poly.containsOrOnBoundary(lineHit));

		{
			ember::Polygon256 incoming(
				px,
				std::vector<ember::Plane3i>{
					ember::Plane3i::fromPointNormal(Vec3i(1, 0, 2), Vec3i(0, 1, 0)),   // y = 0
					ember::Plane3i::fromPointNormal(Vec3i(1, 0, 2), Vec3i(0, 0, 1)),   // z = 2
					ember::Plane3i::fromPointNormal(Vec3i(1, 1, 2), Vec3i(0, -1, 0)),  // y = 2
					ember::Plane3i::fromPointNormal(Vec3i(1, 0, 4), Vec3i(0, 0, -1))}); // z = 4

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

			std::cout << "hit0 " << hit0 << std::endl;
			std::cout << "nit1 " << hit1 << std::endl;
		}

		ember::Plane3i splitter = ember::Plane3i::fromPointNormal(Vec3i(0, 0, 3), Vec3i(-1, 1, 0));
		ember::Polygon256 frontGeometry;
		ember::Polygon256 backGeometry;
		assert(ember::clipLeafGeometryByPlane(poly, splitter, frontGeometry, backGeometry));

		assert(frontGeometry.isValid());
		assert(frontGeometry.edgeCount() == 3);
		assert(backGeometry.isValid());
		assert(backGeometry.edgeCount() == 3);

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
	}

	{
		const ember::Polygon256 square1 = ember::Polygon256(
			ember::Plane3i::fromPointNormal(Vec3i(1, 1, 1), Vec3i(0, 0, -11)),
			std::vector<ember::Plane3i>{
				ember::Plane3i::fromPointNormal(Vec3i(1, 0, 0), Vec3i(0, -1, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(2, 1, 2), Vec3i(1, 0, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 2, 1), Vec3i(0, 1, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 4), Vec3i(-1, 0, 0))
		});

		const ember::Polygon256 square2 = ember::Polygon256(
			ember::Plane3i::fromPointNormal(Vec3i(1, 1, 0), Vec3i(0, 1, 0)),
			std::vector<ember::Plane3i>{
				ember::Plane3i::fromPointNormal(Vec3i(-1, 0, 1), Vec3i(-1, 0, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, -100)),
				ember::Plane3i::fromPointNormal(Vec3i(1, 12, 14), Vec3i(12, 0, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 2), Vec3i(0, 0, 10))
				});


		const ember::Polygon256 square3 = ember::Polygon256(
			ember::Plane3i::fromPointNormal(Vec3i(1, 1, 0), Vec3i(1, 0, 0)),
			std::vector<ember::Plane3i>{
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

		assert(point.classify(above) == ember::classifyByDeterminants(px, py, pz, above));
		assert(point.classify(below) == ember::classifyByDeterminants(px, py, pz, below));
		assert(point.classify(on) == ember::classifyByDeterminants(px, py, pz, on));
	}

	{
		const ember::Polygon256 baseSquare = ember::Polygon256(
			ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
			std::vector<ember::Plane3i>{
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, -1, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(4, 0, 0), Vec3i(1, 0, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 4, 0), Vec3i(0, 1, 0)),
				ember::Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(-1, 0, 0))
			});

		const ember::Polygon256 overlapSquare = ember::Polygon256(
			ember::Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(0, 0, 5)),
			std::vector<ember::Plane3i>{
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
