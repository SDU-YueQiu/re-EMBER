/**
 * @file bool_problem_tests.cpp
 * @brief 实现布尔求解和路径追踪行为的回归测试。
 */
#include "bool_problem_tests.h"

#include "core/bool_problem.h"
#include "algorithm/WNV_tracing.h"
#include "algorithm/path_candidates.h"
#include "algorithm/tracing_geometry.h"
#include "geometry/clipping.h"

#include <cassert>
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <string>

namespace
{
    using ember::BoolOp;
    using ember::Integer;
    using ember::Plane3i;
    using ember::PlanePoint3i;
    using ember::Polygon256;
    using ember::Vec3i;

    Polygon256 makeFaceXY(int z, int xmin, int xmax, int ymin, int ymax, int normalZ)
    {
        return Polygon256(
            Plane3i::fromPointNormal(Vec3i(0, 0, z), Vec3i(0, 0, normalZ)),
            std::vector<Plane3i>{
                Plane3i::fromPointNormal(Vec3i(xmin, ymin, z), Vec3i(0, -1, 0)),
                Plane3i::fromPointNormal(Vec3i(xmax, ymin, z), Vec3i(1, 0, 0)),
                Plane3i::fromPointNormal(Vec3i(xmin, ymax, z), Vec3i(0, 1, 0)),
                Plane3i::fromPointNormal(Vec3i(xmin, ymin, z), Vec3i(-1, 0, 0))});
    }

    Polygon256 makeFaceYZ(int x, int ymin, int ymax, int zmin, int zmax, int normalX)
    {
        return Polygon256(
            Plane3i::fromPointNormal(Vec3i(x, 0, 0), Vec3i(normalX, 0, 0)),
            std::vector<Plane3i>{
                Plane3i::fromPointNormal(Vec3i(x, ymin, zmin), Vec3i(0, -1, 0)),
                Plane3i::fromPointNormal(Vec3i(x, ymin, zmax), Vec3i(0, 0, 1)),
                Plane3i::fromPointNormal(Vec3i(x, ymax, zmin), Vec3i(0, 1, 0)),
                Plane3i::fromPointNormal(Vec3i(x, ymin, zmin), Vec3i(0, 0, -1))});
    }

    Polygon256 makeFaceXZ(int y, int xmin, int xmax, int zmin, int zmax, int normalY)
    {
        return Polygon256(
            Plane3i::fromPointNormal(Vec3i(0, y, 0), Vec3i(0, normalY, 0)),
            std::vector<Plane3i>{
                Plane3i::fromPointNormal(Vec3i(xmin, y, zmin), Vec3i(-1, 0, 0)),
                Plane3i::fromPointNormal(Vec3i(xmin, y, zmax), Vec3i(0, 0, 1)),
                Plane3i::fromPointNormal(Vec3i(xmax, y, zmin), Vec3i(1, 0, 0)),
                Plane3i::fromPointNormal(Vec3i(xmin, y, zmin), Vec3i(0, 0, -1))});
    }

    std::vector<Polygon256> makeAxisAlignedBox(int xmin, int ymin, int zmin, int xmax, int ymax, int zmax)
    {
        return {
            makeFaceYZ(xmin, ymin, ymax, zmin, zmax, -1),
            makeFaceYZ(xmax, ymin, ymax, zmin, zmax, 1),
            makeFaceXZ(ymin, xmin, xmax, zmin, zmax, -1),
            makeFaceXZ(ymax, xmin, xmax, zmin, zmax, 1),
            makeFaceXY(zmin, xmin, xmax, ymin, ymax, -1),
            makeFaceXY(zmax, xmin, xmax, ymin, ymax, 1)};
    }

    void assignWNTV(std::vector<Polygon256> &polygons, const ember::WNV &wntv)
    {
        for (Polygon256 &polygon : polygons)
        {
            polygon.WNTV = wntv;
        }
    }

    Polygon256 makeThinTriangleXY()
    {
        return Polygon256(
            Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
            std::vector<Plane3i>{
                Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, -1, 0)),
                Plane3i(1, 100, 0, -100),
                Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(-1, 0, 0))});
    }

    Polygon256 makeLargeOffsetThinTriangleXY()
    {
        const Integer base = Integer(1) << 75;
        return Polygon256(
            Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
            std::vector<Plane3i>{
                Plane3i(0, -1, 0, 0),
                Plane3i(1, 1, 0, -base - Integer(1)),
                Plane3i(-1, 0, 0, base)});
    }

    Polygon256 makeInvalidInwardSquareXY()
    {
        Polygon256 polygon;
        polygon.plane = Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1));
        polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 1, 0)));
        polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(-1, 0, 0)));
        polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, -1, 0)));
        polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(1, 0, 0)));
        polygon.WNTV = {1, 0};
        return polygon;
    }

    Polygon256 makeSubdivisionClippedFaceYZ()
    {
        Polygon256 polygon = makeFaceYZ(0, 0, 4, 0, 4, 1);
        polygon.WNTV = {1, 0};

        Polygon256 frontPolygon;
        Polygon256 backPolygon;
        if (!ember::detail::clipLeafGeometryByPlaneTrusted(
                polygon,
                Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, 1, 0)),
                frontPolygon,
                backPolygon,
                ember::PolygonEdgeProvenance::SubdivisionClip))
        {
            throw std::runtime_error("bool_problem_tests failed to build a subdivision-clipped polygon.");
        }

        return backPolygon;
    }

    ember::Segment256 makeAxisSegment(const PlanePoint3i &start, const PlanePoint3i &end)
    {
        ember::Segment256 segment;
        if (!ember::detail::buildAxisAlignedSegment(start, end, segment))
        {
            throw std::runtime_error("bool_problem_tests failed to build an axis-aligned segment.");
        }
        if (!ember::areSamePlanePoint(segment.getStartPoint(), start) ||
            !ember::areSamePlanePoint(segment.getEndPoint(), end))
        {
            throw std::runtime_error("bool_problem_tests built a segment with unexpected endpoints.");
        }
        return segment;
    }

    ember::Path makeAxisPath(std::initializer_list<PlanePoint3i> points)
    {
        if (points.size() < 2u)
        {
            return {};
        }

        std::vector<PlanePoint3i> pointList(points);
        ember::Path path;
        path.reserve(pointList.size() - 1u);
        for (std::size_t i = 1; i < pointList.size(); ++i)
        {
            path.push_back(makeAxisSegment(pointList[i - 1], pointList[i]));
        }
        return path;
    }

    bool throwsRuntimeError(const std::function<void()> &fn, const std::string &needle = std::string())
    {
        try
        {
            fn();
        }
        catch (const std::runtime_error &ex)
        {
            return needle.empty() || std::string(ex.what()).find(needle) != std::string::npos;
        }

        return false;
    }

}

#undef assert
#define assert(expr)                                                                   \
    do                                                                                 \
    {                                                                                  \
        if (!(expr))                                                                   \
        {                                                                              \
            throw std::runtime_error("bool_problem_tests assertion failed: " #expr);   \
        }                                                                              \
    } while (false)

namespace
{
    void assertResultFragmentIsGeometryOnly(const Polygon256 &fragment)
    {
        assert(fragment.isValid());
    }
}

void runBoolProblemTests()
{
    const std::vector<Polygon256> lhs = makeAxisAlignedBox(0, 0, 0, 1, 1, 1);
    const std::vector<Polygon256> rhs = makeAxisAlignedBox(3, 3, 3, 4, 4, 4);

    {
        const Polygon256 square = makeFaceXY(0, 0, 2, 0, 2, 1);
        const std::vector<PlanePoint3i> candidates = ember::enumerateLeafClassificationPointCandidates(square);
        assert(!candidates.empty());
        assert(square.containsStrictly(candidates.front()));
        assert(ember::areSamePlanePoint(candidates.front(), ember::makeIntegerPoint(1, 1, 0)));
    }

    {
        const Polygon256 thinTriangle = makeThinTriangleXY();
        assert(thinTriangle.isValid());

        const PlanePoint3i roundedCentroid = ember::makeIntegerPoint(33, 0, 0);
        assert(!thinTriangle.containsStrictly(roundedCentroid));

        const std::vector<PlanePoint3i> candidates = ember::enumerateLeafClassificationPointCandidates(thinTriangle);
        assert(!candidates.empty());
        for (const PlanePoint3i &candidate : candidates)
        {
            assert(thinTriangle.containsStrictly(candidate));
        }
    }

    {
        const Polygon256 shiftedThinTriangle = makeLargeOffsetThinTriangleXY();
        assert(shiftedThinTriangle.isValid());

        const Integer base = Integer(1) << 75;
        const PlanePoint3i roundedCentroid = ember::makeIntegerPoint(base, 0, 0);
        assert(!shiftedThinTriangle.containsStrictly(roundedCentroid));

        const std::vector<PlanePoint3i> primaryCandidates =
            ember::detail::enumerateLeafClassificationPrimaryPointCandidatesUnchecked(shiftedThinTriangle);
        assert(primaryCandidates.empty());

        const std::vector<PlanePoint3i> candidates =
            ember::enumerateLeafClassificationPointCandidates(shiftedThinTriangle);
        assert(!candidates.empty());
        for (const PlanePoint3i &candidate : candidates)
        {
            assert(shiftedThinTriangle.containsStrictly(candidate));
        }
    }

    {
        ember::AABB3i box;
        box.xMin = 0;
        box.xMax = 10;
        box.yMin = 0;
        box.yMax = 10;
        box.zMin = 0;
        box.zMax = 10;
        box.valid = true;

        const PlanePoint3i reference = ember::makeIntegerPoint(1, 1, 1);
        const PlanePoint3i target(
            Plane3i(1, -100, -100, 1791),
            Plane3i(0, 1, 0, -9),
            Plane3i(0, 0, 1, -9));

        std::vector<ember::Segment256> path;
        assert(!ember::detail::buildPlaneReplacementPath(reference, target, box, {0, 1, 2}, path));
        assert(ember::detail::buildPlaneReplacementPath(reference, target, box, {1, 2, 0}, path));
        assert(!path.empty());
        assert(ember::areSamePlanePoint(path.front().getStartPoint(), reference));
        assert(ember::areSamePlanePoint(path.back().getEndPoint(), target));
        for (std::size_t i = 0; i < path.size(); ++i)
        {
            assert(path[i].isValid());
            assert(ember::isPointInsideOrOnAABB(path[i].getStartPoint(), box));
            assert(ember::isPointInsideOrOnAABB(path[i].getEndPoint(), box));
            if (i != 0)
            {
                assert(ember::areSamePlanePoint(path[i - 1].getEndPoint(), path[i].getStartPoint()));
            }
        }
    }

    {
        ember::AABB3i box;
        box.xMin = 0;
        box.xMax = 10;
        box.yMin = 0;
        box.yMax = 10;
        box.zMin = 0;
        box.zMax = 10;
        box.valid = true;

        const PlanePoint3i reference = ember::makeIntegerPoint(1, 1, 1);
        const PlanePoint3i target(
            Plane3i(0, 2, 0, -9),
            Plane3i(0, 0, 2, -9),
            Plane3i(2, 0, 0, -9));
        const std::vector<PlanePoint3i> targetPoints{target};

        const std::vector<ember::LeafClassificationPathCandidate> fastCandidates =
            ember::enumerateLeafClassificationFastPathCandidatesFromPoints(reference, targetPoints, box);
        assert(fastCandidates.empty());

        std::size_t visitedFallbackCandidates = 0;
        const std::size_t emittedFallbackCandidates =
            ember::enumerateLeafClassificationFallbackPathCandidatesFromPoints(
                reference,
                targetPoints,
                box,
                [&](ember::LeafClassificationPathCandidate candidate)
                {
                    ++visitedFallbackCandidates;
                    assert(!candidate.path.empty());
                    assert(ember::areSamePlanePoint(candidate.path.front().getStartPoint(), reference));
                    assert(ember::areSamePlanePoint(candidate.path.back().getEndPoint(), target));
                    return false;
                });

        assert(emittedFallbackCandidates == 1u);
        assert(visitedFallbackCandidates == 1u);
    }

    {
        ember::AABB3i box;
        box.xMin = 0;
        box.xMax = 4;
        box.yMin = 0;
        box.yMax = 4;
        box.zMin = 0;
        box.zMax = 4;
        box.valid = true;

        const PlanePoint3i start = ember::makeIntegerPoint(-1, 1, 1);
        const std::vector<ember::AABBPathCandidate> candidates =
            ember::enumerateAABBPathCandidates(start, box);
        assert(!candidates.empty());

        Integer x;
        Integer y;
        Integer z;
        assert(ember::detail::tryExtractExactIntegerPoint(candidates.front().targetPoint, x, y, z));
        assert(x > box.xMin && x < box.xMax);
        assert(y > box.yMin && y < box.yMax);
        assert(z > box.zMin && z < box.zMax);
    }

    {
        Polygon256 crossingSurface = makeFaceYZ(0, -1, 1, -1, 1, 1);
        crossingSurface.WNTV = {1, 0};
        const std::vector<Polygon256> polygons{crossingSurface};

        const PlanePoint3i refPoint = ember::makeIntegerPoint(-1, 0, 0);
        const ember::refPoint reference(refPoint, ember::WNV{7, 3});

        ember::WNV targetWNV;
        assert(ember::tracePathWNV(reference, ember::Path{}, polygons, targetWNV) == ember::SUCCESS);
        assert(targetWNV == reference.wnv);

        ember::WNV frontWNV;
        ember::WNV backWNV;
        assert(ember::tracePathWNVToSurfacePoint(
                   reference,
                   ember::Path{},
                   polygons,
                   crossingSurface.plane,
                   frontWNV,
                   backWNV) == ember::INPUT_INVALID);
    }

    {
        Polygon256 crossingSurface = makeFaceYZ(0, -1, 1, -1, 1, 1);
        crossingSurface.WNTV = {1, 0};
        const std::vector<Polygon256> polygons{crossingSurface};

        const PlanePoint3i left = ember::makeIntegerPoint(-1, 0, 0);
        const PlanePoint3i right = ember::makeIntegerPoint(1, 0, 0);
        const ember::Path oneCrossing = makeAxisPath({left, right});

        ember::WNV targetWNV;
        assert(ember::tracePathWNV(
                   ember::refPoint(left, ember::WNV{0, 0}),
                   oneCrossing,
                   polygons,
                   targetWNV) == ember::SUCCESS);
        assert(targetWNV == ember::WNV({-1, 0}));

        const ember::Path crossingAndReturn = makeAxisPath({left, right, left});
        assert(ember::tracePathWNV(
                   ember::refPoint(left, ember::WNV{0, 0}),
                   crossingAndReturn,
                   polygons,
                   targetWNV) == ember::SUCCESS);
        assert(targetWNV == ember::WNV({0, 0}));
    }

    {
        Polygon256 crossingSurface = makeFaceYZ(0, -1, 1, -1, 1, 1);
        crossingSurface.WNTV = {1, 0};
        const std::vector<Polygon256> polygons{crossingSurface};

        const PlanePoint3i left = ember::makeIntegerPoint(-1, 0, 0);
        const PlanePoint3i right = ember::makeIntegerPoint(1, 0, 0);
        const ember::Path crossingPath = makeAxisPath({left, right});

        ember::WNV targetWNV;
        Polygon256 dimensionMismatch = crossingSurface;
        dimensionMismatch.WNTV = {1};
        assert(ember::tracePathWNV(
                   ember::refPoint(left, ember::WNV{0, 0}),
                   crossingPath,
                   {dimensionMismatch},
                   targetWNV) == ember::INPUT_INVALID);

        assert(ember::tracePathWNV(
                   ember::refPoint(ember::makeIntegerPoint(-2, 0, 0), ember::WNV{0, 0}),
                   crossingPath,
                   polygons,
                   targetWNV) == ember::INPUT_INVALID);

        const ember::Path discontinuousPath = {
            makeAxisSegment(ember::makeIntegerPoint(-2, 0, 0), ember::makeIntegerPoint(-1, 0, 0)),
            makeAxisSegment(ember::makeIntegerPoint(0, 0, 0), ember::makeIntegerPoint(1, 0, 0))};
        assert(ember::tracePathWNV(
                   ember::refPoint(ember::makeIntegerPoint(-2, 0, 0), ember::WNV{0, 0}),
                   discontinuousPath,
                   polygons,
                   targetWNV) == ember::INPUT_INVALID);
    }

    {
        Polygon256 crossingSurface = makeFaceYZ(0, -1, 1, -1, 1, 1);
        crossingSurface.WNTV = {1, 0};
        const std::vector<Polygon256> polygons{crossingSurface};

        ember::WNV targetWNV;
        const PlanePoint3i left = ember::makeIntegerPoint(-1, 0, 0);
        const PlanePoint3i surfaceInterior = ember::makeIntegerPoint(0, 0, 0);
        assert(ember::tracePathWNV(
                   ember::refPoint(left, ember::WNV{0, 0}),
                   makeAxisPath({left, surfaceInterior}),
                   polygons,
                   targetWNV) == ember::PATH_INVALID);

        assert(ember::tracePathWNV(
                   ember::refPoint(ember::makeIntegerPoint(0, -2, 0), ember::WNV{0, 0}),
                   makeAxisPath({ember::makeIntegerPoint(0, -2, 0), ember::makeIntegerPoint(0, 2, 0)}),
                   polygons,
                   targetWNV) == ember::PATH_INVALID);

        assert(ember::tracePathWNV(
                   ember::refPoint(ember::makeIntegerPoint(-1, 1, 0), ember::WNV{0, 0}),
                   makeAxisPath({ember::makeIntegerPoint(-1, 1, 0), ember::makeIntegerPoint(1, 1, 0)}),
                   polygons,
                   targetWNV) == ember::PATH_INVALID);
    }

    {
        Polygon256 crossingSurface = makeFaceYZ(0, -1, 1, -1, 1, 1);
        crossingSurface.WNTV = {1, 0};
        const std::vector<Polygon256> polygons{crossingSurface};

        const PlanePoint3i left = ember::makeIntegerPoint(-1, 0, 0);
        const PlanePoint3i surfaceInterior = ember::makeIntegerPoint(0, 0, 0);
        ember::WNV frontWNV;
        ember::WNV backWNV;
        assert(ember::tracePathWNVToSurfacePoint(
                   ember::refPoint(left, ember::WNV{0, 0}),
                   makeAxisPath({left, surfaceInterior}),
                   polygons,
                   crossingSurface.plane,
                   frontWNV,
                   backWNV) == ember::SUCCESS);
        assert(frontWNV == ember::WNV({-1, 0}));
        assert(backWNV == ember::WNV({0, 0}));
    }

    {
        const Polygon256 clippedSurface = makeSubdivisionClippedFaceYZ();
        const std::vector<Polygon256> polygons{clippedSurface};
        ember::WNV targetWNV;

        const PlanePoint3i artificialLeft = ember::makeIntegerPoint(-1, 2, 1);
        const PlanePoint3i artificialRight = ember::makeIntegerPoint(1, 2, 1);
        const ember::Path artificialCrossing = makeAxisPath({artificialLeft, artificialRight});
        const ember::detail::PolygonBoundaryContact artificialContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(artificialCrossing.front(), clippedSurface);
        assert(artificialContact.type == ember::detail::PolygonBoundaryContactType::BoundaryPointHit);
        assert(artificialContact.edgeIndices.size() == 1u);
        assert(clippedSurface.edgeProvenance(artificialContact.edgeIndices.front()) == ember::PolygonEdgeProvenance::SubdivisionClip);
        assert(ember::tracePathWNV(
                   ember::refPoint(artificialLeft, ember::WNV{0, 0}),
                   artificialCrossing,
                   polygons,
                   targetWNV) == ember::PATH_INVALID);
        assert(ember::detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                   ember::refPoint(artificialLeft, ember::WNV{0, 0}),
                   artificialCrossing,
                   polygons,
                   targetWNV) == ember::SUCCESS);
        assert(targetWNV == ember::WNV({-1, 0}));

        const PlanePoint3i boundaryEndpoint = ember::makeIntegerPoint(0, 2, 1);
        const ember::Path endpointOnArtificialEdge = makeAxisPath({artificialLeft, boundaryEndpoint});
        const ember::detail::PolygonBoundaryContact endpointContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(
                endpointOnArtificialEdge.front(),
                clippedSurface);
        assert(endpointContact.type == ember::detail::PolygonBoundaryContactType::EndpointOnBoundary);
        assert(ember::detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                   ember::refPoint(artificialLeft, ember::WNV{0, 0}),
                   endpointOnArtificialEdge,
                   polygons,
                   targetWNV) == ember::PATH_INVALID);

        const PlanePoint3i mixedLeft = ember::makeIntegerPoint(-1, 2, 0);
        const PlanePoint3i mixedRight = ember::makeIntegerPoint(1, 2, 0);
        const ember::Path mixedVertexCrossing = makeAxisPath({mixedLeft, mixedRight});
        const ember::detail::PolygonBoundaryContact mixedContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(mixedVertexCrossing.front(), clippedSurface);
        assert(mixedContact.type == ember::detail::PolygonBoundaryContactType::BoundaryPointHit);
        assert(mixedContact.edgeIndices.size() == 2u);
        assert(ember::detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                   ember::refPoint(mixedLeft, ember::WNV{0, 0}),
                   mixedVertexCrossing,
                   polygons,
                   targetWNV) == ember::PATH_INVALID);

        const PlanePoint3i regularLeft = ember::makeIntegerPoint(-1, 0, 1);
        const PlanePoint3i regularRight = ember::makeIntegerPoint(1, 0, 1);
        const ember::Path regularCrossing = makeAxisPath({regularLeft, regularRight});
        const ember::detail::PolygonBoundaryContact regularContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(regularCrossing.front(), clippedSurface);
        assert(regularContact.type == ember::detail::PolygonBoundaryContactType::BoundaryPointHit);
        assert(!regularContact.edgeIndices.empty());
        assert(clippedSurface.edgeProvenance(regularContact.edgeIndices.front()) == ember::PolygonEdgeProvenance::Regular);
        assert(ember::detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                   ember::refPoint(regularLeft, ember::WNV{0, 0}),
                   regularCrossing,
                   polygons,
                   targetWNV) == ember::PATH_INVALID);

        const ember::Path edgeOverlapPath = makeAxisPath(
            {ember::makeIntegerPoint(0, 2, -1), ember::makeIntegerPoint(0, 2, 3)});
        const ember::detail::PolygonBoundaryContact overlapContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(edgeOverlapPath.front(), clippedSurface);
        assert(overlapContact.type == ember::detail::PolygonBoundaryContactType::EdgeOverlap);
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands(lhs, rhs);

        problem.solve();

        const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();

        assert(problem.isSolved());
        assert(!problem.isDiscarded());
        assert(problem.resultFragments().size() == 12u);
        assert(!leaves.empty());
        for (const ember::BoolLeafSummary &leaf : leaves)
        {
            assert(!leaf.discarded);
            assert(leaf.polygonCount <= 2u || !ember::hasSplittableAxis(leaf.aabb));
        }
        for (const Polygon256 &fragment : problem.resultFragments())
        {
            assertResultFragmentIsGeometryOnly(fragment);
        }
    }

    {
        const Polygon256 invalidPolygon = makeInvalidInwardSquareXY();
        assert(!invalidPolygon.isValid());

        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setPolygons({invalidPolygon});

        assert(throwsRuntimeError(
            [&problem]()
            {
                problem.solve();
            },
            "invalid"));
        assert(!problem.isSolved());
        assert(problem.resultFragments().empty());
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands(lhs, rhs);
        problem.solve();
        assert(problem.isSolved());
        assert(!problem.resultFragments().empty());

        const Polygon256 invalidPolygon = makeInvalidInwardSquareXY();
        problem.setPolygons({invalidPolygon});
        assert(throwsRuntimeError(
            [&problem]()
            {
                problem.solve();
            },
            "invalid"));
        assert(!problem.isSolved());
        assert(problem.resultFragments().empty());
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Intersection);
        problem.setOperands(lhs, rhs);
        problem.solve();

        assert(problem.isSolved());
        assert(problem.resultFragments().empty());
    }

    {
        Polygon256 rhsOnlySurface = makeFaceYZ(0, -1, 1, -1, 1, 1);
        rhsOnlySurface.WNTV = {0, 1};

        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Difference);
        problem.setPolygons({rhsOnlySurface});
        problem.solve();

        assert(problem.isSolved());
        assert(problem.isDiscarded());
        assert(problem.resultFragments().empty());
        assert(problem.leafSummaries().empty());
    }

    {
        const std::vector<Polygon256> tallLhs = makeAxisAlignedBox(0, 0, 0, 1, 100, 1);
        const std::vector<Polygon256> tallRhs = makeAxisAlignedBox(3, 0, 0, 4, 100, 1);
        const ember::BoolOperandAssumptions exactOperand{true, true};

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Union);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve();

            const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();
            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 12u);
            assert(leaves.size() == 2u);
            for (const ember::BoolLeafSummary &leaf : leaves)
            {
                assert(leaf.polygonCount == 6u);
                assert(leaf.aabb.xMax <= Integer(2) || leaf.aabb.xMin >= Integer(2));
            }
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Intersection);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(problem.isDiscarded());
            assert(problem.resultFragments().empty());
            assert(problem.leafSummaries().empty());
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Difference);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve();

            const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();
            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
            assert(leaves.size() == 1u);
            assert(leaves.front().polygonCount == 6u);
            assert(leaves.front().aabb.xMax <= Integer(2));
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Union);
            problem.setOperandAssumptions(exactOperand, exactOperand);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 12u);
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Intersection);
            problem.setOperandAssumptions(exactOperand, exactOperand);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(problem.isDiscarded());
            assert(problem.resultFragments().empty());
            assert(problem.leafSummaries().empty());
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Difference);
            problem.setOperandAssumptions(exactOperand, exactOperand);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
        }
    }

    {
        const std::vector<Polygon256> singleLhs = makeAxisAlignedBox(0, 0, 0, 1, 1, 1);
        const std::vector<Polygon256> emptyRhs;

        {
            ember::BoolProblem problem(6);
            ember::BoolOperandAssumptions lhsAssumptions;
            lhsAssumptions.noSelfIntersections = true;
            problem.setOperation(BoolOp::Union);
            problem.setOperandAssumptions(lhsAssumptions, ember::BoolOperandAssumptions{});
            problem.setOperands(singleLhs, emptyRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
        }

        {
            ember::BoolProblem problem(6);
            ember::BoolOperandAssumptions lhsAssumptions;
            lhsAssumptions.noSelfIntersections = true;
            lhsAssumptions.noNestedComponents = true;
            problem.setOperation(BoolOp::Union);
            problem.setOperandAssumptions(lhsAssumptions, ember::BoolOperandAssumptions{});
            problem.setOperands(singleLhs, emptyRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
        }

        {
            ember::BoolProblem problem(6);
            ember::BoolOperandAssumptions lhsAssumptions;
            lhsAssumptions.noSelfIntersections = true;
            lhsAssumptions.noNestedComponents = true;
            problem.setOperation(BoolOp::Difference);
            problem.setOperandAssumptions(lhsAssumptions, ember::BoolOperandAssumptions{});
            problem.setOperands(singleLhs, emptyRhs);
            problem.solve();

            assert(problem.isSolved());
            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
        }
    }

    {
        std::vector<Polygon256> customWntvBox = makeAxisAlignedBox(0, 0, 0, 1, 1, 1);
        assignWNTV(customWntvBox, ember::WNV{1, 1});

        ember::BoolProblem baseline(6);
        baseline.setOperation(BoolOp::Union);
        baseline.setPolygons(customWntvBox);
        baseline.solve();

        ember::BoolProblem assumed(6);
        assumed.setOperation(BoolOp::Union);
        assumed.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        assumed.setPolygons(customWntvBox);
        assumed.solve();

        assert(baseline.isSolved());
        assert(assumed.isSolved());
        assert(baseline.resultFragments().size() == assumed.resultFragments().size());
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Difference);
        problem.setOperands(lhs, rhs);
        problem.solve();

        assert(problem.isSolved());
        assert(problem.resultFragments().size() == 6u);
    }
}

