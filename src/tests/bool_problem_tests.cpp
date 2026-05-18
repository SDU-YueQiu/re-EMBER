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
#include <random>
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
    std::vector<Plane3i> {
        Plane3i::fromPointNormal(Vec3i(xmin, ymin, z), Vec3i(0, -1, 0)),
        Plane3i::fromPointNormal(Vec3i(xmax, ymin, z), Vec3i(1, 0, 0)),
        Plane3i::fromPointNormal(Vec3i(xmin, ymax, z), Vec3i(0, 1, 0)),
        Plane3i::fromPointNormal(Vec3i(xmin, ymin, z), Vec3i(-1, 0, 0))
    });
}

Polygon256 makeFaceYZ(int x, int ymin, int ymax, int zmin, int zmax, int normalX)
{
    return Polygon256(
               Plane3i::fromPointNormal(Vec3i(x, 0, 0), Vec3i(normalX, 0, 0)),
    std::vector<Plane3i> {
        Plane3i::fromPointNormal(Vec3i(x, ymin, zmin), Vec3i(0, -1, 0)),
        Plane3i::fromPointNormal(Vec3i(x, ymin, zmax), Vec3i(0, 0, 1)),
        Plane3i::fromPointNormal(Vec3i(x, ymax, zmin), Vec3i(0, 1, 0)),
        Plane3i::fromPointNormal(Vec3i(x, ymin, zmin), Vec3i(0, 0, -1))
    });
}

Polygon256 makeFaceXZ(int y, int xmin, int xmax, int zmin, int zmax, int normalY)
{
    return Polygon256(
               Plane3i::fromPointNormal(Vec3i(0, y, 0), Vec3i(0, normalY, 0)),
    std::vector<Plane3i> {
        Plane3i::fromPointNormal(Vec3i(xmin, y, zmin), Vec3i(-1, 0, 0)),
        Plane3i::fromPointNormal(Vec3i(xmin, y, zmax), Vec3i(0, 0, 1)),
        Plane3i::fromPointNormal(Vec3i(xmax, y, zmin), Vec3i(1, 0, 0)),
        Plane3i::fromPointNormal(Vec3i(xmin, y, zmin), Vec3i(0, 0, -1))
    });
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

ember::AABB3i makeSceneAABB(int xmin, int ymin, int zmin, int xmax, int ymax, int zmax)
{
    ember::AABB3i box;
    box.xMin = xmin;
    box.xMax = xmax;
    box.yMin = ymin;
    box.yMax = ymax;
    box.zMin = zmin;
    box.zMax = zmax;
    box.valid = true;
    ember::expandAABB(box, 1);
    return box;
}

void assignWNTV(std::vector<Polygon256> &polygons, const ember::WNV &wntv)
{
    for (Polygon256 &polygon : polygons)
        polygon.WNTV = wntv;
}

std::vector<Polygon256> makeInteriorGridSupportPlanesForBox(int minCoord, int maxCoord)
{
    std::vector<Polygon256> polygons;
    for (int coordinate = minCoord + 1; coordinate < maxCoord; ++coordinate)
    {
        polygons.push_back(makeFaceYZ(coordinate, minCoord, maxCoord, minCoord, maxCoord, 1));
        polygons.push_back(makeFaceXZ(coordinate, minCoord, maxCoord, minCoord, maxCoord, 1));
        polygons.push_back(makeFaceXY(coordinate, minCoord, maxCoord, minCoord, maxCoord, 1));
    }

    assignWNTV(polygons, ember::WNV{1, 0});
    return polygons;
}

bool tryPropagateReferenceViaAABBSeedTiers(
    const PlanePoint3i &startPoint,
    const ember::WNV &startWNV,
    const ember::AABB3i &box,
    const std::vector<Polygon256> &polygons,
    bool useFastTier,
    bool useExhaustiveTier,
    PlanePoint3i &outPoint,
    std::size_t &outVisitedCount,
    std::size_t &outTriedCount)
{
    outPoint = PlanePoint3i();
    outVisitedCount = 0;
    outTriedCount = 0;

    const ember::refPoint reference(startPoint, startWNV);
    std::vector<ember::Segment256> path;
    path.reserve(3);
    bool success = false;
    bool hardFailure = false;
    auto processSeed =
        [&](const ember::detail::AABBPathCandidateSeed &seed)
    {
        ++outVisitedCount;

        bool onSurface = false;
        for (const Polygon256 &polygon : polygons)
        {
            if (seed.targetPoint.classify(polygon.plane) == 0)
            {
                onSurface = true;
                break;
            }
        }
        if (onSurface)
            return true;

        if (!ember::detail::buildAABBPathFromSeed(startPoint, seed, path))
            return true;

        ember::WNV propagatedWNV;
        ++outTriedCount;
        const ember::traceStatus status =
            ember::detail::tracePathWNVAllowSubdivisionClipCrossingTrusted(
                reference,
                path,
                polygons,
                propagatedWNV);
        if (status == ember::SUCCESS)
        {
            outPoint = seed.targetPoint;
            success = true;
            return false;
        }

        if (status != ember::PATH_INVALID)
        {
            hardFailure = true;
            return false;
        }

        return true;
    };

    if (useFastTier)
        ember::detail::visitFastAABBPathCandidateSeeds(startPoint, box, processSeed);
    if (!success && !hardFailure && useExhaustiveTier)
        ember::detail::visitExhaustiveAABBPathCandidateSeeds(startPoint, box, processSeed);

    return success;
}

Polygon256 makeThinTriangleXY()
{
    return Polygon256(
               Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
    std::vector<Plane3i> {
        Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, -1, 0)),
        Plane3i(1, 100, 0, -100),
        Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(-1, 0, 0))
    });
}

Polygon256 makeLargeOffsetThinTriangleXY()
{
    const Integer base = Integer(1) << 75;
    return Polygon256(
               Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
    std::vector<Plane3i> {
        Plane3i(0, -1, 0, 0),
        Plane3i(1, 1, 0, -base - Integer(1)),
        Plane3i(-1, 0, 0, base)
    });
}

Polygon256 makeInvalidInwardSquareXY()
{
    Polygon256 polygon;
    polygon.plane = Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1));
    polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 1, 0)));
    polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(-1, 0, 0)));
    polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, -1, 0)));
    polygon.addEdgePlane(Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(1, 0, 0)));
    polygon.precomputeVertices();
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
        throw std::runtime_error("bool_problem_tests failed to build a subdivision-clipped polygon.");

    return backPolygon;
}

ember::Segment256 makeAxisSegment(const PlanePoint3i &start, const PlanePoint3i &end)
{
    Integer x0;
    Integer y0;
    Integer z0;
    Integer x1;
    Integer y1;
    Integer z1;
    if (!ember::detail::tryExtractExactIntegerPoint(start, x0, y0, z0) ||
            !ember::detail::tryExtractExactIntegerPoint(end, x1, y1, z1))
        throw std::runtime_error("bool_problem_tests received a non-integer axis segment endpoint.");

    ember::Segment256 segment;
    if (!ember::detail::buildAxisAlignedSegment(x0, y0, z0, x1, y1, z1, segment))
        throw std::runtime_error("bool_problem_tests failed to build an axis-aligned segment.");
    if (!ember::areSamePlanePoint(segment.getStartPointRef(), start) ||
            !ember::areSamePlanePoint(segment.getEndPointRef(), end))
        throw std::runtime_error("bool_problem_tests built a segment with unexpected endpoints.");
    return segment;
}

std::vector<ember::AABBPathCandidate> collectAABBPathCandidates(const PlanePoint3i &startPoint, const ember::AABB3i &box)
{
    std::vector<ember::AABBPathCandidate> candidates;
    auto materializeSeed =
        [&](const ember::detail::AABBPathCandidateSeed &seed)
    {
        std::vector<ember::Segment256> path;
        if (ember::detail::buildAABBPathFromSeed(startPoint, seed, path))
        {
            candidates.push_back(ember::AABBPathCandidate{seed.targetPoint, std::move(path)});
        }
        return true;
    };

    ember::detail::visitFastAABBPathCandidateSeeds(startPoint, box, materializeSeed);
    ember::detail::visitExhaustiveAABBPathCandidateSeeds(startPoint, box, materializeSeed);
    return candidates;
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
        path.push_back(makeAxisSegment(pointList[i - 1], pointList[i]));
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
    const ember::AABB3i separatedSceneAABB = makeSceneAABB(0, 0, 0, 4, 4, 4);

    {
        const Polygon256 square = makeFaceXY(0, 0, 2, 0, 2, 1);
        PlanePoint3i centroidPoint;
        assert(ember::detail::buildLeafClassificationCentroidTargetPoint(square, centroidPoint));
        assert(square.containsStrictly(centroidPoint));
        assert(ember::areSamePlanePoint(centroidPoint, ember::makeIntegerPoint(1, 1, 0)));
    }

    {
        const Polygon256 thinTriangle = makeThinTriangleXY();
        assert(thinTriangle.isValid());

        PlanePoint3i centroidPoint;
        assert(!ember::detail::buildLeafClassificationCentroidTargetPoint(thinTriangle, centroidPoint));

        std::mt19937 rng(42u);
        const ember::detail::LeafClassificationInsetPointSequence insetPoints =
            ember::detail::enumerateLeafClassificationInsetPointCandidates(thinTriangle, rng);
        assert(insetPoints.attemptCount == 9u);
        assert(!insetPoints.candidates.empty());
        for (const PlanePoint3i &candidate : insetPoints.candidates)
            assert(thinTriangle.containsStrictly(candidate));
    }

    {
        const Polygon256 shiftedThinTriangle = makeLargeOffsetThinTriangleXY();
        assert(shiftedThinTriangle.isValid());

        const Integer base = Integer(1) << 75;
        const PlanePoint3i roundedCentroid = ember::makeIntegerPoint(base, 0, 0);
        assert(!shiftedThinTriangle.containsStrictly(roundedCentroid));

        PlanePoint3i centroidPoint;
        assert(!ember::detail::buildLeafClassificationCentroidTargetPoint(shiftedThinTriangle, centroidPoint));

        std::mt19937 rngA(42u);
        std::mt19937 rngB(42u);
        const ember::detail::LeafClassificationInsetPointSequence insetPointsA =
            ember::detail::enumerateLeafClassificationInsetPointCandidates(shiftedThinTriangle, rngA);
        const ember::detail::LeafClassificationInsetPointSequence insetPointsB =
            ember::detail::enumerateLeafClassificationInsetPointCandidates(shiftedThinTriangle, rngB);
        assert(insetPointsA.attemptCount == 9u);
        assert(insetPointsA.attemptCount == insetPointsB.attemptCount);
        assert(!insetPointsA.candidates.empty());
        assert(insetPointsA.candidates.size() == insetPointsB.candidates.size());
        for (std::size_t i = 0; i < insetPointsA.candidates.size(); ++i)
        {
            assert(shiftedThinTriangle.containsStrictly(insetPointsA.candidates[i]));
            assert(shiftedThinTriangle.containsStrictly(insetPointsB.candidates[i]));
            assert(ember::areSamePlanePoint(insetPointsA.candidates[i], insetPointsB.candidates[i]));
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
        assert(ember::areSamePlanePoint(path.front().getStartPointRef(), reference));
        assert(ember::areSamePlanePoint(path.back().getEndPointRef(), target));
        for (std::size_t i = 0; i < path.size(); ++i)
        {
            assert(path[i].isValid());
            assert(ember::isPointInsideOrOnAABB(path[i].getStartPointRef(), box));
            assert(ember::isPointInsideOrOnAABB(path[i].getEndPointRef(), box));
            if (i != 0)
                assert(ember::areSamePlanePoint(path[i - 1].getEndPointRef(), path[i].getStartPointRef()));
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
        const PlanePoint3i target = ember::makeIntegerPoint(9, 9, 9);
        const std::vector<PlanePoint3i> targetPoints{target};

        const std::vector<ember::LeafClassificationPathCandidate> axisCandidates =
            ember::enumerateLeafClassificationAxisPathCandidatesFromPoints(reference, targetPoints, box);
        assert(axisCandidates.size() == 1u);
        for (const ember::LeafClassificationPathCandidate &candidate : axisCandidates)
        {
            assert(!candidate.path.empty());
            assert(ember::areSamePlanePoint(candidate.path.front().getStartPointRef(), reference));
            assert(ember::areSamePlanePoint(candidate.path.back().getEndPointRef(), target));
            for (std::size_t i = 0; i < candidate.path.size(); ++i)
            {
                assert(candidate.path[i].isValid());
                assert(ember::isPointInsideOrOnAABB(candidate.path[i].getStartPointRef(), box));
                assert(ember::isPointInsideOrOnAABB(candidate.path[i].getEndPointRef(), box));
                if (i != 0)
                    assert(ember::areSamePlanePoint(candidate.path[i - 1].getEndPointRef(), candidate.path[i].getStartPointRef()));
            }
        }

        const std::vector<ember::LeafClassificationPathCandidate> planeReplacementCandidates =
            ember::enumerateLeafClassificationPlaneReplacementPathCandidatesFromPoints(reference, targetPoints, box);
        assert(planeReplacementCandidates.size() == 6u);
        for (const ember::LeafClassificationPathCandidate &candidate : planeReplacementCandidates)
        {
            assert(!candidate.path.empty());
            assert(ember::areSamePlanePoint(candidate.path.front().getStartPointRef(), reference));
            assert(ember::areSamePlanePoint(candidate.path.back().getEndPointRef(), target));
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

        const std::vector<ember::LeafClassificationPathCandidate> axisCandidates =
            ember::enumerateLeafClassificationAxisPathCandidatesFromPoints(reference, targetPoints, box);
        assert(axisCandidates.empty());

        const std::vector<ember::LeafClassificationPathCandidate> planeReplacementCandidates =
            ember::enumerateLeafClassificationPlaneReplacementPathCandidatesFromPoints(reference, targetPoints, box);
        assert(planeReplacementCandidates.empty());

        std::size_t visitedExhaustiveCandidates = 0;
        const std::size_t emittedExhaustiveCandidates =
            ember::enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints(
                reference,
                targetPoints,
                box,
                [&](ember::LeafClassificationPathCandidate candidate)
        {
            ++visitedExhaustiveCandidates;
            assert(!candidate.path.empty());
            assert(ember::areSamePlanePoint(candidate.path.front().getStartPointRef(), reference));
            assert(ember::areSamePlanePoint(candidate.path.back().getEndPointRef(), target));
            return true;
        });
        assert(emittedExhaustiveCandidates == visitedExhaustiveCandidates);
        assert(emittedExhaustiveCandidates > 0u);
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
        const std::vector<ember::AABBPathCandidate> candidates = collectAABBPathCandidates(start, box);
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
        ember::AABB3i box;
        box.xMin = 0;
        box.xMax = 4;
        box.yMin = 0;
        box.yMax = 4;
        box.zMin = 0;
        box.zMax = 4;
        box.valid = true;

        const PlanePoint3i start = ember::makeIntegerPoint(0, 0, 0);
        const std::vector<ember::AABBPathCandidate> candidates = collectAABBPathCandidates(start, box);
        assert(!candidates.empty());
        assert(candidates.front().path.empty());
        assert(ember::areSamePlanePoint(candidates.front().targetPoint, start));

        std::vector<Polygon256> distantSurface = {makeFaceYZ(3, 0, 4, 0, 4, 1)};
        assignWNTV(distantSurface, ember::WNV{1, 0});

        PlanePoint3i propagatedPoint;
        std::size_t visitedSeeds = 0;
        std::size_t triedSeeds = 0;
        assert(tryPropagateReferenceViaAABBSeedTiers(
                   start,
                   ember::WNV{5, 7},
                   box,
                   distantSurface,
                   true,
                   false,
                   propagatedPoint,
                   visitedSeeds,
                   triedSeeds));
        assert(visitedSeeds == 1u);
        assert(triedSeeds == 1u);
        assert(ember::areSamePlanePoint(propagatedPoint, start));
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

        std::vector<Polygon256> distantSurface = {makeFaceYZ(3, 0, 4, 0, 4, 1)};
        assignWNTV(distantSurface, ember::WNV{1, 0});

        PlanePoint3i propagatedPoint;
        std::size_t visitedSeeds = 0;
        std::size_t triedSeeds = 0;
        assert(tryPropagateReferenceViaAABBSeedTiers(
                   ember::makeIntegerPoint(-1, 1, 1),
                   ember::WNV{0, 0},
                   box,
                   distantSurface,
                   true,
                   false,
                   propagatedPoint,
                   visitedSeeds,
                   triedSeeds));
        assert(visitedSeeds == 1u);
        assert(triedSeeds == 1u);
        assert(ember::areSamePlanePoint(propagatedPoint, ember::makeIntegerPoint(1, 1, 1)));
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

        const std::vector<Polygon256> blockingSurfaces = makeInteriorGridSupportPlanesForBox(0, 4);
        PlanePoint3i propagatedPoint;
        std::size_t fastVisitedSeeds = 0;
        std::size_t fastTriedSeeds = 0;
        assert(!tryPropagateReferenceViaAABBSeedTiers(
                   ember::makeIntegerPoint(-1, 0, 0),
                   ember::WNV{0, 0},
                   box,
                   blockingSurfaces,
                   true,
                   false,
                   propagatedPoint,
                   fastVisitedSeeds,
                   fastTriedSeeds));
        assert(fastVisitedSeeds > 0u);
        assert(fastTriedSeeds == 0u);

        std::size_t allVisitedSeeds = 0;
        std::size_t allTriedSeeds = 0;
        assert(tryPropagateReferenceViaAABBSeedTiers(
                   ember::makeIntegerPoint(-1, 0, 0),
                   ember::WNV{0, 0},
                   box,
                   blockingSurfaces,
                   true,
                   true,
                   propagatedPoint,
                   allVisitedSeeds,
                   allTriedSeeds));
        assert(allVisitedSeeds > fastVisitedSeeds);
        assert(allTriedSeeds == 1u);
        assert(ember::areSamePlanePoint(propagatedPoint, ember::makeIntegerPoint(0, 0, 0)));
    }

    {
        ember::AABB3i leftBox;
        leftBox.xMin = 0;
        leftBox.xMax = 2;
        leftBox.yMin = 0;
        leftBox.yMax = 2;
        leftBox.zMin = 0;
        leftBox.zMax = 2;
        leftBox.valid = true;

        ember::AABB3i touchingBox = leftBox;
        touchingBox.xMin = 2;
        touchingBox.xMax = 4;
        assert(ember::doAABBsOverlap(leftBox, touchingBox));

        ember::AABB3i separatedBox = leftBox;
        separatedBox.xMin = 3;
        separatedBox.xMax = 4;
        assert(!ember::doAABBsOverlap(leftBox, separatedBox));

        assert(ember::doesPlaneIntersectAABB(
                   Plane3i::fromPointNormal(Vec3i(2, 0, 0), Vec3i(1, 0, 0)),
                   leftBox));
        assert(!ember::doesPlaneIntersectAABB(
                   Plane3i::fromPointNormal(Vec3i(3, 0, 0), Vec3i(1, 0, 0)),
                   leftBox));

        const Polygon256 square = makeFaceXY(0, 0, 2, 0, 2, 1);
        const Polygon256 farFace = makeFaceYZ(10, 0, 2, 0, 2, 1);
        Plane3i splitPlane;
        Plane3i v0;
        Plane3i v1;
        assert(!ember::computePolygonIntersectionCarrier(square, farFace, splitPlane, v0, v1));
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

        const PlanePoint3i farLeft = ember::makeIntegerPoint(-1, 5, 0);
        const PlanePoint3i farRight = ember::makeIntegerPoint(1, 5, 0);
        const ember::Path missedCrossing = makeAxisPath({farLeft, farRight});

        const ember::detail::PolygonBoundaryContact missContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(
                missedCrossing.front(),
                crossingSurface);
        assert(missContact.type == ember::detail::PolygonBoundaryContactType::None);

        PlanePoint3i hitPoint;
        assert(!ember::intersectionSegmentPolygon(missedCrossing.front(), crossingSurface, hitPoint));

        ember::WNV targetWNV;
        assert(ember::tracePathWNV(
                   ember::refPoint(farLeft, ember::WNV{4, 2}),
                   missedCrossing,
                   polygons,
                   targetWNV) == ember::SUCCESS);
        assert(targetWNV == ember::WNV({4, 2}));
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
            makeAxisSegment(ember::makeIntegerPoint(0, 0, 0), ember::makeIntegerPoint(1, 0, 0))
        };
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
                   targetWNV) == ember::SUCCESS);
        assert(targetWNV == ember::WNV({-1, 0}));
        assert(ember::detail::tracePathWNVTrusted(
                   ember::refPoint(artificialLeft, ember::WNV{0, 0}),
                   artificialCrossing,
                   polygons,
                   targetWNV) == ember::SUCCESS);
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
        Polygon256 clippedSurface = makeFaceYZ(0, 0, 4, 0, 4, 1);
        clippedSurface.WNTV = {1, 0};
        Polygon256 frontPolygon;
        Polygon256 backPolygon;
        assert(ember::detail::clipLeafGeometryByPlaneTrusted(
                   clippedSurface,
                   Plane3i::fromPointNormal(Vec3i(0, 2, 0), Vec3i(0, 1, 0)),
                   frontPolygon,
                   backPolygon,
                   ember::PolygonEdgeProvenance::SubdivisionClip));

        const std::vector<Polygon256> polygons{backPolygon};
        ember::WNV frontWNV;
        ember::WNV backWNV;

        const PlanePoint3i clipStart = ember::makeIntegerPoint(-1, 2, 2);
        const PlanePoint3i clipTurn = ember::makeIntegerPoint(1, 2, 2);
        const PlanePoint3i surfaceTurn = ember::makeIntegerPoint(1, 1, 2);
        const PlanePoint3i surfaceTarget = ember::makeIntegerPoint(0, 1, 2);
        const ember::Path clipSurfacePath = makeAxisPath({clipStart, clipTurn, surfaceTurn, surfaceTarget});
        const ember::detail::PolygonBoundaryContact clipContact =
            ember::detail::classifySegmentPolygonBoundaryContactUnchecked(clipSurfacePath.front(), backPolygon);
        assert(clipContact.type == ember::detail::PolygonBoundaryContactType::BoundaryPointHit);
        assert(clipContact.edgeIndices.size() == 1u);
        assert(backPolygon.edgeProvenance(clipContact.edgeIndices.front()) == ember::PolygonEdgeProvenance::SubdivisionClip);

        assert(ember::tracePathWNVToSurfacePoint(
                   ember::refPoint(clipStart, ember::WNV{0, 0}),
                   clipSurfacePath,
                   polygons,
                   backPolygon.plane,
                   frontWNV,
                   backWNV) == ember::SUCCESS);
        assert(frontWNV == ember::WNV({-1, 0}));
        assert(backWNV == ember::WNV({0, 0}));
        assert(ember::detail::tracePathWNVToSurfacePointTrusted(
                   ember::refPoint(clipStart, ember::WNV{0, 0}),
                   clipSurfacePath,
                   polygons,
                   backPolygon.plane,
                   frontWNV,
                   backWNV) == ember::SUCCESS);
        assert(frontWNV == ember::WNV({-1, 0}));
        assert(backWNV == ember::WNV({0, 0}));
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands(lhs, rhs);

        problem.solve(separatedSceneAABB);

        const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();

        assert(!problem.isDiscarded());
        assert(problem.resultFragments().size() == 12u);
        assert(!leaves.empty());
        for (const ember::BoolLeafSummary &leaf : leaves)
        {
            assert(!leaf.discarded);
            assert(leaf.polygonCount <= 2u || !ember::hasSplittableAxis(leaf.aabb));
        }
        for (const Polygon256 &fragment : problem.resultFragments())
            assertResultFragmentIsGeometryOnly(fragment);
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands(lhs, rhs);
        problem.solve(separatedSceneAABB);

        assert(problem.resultFragments().size() == 12u);
        assert(throwsRuntimeError(
                   [&problem, &separatedSceneAABB]()
        {
            problem.solve(separatedSceneAABB);
        },
        "single-use"));
        assert(problem.resultFragments().size() == 12u);
        assert(throwsRuntimeError(
                   [&problem]()
        {
            problem.setOperation(BoolOp::Difference);
        },
        "single-use"));
        assert(throwsRuntimeError(
                   [&problem]()
        {
            problem.setOperandAssumptions(ember::BoolOperandAssumptions{}, ember::BoolOperandAssumptions{});
        },
        "single-use"));
        assert(throwsRuntimeError(
                   [&problem]()
        {
            problem.setThreadCount(4);
        },
        "single-use"));
        assert(throwsRuntimeError(
                   [&problem, &lhs, &rhs]()
        {
            problem.setOperands(lhs, rhs);
        },
        "single-use"));
    }

    {
        const Polygon256 invalidPolygon = makeInvalidInwardSquareXY();
        assert(!invalidPolygon.isValid());

        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands({invalidPolygon}, {});

        assert(throwsRuntimeError(
                   [&problem, &separatedSceneAABB]()
        {
            problem.solve(separatedSceneAABB);
        },
        "invalid"));
        assert(throwsRuntimeError(
                   [&problem, &separatedSceneAABB]()
        {
            problem.solve(separatedSceneAABB);
        },
        "single-use"));
        assert(problem.resultFragments().empty());
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Intersection);
        problem.setOperands(lhs, rhs);
        problem.solve(separatedSceneAABB);

        assert(problem.resultFragments().empty());
        assert(problem.solveMetrics().constantDiscardCount > 0u);
    }

    {
        const std::vector<Polygon256> rhsOnlySurface{
            makeFaceYZ(0, -1, 1, -1, 1, 1)};

        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Difference);
        problem.setOperands({}, rhsOnlySurface);
        problem.solve(separatedSceneAABB);

        assert(problem.isDiscarded());
        assert(problem.resultFragments().empty());
        assert(problem.leafSummaries().size() == 1u);
    }

    {
        const std::vector<Polygon256> tallLhs = makeAxisAlignedBox(0, 0, 0, 1, 100, 1);
        const std::vector<Polygon256> tallRhs = makeAxisAlignedBox(3, 0, 0, 4, 100, 1);
        const ember::AABB3i tallSceneAABB = makeSceneAABB(0, 0, 0, 4, 100, 1);
        const ember::BoolOperandAssumptions exactOperand{true, true};

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Union);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve(tallSceneAABB);

            const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();
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
            problem.solve(tallSceneAABB);

            assert(problem.isDiscarded());
            assert(problem.resultFragments().empty());
            assert(problem.leafSummaries().empty());
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Difference);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve(tallSceneAABB);

            const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();
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
            problem.solve(tallSceneAABB);

            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 12u);
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Intersection);
            problem.setOperandAssumptions(exactOperand, exactOperand);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve(tallSceneAABB);

            assert(problem.isDiscarded());
            assert(problem.resultFragments().empty());
            assert(problem.leafSummaries().empty());
        }

        {
            ember::BoolProblem problem(6);
            problem.setOperation(BoolOp::Difference);
            problem.setOperandAssumptions(exactOperand, exactOperand);
            problem.setOperands(tallLhs, tallRhs);
            problem.solve(tallSceneAABB);

            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
        }

        {
            ember::BoolProblem serialProblem(2);
            serialProblem.setOperation(BoolOp::Union);
            serialProblem.setThreadCount(1);
            serialProblem.setOperands(lhs, rhs);
            serialProblem.solve(separatedSceneAABB);

            ember::BoolProblem parallelProblem(2);
            parallelProblem.setOperation(BoolOp::Union);
            parallelProblem.setThreadCount(4);
            parallelProblem.setOperands(lhs, rhs);
            parallelProblem.solve(separatedSceneAABB);

            assert(!serialProblem.isDiscarded());
            assert(!parallelProblem.isDiscarded());
            assert(serialProblem.resultFragments().size() == 12u);
            assert(parallelProblem.resultFragments().size() == serialProblem.resultFragments().size());
            for (const Polygon256 &fragment : serialProblem.resultFragments())
                assertResultFragmentIsGeometryOnly(fragment);
            for (const Polygon256 &fragment : parallelProblem.resultFragments())
                assertResultFragmentIsGeometryOnly(fragment);

            const std::vector<ember::BoolLeafSummary> &serialLeaves = serialProblem.leafSummaries();
            const std::vector<ember::BoolLeafSummary> &parallelLeaves = parallelProblem.leafSummaries();
            assert(serialLeaves.size() == parallelLeaves.size());
            for (std::size_t leafIndex = 0; leafIndex < serialLeaves.size(); ++leafIndex)
            {
                assert(serialLeaves[leafIndex].depth == parallelLeaves[leafIndex].depth);
                assert(serialLeaves[leafIndex].polygonCount == parallelLeaves[leafIndex].polygonCount);
                assert(serialLeaves[leafIndex].discarded == parallelLeaves[leafIndex].discarded);
                assert(serialLeaves[leafIndex].aabb.valid == parallelLeaves[leafIndex].aabb.valid);
                assert(serialLeaves[leafIndex].aabb.xMin == parallelLeaves[leafIndex].aabb.xMin);
                assert(serialLeaves[leafIndex].aabb.xMax == parallelLeaves[leafIndex].aabb.xMax);
                assert(serialLeaves[leafIndex].aabb.yMin == parallelLeaves[leafIndex].aabb.yMin);
                assert(serialLeaves[leafIndex].aabb.yMax == parallelLeaves[leafIndex].aabb.yMax);
                assert(serialLeaves[leafIndex].aabb.zMin == parallelLeaves[leafIndex].aabb.zMin);
                assert(serialLeaves[leafIndex].aabb.zMax == parallelLeaves[leafIndex].aabb.zMax);
            }

            const ember::BoolSolveMetrics &serialMetrics = serialProblem.solveMetrics();
            const ember::BoolSolveMetrics &parallelMetrics = parallelProblem.solveMetrics();
            assert(serialMetrics.resultFragmentCount == parallelMetrics.resultFragmentCount);
            assert(serialMetrics.effectiveThreadCount == 1u);
            assert(serialMetrics.parallelSiblingSpawnCount == 0u);
            assert(parallelMetrics.effectiveThreadCount == 4u);
        }
    }

    {
        const std::vector<Polygon256> singleLhs = makeAxisAlignedBox(0, 0, 0, 1, 1, 1);
        const std::vector<Polygon256> emptyRhs;
        const ember::AABB3i singleSceneAABB = makeSceneAABB(0, 0, 0, 1, 1, 1);

        {
            ember::BoolProblem problem(2);
            ember::BoolOperandAssumptions lhsAssumptions;
            lhsAssumptions.noSelfIntersections = true;
            problem.setOperation(BoolOp::Union);
            problem.setOperandAssumptions(lhsAssumptions, ember::BoolOperandAssumptions{});
            problem.setOperands(singleLhs, emptyRhs);
            problem.solve(singleSceneAABB);

            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
            assert(problem.solveMetrics().singleOperandAssumptionStopCount == 0u);
            assert(problem.solveMetrics().singleOperandAssumptionFallbackCount == 0u);
            assert(problem.solveMetrics().singleOperandLeafBspSkipCount != 0u);
            assert(problem.solveMetrics().singleOperandLeafBspSkipCount ==
                   problem.solveMetrics().leafNodeCount);
            assert(problem.solveMetrics().singleOperandClassificationReuseCount == 0u);
            assert(problem.solveMetrics().leafBspBuildCount == 0u);
        }

        {
            ember::BoolProblem problem(2);
            ember::BoolOperandAssumptions lhsAssumptions;
            lhsAssumptions.noSelfIntersections = true;
            lhsAssumptions.noNestedComponents = true;
            problem.setOperation(BoolOp::Union);
            problem.setOperandAssumptions(lhsAssumptions, ember::BoolOperandAssumptions{});
            problem.setOperands(singleLhs, emptyRhs);
            problem.solve(singleSceneAABB);

            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
            assert(problem.solveMetrics().nodeCount == 1u);
            assert(problem.solveMetrics().leafNodeCount == 1u);
            assert(problem.solveMetrics().maxDepth == 0u);
            assert(problem.solveMetrics().singleOperandAssumptionStopCount == 1u);
            assert(problem.solveMetrics().singleOperandAssumptionFallbackCount == 0u);
            assert(problem.solveMetrics().singleOperandLeafBspSkipCount == 1u);
            assert(problem.solveMetrics().singleOperandClassificationReuseCount == 5u);
            assert(problem.solveMetrics().leafBspBuildCount == 0u);
        }

        {
            ember::BoolProblem problem(2);
            ember::BoolOperandAssumptions lhsAssumptions;
            lhsAssumptions.noSelfIntersections = true;
            lhsAssumptions.noNestedComponents = true;
            problem.setOperation(BoolOp::Difference);
            problem.setOperandAssumptions(lhsAssumptions, ember::BoolOperandAssumptions{});
            problem.setOperands(singleLhs, emptyRhs);
            problem.solve(singleSceneAABB);

            assert(!problem.isDiscarded());
            assert(problem.resultFragments().size() == 6u);
            assert(problem.solveMetrics().nodeCount == 1u);
            assert(problem.solveMetrics().leafNodeCount == 1u);
            assert(problem.solveMetrics().maxDepth == 0u);
            assert(problem.solveMetrics().singleOperandAssumptionStopCount == 1u);
            assert(problem.solveMetrics().singleOperandAssumptionFallbackCount == 0u);
            assert(problem.solveMetrics().singleOperandLeafBspSkipCount == 1u);
            assert(problem.solveMetrics().singleOperandClassificationReuseCount == 5u);
            assert(problem.solveMetrics().leafBspBuildCount == 0u);
        }
    }

    {
        std::vector<Polygon256> dirtyTaggedLhs = lhs;
        std::vector<Polygon256> dirtyTaggedRhs = rhs;
        assignWNTV(dirtyTaggedLhs, ember::WNV{1, 1});
        assignWNTV(dirtyTaggedRhs, ember::WNV{-3, 7});

        ember::BoolProblem clean(2);
        clean.setOperation(BoolOp::Union);
        clean.setOperands(lhs, rhs);
        clean.solve(separatedSceneAABB);

        ember::BoolProblem retagged(2);
        retagged.setOperation(BoolOp::Union);
        retagged.setOperands(dirtyTaggedLhs, dirtyTaggedRhs);
        retagged.solve(separatedSceneAABB);

        assert(clean.resultFragments().size() == retagged.resultFragments().size());
        assert(clean.solveMetrics().resultFragmentCount == retagged.solveMetrics().resultFragmentCount);
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Difference);
        problem.setOperands(lhs, rhs);
        problem.solve(separatedSceneAABB);

        assert(problem.resultFragments().size() == 6u);
    }
}
