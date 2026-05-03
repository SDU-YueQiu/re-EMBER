#include "bool_problem_tests.h"

#include "core/bool_problem.h"

#include <cassert>
#include <stdexcept>

namespace
{
    using ember::BoolOp;
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

    Polygon256 makeThinTriangleXY()
    {
        return Polygon256(
            Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, 0, 1)),
            std::vector<Plane3i>{
                Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(0, -1, 0)),
                Plane3i(1, 100, 0, -100),
                Plane3i::fromPointNormal(Vec3i(0, 0, 0), Vec3i(-1, 0, 0))});
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
        assert(ember::detail::buildPlaneReplacementPath(reference, target, box, {0, 1, 2}, path));
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
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands(lhs, rhs);
        problem.solve();

        std::vector<const ember::BoolProblem *> leaves;
        problem.collectLeafProblems(leaves);

        assert(problem.isSolved());
        assert(!problem.isDiscarded());
        assert(problem.resultFragments().size() == 12u);
        assert(!leaves.empty());
        for (const ember::BoolProblem *leaf : leaves)
        {
            assert(leaf->polygonCount() <= 2u || !ember::hasSplittableAxis(leaf->aabb()));
        }
        for (const Polygon256 &fragment : problem.resultFragments())
        {
            assert(fragment.WNVF.size() == 2u);
            assert(fragment.WNVB.size() == 2u);
        }
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
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Difference);
        problem.setOperands(lhs, rhs);
        problem.solve();

        assert(problem.isSolved());
        assert(problem.resultFragments().size() == 6u);
    }
}

