#include "bool_problem_tests.h"

#include "core/bool_problem.h"

#include <cassert>
#include <iostream>
#include <stdexcept>

namespace
{
    using ember::BoolOp;
    using ember::Plane3i;
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
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperands(lhs, rhs);
        problem.solve();

        std::vector<const ember::BoolProblem *> leaves;
        problem.collectLeafProblems(leaves);

        std::cout
            << "[BoolTest] union solved=" << problem.isSolved()
            << " leaves=" << leaves.size()
            << " results=" << problem.resultFragments().size()
            << std::endl;

        assert(problem.isSolved());
        assert(!problem.isDiscarded());
        assert(problem.resultFragments().size() == 12u);
        assert(!leaves.empty());
        for (const ember::BoolProblem *leaf : leaves)
        {
            assert(leaf->polygonCount() <= 2u);
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

        std::cout
            << "[BoolTest] intersection solved=" << problem.isSolved()
            << " results=" << problem.resultFragments().size()
            << std::endl;

        assert(problem.isSolved());
        assert(problem.resultFragments().empty());
    }

    {
        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Difference);
        problem.setOperands(lhs, rhs);
        problem.solve();

        std::cout
            << "[BoolTest] difference solved=" << problem.isSolved()
            << " results=" << problem.resultFragments().size()
            << std::endl;

        assert(problem.isSolved());
        assert(problem.resultFragments().size() == 6u);
    }
}

