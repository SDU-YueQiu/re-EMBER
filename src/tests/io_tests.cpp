/**
 * @file io_tests.cpp
 * @brief 实现 OBJ I/O 和 CLI 相邻流程的回归测试。
 */
#include "io_tests.h"

#include "core/bool_problem.h"
#include "io/io.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
    using ember::BoolOp;
    using ember::ObjMeshData;
    using ember::ObjVertex;
    using ember::Plane3i;
    using ember::PolygonSoupBuildOptions;
    using ember::Polygon256;
    using ember::Vec3i;

    struct MeshVec3
    {
        double x;
        double y;
        double z;
    };

    std::filesystem::path makeTestPath(const std::string &filename)
    {
        const std::filesystem::path outputRoot = std::filesystem::current_path() / "build" / "test-output";
        std::filesystem::create_directories(outputRoot);
        return outputRoot / filename;
    }

    void writeTextFile(const std::filesystem::path &path, const std::string &contents)
    {
        std::ofstream output(path, std::ios::trunc);
        if (!output)
        {
            throw std::runtime_error("io_tests failed to open file for writing: " + path.string());
        }

        output << contents;
    }

    std::string readTextFile(const std::filesystem::path &path)
    {
        std::ifstream input(path);
        if (!input)
        {
            throw std::runtime_error("io_tests failed to open file for reading: " + path.string());
        }

        std::ostringstream buffer;
        buffer << input.rdbuf();
        return buffer.str();
    }

    std::filesystem::path findRepoPath(const std::filesystem::path &relativePath)
    {
        std::filesystem::path current = std::filesystem::current_path();
        for (int depth = 0; depth < 8; ++depth)
        {
            const std::filesystem::path candidate = current / relativePath;
            if (std::filesystem::exists(candidate))
            {
                return candidate;
            }

            const std::filesystem::path parent = current.parent_path();
            if (parent == current || parent.empty())
            {
                break;
            }
            current = parent;
        }

        throw std::runtime_error("io_tests could not locate repo path: " + relativePath.string());
    }

    ObjMeshData readRepoObjMesh(const std::filesystem::path &relativePath)
    {
        ObjMeshData mesh;
        std::string error;
        const std::filesystem::path path = findRepoPath(relativePath);
        if (!ember::readObjMesh(path.string(), mesh, error))
        {
            throw std::runtime_error("io_tests failed to read OBJ asset: " + error);
        }
        return mesh;
    }

    void assertWellFormedObjMesh(const ObjMeshData &mesh)
    {
        if (mesh.vertices.empty() || mesh.faces.empty())
        {
            throw std::runtime_error("io_tests produced an empty OBJ mesh.");
        }
        for (const ObjVertex &vertex : mesh.vertices)
        {
            if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y) || !std::isfinite(vertex.z))
            {
                throw std::runtime_error("io_tests produced a non-finite OBJ vertex.");
            }
        }
        for (const std::vector<std::size_t> &face : mesh.faces)
        {
            if (face.size() < 3u)
            {
                throw std::runtime_error("io_tests produced an OBJ face with fewer than three vertices.");
            }
            for (const std::size_t vertexIndex : face)
            {
                if (vertexIndex >= mesh.vertices.size())
                {
                    throw std::runtime_error("io_tests produced an OBJ face with an out-of-range vertex index.");
                }
            }
        }
    }

    void assertFaceCountAtLeast(const ObjMeshData &mesh, std::size_t minimum, const std::string &label)
    {
        if (mesh.faces.size() >= minimum)
        {
            return;
        }

        std::ostringstream message;
        message << "io_tests " << label << " produced " << mesh.faces.size()
                << " faces, expected at least " << minimum << ".";
        throw std::runtime_error(message.str());
    }

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

    ObjMeshData makeObjBox(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
    {
        ObjMeshData box;
        box.vertices = {
            ObjVertex{xmin, ymin, zmin},
            ObjVertex{xmax, ymin, zmin},
            ObjVertex{xmax, ymax, zmin},
            ObjVertex{xmin, ymax, zmin},
            ObjVertex{xmin, ymin, zmax},
            ObjVertex{xmax, ymin, zmax},
            ObjVertex{xmax, ymax, zmax},
            ObjVertex{xmin, ymax, zmax}};
        box.faces = {
            {3, 2, 1, 0},
            {4, 5, 6, 7},
            {0, 1, 5, 4},
            {1, 2, 6, 5},
            {2, 3, 7, 6},
            {3, 0, 4, 7}};
        return box;
    }

    MeshVec3 normalized(MeshVec3 point)
    {
        const double length = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        return MeshVec3{point.x / length, point.y / length, point.z / length};
    }

    std::size_t appendIcosphereVertex(ObjMeshData &mesh, MeshVec3 point)
    {
        const MeshVec3 unit = normalized(point);
        mesh.vertices.push_back(ObjVertex{unit.x, unit.y, unit.z});
        return mesh.vertices.size() - 1u;
    }

    ObjMeshData makeIcosphere80()
    {
        ObjMeshData mesh;
        const double t = (1.0 + std::sqrt(5.0)) / 2.0;
        const std::array<MeshVec3, 12> vertices = {
            MeshVec3{-1.0, t, 0.0},
            MeshVec3{1.0, t, 0.0},
            MeshVec3{-1.0, -t, 0.0},
            MeshVec3{1.0, -t, 0.0},
            MeshVec3{0.0, -1.0, t},
            MeshVec3{0.0, 1.0, t},
            MeshVec3{0.0, -1.0, -t},
            MeshVec3{0.0, 1.0, -t},
            MeshVec3{t, 0.0, -1.0},
            MeshVec3{t, 0.0, 1.0},
            MeshVec3{-t, 0.0, -1.0},
            MeshVec3{-t, 0.0, 1.0}};
        for (const MeshVec3 &vertex : vertices)
        {
            appendIcosphereVertex(mesh, vertex);
        }

        const std::array<std::array<std::size_t, 3>, 20> baseFaces = {
            std::array<std::size_t, 3>{0, 11, 5},
            std::array<std::size_t, 3>{0, 5, 1},
            std::array<std::size_t, 3>{0, 1, 7},
            std::array<std::size_t, 3>{0, 7, 10},
            std::array<std::size_t, 3>{0, 10, 11},
            std::array<std::size_t, 3>{1, 5, 9},
            std::array<std::size_t, 3>{5, 11, 4},
            std::array<std::size_t, 3>{11, 10, 2},
            std::array<std::size_t, 3>{10, 7, 6},
            std::array<std::size_t, 3>{7, 1, 8},
            std::array<std::size_t, 3>{3, 9, 4},
            std::array<std::size_t, 3>{3, 4, 2},
            std::array<std::size_t, 3>{3, 2, 6},
            std::array<std::size_t, 3>{3, 6, 8},
            std::array<std::size_t, 3>{3, 8, 9},
            std::array<std::size_t, 3>{4, 9, 5},
            std::array<std::size_t, 3>{2, 4, 11},
            std::array<std::size_t, 3>{6, 2, 10},
            std::array<std::size_t, 3>{8, 6, 7},
            std::array<std::size_t, 3>{9, 8, 1}};

        std::map<std::pair<std::size_t, std::size_t>, std::size_t> midpointByEdge;
        auto midpointIndex = [&mesh, &midpointByEdge](std::size_t lhs, std::size_t rhs)
        {
            const std::pair<std::size_t, std::size_t> key =
                lhs < rhs ? std::make_pair(lhs, rhs) : std::make_pair(rhs, lhs);
            const auto found = midpointByEdge.find(key);
            if (found != midpointByEdge.end())
            {
                return found->second;
            }

            const ObjVertex &a = mesh.vertices[lhs];
            const ObjVertex &b = mesh.vertices[rhs];
            const std::size_t index = appendIcosphereVertex(
                mesh,
                MeshVec3{
                    (a.x + b.x) * 0.5,
                    (a.y + b.y) * 0.5,
                    (a.z + b.z) * 0.5});
            midpointByEdge.emplace(key, index);
            return index;
        };

        for (const auto &face : baseFaces)
        {
            const std::size_t ab = midpointIndex(face[0], face[1]);
            const std::size_t bc = midpointIndex(face[1], face[2]);
            const std::size_t ca = midpointIndex(face[2], face[0]);
            mesh.faces.push_back({face[0], ab, ca});
            mesh.faces.push_back({face[1], bc, ab});
            mesh.faces.push_back({face[2], ca, bc});
            mesh.faces.push_back({ab, bc, ca});
        }

        return mesh;
    }

    ObjMeshData makeTorus256()
    {
        ObjMeshData mesh;
        constexpr int majorSegments = 16;
        constexpr int minorSegments = 8;
        constexpr double majorRadius = 1.15;
        constexpr double minorRadius = 0.32;
        const double pi = std::acos(-1.0);

        for (int major = 0; major < majorSegments; ++major)
        {
            const double theta = 2.0 * pi * static_cast<double>(major) / static_cast<double>(majorSegments);
            const double cosTheta = std::cos(theta);
            const double sinTheta = std::sin(theta);
            for (int minor = 0; minor < minorSegments; ++minor)
            {
                const double phi = 2.0 * pi * static_cast<double>(minor) / static_cast<double>(minorSegments);
                const double radial = majorRadius + minorRadius * std::cos(phi);
                mesh.vertices.push_back(ObjVertex{
                    radial * cosTheta,
                    minorRadius * std::sin(phi),
                    radial * sinTheta});
            }
        }

        for (int major = 0; major < majorSegments; ++major)
        {
            const int nextMajor = (major + 1) % majorSegments;
            for (int minor = 0; minor < minorSegments; ++minor)
            {
                const int nextMinor = (minor + 1) % minorSegments;
                const std::size_t a = static_cast<std::size_t>(major * minorSegments + minor);
                const std::size_t b = static_cast<std::size_t>(nextMajor * minorSegments + minor);
                const std::size_t c = static_cast<std::size_t>(nextMajor * minorSegments + nextMinor);
                const std::size_t d = static_cast<std::size_t>(major * minorSegments + nextMinor);
                mesh.faces.push_back({a, b, c});
                mesh.faces.push_back({a, c, d});
            }
        }

        return mesh;
    }

    ObjMeshData transformObjMesh(
        const ObjMeshData &mesh,
        double tx,
        double ty,
        double tz,
        double rxDegrees,
        double ryDegrees,
        double rzDegrees)
    {
        ObjMeshData transformed = mesh;
        const double pi = std::acos(-1.0);
        const double rx = rxDegrees * pi / 180.0;
        const double ry = ryDegrees * pi / 180.0;
        const double rz = rzDegrees * pi / 180.0;
        const double cx = std::cos(rx);
        const double sx = std::sin(rx);
        const double cy = std::cos(ry);
        const double sy = std::sin(ry);
        const double cz = std::cos(rz);
        const double sz = std::sin(rz);

        for (ObjVertex &vertex : transformed.vertices)
        {
            const double x1 = vertex.x;
            const double y1 = cx * vertex.y - sx * vertex.z;
            const double z1 = sx * vertex.y + cx * vertex.z;
            const double x2 = cy * x1 + sy * z1;
            const double y2 = y1;
            const double z2 = -sy * x1 + cy * z1;
            vertex.x = cz * x2 - sz * y2 + tx;
            vertex.y = sz * x2 + cz * y2 + ty;
            vertex.z = z2 + tz;
        }

        return transformed;
    }

    ObjMeshData solveObjBooleanMesh(
        const ObjMeshData &lhs,
        const ObjMeshData &rhs,
        BoolOp operation,
        std::size_t leafThreshold)
    {
        ember::QuantizeOptions options;
        std::uint64_t scale = 0;
        std::string error;
        if (!ember::chooseSharedScale({lhs, rhs}, options, scale, error))
        {
            throw std::runtime_error("io_tests failed to choose shared scale: " + error);
        }

        PolygonSoupBuildOptions polygonBuildOptions;
        polygonBuildOptions.triangulateNonCoplanarFaces = true;

        std::vector<Polygon256> lhsPolygons;
        std::vector<Polygon256> rhsPolygons;
        if (!ember::buildPolygonSoup(lhs, scale, polygonBuildOptions, lhsPolygons, error) ||
            !ember::buildPolygonSoup(rhs, scale, polygonBuildOptions, rhsPolygons, error))
        {
            throw std::runtime_error("io_tests failed to build polygon soup: " + error);
        }

        ember::BoolProblem problem(leafThreshold);
        problem.setOperation(operation);
        problem.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        problem.setOperands(lhsPolygons, rhsPolygons);
        problem.solve();
        if (!problem.isSolved())
        {
            throw std::runtime_error("io_tests BoolProblem did not report solved.");
        }

        ObjMeshData result;
        if (!ember::buildObjMeshFromPolygonSoup(problem.resultFragments(), result, error, scale))
        {
            throw std::runtime_error("io_tests failed to build result mesh: " + error);
        }
        return result;
    }

    ObjMeshData solveObjDifferenceMesh(const ObjMeshData &lhs, const ObjMeshData &rhs, std::size_t leafThreshold)
    {
        return solveObjBooleanMesh(lhs, rhs, BoolOp::Difference, leafThreshold);
    }

    std::size_t solveObjDifferenceFragmentCount(const ObjMeshData &lhs, const ObjMeshData &rhs, std::size_t leafThreshold)
    {
        return solveObjDifferenceMesh(lhs, rhs, leafThreshold).faces.size();
    }

    bool runExpensiveIoRegressionTests()
    {
        const char *value = std::getenv("REEMBER_RUN_EXPENSIVE_IO_TESTS");
        return value != nullptr && std::string(value) == "1";
    }
}

#undef assert
#define assert(expr)                                                             \
    do                                                                           \
    {                                                                            \
        if (!(expr))                                                             \
        {                                                                        \
            throw std::runtime_error("io_tests assertion failed: " #expr);       \
        }                                                                        \
    } while (false)

void runIoTests()
{
    {
        const std::filesystem::path inputPath = makeTestPath("io_explicit_scale_face.obj");
        writeTextFile(
            inputPath,
            "v 0.0 0.0 0.0\n"
            "v 1.0 0.0 0.0\n"
            "v 0.0 1.0 0.0\n"
            "f 1 2 3\n");

        ObjMeshData mesh;
        std::string error;
        assert(ember::readObjMesh(inputPath.string(), mesh, error));
        assert(mesh.vertices.size() == 3u);
        assert(mesh.faces.size() == 1u);

        std::vector<Polygon256> polygons;
        assert(ember::buildPolygonSoup(mesh, 10u, polygons, error));
        assert(polygons.size() == 1u);
        assert(polygons.front().isValid());
    }

    {
        ObjMeshData lhs;
        lhs.vertices = {
            ObjVertex{-12.5, 0.0, 0.0},
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{0.0, 12.5, 0.0}};
        lhs.faces = {{0, 1, 2}};

        ObjMeshData rhs;
        rhs.vertices = {
            ObjVertex{0.125, 0.0, 0.0},
            ObjVertex{0.0, 0.125, 0.0},
            ObjVertex{0.0, 0.0, 0.125}};
        rhs.faces = {{0, 1, 2}};

        ember::QuantizeOptions options;
        std::uint64_t scale = 0;
        std::string error;
        const std::vector<ObjMeshData> meshes{lhs, rhs};
        assert(ember::chooseSharedScale(meshes, options, scale, error));
        assert(scale == 1000000u);
    }

    {
        ObjMeshData lhs;
        lhs.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{1.0, 0.0, 0.0},
            ObjVertex{1.0, 1.0, 0.0},
            ObjVertex{0.0, 1.0, 0.0},
            ObjVertex{0.0, 0.0, 1.0},
            ObjVertex{1.0, 0.0, 1.0},
            ObjVertex{1.0, 1.0, 1.0},
            ObjVertex{0.0, 1.0, 1.0}};
        lhs.faces = {
            {3, 2, 1, 0},
            {4, 5, 6, 7},
            {0, 1, 5, 4},
            {1, 2, 6, 5},
            {2, 3, 7, 6},
            {3, 0, 4, 7}};

        ObjMeshData rhs;
        rhs.vertices = {
            ObjVertex{-0.08, -0.08, -0.5},
            ObjVertex{0.08, -0.08, -0.5},
            ObjVertex{0.08, 0.08, -0.5},
            ObjVertex{-0.08, 0.08, -0.5},
            ObjVertex{-0.08, -0.08, 0.5},
            ObjVertex{0.08, -0.08, 0.5},
            ObjVertex{0.08, 0.08, 0.5},
            ObjVertex{-0.08, 0.08, 0.5}};
        rhs.faces = {
            {3, 2, 1, 0},
            {4, 5, 6, 7},
            {0, 1, 5, 4},
            {1, 2, 6, 5},
            {2, 3, 7, 6},
            {3, 0, 4, 7}};

        ember::QuantizeOptions options;
        std::uint64_t scale = 0;
        std::string error;
        assert(ember::chooseSharedScale({lhs, rhs}, options, scale, error));
        assert(scale == 10000000u);

        std::vector<Polygon256> lhsPolygons;
        std::vector<Polygon256> rhsPolygons;
        assert(ember::buildPolygonSoup(lhs, scale, lhsPolygons, error));
        assert(ember::buildPolygonSoup(rhs, scale, rhsPolygons, error));

        ember::BoolProblem problem(25);
        problem.setOperation(BoolOp::Difference);
        problem.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        problem.setOperands(lhsPolygons, rhsPolygons);
        problem.solve();
        assert(problem.resultFragments().size() == 12u);
    }

    if (runExpensiveIoRegressionTests())
    {
        const ObjMeshData tool = makeObjBox(-0.08, -0.08, -0.5, 0.08, 0.08, 0.5);

        const std::size_t icosphereDefaultFragments =
            solveObjDifferenceFragmentCount(makeIcosphere80(), tool, 25u);
        assert(icosphereDefaultFragments > 0u);

        const std::size_t icosphereFirstSplitFragments =
            solveObjDifferenceFragmentCount(makeIcosphere80(), tool, 85u);
        assert(icosphereFirstSplitFragments > 0u);

        const std::size_t icosphereLargeLeafFragments =
            solveObjDifferenceFragmentCount(makeIcosphere80(), tool, 100u);
        assert(icosphereLargeLeafFragments > 0u);

        const std::size_t torusFragments =
            solveObjDifferenceFragmentCount(makeTorus256(), tool, 25u);
        assert(torusFragments > 0u);

        const ObjMeshData visualPoseTool = transformObjMesh(
            tool,
            0.5,
            0.5,
            0.35,
            123.582094,
            -26.865671,
            118.208964);
        const ObjMeshData visualPoseResult =
            solveObjDifferenceMesh(makeIcosphere80(), visualPoseTool, 25u);
        assert(!visualPoseResult.faces.empty());
        for (const ObjVertex &vertex : visualPoseResult.vertices)
        {
            const double radius =
                std::sqrt(vertex.x * vertex.x + vertex.y * vertex.y + vertex.z * vertex.z);
            assert(radius <= 1.000001);
        }
    }

    if (runExpensiveIoRegressionTests())
    {
        const ObjMeshData workpiece = readRepoObjMesh("assets/visual_test/workpiece_block.obj");
        const ObjMeshData tool = readRepoObjMesh("assets/visual_test/tool_box.obj");

        const ObjMeshData screenshotPoseTool = transformObjMesh(
            tool,
            0.920000,
            0.231343,
            0.500000,
            0.0,
            0.0,
            0.0);
        const ObjMeshData screenshotIntersectionT25 =
            solveObjBooleanMesh(workpiece, screenshotPoseTool, BoolOp::Intersection, 25u);
        assertWellFormedObjMesh(screenshotIntersectionT25);
        assertFaceCountAtLeast(screenshotIntersectionT25, 41u, "visual screenshot intersection threshold 25");

        const ObjMeshData screenshotIntersectionT100 =
            solveObjBooleanMesh(workpiece, screenshotPoseTool, BoolOp::Intersection, 100u);
        assertWellFormedObjMesh(screenshotIntersectionT100);
        assertFaceCountAtLeast(screenshotIntersectionT100, 28u, "visual screenshot intersection threshold 100");

        const ObjMeshData coplanarPresetTool = transformObjMesh(
            tool,
            0.920000,
            0.500000,
            0.500000,
            0.0,
            0.0,
            0.0);
        const ObjMeshData blockWorkpiece = readRepoObjMesh("assets/models/workpiece_block.obj");
        const ObjMeshData coplanarIntersection =
            solveObjBooleanMesh(blockWorkpiece, coplanarPresetTool, BoolOp::Intersection, 25u);
        assertWellFormedObjMesh(coplanarIntersection);
        assertFaceCountAtLeast(coplanarIntersection, 6u, "visual coplanar x-max intersection");
    }

    {
        ObjMeshData lhs;
        lhs.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{1.0, 0.0, 0.0},
            ObjVertex{1.0, 1.0, 0.0},
            ObjVertex{0.0, 1.0, 0.0},
            ObjVertex{0.0, 0.0, 1.0},
            ObjVertex{1.0, 0.0, 1.0},
            ObjVertex{1.0, 1.0, 1.0},
            ObjVertex{0.0, 1.0, 1.0}};
        lhs.faces = {
            {3, 2, 1, 0},
            {4, 5, 6, 7},
            {0, 1, 5, 4},
            {1, 2, 6, 5},
            {2, 3, 7, 6},
            {3, 0, 4, 7}};

        ObjMeshData rhs;
        rhs.vertices = {
            ObjVertex{0.6740791455178619, 0.2857432247353239, 0.10450414972599636},
            ObjVertex{0.7982417133188726, 0.3486409791457349, 0.025589453710697374},
            ObjVertex{0.725937664677665, 0.49137193076482094, 0.025589453710697374},
            ObjVertex{0.6017750968766542, 0.42847417635440993, 0.10450414972599636},
            ObjVertex{1.1140623353223351, 0.5086280692351791, 0.9744105462893027},
            ObjVertex{1.2382249031233459, 0.5715258236455901, 0.8954958502740036},
            ObjVertex{1.1659208544821382, 0.7142567752646761, 0.8954958502740036},
            ObjVertex{1.0417582866811275, 0.6513590208542651, 0.9744105462893027}};
        rhs.faces = {
            {3, 2, 1},
            {3, 1, 0},
            {4, 5, 6},
            {4, 6, 7},
            {0, 1, 5},
            {0, 5, 4},
            {1, 2, 6},
            {1, 6, 5},
            {2, 3, 7},
            {2, 7, 6},
            {3, 0, 4},
            {3, 4, 7}};

        ember::QuantizeOptions options;
        std::uint64_t scale = 0;
        std::string error;
        assert(ember::chooseSharedScale({lhs, rhs}, options, scale, error));
        assert(scale == 10000000u);

        std::vector<Polygon256> lhsPolygons;
        std::vector<Polygon256> rhsPolygons;
        assert(ember::buildPolygonSoup(lhs, scale, lhsPolygons, error));
        assert(ember::buildPolygonSoup(rhs, scale, rhsPolygons, error));

        ember::BoolProblem problem(25);
        problem.setOperation(BoolOp::Difference);
        problem.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        problem.setOperands(lhsPolygons, rhsPolygons);
        problem.solve();

        ObjMeshData resultMesh;
        assert(ember::buildObjMeshFromPolygonSoup(problem.resultFragments(), resultMesh, error, scale));
        assert(!resultMesh.vertices.empty());
        for (const ObjVertex &vertex : resultMesh.vertices)
        {
            assert(vertex.x >= -1e-9 && vertex.x <= 1.0 + 1e-9);
            assert(vertex.y >= -1e-9 && vertex.y <= 1.0 + 1e-9);
            assert(vertex.z >= -1e-9 && vertex.z <= 1.0 + 1e-9);
        }
    }

    {
        ObjMeshData quad;
        quad.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{2.0, 0.0, 0.0},
            ObjVertex{2.0, 1.0, 0.0},
            ObjVertex{0.0, 1.0, 0.0}};
        quad.faces = {{0, 1, 2, 3}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(ember::buildPolygonSoup(quad, 1u, polygons, error));
        assert(polygons.size() == 1u);
        assert(polygons.front().edgeCount() == 4u);
        assert(polygons.front().isValid());
    }

    {
        ObjMeshData quad;
        quad.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{2.0, 0.0, 0.0},
            ObjVertex{2.0, 1.0, 0.0},
            ObjVertex{0.0, 1.0, 0.0}};
        quad.faces = {{0, 1, 2, 3}};

        PolygonSoupBuildOptions options;
        options.triangulateNonCoplanarFaces = true;

        std::string error;
        std::vector<Polygon256> polygons;
        assert(ember::buildPolygonSoup(quad, 1u, options, polygons, error));
        assert(polygons.size() == 1u);
        assert(polygons.front().edgeCount() == 4u);
        assert(polygons.front().isValid());
    }

    {
        ObjMeshData roundedNonCoplanarQuad;
        roundedNonCoplanarQuad.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{1.0, 0.0, 0.04},
            ObjVertex{1.0, 1.0, 0.08},
            ObjVertex{0.0, 1.0, 0.04}};
        roundedNonCoplanarQuad.faces = {{0, 1, 2, 3}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(!ember::buildPolygonSoup(roundedNonCoplanarQuad, 10u, polygons, error));
        assert(!error.empty());

        PolygonSoupBuildOptions options;
        options.triangulateNonCoplanarFaces = true;

        error.clear();
        assert(ember::buildPolygonSoup(roundedNonCoplanarQuad, 10u, options, polygons, error));
        assert(polygons.size() == 2u);
        assert(polygons[0].edgeCount() == 3u);
        assert(polygons[1].edgeCount() == 3u);
        assert(polygons[0].isValid());
        assert(polygons[1].isValid());
    }

    {
        ObjMeshData concave;
        concave.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{2.0, 0.0, 0.0},
            ObjVertex{1.0, 1.0, 0.0},
            ObjVertex{2.0, 2.0, 0.0},
            ObjVertex{0.0, 2.0, 0.0}};
        concave.faces = {{0, 1, 2, 3, 4}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(!ember::buildPolygonSoup(concave, 1u, polygons, error));
        assert(!error.empty());
    }

    {
        ObjMeshData nonCoplanar;
        nonCoplanar.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{1.0, 0.0, 0.0},
            ObjVertex{1.0, 1.0, 1.0},
            ObjVertex{0.0, 1.0, 0.0}};
        nonCoplanar.faces = {{0, 1, 2, 3}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(!ember::buildPolygonSoup(nonCoplanar, 1u, polygons, error));
        assert(!error.empty());
    }

    {
        ObjMeshData repeatedVertex;
        repeatedVertex.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{1.0, 0.0, 0.0},
            ObjVertex{0.0, 1.0, 0.0}};
        repeatedVertex.faces = {{0, 1, 1, 2}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(!ember::buildPolygonSoup(repeatedVertex, 1u, polygons, error));
        assert(!error.empty());
    }

    {
        ObjMeshData outOfRange;
        outOfRange.vertices = {
            ObjVertex{33554432.0, 0.0, 0.0},
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{0.0, 1.0, 0.0}};
        outOfRange.faces = {{0, 1, 2}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(!ember::buildPolygonSoup(outOfRange, 1u, polygons, error));
        assert(!error.empty());
    }

    {
        const std::filesystem::path inputPath = makeTestPath("io_face_with_vt_vn.obj");
        writeTextFile(
            inputPath,
            "v 0.0 0.0 0.0\n"
            "v 1.0 0.0 0.0\n"
            "v 0.0 1.0 0.0\n"
            "vt 0.0 0.0\n"
            "vt 1.0 0.0\n"
            "vt 0.0 1.0\n"
            "vn 0.0 0.0 1.0\n"
            "vn 0.0 0.0 1.0\n"
            "vn 0.0 0.0 1.0\n"
            "f 1/1/1 2/2/2 3/3/3\n");

        ObjMeshData mesh;
        std::string error;
        assert(ember::readObjMesh(inputPath.string(), mesh, error));
        assert(mesh.faces.size() == 1u);

        std::vector<Polygon256> polygons;
        assert(ember::buildPolygonSoup(mesh, 1u, polygons, error));
        assert(polygons.size() == 1u);
    }

    {
        const std::filesystem::path inputPath = makeTestPath("io_relative_index_quad.obj");
        writeTextFile(
            inputPath,
            "v 0.0 0.0 0.0\n"
            "v 2.0 0.0 0.0\n"
            "v 2.0 1.0 0.0\n"
            "v 0.0 1.0 0.0\n"
            "f -4 -3 -2 -1\n");

        ObjMeshData mesh;
        std::string error;
        assert(ember::readObjMesh(inputPath.string(), mesh, error));
        assert(mesh.vertices.size() == 4u);
        assert(mesh.faces.size() == 1u);
        assert(mesh.faces.front().size() == 4u);
        assert(mesh.faces.front()[0] == 0u);
        assert(mesh.faces.front()[1] == 1u);
        assert(mesh.faces.front()[2] == 2u);
        assert(mesh.faces.front()[3] == 3u);

        std::vector<Polygon256> polygons;
        assert(ember::buildPolygonSoup(mesh, 1u, polygons, error));
        assert(polygons.size() == 1u);
        assert(polygons.front().edgeCount() == 4u);
    }

    {
        const Polygon256 square = makeFaceXY(0, 0, 2, 0, 2, 1);

        ObjMeshData mesh;
        std::string error;
        assert(ember::buildObjMeshFromPolygonSoup({square}, mesh, error));
        assert(mesh.vertices.size() == 4u);
        assert(mesh.faces.size() == 1u);
        assert(mesh.faces.front().size() == 4u);
    }

    {
        const Polygon256 scaledSquare = makeFaceXY(0, 0, 20, 0, 20, 1);

        ObjMeshData mesh;
        std::string error;
        assert(ember::buildObjMeshFromPolygonSoup({scaledSquare}, mesh, error, 10u));
        assert(mesh.vertices.size() == 4u);
        assert(mesh.vertices[0].x == 0.0);
        assert(mesh.vertices[0].y == 0.0);
        assert(mesh.vertices[1].x == 2.0);
        assert(mesh.vertices[2].y == 2.0);
    }

    {
        const Polygon256 square = makeFaceXY(0, 0, 2, 0, 2, 1);
        const std::filesystem::path outputPath = makeTestPath("io_single_square_export.obj");

        std::size_t faceCount = 0;
        std::string error;
        const std::vector<Polygon256> fragments{square};
        assert(ember::writePolygonSoupObj(fragments, outputPath.string(), faceCount, error));
        assert(faceCount == 1u);

        const std::string contents = readTextFile(outputPath);
        std::size_t vertexLineCount = 0;
        std::size_t faceLineCount = 0;
        std::istringstream input(contents);
        std::string line;
        while (std::getline(input, line))
        {
            if (line.rfind("v ", 0) == 0)
            {
                ++vertexLineCount;
            }
            else if (line.rfind("f ", 0) == 0)
            {
                ++faceLineCount;
                std::istringstream lineStream(line);
                std::string prefix;
                lineStream >> prefix;
                std::size_t indices = 0;
                std::string token;
                while (lineStream >> token)
                {
                    ++indices;
                }
                assert(indices == 4u);
            }
        }

        assert(vertexLineCount == 4u);
        assert(faceLineCount == 1u);
    }

    {
        ObjMeshData concave;
        concave.vertices = {
            ObjVertex{0.0, 0.0, 0.0},
            ObjVertex{2.0, 0.0, 0.0},
            ObjVertex{1.0, 1.0, 0.0},
            ObjVertex{2.0, 2.0, 0.0},
            ObjVertex{0.0, 2.0, 0.0}};
        concave.faces = {{0, 1, 2, 3, 4}};

        std::string error;
        std::vector<Polygon256> polygons;
        assert(!ember::buildPolygonSoup(concave, 1u, polygons, error));
        assert(!error.empty());
    }

    {
        const std::vector<Polygon256> lhs = makeAxisAlignedBox(0, 0, 0, 1, 1, 1);
        const std::vector<Polygon256> rhs = makeAxisAlignedBox(3, 3, 3, 4, 4, 4);

        ember::BoolProblem problem(2);
        problem.setOperation(BoolOp::Union);
        problem.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        problem.setOperands(lhs, rhs);
        problem.solve();
        assert(problem.isSolved());
        assert(problem.resultFragments().size() == 12u);

        const std::filesystem::path outputPath = makeTestPath("io_boolean_export.obj");
        std::size_t faceCount = 0;
        std::string error;
        assert(ember::writePolygonSoupObj(problem.resultFragments(), outputPath.string(), faceCount, error));
        assert(faceCount == problem.resultFragments().size());

        const std::string contents = readTextFile(outputPath);
        std::size_t exportedFaceLines = 0;
        std::istringstream input(contents);
        std::string line;
        while (std::getline(input, line))
        {
            if (line.rfind("f ", 0) == 0)
            {
                ++exportedFaceLines;
            }
        }

        assert(exportedFaceLines == problem.resultFragments().size());
    }

    {
        auto makeObjBox = [](double xmin, double ymin, double zmin, double xmax, double ymax, double zmax)
        {
            ObjMeshData box;
            box.vertices = {
                ObjVertex{xmin, ymin, zmin},
                ObjVertex{xmax, ymin, zmin},
                ObjVertex{xmax, ymax, zmin},
                ObjVertex{xmin, ymax, zmin},
                ObjVertex{xmin, ymin, zmax},
                ObjVertex{xmax, ymin, zmax},
                ObjVertex{xmax, ymax, zmax},
                ObjVertex{xmin, ymax, zmax}};
            box.faces = {
                {3, 2, 1, 0},
                {4, 5, 6, 7},
                {0, 1, 5, 4},
                {1, 2, 6, 5},
                {2, 3, 7, 6},
                {3, 0, 4, 7}};
            return box;
        };

        const ObjMeshData workpiece = makeObjBox(0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
        const ObjMeshData tool = makeObjBox(0.42, 0.42, -0.15, 0.58, 0.58, 0.85);

        std::string error;
        std::vector<Polygon256> lhsPolygons;
        std::vector<Polygon256> rhsPolygons;
        assert(ember::buildPolygonSoup(workpiece, 1000u, lhsPolygons, error));
        assert(ember::buildPolygonSoup(tool, 1000u, rhsPolygons, error));

        ember::BoolProblem problem(25);
        problem.setOperation(BoolOp::Union);
        problem.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        problem.setOperands(lhsPolygons, rhsPolygons);
        problem.solve();
        assert(problem.resultFragments().size() == 14u);

        ObjMeshData result;
        assert(ember::buildObjMeshFromPolygonSoup(problem.resultFragments(), result, error, 1000u));
        assert(result.faces.size() == 14u);
    }
}
