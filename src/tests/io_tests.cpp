#include "io_tests.h"

#include "core/bool_problem.h"
#include "io/io.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    using ember::BoolOp;
    using ember::ObjMeshData;
    using ember::ObjVertex;
    using ember::Plane3i;
    using ember::Polygon256;
    using ember::Vec3i;

    std::filesystem::path makeTestPath(const std::string &filename)
    {
        return std::filesystem::current_path() / filename;
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
        problem.setOperands(lhsPolygons, rhsPolygons);
        problem.solve();
        assert(problem.resultFragments().size() == 14u);

        ObjMeshData result;
        assert(ember::buildObjMeshFromPolygonSoup(problem.resultFragments(), result, error, 1000u));
        assert(result.faces.size() == 14u);
    }
}
