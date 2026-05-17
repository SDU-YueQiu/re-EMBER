/**
 * @file verify.cpp
 * @brief 实现基于 CGAL Nef oracle 的 re-EMBER 结果校验工具。
 */
#include "core/bool_problem.h"
#include "geometry/geometry256.h"
#include "io/io.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/version.h>

#include <boost/uuid/detail/sha1.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using ember::BoolOp;
using ember::BoolOperandAssumptions;
using Clock = std::chrono::steady_clock;
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using SurfaceMesh = CGAL::Surface_mesh<Kernel::Point_3>;
using NefPolyhedron = CGAL::Nef_polyhedron_3<Kernel>;

constexpr const char *kOracleSchema = "re-EMBER-nef-oracle-v1";

struct VerifyOptions
{
    std::string lhsPath;
    std::string rhsPath;
    BoolOp operation = BoolOp::Intersection;
    std::optional<std::uint64_t> scale;
    std::size_t leafThreshold = 25;
    std::size_t threadCount = 0;
    BoolOperandAssumptions lhsAssumptions;
    BoolOperandAssumptions rhsAssumptions;
    std::filesystem::path oracleCacheDir = std::filesystem::path("build") / "oracle_cache" / "nef";
    bool refreshOracle = false;
    std::filesystem::path reportPath;
};

struct PreparedProblem
{
    ember::ObjMeshData lhsMesh;
    ember::ObjMeshData rhsMesh;
    std::uint64_t sharedScale = 0;
    std::vector<ember::Polygon256> lhsPolygons;
    std::vector<ember::Polygon256> rhsPolygons;
    ember::AABB3i sceneAABB;
};

struct VerificationReport
{
    bool passed = false;
    bool cacheHit = false;
    std::string oracleKey;
    std::filesystem::path oraclePath;
    std::size_t lhsPolygons = 0;
    std::size_t rhsPolygons = 0;
    std::size_t resultFragments = 0;
    std::uint64_t sharedScale = 0;
    double prepareMs = 0.0;
    double solveMs = 0.0;
    double oracleMs = 0.0;
    double compareMs = 0.0;
};

double elapsedMilliseconds(const Clock::time_point &start, const Clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

void printUsage()
{
    std::cerr
            << "Usage: re-EMBER_verify --lhs <file.obj|file.stl> --rhs <file.obj|file.stl> "
            << "--op union|intersection|difference "
            << "[--scale <positive_integer>] [--leaf-threshold <positive_integer>] "
            << "[--threads <positive_integer>] "
            << "[--assume-lhs-nsi] [--assume-lhs-nnc] "
            << "[--assume-rhs-nsi] [--assume-rhs-nnc] "
            << "[--oracle-cache-dir <dir>] [--refresh-oracle] [--report-out <file>]"
            << std::endl;
}

bool parsePositiveUInt64(const std::string &token, std::uint64_t &outValue)
{
    try
    {
        std::size_t consumed = 0;
        const unsigned long long parsed = std::stoull(token, &consumed, 10);
        if (consumed != token.size() || parsed == 0)
            return false;

        outValue = static_cast<std::uint64_t>(parsed);
        return true;
    }
    catch (...)
    {
        return false;
    }
}

bool parseBoolOp(const std::string &token, BoolOp &outOp)
{
    if (token == "union")
    {
        outOp = BoolOp::Union;
        return true;
    }
    if (token == "intersection")
    {
        outOp = BoolOp::Intersection;
        return true;
    }
    if (token == "difference")
    {
        outOp = BoolOp::Difference;
        return true;
    }

    return false;
}

const char *toString(BoolOp op)
{
    switch (op)
    {
    case BoolOp::Union:
        return "union";
    case BoolOp::Intersection:
        return "intersection";
    case BoolOp::Difference:
        return "difference";
    }

    return "unknown";
}

bool parseArgs(int argc, char **argv, VerifyOptions &outOptions)
{
    outOptions = VerifyOptions();
    bool hasOperation = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--lhs" || arg == "--rhs" || arg == "--op" ||
                arg == "--scale" || arg == "--leaf-threshold" || arg == "--threads" ||
                arg == "--oracle-cache-dir" || arg == "--report-out")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "Missing value for argument: " << arg << std::endl;
                return false;
            }

            const std::string value(argv[++i]);
            if (arg == "--lhs")
                outOptions.lhsPath = value;
            else if (arg == "--rhs")
                outOptions.rhsPath = value;
            else if (arg == "--op")
            {
                if (!parseBoolOp(value, outOptions.operation))
                {
                    std::cerr << "Unsupported boolean operation: " << value << std::endl;
                    return false;
                }
                hasOperation = true;
            }
            else if (arg == "--scale")
            {
                std::uint64_t scaleValue = 0;
                if (!parsePositiveUInt64(value, scaleValue))
                {
                    std::cerr << "Scale must be a positive integer." << std::endl;
                    return false;
                }
                outOptions.scale = scaleValue;
            }
            else if (arg == "--leaf-threshold")
            {
                std::uint64_t thresholdValue = 0;
                if (!parsePositiveUInt64(value, thresholdValue))
                {
                    std::cerr << "Leaf threshold must be a positive integer." << std::endl;
                    return false;
                }
                outOptions.leafThreshold = static_cast<std::size_t>(thresholdValue);
            }
            else if (arg == "--threads")
            {
                std::uint64_t threadCountValue = 0;
                if (!parsePositiveUInt64(value, threadCountValue))
                {
                    std::cerr << "Thread count must be a positive integer." << std::endl;
                    return false;
                }
                outOptions.threadCount = static_cast<std::size_t>(threadCountValue);
            }
            else if (arg == "--oracle-cache-dir")
                outOptions.oracleCacheDir = value;
            else if (arg == "--report-out")
                outOptions.reportPath = value;

            continue;
        }

        if (arg == "--refresh-oracle")
        {
            outOptions.refreshOracle = true;
            continue;
        }
        if (arg == "--assume-lhs-nsi")
        {
            outOptions.lhsAssumptions.noSelfIntersections = true;
            continue;
        }
        if (arg == "--assume-lhs-nnc")
        {
            outOptions.lhsAssumptions.noNestedComponents = true;
            continue;
        }
        if (arg == "--assume-rhs-nsi")
        {
            outOptions.rhsAssumptions.noSelfIntersections = true;
            continue;
        }
        if (arg == "--assume-rhs-nnc")
        {
            outOptions.rhsAssumptions.noNestedComponents = true;
            continue;
        }

        std::cerr << "Unknown argument: " << arg << std::endl;
        return false;
    }

    if (outOptions.lhsPath.empty() || outOptions.rhsPath.empty() || !hasOperation)
    {
        std::cerr << "Missing required arguments." << std::endl;
        return false;
    }
    if (outOptions.lhsAssumptions.noNestedComponents &&
            !outOptions.lhsAssumptions.noSelfIntersections)
    {
        std::cerr << "--assume-lhs-nnc requires --assume-lhs-nsi." << std::endl;
        return false;
    }
    if (outOptions.rhsAssumptions.noNestedComponents &&
            !outOptions.rhsAssumptions.noSelfIntersections)
    {
        std::cerr << "--assume-rhs-nnc requires --assume-rhs-nsi." << std::endl;
        return false;
    }

    return true;
}

PreparedProblem prepareProblem(const VerifyOptions &options)
{
    PreparedProblem prepared;
    std::string error;
    if (!ember::readMesh(options.lhsPath, prepared.lhsMesh, error))
        throw std::runtime_error(error);
    if (!ember::readMesh(options.rhsPath, prepared.rhsMesh, error))
        throw std::runtime_error(error);

    ember::QuantizeOptions quantizeOptions;
    quantizeOptions.explicitScale = options.scale;
    if (!ember::chooseSharedScale(
                {prepared.lhsMesh, prepared.rhsMesh},
                quantizeOptions,
                prepared.sharedScale,
                error))
        throw std::runtime_error(error);

    ember::PolygonSoupBuildOptions buildOptions;
    buildOptions.triangulateNonCoplanarFaces = true;

    ember::AABB3i lhsAABB;
    ember::AABB3i rhsAABB;
    if (!ember::computeScaledMeshAABB(prepared.lhsMesh, prepared.sharedScale, lhsAABB, error))
        throw std::runtime_error(error);
    if (!ember::computeScaledMeshAABB(prepared.rhsMesh, prepared.sharedScale, rhsAABB, error))
        throw std::runtime_error(error);
    if (!ember::buildPolygonSoup(
                prepared.lhsMesh,
                prepared.sharedScale,
                buildOptions,
                prepared.lhsPolygons,
                error))
        throw std::runtime_error(error);
    if (!ember::buildPolygonSoup(
                prepared.rhsMesh,
                prepared.sharedScale,
                buildOptions,
                prepared.rhsPolygons,
                error))
        throw std::runtime_error(error);

    ember::mergeAABB(prepared.sceneAABB, lhsAABB);
    ember::mergeAABB(prepared.sceneAABB, rhsAABB);
    ember::expandAABB(prepared.sceneAABB, 1);
    return prepared;
}

Kernel::FT parseIntegerKernelFT(const std::string &text)
{
    bool negative = false;
    std::size_t offset = 0;
    if (!text.empty() && text[0] == '-')
    {
        negative = true;
        offset = 1;
    }

    Kernel::FT value(0);
    for (std::size_t i = offset; i < text.size(); ++i)
    {
        if (text[i] < '0' || text[i] > '9')
            throw std::runtime_error("Invalid integer text while converting to CGAL exact number.");
        value = value * 10 + static_cast<int>(text[i] - '0');
    }

    return negative ? -value : value;
}

Kernel::FT toKernelFT(const ember::Integer &numerator, const ember::Integer &denominator)
{
    return parseIntegerKernelFT(ember::integerToString(numerator)) /
           parseIntegerKernelFT(ember::integerToString(denominator));
}

std::string homPointKey(const ember::HomPoint4i &point)
{
    const ember::HomPoint4i primitive = ember::primitiveHomPoint(point);
    return ember::integerToString(primitive.x) + "/" +
           ember::integerToString(primitive.y) + "/" +
           ember::integerToString(primitive.z) + "/" +
           ember::integerToString(primitive.w);
}

Kernel::Point_3 toKernelPoint(const ember::PlanePoint3i &point)
{
    if (!point.hasUniqueIntersection() || ember::isZero(point.x.w))
        throw std::runtime_error("Polygon contains a vertex without a unique finite homogeneous point.");

    return Kernel::Point_3(
               toKernelFT(point.x.x, point.x.w),
               toKernelFT(point.x.y, point.x.w),
               toKernelFT(point.x.z, point.x.w));
}

Kernel::FT coordinateByAxis(const Kernel::Point_3 &point, int axis)
{
    if (axis == 0)
        return point.x();
    if (axis == 1)
        return point.y();
    return point.z();
}

int chooseSegmentSortAxis(const Kernel::Point_3 &start, const Kernel::Point_3 &end)
{
    if (start.x() != end.x())
        return 0;
    if (start.y() != end.y())
        return 1;
    return 2;
}

bool pointIsOnSegment(
    const Kernel::Point_3 &start,
    const Kernel::Point_3 &point,
    const Kernel::Point_3 &end)
{
    return CGAL::collinear(start, point, end) &&
           CGAL::collinear_are_ordered_along_line(start, point, end);
}

bool isDegenerateTriangle(
    const std::vector<Kernel::Point_3> &points,
    std::size_t a,
    std::size_t b,
    std::size_t c)
{
    return a == b || b == c || a == c ||
           CGAL::collinear(points[a], points[b], points[c]);
}

bool hasNondegenerateTriple(
    const std::vector<Kernel::Point_3> &points,
    const std::vector<std::size_t> &face)
{
    if (face.size() < 3)
        return false;
    for (std::size_t i = 0; i < face.size(); ++i)
    {
        const std::size_t a = face[(i + face.size() - 1) % face.size()];
        const std::size_t b = face[i];
        const std::size_t c = face[(i + 1) % face.size()];
        if (!isDegenerateTriangle(points, a, b, c))
            return true;
    }
    return false;
}

void appendConvexEarTriangles(
    const std::vector<Kernel::Point_3> &points,
    const std::vector<std::size_t> &face,
    std::vector<std::vector<std::size_t>> &triangles,
    const char *label,
    std::size_t polygonIndex)
{
    std::vector<std::size_t> remaining = face;
    while (remaining.size() > 3)
    {
        bool clipped = false;
        for (std::size_t i = 0; i < remaining.size(); ++i)
        {
            const std::size_t a = remaining[(i + remaining.size() - 1) % remaining.size()];
            const std::size_t b = remaining[i];
            const std::size_t c = remaining[(i + 1) % remaining.size()];
            if (isDegenerateTriangle(points, a, b, c))
                continue;
            std::vector<std::size_t> candidateRemaining = remaining;
            candidateRemaining.erase(candidateRemaining.begin() + static_cast<std::ptrdiff_t>(i));
            if (candidateRemaining.size() >= 3 && !hasNondegenerateTriple(points, candidateRemaining))
                continue;

            triangles.push_back({a, b, c});
            remaining = std::move(candidateRemaining);
            clipped = true;
            break;
        }
        if (!clipped)
            throw std::runtime_error(
                std::string("Failed to triangulate ") + label + " polygon " + std::to_string(polygonIndex) +
                " after exact edge refinement; remaining_vertices=" + std::to_string(remaining.size()) + ".");
    }

    if (!isDegenerateTriangle(points, remaining[0], remaining[1], remaining[2]))
        triangles.push_back({remaining[0], remaining[1], remaining[2]});
}

NefPolyhedron makeNefFromPolygons(
    const std::vector<ember::Polygon256> &polygons,
    const char *label)
{
    if (polygons.empty())
        return NefPolyhedron(NefPolyhedron::EMPTY);

    std::vector<Kernel::Point_3> points;
    std::vector<std::vector<std::size_t>> polygonFaces;
    std::map<std::string, std::size_t> pointIndexByKey;

    auto getPointIndex = [&](const ember::PlanePoint3i &point) -> std::size_t
    {
        const std::string key = homPointKey(point.x);
        const auto found = pointIndexByKey.find(key);
        if (found != pointIndexByKey.end())
            return found->second;

        const std::size_t index = points.size();
        pointIndexByKey.emplace(key, index);
        points.push_back(toKernelPoint(point));
        return index;
    };

    for (const ember::Polygon256 &polygon : polygons)
    {
        const std::vector<ember::PlanePoint3i> &vertices = polygon.vertices();
        if (vertices.size() < 3)
            throw std::runtime_error("Polygon has fewer than three vertices.");

        std::vector<std::size_t> face;
        face.reserve(vertices.size());
        for (const ember::PlanePoint3i &vertex : vertices)
            face.push_back(getPointIndex(vertex));

        polygonFaces.push_back(std::move(face));
    }

    std::vector<std::vector<std::size_t>> triangles;
    for (std::size_t polygonIndex = 0; polygonIndex < polygonFaces.size(); ++polygonIndex)
    {
        const std::vector<std::size_t> &face = polygonFaces[polygonIndex];
        std::vector<std::size_t> refinedFace;
        refinedFace.reserve(face.size());
        for (std::size_t edgeIndex = 0; edgeIndex < face.size(); ++edgeIndex)
        {
            const std::size_t startIndex = face[edgeIndex];
            const std::size_t endIndex = face[(edgeIndex + 1) % face.size()];
            refinedFace.push_back(startIndex);

            std::vector<std::size_t> edgeInteriorPoints;
            const Kernel::Point_3 &start = points[startIndex];
            const Kernel::Point_3 &end = points[endIndex];
            for (std::size_t candidateIndex = 0; candidateIndex < points.size(); ++candidateIndex)
            {
                if (candidateIndex == startIndex || candidateIndex == endIndex)
                    continue;
                if (pointIsOnSegment(start, points[candidateIndex], end))
                    edgeInteriorPoints.push_back(candidateIndex);
            }

            const int sortAxis = chooseSegmentSortAxis(start, end);
            const bool ascending = coordinateByAxis(start, sortAxis) < coordinateByAxis(end, sortAxis);
            std::sort(
                edgeInteriorPoints.begin(),
                edgeInteriorPoints.end(),
                [&](std::size_t lhs, std::size_t rhs)
                {
                    const Kernel::FT lhsCoord = coordinateByAxis(points[lhs], sortAxis);
                    const Kernel::FT rhsCoord = coordinateByAxis(points[rhs], sortAxis);
                    return ascending ? lhsCoord < rhsCoord : rhsCoord < lhsCoord;
                });

            refinedFace.insert(refinedFace.end(), edgeInteriorPoints.begin(), edgeInteriorPoints.end());
        }

        refinedFace.erase(
            std::unique(refinedFace.begin(), refinedFace.end()),
            refinedFace.end());
        if (refinedFace.size() > 1 && refinedFace.front() == refinedFace.back())
            refinedFace.pop_back();
        if (refinedFace.size() < 3)
            continue;

        appendConvexEarTriangles(points, refinedFace, triangles, label, polygonIndex);
    }

    if (triangles.empty())
        return NefPolyhedron(NefPolyhedron::EMPTY);

    SurfaceMesh mesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, triangles, mesh);
    if (mesh.number_of_faces() == 0)
        return NefPolyhedron(NefPolyhedron::EMPTY);
    if (CGAL::is_closed(mesh) && CGAL::is_triangle_mesh(mesh) &&
            CGAL::Polygon_mesh_processing::does_bound_a_volume(mesh))
    {
        CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(mesh);
    }

    return NefPolyhedron(mesh).regularization();
}

NefPolyhedron applyBoolean(const NefPolyhedron &lhs, const NefPolyhedron &rhs, BoolOp op)
{
    switch (op)
    {
    case BoolOp::Union:
        return (lhs + rhs).regularization();
    case BoolOp::Intersection:
        return (lhs * rhs).regularization();
    case BoolOp::Difference:
        return (lhs - rhs).regularization();
    }

    return NefPolyhedron(NefPolyhedron::EMPTY);
}

void feedSha1(boost::uuids::detail::sha1 &sha, const std::string &value)
{
    sha.process_bytes(value.data(), value.size());
    const char separator = '\n';
    sha.process_byte(static_cast<unsigned char>(separator));
}

void feedInteger(boost::uuids::detail::sha1 &sha, const ember::Integer &value)
{
    feedSha1(sha, ember::integerToString(value));
}

void feedPlane(boost::uuids::detail::sha1 &sha, const ember::Plane3i &plane)
{
    feedInteger(sha, plane.a);
    feedInteger(sha, plane.b);
    feedInteger(sha, plane.c);
    feedInteger(sha, plane.d);
}

void feedPolygons(
    boost::uuids::detail::sha1 &sha,
    const char *label,
    const std::vector<ember::Polygon256> &polygons)
{
    feedSha1(sha, label);
    feedSha1(sha, std::to_string(polygons.size()));
    for (const ember::Polygon256 &polygon : polygons)
    {
        feedPlane(sha, polygon.plane);
        feedSha1(sha, std::to_string(polygon.edgePlanes.size()));
        for (const ember::Plane3i &edge : polygon.edgePlanes)
            feedPlane(sha, edge);

        const std::vector<ember::PlanePoint3i> &vertices = polygon.vertices();
        feedSha1(sha, std::to_string(vertices.size()));
        for (const ember::PlanePoint3i &vertex : vertices)
        {
            const ember::HomPoint4i primitive = ember::primitiveHomPoint(vertex.x);
            feedInteger(sha, primitive.x);
            feedInteger(sha, primitive.y);
            feedInteger(sha, primitive.z);
            feedInteger(sha, primitive.w);
        }
    }
}

std::string digestToHex(const boost::uuids::detail::sha1::digest_type &digest)
{
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (unsigned char byte : digest)
        out << std::setw(2) << static_cast<unsigned int>(byte);
    return out.str();
}

std::string computeOracleKey(
    const PreparedProblem &prepared,
    BoolOp operation)
{
    boost::uuids::detail::sha1 sha;
    feedSha1(sha, kOracleSchema);
    feedSha1(sha, CGAL_VERSION_STR);
    feedSha1(sha, toString(operation));
    feedSha1(sha, std::to_string(prepared.sharedScale));
    feedSha1(sha, "triangulateNonCoplanarFaces=true");
    feedPolygons(sha, "lhs", prepared.lhsPolygons);
    feedPolygons(sha, "rhs", prepared.rhsPolygons);

    boost::uuids::detail::sha1::digest_type digest;
    sha.get_digest(digest);
    return digestToHex(digest);
}

void saveNef(const NefPolyhedron &nef, const std::filesystem::path &path)
{
    std::filesystem::create_directories(path.parent_path());
    const std::filesystem::path tmpPath = path.string() + ".tmp";
    NefPolyhedron copy = nef;
    {
        std::ofstream output(tmpPath, std::ios::trunc);
        if (!output)
            throw std::runtime_error("Failed to open oracle cache for writing: " + tmpPath.string());
        output << copy;
        if (!output)
            throw std::runtime_error("Failed to write oracle cache: " + tmpPath.string());
    }

    std::error_code ec;
    std::filesystem::rename(tmpPath, path, ec);
    if (ec)
    {
        std::filesystem::remove(path, ec);
        ec.clear();
        std::filesystem::rename(tmpPath, path, ec);
        if (ec)
            throw std::runtime_error("Failed to replace oracle cache: " + path.string());
    }
}

NefPolyhedron loadNef(const std::filesystem::path &path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Failed to open oracle cache for reading: " + path.string());

    NefPolyhedron nef;
    input >> nef;
    if (!input)
        throw std::runtime_error("Failed to read oracle cache: " + path.string());
    return nef;
}

void writeMetadata(
    const std::filesystem::path &path,
    const std::string &key,
    const PreparedProblem &prepared,
    BoolOp operation)
{
    std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open oracle metadata for writing: " + path.string());

    output << "{\n"
           << "  \"schema\": \"" << kOracleSchema << "\",\n"
           << "  \"key\": \"" << key << "\",\n"
           << "  \"cgal_version\": \"" << CGAL_VERSION_STR << "\",\n"
           << "  \"operation\": \"" << toString(operation) << "\",\n"
           << "  \"shared_scale\": " << prepared.sharedScale << ",\n"
           << "  \"lhs_polygons\": " << prepared.lhsPolygons.size() << ",\n"
           << "  \"rhs_polygons\": " << prepared.rhsPolygons.size() << "\n"
           << "}\n";
}

NefPolyhedron loadOrBuildOracle(
    const VerifyOptions &options,
    const PreparedProblem &prepared,
    const std::string &key,
    VerificationReport &report)
{
    std::filesystem::create_directories(options.oracleCacheDir);
    const std::filesystem::path nefPath = options.oracleCacheDir / (key + ".nef3");
    const std::filesystem::path metadataPath = options.oracleCacheDir / (key + ".json");
    report.oraclePath = nefPath;

    if (!options.refreshOracle && std::filesystem::exists(nefPath))
    {
        report.cacheHit = true;
        return loadNef(nefPath);
    }

    report.cacheHit = false;
    const NefPolyhedron lhs = makeNefFromPolygons(prepared.lhsPolygons, "lhs");
    const NefPolyhedron rhs = makeNefFromPolygons(prepared.rhsPolygons, "rhs");
    const NefPolyhedron oracle = applyBoolean(lhs, rhs, options.operation);
    saveNef(oracle, nefPath);
    writeMetadata(metadataPath, key, prepared, options.operation);
    return oracle;
}

ember::BoolProblem solveCandidate(const VerifyOptions &options, const PreparedProblem &prepared)
{
    ember::BoolProblem problem(options.leafThreshold);
    problem.setOperation(options.operation);
    problem.setOperandAssumptions(options.lhsAssumptions, options.rhsAssumptions);
    problem.setThreadCount(options.threadCount);
    problem.setOperands(prepared.lhsPolygons, prepared.rhsPolygons);
    problem.solve(prepared.sceneAABB);
    return problem;
}

void writeReport(const std::filesystem::path &path, const VerificationReport &report)
{
    if (path.empty())
        return;

    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open verification report: " + path.string());

    output << "passed=" << (report.passed ? 1 : 0) << '\n'
           << "cache_hit=" << (report.cacheHit ? 1 : 0) << '\n'
           << "oracle_key=" << report.oracleKey << '\n'
           << "oracle_path=" << report.oraclePath.string() << '\n'
           << "shared_scale=" << report.sharedScale << '\n'
           << "lhs_polygons=" << report.lhsPolygons << '\n'
           << "rhs_polygons=" << report.rhsPolygons << '\n'
           << "result_fragments=" << report.resultFragments << '\n'
           << std::fixed << std::setprecision(6)
           << "prepare_ms=" << report.prepareMs << '\n'
           << "solve_ms=" << report.solveMs << '\n'
           << "oracle_ms=" << report.oracleMs << '\n'
           << "compare_ms=" << report.compareMs << '\n';
}

std::string sanitizeReportValue(std::string value)
{
    for (char &ch : value)
    {
        if (ch == '\r' || ch == '\n')
            ch = ' ';
    }
    return value;
}

void writeFailureReport(const std::filesystem::path &path, const std::string &message)
{
    if (path.empty())
        return;

    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        return;

    output << "passed=0\n"
           << "cache_hit=0\n"
           << "error=" << sanitizeReportValue(message) << '\n';
}
}

int main(int argc, char **argv)
{
    VerifyOptions options;
    if (!parseArgs(argc, argv, options))
    {
        printUsage();
        return 1;
    }

    try
    {
        VerificationReport report;

        const Clock::time_point prepareStart = Clock::now();
        const PreparedProblem prepared = prepareProblem(options);
        const Clock::time_point prepareEnd = Clock::now();
        report.prepareMs = elapsedMilliseconds(prepareStart, prepareEnd);
        report.sharedScale = prepared.sharedScale;
        report.lhsPolygons = prepared.lhsPolygons.size();
        report.rhsPolygons = prepared.rhsPolygons.size();

        const Clock::time_point solveStart = Clock::now();
        ember::BoolProblem problem = solveCandidate(options, prepared);
        const Clock::time_point solveEnd = Clock::now();
        report.solveMs = elapsedMilliseconds(solveStart, solveEnd);
        report.resultFragments = problem.resultFragments().size();

        report.oracleKey = computeOracleKey(prepared, options.operation);
        const Clock::time_point oracleStart = Clock::now();
        const NefPolyhedron oracle = loadOrBuildOracle(options, prepared, report.oracleKey, report);
        const Clock::time_point oracleEnd = Clock::now();
        report.oracleMs = elapsedMilliseconds(oracleStart, oracleEnd);

        const Clock::time_point compareStart = Clock::now();
        const NefPolyhedron candidate = makeNefFromPolygons(problem.resultFragments(), "candidate");
        const bool equal = (candidate ^ oracle).regularization().is_empty();
        const Clock::time_point compareEnd = Clock::now();
        report.compareMs = elapsedMilliseconds(compareStart, compareEnd);
        report.passed = equal;

        writeReport(options.reportPath, report);

        std::cout << "verification=" << (report.passed ? "pass" : "fail")
                  << " operation=" << toString(options.operation)
                  << " scale=" << report.sharedScale
                  << " lhs_polygons=" << report.lhsPolygons
                  << " rhs_polygons=" << report.rhsPolygons
                  << " result_fragments=" << report.resultFragments
                  << " cache_hit=" << (report.cacheHit ? 1 : 0)
                  << " oracle_key=" << report.oracleKey
                  << " oracle_path=" << report.oraclePath.string()
                  << std::fixed << std::setprecision(6)
                  << " prepare_ms=" << report.prepareMs
                  << " solve_ms=" << report.solveMs
                  << " oracle_ms=" << report.oracleMs
                  << " compare_ms=" << report.compareMs
                  << std::endl;

        return report.passed ? 0 : 2;
    }
    catch (const std::exception &ex)
    {
        writeFailureReport(options.reportPath, ex.what());
        std::cerr << ex.what() << std::endl;
        return 1;
    }
}
