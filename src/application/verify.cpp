/**
 * @file verify.cpp
 * @brief 实现基于 CGAL Nef oracle 的 re-EMBER 结果校验工具。
 */
#include "core/bool_problem.h"
#include "geometry/geometry256.h"
#include "io/io.h"
#include "nef_postprocess.h"

#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/boost/graph/iterator.h>
#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
#include <CGAL/version.h>

#include <boost/uuid/detail/sha1.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <map>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
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
using NefPolyhedron = ember::app::NefPolyhedron;

constexpr const char *kOracleSchema = "re-EMBER-nef-oracle-v1";

enum class CandidateMode
{
    FragmentsNef,
    ExportConforming,
    ExportNef
};

enum class NefCompareOp
{
    Xor,
    CandidateMinusOracle,
    OracleMinusCandidate,
    Skip
};

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
    CandidateMode candidateMode = CandidateMode::FragmentsNef;
    std::filesystem::path oracleCacheDir = std::filesystem::path("build") / "oracle_cache" / "nef";
    bool refreshOracle = false;
    bool diagnoseNef = false;
    bool disableSurfaceCompare = false;
    NefCompareOp nefCompareOp = NefCompareOp::Xor;
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
    std::string candidateMode;
    std::string nefCompareOp;
    std::uint64_t sharedScale = 0;
    double prepareMs = 0.0;
    double solveMs = 0.0;
    double oracleMs = 0.0;
    double compareMs = 0.0;
    bool surfaceCompareUsed = false;
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
            << "[--candidate-mode fragments-nef|export-conforming|export-nef] "
            << "[--oracle-cache-dir <dir>] [--refresh-oracle] "
            << "[--diagnose-nef] [--nef-compare-op xor|candidate-minus-oracle|oracle-minus-candidate|skip] "
            << "[--disable-surface-compare] "
            << "[--report-out <file>]"
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

const char *toString(CandidateMode mode)
{
    switch (mode)
    {
    case CandidateMode::FragmentsNef:
        return "fragments-nef";
    case CandidateMode::ExportConforming:
        return "export-conforming";
    case CandidateMode::ExportNef:
        return "export-nef";
    }

    return "fragments-nef";
}

const char *toString(NefCompareOp op)
{
    switch (op)
    {
    case NefCompareOp::Xor:
        return "xor";
    case NefCompareOp::CandidateMinusOracle:
        return "candidate-minus-oracle";
    case NefCompareOp::OracleMinusCandidate:
        return "oracle-minus-candidate";
    case NefCompareOp::Skip:
        return "skip";
    }

    return "xor";
}

bool parseCandidateMode(const std::string &token, CandidateMode &outMode)
{
    if (token == "fragments-nef")
    {
        outMode = CandidateMode::FragmentsNef;
        return true;
    }
    if (token == "export-conforming")
    {
        outMode = CandidateMode::ExportConforming;
        return true;
    }
    if (token == "export-nef")
    {
        outMode = CandidateMode::ExportNef;
        return true;
    }

    return false;
}

bool parseNefCompareOp(const std::string &token, NefCompareOp &outOp)
{
    if (token == "xor")
    {
        outOp = NefCompareOp::Xor;
        return true;
    }
    if (token == "candidate-minus-oracle")
    {
        outOp = NefCompareOp::CandidateMinusOracle;
        return true;
    }
    if (token == "oracle-minus-candidate")
    {
        outOp = NefCompareOp::OracleMinusCandidate;
        return true;
    }
    if (token == "skip")
    {
        outOp = NefCompareOp::Skip;
        return true;
    }

    return false;
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
                arg == "--candidate-mode" || arg == "--oracle-cache-dir" ||
                arg == "--nef-compare-op" || arg == "--report-out")
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
            else if (arg == "--candidate-mode")
            {
                if (!parseCandidateMode(value, outOptions.candidateMode))
                {
                    std::cerr << "Unsupported candidate mode: " << value << std::endl;
                    return false;
                }
            }
            else if (arg == "--nef-compare-op")
            {
                if (!parseNefCompareOp(value, outOptions.nefCompareOp))
                {
                    std::cerr << "Unsupported Nef compare operation: " << value << std::endl;
                    return false;
                }
            }
            else if (arg == "--report-out")
                outOptions.reportPath = value;

            continue;
        }

        if (arg == "--refresh-oracle")
        {
            outOptions.refreshOracle = true;
            continue;
        }
        if (arg == "--diagnose-nef")
        {
            outOptions.diagnoseNef = true;
            continue;
        }
        if (arg == "--disable-surface-compare")
        {
            outOptions.disableSurfaceCompare = true;
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

ember::app::ExactKernel::FT parseExactKernelInteger(const std::string &text)
{
    bool negative = false;
    std::size_t offset = 0;
    if (!text.empty() && text[0] == '-')
    {
        negative = true;
        offset = 1;
    }

    ember::app::ExactKernel::FT value(0);
    for (std::size_t i = offset; i < text.size(); ++i)
    {
        if (text[i] < '0' || text[i] > '9')
            throw std::runtime_error("Invalid integer text while building Nef diagnostics.");
        value = value * 10 + static_cast<int>(text[i] - '0');
    }

    return negative ? -value : value;
}

ember::app::ExactKernel::FT toExactKernelFT(
    const ember::Integer &numerator,
    const ember::Integer &denominator)
{
    return parseExactKernelInteger(ember::integerToString(numerator)) /
           parseExactKernelInteger(ember::integerToString(denominator));
}

std::string homPointKey(const ember::HomPoint4i &point)
{
    const ember::HomPoint4i primitive = ember::primitiveHomPoint(point);
    return ember::integerToString(primitive.x) + "/" +
           ember::integerToString(primitive.y) + "/" +
           ember::integerToString(primitive.z) + "/" +
           ember::integerToString(primitive.w);
}

ember::app::ExactKernel::Point_3 toExactKernelPoint(const ember::PlanePoint3i &point)
{
    if (!point.hasUniqueIntersection() || ember::isZero(point.x.w))
        throw std::runtime_error("Nef diagnostics found a non-finite exact point.");

    return ember::app::ExactKernel::Point_3(
               toExactKernelFT(point.x.x, point.x.w),
               toExactKernelFT(point.x.y, point.x.w),
               toExactKernelFT(point.x.z, point.x.w));
}

struct IndexedExactMesh
{
    std::size_t sourceVertexCount = 0;
    std::vector<ember::app::ExactKernel::Point_3> points;
    std::vector<std::vector<std::size_t>> faces;
};

struct ExactMeshDiagnostics
{
    std::size_t sourceVertices = 0;
    std::size_t uniqueVertices = 0;
    std::size_t faces = 0;
    std::size_t shortFaces = 0;
    std::size_t facesWithDuplicateRefs = 0;
    std::size_t degenerateFaces = 0;
    std::size_t duplicateFaceVertexSets = 0;
    std::size_t uniqueEdges = 0;
    std::size_t boundaryEdges = 0;
    std::size_t nonmanifoldEdges = 0;
    std::size_t sameDirectionPairedEdges = 0;
    std::size_t edgesWithInteriorVertices = 0;
    std::size_t edgeInteriorVertexHits = 0;
};

std::string indexListKey(std::vector<std::size_t> indices)
{
    std::sort(indices.begin(), indices.end());
    std::ostringstream out;
    for (const std::size_t index : indices)
        out << index << ',';
    return out.str();
}

std::vector<std::size_t> cleanFace(const std::vector<std::size_t> &face)
{
    std::vector<std::size_t> cleaned;
    cleaned.reserve(face.size());
    for (const std::size_t index : face)
    {
        if (cleaned.empty() || cleaned.back() != index)
            cleaned.push_back(index);
    }
    if (cleaned.size() > 1u && cleaned.front() == cleaned.back())
        cleaned.pop_back();
    return cleaned;
}

bool isDegenerateTriangle(
    const std::vector<ember::app::ExactKernel::Point_3> &points,
    std::size_t a,
    std::size_t b,
    std::size_t c)
{
    return a == b || b == c || a == c ||
           CGAL::collinear(points[a], points[b], points[c]);
}

bool hasNondegenerateTriple(
    const std::vector<ember::app::ExactKernel::Point_3> &points,
    const std::vector<std::size_t> &face)
{
    if (face.size() < 3u)
        return false;
    for (std::size_t i = 0; i < face.size(); ++i)
    {
        const std::size_t a = face[(i + face.size() - 1u) % face.size()];
        const std::size_t b = face[i];
        const std::size_t c = face[(i + 1u) % face.size()];
        if (!isDegenerateTriangle(points, a, b, c))
            return true;
    }
    return false;
}

bool pointIsStrictlyOnSegment(
    const ember::app::ExactKernel::Point_3 &start,
    const ember::app::ExactKernel::Point_3 &point,
    const ember::app::ExactKernel::Point_3 &end)
{
    return point != start && point != end &&
           CGAL::collinear(start, point, end) &&
           CGAL::collinear_are_ordered_along_line(start, point, end);
}

IndexedExactMesh makeIndexedExactMesh(const ember::ExactMeshData &mesh)
{
    IndexedExactMesh indexed;
    indexed.sourceVertexCount = mesh.vertices.size();
    indexed.faces.reserve(mesh.faces.size());

    std::map<std::string, std::size_t> indexByPoint;
    std::vector<std::size_t> remap(mesh.vertices.size(), 0u);
    for (std::size_t vertexIndex = 0; vertexIndex < mesh.vertices.size(); ++vertexIndex)
    {
        const std::string key = homPointKey(mesh.vertices[vertexIndex].x);
        const auto found = indexByPoint.find(key);
        if (found != indexByPoint.end())
        {
            remap[vertexIndex] = found->second;
            continue;
        }

        const std::size_t pointIndex = indexed.points.size();
        indexByPoint.emplace(key, pointIndex);
        remap[vertexIndex] = pointIndex;
        indexed.points.push_back(toExactKernelPoint(mesh.vertices[vertexIndex]));
    }

    for (const std::vector<std::size_t> &sourceFace : mesh.faces)
    {
        std::vector<std::size_t> face;
        face.reserve(sourceFace.size());
        for (const std::size_t vertexIndex : sourceFace)
        {
            if (vertexIndex >= remap.size())
                throw std::runtime_error("Nef diagnostics found an out-of-range exact mesh index.");
            face.push_back(remap[vertexIndex]);
        }
        indexed.faces.push_back(std::move(face));
    }

    return indexed;
}

ExactMeshDiagnostics computeMeshDiagnostics(const IndexedExactMesh &mesh)
{
    ExactMeshDiagnostics diag;
    diag.sourceVertices = mesh.sourceVertexCount;
    diag.uniqueVertices = mesh.points.size();
    diag.faces = mesh.faces.size();

    std::map<std::pair<std::size_t, std::size_t>, std::size_t> undirectedEdges;
    std::map<std::pair<std::size_t, std::size_t>, std::size_t> directedEdges;
    std::map<std::string, std::size_t> faceVertexSets;

    for (const std::vector<std::size_t> &sourceFace : mesh.faces)
    {
        if (sourceFace.size() < 3u)
            ++diag.shortFaces;

        std::set<std::size_t> refs(sourceFace.begin(), sourceFace.end());
        if (refs.size() != sourceFace.size())
            ++diag.facesWithDuplicateRefs;

        const std::vector<std::size_t> face = cleanFace(sourceFace);
        if (!hasNondegenerateTriple(mesh.points, face))
        {
            ++diag.degenerateFaces;
            continue;
        }

        ++faceVertexSets[indexListKey(face)];
        for (std::size_t i = 0; i < face.size(); ++i)
        {
            const std::size_t a = face[i];
            const std::size_t b = face[(i + 1u) % face.size()];
            if (a == b)
                continue;

            ++directedEdges[{a, b}];
            ++undirectedEdges[{std::min(a, b), std::max(a, b)}];
        }
    }

    for (const auto &[key, count] : faceVertexSets)
    {
        (void)key;
        if (count > 1u)
            diag.duplicateFaceVertexSets += count - 1u;
    }

    diag.uniqueEdges = undirectedEdges.size();
    for (const auto &[edge, count] : undirectedEdges)
    {
        if (count == 1u)
            ++diag.boundaryEdges;
        else if (count > 2u)
            ++diag.nonmanifoldEdges;

        if (count == 2u)
        {
            const std::size_t forwardCount = directedEdges[{edge.first, edge.second}];
            const std::size_t reverseCount = directedEdges[{edge.second, edge.first}];
            if (forwardCount == 2u || reverseCount == 2u)
                ++diag.sameDirectionPairedEdges;
        }

        bool hasInteriorPoint = false;
        const auto &start = mesh.points[edge.first];
        const auto &end = mesh.points[edge.second];
        for (std::size_t pointIndex = 0; pointIndex < mesh.points.size(); ++pointIndex)
        {
            if (pointIndex == edge.first || pointIndex == edge.second)
                continue;
            if (pointIsStrictlyOnSegment(start, mesh.points[pointIndex], end))
            {
                hasInteriorPoint = true;
                ++diag.edgeInteriorVertexHits;
            }
        }
        if (hasInteriorPoint)
            ++diag.edgesWithInteriorVertices;
    }

    return diag;
}

void printMeshDiagnostics(const char *label, const ExactMeshDiagnostics &diag)
{
    std::cerr << "[nef-diagnose] " << label
              << " source_vertices=" << diag.sourceVertices
              << " unique_vertices=" << diag.uniqueVertices
              << " faces=" << diag.faces
              << " short_faces=" << diag.shortFaces
              << " duplicate_ref_faces=" << diag.facesWithDuplicateRefs
              << " degenerate_faces=" << diag.degenerateFaces
              << " duplicate_face_vertex_sets=" << diag.duplicateFaceVertexSets
              << " unique_edges=" << diag.uniqueEdges
              << " boundary_edges=" << diag.boundaryEdges
              << " nonmanifold_edges=" << diag.nonmanifoldEdges
              << " same_direction_paired_edges=" << diag.sameDirectionPairedEdges
              << " t_junction_edges=" << diag.edgesWithInteriorVertices
              << " t_junction_hits=" << diag.edgeInteriorVertexHits
              << std::endl;
}

void diagnosePolygonSoup(
    const char *label,
    const std::vector<ember::Polygon256> &polygons,
    ember::PolygonSoupTopologyMode topologyMode)
{
    ember::ExactMeshData mesh;
    std::string error;
    if (!ember::buildExactMeshFromPolygonSoup(polygons, mesh, error, topologyMode))
    {
        std::cerr << "[nef-diagnose] " << label << " build_failed=" << error << std::endl;
        return;
    }

    printMeshDiagnostics(label, computeMeshDiagnostics(makeIndexedExactMesh(mesh)));
}

IndexedExactMesh makeIndexedSurfaceMesh(const ember::app::SurfaceMesh &surfaceMesh)
{
    IndexedExactMesh indexed;
    indexed.sourceVertexCount = surfaceMesh.number_of_vertices();
    indexed.points.resize(surfaceMesh.number_of_vertices());
    for (const auto vertex : surfaceMesh.vertices())
        indexed.points[static_cast<std::size_t>(vertex.idx())] = surfaceMesh.point(vertex);

    indexed.faces.reserve(surfaceMesh.number_of_faces());
    for (const auto faceDescriptor : surfaceMesh.faces())
    {
        std::vector<std::size_t> face;
        for (const auto vertex : CGAL::vertices_around_face(surfaceMesh.halfedge(faceDescriptor), surfaceMesh))
            face.push_back(static_cast<std::size_t>(vertex.idx()));
        indexed.faces.push_back(std::move(face));
    }

    return indexed;
}

std::optional<IndexedExactMesh> extractSimpleNefSurface(const NefPolyhedron &nef)
{
    if (nef.is_empty() || !nef.is_simple())
        return std::nullopt;

    ember::app::SurfaceMesh surfaceMesh;
    NefPolyhedron copy = nef;
    CGAL::convert_nef_polyhedron_to_polygon_mesh(copy, surfaceMesh, false);
    return makeIndexedSurfaceMesh(surfaceMesh);
}

std::optional<IndexedExactMesh> diagnoseNef(const char *label, const NefPolyhedron &nef)
{
    std::cerr << "[nef-diagnose] " << label
              << " nef_empty=" << (nef.is_empty() ? 1 : 0)
              << " nef_simple=" << (nef.is_simple() ? 1 : 0)
              << std::endl;
    std::optional<IndexedExactMesh> indexed = extractSimpleNefSurface(nef);
    if (!indexed)
        return std::nullopt;

    printMeshDiagnostics((std::string(label) + "_surface").c_str(), computeMeshDiagnostics(*indexed));
    return indexed;
}

std::map<std::string, std::size_t> makeFaceVertexSetMultiset(
    const IndexedExactMesh &mesh,
    const std::vector<std::size_t> *remap)
{
    std::map<std::string, std::size_t> multiset;
    for (const std::vector<std::size_t> &sourceFace : mesh.faces)
    {
        std::vector<std::size_t> face = cleanFace(sourceFace);
        if (remap != nullptr)
        {
            for (std::size_t &index : face)
                index = (*remap)[index];
        }
        ++multiset[indexListKey(face)];
    }
    return multiset;
}

bool equivalentSurfaceMeshes(
    const IndexedExactMesh &candidate,
    const IndexedExactMesh &oracle,
    std::string &outReason)
{
    if (candidate.points.size() != oracle.points.size())
    {
        outReason = "different vertex count";
        return false;
    }
    if (candidate.faces.size() != oracle.faces.size())
    {
        outReason = "different face count";
        return false;
    }

    std::vector<std::size_t> candidateToOracle(candidate.points.size(), 0u);
    std::vector<bool> oracleUsed(oracle.points.size(), false);
    for (std::size_t candidateIndex = 0; candidateIndex < candidate.points.size(); ++candidateIndex)
    {
        bool found = false;
        for (std::size_t oracleIndex = 0; oracleIndex < oracle.points.size(); ++oracleIndex)
        {
            if (oracleUsed[oracleIndex])
                continue;
            if (candidate.points[candidateIndex] == oracle.points[oracleIndex])
            {
                candidateToOracle[candidateIndex] = oracleIndex;
                oracleUsed[oracleIndex] = true;
                found = true;
                break;
            }
        }
        if (!found)
        {
            outReason = "candidate vertex has no exact oracle match";
            return false;
        }
    }

    const std::map<std::string, std::size_t> candidateFaces =
        makeFaceVertexSetMultiset(candidate, &candidateToOracle);
    const std::map<std::string, std::size_t> oracleFaces =
        makeFaceVertexSetMultiset(oracle, nullptr);
    if (candidateFaces != oracleFaces)
    {
        outReason = "different exact face vertex sets";
        return false;
    }

    outReason = "exact surface vertex and face sets match";
    return true;
}

bool runNefCompare(
    const NefPolyhedron &candidate,
    const NefPolyhedron &oracle,
    NefCompareOp op)
{
    switch (op)
    {
    case NefCompareOp::Xor:
        return (candidate ^ oracle).regularization().is_empty();
    case NefCompareOp::CandidateMinusOracle:
        return (candidate - oracle).regularization().is_empty();
    case NefCompareOp::OracleMinusCandidate:
        return (oracle - candidate).regularization().is_empty();
    case NefCompareOp::Skip:
        return false;
    }

    return false;
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
    const NefPolyhedron lhs = ember::app::makeNefFromPolygons(prepared.lhsPolygons, "lhs");
    const NefPolyhedron rhs = ember::app::makeNefFromPolygons(prepared.rhsPolygons, "rhs");
    const NefPolyhedron oracle = ember::app::applyBoolean(lhs, rhs, options.operation);
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

NefPolyhedron buildCandidateNef(
    const VerifyOptions &options,
    const std::vector<ember::Polygon256> &fragments)
{
    switch (options.candidateMode)
    {
    case CandidateMode::FragmentsNef:
        return ember::app::makeNefFromPolygons(fragments, "candidate");
    case CandidateMode::ExportConforming:
    {
        ember::ExactMeshData exactMesh;
        std::string error;
        if (!ember::buildExactMeshFromPolygonSoup(
                    fragments,
                    exactMesh,
                    error,
                    ember::PolygonSoupTopologyMode::Conforming))
        {
            throw std::runtime_error("Failed to build conforming candidate mesh: " + error);
        }

        ember::app::NefBuildOptions options;
        options.refineEdgeInteriorPoints = false;
        return ember::app::makeNefFromExactMesh(exactMesh, "candidate", options);
    }
    case CandidateMode::ExportNef:
    {
        ember::ExactMeshData exactMesh;
        std::string error;
        if (!ember::buildExactMeshFromPolygonSoup(
                    fragments,
                    exactMesh,
                    error,
                    ember::PolygonSoupTopologyMode::Conforming))
        {
            throw std::runtime_error("Failed to build Nef output-topology candidate mesh: " + error);
        }

        ember::app::NefBuildOptions options;
        options.refineEdgeInteriorPoints = false;
        options.rejectEmptyRegularizedResult = !fragments.empty();
        return ember::app::makeNefFromExactMesh(exactMesh, "candidate", options);
    }
    }

    return ember::app::makeNefFromPolygons(fragments, "candidate");
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
           << "candidate_mode=" << report.candidateMode << '\n'
           << "nef_compare_op=" << report.nefCompareOp << '\n'
           << "surface_compare_used=" << (report.surfaceCompareUsed ? 1 : 0) << '\n'
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
        report.candidateMode = toString(options.candidateMode);
        report.nefCompareOp = toString(options.nefCompareOp);

        if (options.diagnoseNef)
        {
            diagnosePolygonSoup("lhs_raw", prepared.lhsPolygons, ember::PolygonSoupTopologyMode::Raw);
            diagnosePolygonSoup("rhs_raw", prepared.rhsPolygons, ember::PolygonSoupTopologyMode::Raw);
            diagnosePolygonSoup("candidate_raw", problem.resultFragments(), ember::PolygonSoupTopologyMode::Raw);
            diagnosePolygonSoup("candidate_conforming", problem.resultFragments(), ember::PolygonSoupTopologyMode::Conforming);
        }

        report.oracleKey = computeOracleKey(prepared, options.operation);
        const Clock::time_point oracleStart = Clock::now();
        const NefPolyhedron oracle = loadOrBuildOracle(options, prepared, report.oracleKey, report);
        const Clock::time_point oracleEnd = Clock::now();
        report.oracleMs = elapsedMilliseconds(oracleStart, oracleEnd);
        std::optional<IndexedExactMesh> oracleSurface;
        if (options.diagnoseNef)
            oracleSurface = diagnoseNef("oracle", oracle);
        else if (!options.disableSurfaceCompare)
            oracleSurface = extractSimpleNefSurface(oracle);

        const Clock::time_point compareStart = Clock::now();
        const NefPolyhedron candidate = buildCandidateNef(options, problem.resultFragments());
        std::optional<IndexedExactMesh> candidateSurface;
        if (options.diagnoseNef)
        {
            candidateSurface = diagnoseNef("candidate", candidate);
            if (candidateSurface && oracleSurface)
            {
                std::string surfaceReason;
                const bool surfacesEqual = equivalentSurfaceMeshes(*candidateSurface, *oracleSurface, surfaceReason);
                std::cerr << "[nef-diagnose] exact_surface_equal=" << (surfacesEqual ? 1 : 0)
                          << " reason=\"" << surfaceReason << "\"" << std::endl;
            }
            std::cerr << "[nef-diagnose] compare_begin op=" << toString(options.nefCompareOp) << std::endl;
        }
        else if (!options.disableSurfaceCompare)
        {
            candidateSurface = extractSimpleNefSurface(candidate);
        }

        bool equal = false;
        if (!options.disableSurfaceCompare && candidateSurface && oracleSurface)
        {
            std::string surfaceReason;
            if (equivalentSurfaceMeshes(*candidateSurface, *oracleSurface, surfaceReason))
            {
                equal = true;
                report.surfaceCompareUsed = true;
            }
        }
        if (!equal && options.nefCompareOp != NefCompareOp::Skip)
            equal = runNefCompare(candidate, oracle, options.nefCompareOp);
        if (options.diagnoseNef)
            std::cerr << "[nef-diagnose] compare_end op=" << toString(options.nefCompareOp)
                      << " empty=" << (equal ? 1 : 0) << std::endl;
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
                  << " candidate_mode=" << report.candidateMode
                  << " nef_compare_op=" << toString(options.nefCompareOp)
                  << " surface_compare_used=" << (report.surfaceCompareUsed ? 1 : 0)
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
