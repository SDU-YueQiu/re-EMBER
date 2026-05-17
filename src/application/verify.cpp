/**
 * @file verify.cpp
 * @brief 实现基于 CGAL Nef oracle 的 re-EMBER 结果校验工具。
 */
#include "core/bool_problem.h"
#include "geometry/geometry256.h"
#include "io/io.h"
#include "nef_postprocess.h"

#include <CGAL/IO/Nef_polyhedron_iostream_3.h>
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
using NefPolyhedron = ember::app::NefPolyhedron;

constexpr const char *kOracleSchema = "re-EMBER-nef-oracle-v1";

enum class CandidateMode
{
    FragmentsNef,
    ExportConforming,
    ExportNef
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
            << "[--candidate-mode fragments-nef|export-conforming|export-nef] "
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

bool parseArgs(int argc, char **argv, VerifyOptions &outOptions)
{
    outOptions = VerifyOptions();
    bool hasOperation = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--lhs" || arg == "--rhs" || arg == "--op" ||
                arg == "--scale" || arg == "--leaf-threshold" || arg == "--threads" ||
                arg == "--candidate-mode" || arg == "--oracle-cache-dir" || arg == "--report-out")
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

        report.oracleKey = computeOracleKey(prepared, options.operation);
        const Clock::time_point oracleStart = Clock::now();
        const NefPolyhedron oracle = loadOrBuildOracle(options, prepared, report.oracleKey, report);
        const Clock::time_point oracleEnd = Clock::now();
        report.oracleMs = elapsedMilliseconds(oracleStart, oracleEnd);

        const Clock::time_point compareStart = Clock::now();
        const NefPolyhedron candidate = buildCandidateNef(options, problem.resultFragments());
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
                  << " candidate_mode=" << report.candidateMode
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
