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
#include <CGAL/number_utils.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/version.h>

#include <boost/uuid/detail/sha1.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <future>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{
using ember::BoolOp;
using ember::BoolOperandAssumptions;
using Clock = std::chrono::steady_clock;
using NefPolyhedron = ember::app::NefPolyhedron;

constexpr const char *kOracleSchema = "re-EMBER-surface-oracle-v2";
constexpr const char *kCandidateCacheSchema = "re-EMBER-verify-candidate-cache-v1";

enum class CandidateMode
{
    FragmentsNef,
    ExportConforming,
    ExportNef
};

enum class NefCompareOp
{
    Xor,
    Equal,
    CandidateMinusOracle,
    OracleMinusCandidate,
    Skip
};

struct VerifyOptions
{
    std::string lhsPath;
    std::string rhsPath;
    BoolOp operation = BoolOp::Intersection;
    bool operationExplicit = false;
    std::optional<std::uint64_t> scale;
    std::size_t leafThreshold = ember::kDefaultLeafPolygonThreshold;
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
    std::filesystem::path diffOutPath;
    std::filesystem::path inputDumpPath;
    std::filesystem::path chunkDumpPath;
    std::filesystem::path fragmentDumpPath;
    std::filesystem::path batchInputRoot;
    std::filesystem::path batchManifest;
    std::filesystem::path batchOutDir;
    std::size_t batchSize = 0;
};

struct BatchWorkload
{
    std::string name;
    std::string lhsPath;
    std::string rhsPath;
    BoolOp operation = BoolOp::Intersection;
};

struct CandidateCacheData
{
    BatchWorkload workload;
    CandidateMode candidateMode = CandidateMode::FragmentsNef;
    std::optional<std::uint64_t> explicitScale;
    std::string oracleKey;
    std::uint64_t sharedScale = 0;
    std::size_t lhsPolygons = 0;
    std::size_t rhsPolygons = 0;
    std::size_t resultFragments = 0;
    double prepareMs = 0.0;
    double solveMs = 0.0;
    ember::ExactMeshData candidateMesh;
};

struct BatchVerificationRow
{
    std::string workload;
    bool passed = false;
    std::string error;
    std::filesystem::path cachePath;
    std::filesystem::path reportPath;
    std::string oracleKey;
    std::filesystem::path oraclePath;
    bool cacheHit = false;
    std::uint64_t sharedScale = 0;
    std::size_t lhsPolygons = 0;
    std::size_t rhsPolygons = 0;
    std::size_t resultFragments = 0;
    double prepareMs = 0.0;
    double solveMs = 0.0;
    double oracleMs = 0.0;
    double compareMs = 0.0;
    bool surfaceCompareUsed = false;
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
            << "[--threads <nonnegative_integer>] "
            << "[--assume-lhs-nsi] [--assume-lhs-nnc] "
            << "[--assume-rhs-nsi] [--assume-rhs-nnc] "
            << "[--candidate-mode fragments-nef|export-conforming|export-nef] "
            << "[--oracle-cache-dir <dir>] [--refresh-oracle] "
            << "[--diagnose-nef] [--nef-compare-op xor|equal|candidate-minus-oracle|oracle-minus-candidate|skip] "
            << "[--disable-surface-compare] "
            << "[--report-out <file>] [--diff-out <file.obj|file.stl>] "
            << "[--input-dump-out <file.csv>] "
            << "[--chunk-dump-out <file.csv>] [--fragment-dump-out <file.csv>]"
            << "\n       re-EMBER_verify (--batch-input-root <dir>|--batch-manifest <csv>) "
            << "--batch-out-dir <dir> [--op union|intersection|difference] "
            << "[--batch-size <positive_integer>] [single-workload options except dump/diff outputs]"
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

bool parseNonNegativeUInt64(const std::string &token, std::uint64_t &outValue)
{
    try
    {
        std::size_t consumed = 0;
        const unsigned long long parsed = std::stoull(token, &consumed, 10);
        if (consumed != token.size())
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
    case NefCompareOp::Equal:
        return "equal";
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
    if (token == "equal")
    {
        outOp = NefCompareOp::Equal;
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
                arg == "--nef-compare-op" || arg == "--report-out" || arg == "--diff-out" ||
                arg == "--input-dump-out" || arg == "--chunk-dump-out" || arg == "--fragment-dump-out" ||
                arg == "--batch-input-root" || arg == "--batch-manifest" ||
                arg == "--batch-out-dir" || arg == "--batch-size")
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
                outOptions.operationExplicit = true;
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
                if (!parseNonNegativeUInt64(value, threadCountValue))
                {
                    std::cerr << "Thread count must be 0 or a positive integer." << std::endl;
                    return false;
                }
                outOptions.threadCount = static_cast<std::size_t>(threadCountValue);
            }
            else if (arg == "--batch-size")
            {
                std::uint64_t batchSizeValue = 0;
                if (!parsePositiveUInt64(value, batchSizeValue))
                {
                    std::cerr << "Batch size must be a positive integer." << std::endl;
                    return false;
                }
                outOptions.batchSize = static_cast<std::size_t>(batchSizeValue);
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
            else if (arg == "--diff-out")
                outOptions.diffOutPath = value;
            else if (arg == "--input-dump-out")
                outOptions.inputDumpPath = value;
            else if (arg == "--chunk-dump-out")
                outOptions.chunkDumpPath = value;
            else if (arg == "--fragment-dump-out")
                outOptions.fragmentDumpPath = value;
            else if (arg == "--batch-input-root")
                outOptions.batchInputRoot = value;
            else if (arg == "--batch-manifest")
                outOptions.batchManifest = value;
            else if (arg == "--batch-out-dir")
                outOptions.batchOutDir = value;

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

    const bool isBatchMode = !outOptions.batchInputRoot.empty() || !outOptions.batchManifest.empty();
    if (isBatchMode)
    {
        if (!outOptions.lhsPath.empty() || !outOptions.rhsPath.empty())
        {
            std::cerr << "Batch mode cannot be combined with --lhs/--rhs." << std::endl;
            return false;
        }
        if (!outOptions.reportPath.empty() || !outOptions.diffOutPath.empty() ||
                !outOptions.inputDumpPath.empty() || !outOptions.chunkDumpPath.empty() ||
                !outOptions.fragmentDumpPath.empty())
        {
            std::cerr << "Batch mode writes reports under --batch-out-dir and cannot use single-workload output paths." << std::endl;
            return false;
        }
        if (!outOptions.batchInputRoot.empty() && !outOptions.batchManifest.empty())
        {
            std::cerr << "Use only one of --batch-input-root or --batch-manifest." << std::endl;
            return false;
        }
        if (outOptions.batchOutDir.empty())
        {
            std::cerr << "Batch mode requires --batch-out-dir." << std::endl;
            return false;
        }
        if (!outOptions.batchInputRoot.empty() && !hasOperation)
        {
            std::cerr << "--batch-input-root requires a global --op." << std::endl;
            return false;
        }
    }
    else if (outOptions.lhsPath.empty() || outOptions.rhsPath.empty() || !hasOperation)
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

std::size_t hardwareThreadCount()
{
    const unsigned int count = std::thread::hardware_concurrency();
    return count == 0 ? 1u : static_cast<std::size_t>(count);
}

std::string trim(std::string value)
{
    auto isSpace = [](unsigned char ch)
    {
        return std::isspace(ch) != 0;
    };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [&](char ch)
    {
        return !isSpace(static_cast<unsigned char>(ch));
    }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [&](char ch)
    {
        return !isSpace(static_cast<unsigned char>(ch));
    }).base(), value.end());
    return value;
}

std::vector<std::string> parseCsvLine(const std::string &line)
{
    std::vector<std::string> fields;
    std::string field;
    bool inQuotes = false;
    for (std::size_t i = 0; i < line.size(); ++i)
    {
        const char ch = line[i];
        if (inQuotes)
        {
            if (ch == '"' && i + 1u < line.size() && line[i + 1u] == '"')
            {
                field.push_back('"');
                ++i;
            }
            else if (ch == '"')
            {
                inQuotes = false;
            }
            else
            {
                field.push_back(ch);
            }
            continue;
        }

        if (ch == '"')
        {
            inQuotes = true;
        }
        else if (ch == ',')
        {
            fields.push_back(trim(field));
            field.clear();
        }
        else
        {
            field.push_back(ch);
        }
    }
    fields.push_back(trim(field));
    return fields;
}

std::string sanitizeFileStem(const std::string &name)
{
    std::string result;
    result.reserve(name.size());
    for (char ch : name)
    {
        const unsigned char byte = static_cast<unsigned char>(ch);
        if (std::isalnum(byte) || ch == '-' || ch == '_' || ch == '.')
            result.push_back(ch);
        else
            result.push_back('_');
    }
    return result.empty() ? std::string("workload") : result;
}

std::string csvEscape(const std::string &value)
{
    bool needsQuotes = value.find_first_of(",\"\r\n") != std::string::npos;
    if (!needsQuotes)
        return value;

    std::string escaped;
    escaped.reserve(value.size() + 2u);
    escaped.push_back('"');
    for (char ch : value)
    {
        if (ch == '"')
            escaped.push_back('"');
        escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

std::filesystem::path resolveExistingPath(const std::filesystem::path &base, const std::string &pathText)
{
    std::filesystem::path path(pathText);
    if (path.is_relative())
        path = base / path;
    return std::filesystem::weakly_canonical(path);
}

std::optional<std::size_t> findColumn(
    const std::map<std::string, std::size_t> &columns,
    const std::string &primary,
    const std::string &fallback = std::string())
{
    const auto primaryIt = columns.find(primary);
    if (primaryIt != columns.end())
        return primaryIt->second;
    if (!fallback.empty())
    {
        const auto fallbackIt = columns.find(fallback);
        if (fallbackIt != columns.end())
            return fallbackIt->second;
    }
    return std::nullopt;
}

std::string fieldAt(const std::vector<std::string> &fields, std::size_t index)
{
    return index < fields.size() ? fields[index] : std::string();
}

std::filesystem::path resolveBatchMeshPath(const std::filesystem::path &caseDir, const char *stem)
{
    std::vector<std::filesystem::path> matches;
    for (const char *extension : {".obj", ".stl"})
    {
        const std::filesystem::path candidate = caseDir / (std::string(stem) + extension);
        if (std::filesystem::exists(candidate))
            matches.push_back(std::filesystem::weakly_canonical(candidate));
    }

    if (matches.empty())
        throw std::runtime_error("Batch case is missing " + std::string(stem) + ".obj or " + stem + ".stl: " + caseDir.string());
    if (matches.size() > 1u)
        throw std::runtime_error("Batch case contains multiple " + std::string(stem) + " inputs: " + caseDir.string());
    return matches.front();
}

std::vector<BatchWorkload> loadBatchInputRootWorkloads(const VerifyOptions &options)
{
    std::vector<BatchWorkload> workloads;
    const std::filesystem::path root = std::filesystem::weakly_canonical(options.batchInputRoot);
    if (!std::filesystem::is_directory(root))
        throw std::runtime_error("Batch input root is not a directory: " + root.string());

    std::vector<std::filesystem::path> caseDirs;
    for (const std::filesystem::directory_entry &entry : std::filesystem::directory_iterator(root))
    {
        if (entry.is_directory())
            caseDirs.push_back(entry.path());
    }
    std::sort(caseDirs.begin(), caseDirs.end());
    if (caseDirs.empty())
        throw std::runtime_error("Batch input root contains no case subdirectories: " + root.string());

    for (const std::filesystem::path &caseDir : caseDirs)
    {
        BatchWorkload workload;
        workload.name = caseDir.filename().string();
        workload.lhsPath = resolveBatchMeshPath(caseDir, "lhs").string();
        workload.rhsPath = resolveBatchMeshPath(caseDir, "rhs").string();
        workload.operation = options.operation;
        workloads.push_back(std::move(workload));
    }
    return workloads;
}

std::vector<BatchWorkload> loadBatchManifestWorkloads(const VerifyOptions &options)
{
    std::vector<BatchWorkload> workloads;
    const std::filesystem::path manifestPath = std::filesystem::weakly_canonical(options.batchManifest);
    std::ifstream input(manifestPath);
    if (!input)
        throw std::runtime_error("Failed to open batch manifest: " + manifestPath.string());

    std::string headerLine;
    if (!std::getline(input, headerLine))
        throw std::runtime_error("Batch manifest is empty: " + manifestPath.string());
    if (!headerLine.empty() && headerLine.back() == '\r')
        headerLine.pop_back();

    const std::vector<std::string> headers = parseCsvLine(headerLine);
    std::map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < headers.size(); ++i)
        columns.emplace(headers[i], i);

    const std::optional<std::size_t> nameColumn = findColumn(columns, "name", "pair_id");
    const std::optional<std::size_t> lhsColumn = findColumn(columns, "lhs", "lhs_path");
    const std::optional<std::size_t> rhsColumn = findColumn(columns, "rhs", "rhs_path");
    const std::optional<std::size_t> opColumn = findColumn(columns, "op", "operation");
    if (!nameColumn || !lhsColumn || !rhsColumn)
        throw std::runtime_error("Batch manifest must contain name,lhs,rhs or pair_id,lhs_path,rhs_path columns.");
    if (!opColumn && !options.operationExplicit)
        throw std::runtime_error("Batch manifest must contain op/operation or be used with a global --op.");

    const std::filesystem::path base = manifestPath.parent_path();
    std::string line;
    std::size_t rowNumber = 1;
    while (std::getline(input, line))
    {
        ++rowNumber;
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (trim(line).empty())
            continue;

        const std::vector<std::string> fields = parseCsvLine(line);
        BatchWorkload workload;
        workload.name = fieldAt(fields, *nameColumn);
        workload.lhsPath = resolveExistingPath(base, fieldAt(fields, *lhsColumn)).string();
        workload.rhsPath = resolveExistingPath(base, fieldAt(fields, *rhsColumn)).string();
        if (workload.name.empty() || workload.lhsPath.empty() || workload.rhsPath.empty())
            throw std::runtime_error("Batch manifest row has an empty name/lhs/rhs field at line " + std::to_string(rowNumber));

        std::string opText = opColumn ? fieldAt(fields, *opColumn) : std::string();
        if (opText.empty())
            workload.operation = options.operation;
        else if (!parseBoolOp(opText, workload.operation))
            throw std::runtime_error("Unsupported boolean operation in batch manifest at line " + std::to_string(rowNumber) + ": " + opText);

        workloads.push_back(std::move(workload));
    }

    if (workloads.empty())
        throw std::runtime_error("Batch manifest contains no workloads: " + manifestPath.string());
    return workloads;
}

std::vector<BatchWorkload> loadBatchWorkloads(const VerifyOptions &options)
{
    if (!options.batchInputRoot.empty())
        return loadBatchInputRootWorkloads(options);
    return loadBatchManifestWorkloads(options);
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
                prepared.lhsMesh,
                prepared.rhsMesh,
                quantizeOptions,
                prepared.sharedScale,
                error))
        throw std::runtime_error(error);

    ember::PolygonSoupBuildOptions buildOptions;
    buildOptions.triangulateNonCoplanarFaces = true;

    ember::AABB3i lhsAABB;
    ember::AABB3i rhsAABB;
    if (!ember::buildPolygonSoupWithAABB(
                prepared.lhsMesh,
                prepared.sharedScale,
                buildOptions,
                lhsAABB,
                prepared.lhsPolygons,
                error))
        throw std::runtime_error(error);
    if (!ember::buildPolygonSoupWithAABB(
                prepared.rhsMesh,
                prepared.sharedScale,
                buildOptions,
                rhsAABB,
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

ember::app::ExactKernel::FT parseExactKernelFTToken(const std::string &text)
{
    const std::size_t slash = text.find('/');
    if (slash == std::string::npos)
        return parseExactKernelInteger(text);

    const std::string numerator = text.substr(0, slash);
    const std::string denominator = text.substr(slash + 1u);
    if (denominator.empty())
        throw std::runtime_error("Invalid rational text in oracle surface cache.");

    const ember::app::ExactKernel::FT den = parseExactKernelInteger(denominator);
    if (den == ember::app::ExactKernel::FT(0))
        throw std::runtime_error("Zero denominator in oracle surface cache.");
    return parseExactKernelInteger(numerator) / den;
}

std::string exactKernelFTToString(const ember::app::ExactKernel::FT &value)
{
    std::ostringstream output;
    output << CGAL::exact(value);
    return output.str();
}

void writeExactKernelPoint(std::ostream &output, const ember::app::ExactKernel::Point_3 &point)
{
    output << exactKernelFTToString(point.x()) << ' '
           << exactKernelFTToString(point.y()) << ' '
           << exactKernelFTToString(point.z());
}

ember::app::ExactKernel::Point_3 readExactKernelPoint(std::istream &input)
{
    std::string x;
    std::string y;
    std::string z;
    if (!(input >> x >> y >> z))
        throw std::runtime_error("Oracle surface cache ended while reading a point.");
    return ember::app::ExactKernel::Point_3(
               parseExactKernelFTToken(x),
               parseExactKernelFTToken(y),
               parseExactKernelFTToken(z));
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

std::string rationalCoordinateKey(ember::Integer numerator, ember::Integer denominator)
{
    if (ember::isZero(denominator))
        throw std::runtime_error("Cannot build an exact point key for a point at infinity.");
    if (ember::isZero(numerator))
        return "0";
    if (denominator < 0)
    {
        numerator = -numerator;
        denominator = -denominator;
    }

    const ember::Integer divisor = ember::gcdMagnitude(numerator, denominator);
    numerator /= divisor;
    denominator /= divisor;
    if (denominator == ember::Integer(1))
        return ember::integerToString(numerator);
    return ember::integerToString(numerator) + "/" + ember::integerToString(denominator);
}

std::string homPointCoordinateKey(const ember::HomPoint4i &point)
{
    const ember::HomPoint4i primitive = ember::primitiveHomPoint(point);
    return rationalCoordinateKey(primitive.x, primitive.w) + "|" +
           rationalCoordinateKey(primitive.y, primitive.w) + "|" +
           rationalCoordinateKey(primitive.z, primitive.w);
}

ember::Integer parseIntegerToken(const std::string &text)
{
    if (text.empty())
        throw std::runtime_error("Candidate cache contains an empty integer token.");

    bool negative = false;
    std::size_t offset = 0;
    if (text[0] == '-')
    {
        negative = true;
        offset = 1;
    }
    if (offset == text.size())
        throw std::runtime_error("Candidate cache contains an invalid integer token.");

    ember::Integer value = 0;
    for (std::size_t i = offset; i < text.size(); ++i)
    {
        if (text[i] < '0' || text[i] > '9')
            throw std::runtime_error("Candidate cache contains an invalid integer token: " + text);
        value = value * ember::Integer(10) + ember::Integer(text[i] - '0');
    }
    return negative ? -value : value;
}

void writeIntegerToken(std::ostream &output, const ember::Integer &value)
{
    output << ember::integerToString(value);
}

void writePlaneToken(std::ostream &output, const ember::Plane3i &plane)
{
    writeIntegerToken(output, plane.a);
    output << ' ';
    writeIntegerToken(output, plane.b);
    output << ' ';
    writeIntegerToken(output, plane.c);
    output << ' ';
    writeIntegerToken(output, plane.d);
}

void writeHomPointToken(std::ostream &output, const ember::HomPoint4i &point)
{
    writeIntegerToken(output, point.x);
    output << ' ';
    writeIntegerToken(output, point.y);
    output << ' ';
    writeIntegerToken(output, point.z);
    output << ' ';
    writeIntegerToken(output, point.w);
}

ember::Plane3i readPlaneToken(std::istream &input)
{
    std::string a;
    std::string b;
    std::string c;
    std::string d;
    if (!(input >> a >> b >> c >> d))
        throw std::runtime_error("Candidate cache ended while reading a plane.");
    return ember::Plane3i(parseIntegerToken(a), parseIntegerToken(b), parseIntegerToken(c), parseIntegerToken(d));
}

ember::HomPoint4i readHomPointToken(std::istream &input)
{
    std::string x;
    std::string y;
    std::string z;
    std::string w;
    if (!(input >> x >> y >> z >> w))
        throw std::runtime_error("Candidate cache ended while reading a homogeneous point.");
    return ember::HomPoint4i(parseIntegerToken(x), parseIntegerToken(y), parseIntegerToken(z), parseIntegerToken(w));
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
    std::vector<std::string> pointKeys;
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

std::string faceCycleKeyForRotation(
    const std::vector<std::size_t> &face,
    std::size_t start,
    int step)
{
    std::ostringstream out;
    const std::size_t n = face.size();
    std::size_t index = start;
    for (std::size_t i = 0; i < n; ++i)
    {
        out << face[index] << ',';
        index = step > 0 ? (index + 1u) % n : (index + n - 1u) % n;
    }
    return out.str();
}

std::string canonicalFaceCycleKey(const std::vector<std::size_t> &face)
{
    if (face.empty())
        return std::string();

    std::string best = faceCycleKeyForRotation(face, 0u, 1);
    for (std::size_t start = 0; start < face.size(); ++start)
    {
        best = std::min(best, faceCycleKeyForRotation(face, start, 1));
        best = std::min(best, faceCycleKeyForRotation(face, start, -1));
    }
    return best;
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
        indexed.pointKeys.push_back(homPointCoordinateKey(mesh.vertices[vertexIndex].x));
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

std::string exactPointKey(const ember::app::ExactKernel::Point_3 &point);

IndexedExactMesh makeIndexedSurfaceMesh(const ember::app::SurfaceMesh &surfaceMesh)
{
    IndexedExactMesh indexed;
    indexed.sourceVertexCount = surfaceMesh.number_of_vertices();
    indexed.points.resize(surfaceMesh.number_of_vertices());
    indexed.pointKeys.resize(surfaceMesh.number_of_vertices());
    for (const auto vertex : surfaceMesh.vertices())
    {
        const std::size_t index = static_cast<std::size_t>(vertex.idx());
        indexed.points[index] = surfaceMesh.point(vertex);
        indexed.pointKeys[index] = exactPointKey(indexed.points[index]);
    }

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

IndexedExactMesh extractNefSurfaceOrEmpty(const NefPolyhedron &nef)
{
    if (nef.is_empty())
        return IndexedExactMesh();

    std::optional<IndexedExactMesh> surface = extractSimpleNefSurface(nef);
    if (!surface)
        throw std::runtime_error("CGAL Nef oracle is non-empty but not simple; cannot cache an exact surface.");
    return *surface;
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

std::map<std::string, std::size_t> makeFaceCycleMultiset(
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
        ++multiset[canonicalFaceCycleKey(face)];
    }
    return multiset;
}

int compareExactPoints(
    const ember::app::ExactKernel::Point_3 &lhs,
    const ember::app::ExactKernel::Point_3 &rhs)
{
    if (lhs.x() < rhs.x())
        return -1;
    if (rhs.x() < lhs.x())
        return 1;
    if (lhs.y() < rhs.y())
        return -1;
    if (rhs.y() < lhs.y())
        return 1;
    if (lhs.z() < rhs.z())
        return -1;
    if (rhs.z() < lhs.z())
        return 1;
    return 0;
}

struct PointKeyIndex
{
    std::string key;
    std::size_t index = 0;
};

std::string exactPointKey(const ember::app::ExactKernel::Point_3 &point)
{
    return exactKernelFTToString(point.x()) + "|" +
           exactKernelFTToString(point.y()) + "|" +
           exactKernelFTToString(point.z());
}

std::vector<PointKeyIndex> sortedPointKeyIndices(const IndexedExactMesh &mesh)
{
    std::vector<PointKeyIndex> entries;
    entries.reserve(mesh.points.size());
    for (std::size_t i = 0; i < mesh.points.size(); ++i)
        entries.push_back(PointKeyIndex{exactPointKey(mesh.points[i]), i});
    std::sort(entries.begin(), entries.end(), [](const PointKeyIndex &lhs, const PointKeyIndex &rhs)
    {
        if (lhs.key != rhs.key)
            return lhs.key < rhs.key;
        return lhs.index < rhs.index;
    });
    return entries;
}

bool buildCandidateToOraclePointMap(
    const IndexedExactMesh &candidate,
    const IndexedExactMesh &oracle,
    std::vector<std::size_t> &outCandidateToOracle,
    std::string &outReason)
{
    outCandidateToOracle.assign(candidate.points.size(), 0u);
    if (candidate.pointKeys.size() != candidate.points.size() ||
            oracle.pointKeys.size() != oracle.points.size())
    {
        outReason = "surface mesh is missing exact point keys";
        return false;
    }

    std::unordered_map<std::string, std::vector<std::size_t>> oracleBuckets;
    oracleBuckets.reserve(oracle.points.size() * 2u + 1u);
    for (std::size_t oracleIndex = 0; oracleIndex < oracle.points.size(); ++oracleIndex)
        oracleBuckets[oracle.pointKeys[oracleIndex]].push_back(oracleIndex);

    std::vector<bool> oracleUsed(oracle.points.size(), false);
    for (std::size_t candidateIndex = 0; candidateIndex < candidate.points.size(); ++candidateIndex)
    {
        const auto bucket = oracleBuckets.find(candidate.pointKeys[candidateIndex]);
        if (bucket == oracleBuckets.end())
        {
            outReason = "candidate vertex has no exact oracle key";
            return false;
        }

        bool found = false;
        for (std::size_t oracleIndex : bucket->second)
        {
            if (oracleUsed[oracleIndex])
                continue;
            if (candidate.pointKeys[candidateIndex] == oracle.pointKeys[oracleIndex])
            {
                outCandidateToOracle[candidateIndex] = oracleIndex;
                oracleUsed[oracleIndex] = true;
                found = true;
                break;
            }
        }
        if (!found)
        {
            outReason = "candidate vertex has no unused exact oracle key match";
            return false;
        }
    }

    if (std::find(oracleUsed.begin(), oracleUsed.end(), false) != oracleUsed.end())
    {
        outReason = "oracle vertex has no exact candidate match";
        return false;
    }
    return true;
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

    std::vector<std::size_t> candidateToOracle;
    if (!buildCandidateToOraclePointMap(candidate, oracle, candidateToOracle, outReason))
        return false;

    const std::map<std::string, std::size_t> candidateFaces =
        makeFaceCycleMultiset(candidate, &candidateToOracle);
    const std::map<std::string, std::size_t> oracleFaces =
        makeFaceCycleMultiset(oracle, nullptr);
    if (candidateFaces != oracleFaces)
    {
        outReason = "different exact face cycles";
        return false;
    }

    outReason = "exact surface vertices and face cycles match";
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
    case NefCompareOp::Equal:
        return candidate == oracle;
    case NefCompareOp::CandidateMinusOracle:
        return (candidate - oracle).regularization().is_empty();
    case NefCompareOp::OracleMinusCandidate:
        return (oracle - candidate).regularization().is_empty();
    case NefCompareOp::Skip:
        return false;
    }

    return false;
}

NefPolyhedron buildNefDifferenceForOp(
    const NefPolyhedron &candidate,
    const NefPolyhedron &oracle,
    NefCompareOp op)
{
    switch (op)
    {
    case NefCompareOp::Xor:
        return (candidate ^ oracle).regularization();
    case NefCompareOp::Equal:
        return (candidate ^ oracle).regularization();
    case NefCompareOp::CandidateMinusOracle:
        return (candidate - oracle).regularization();
    case NefCompareOp::OracleMinusCandidate:
        return (oracle - candidate).regularization();
    case NefCompareOp::Skip:
        return NefPolyhedron(NefPolyhedron::EMPTY);
    }

    return NefPolyhedron(NefPolyhedron::EMPTY);
}

void writeNefDifferenceMesh(
    const std::filesystem::path &path,
    const NefPolyhedron &candidate,
    const NefPolyhedron &oracle,
    NefCompareOp op,
    std::uint64_t coordinateScale)
{
    if (path.empty())
        return;

    const NefPolyhedron diff = buildNefDifferenceForOp(candidate, oracle, op);
    const ember::ObjMeshData mesh = ember::app::makeObjMeshData(diff, coordinateScale);
    std::size_t faceCount = 0;
    std::string error;
    if (!ember::writeMesh(mesh, path.string(), faceCount, error))
        throw std::runtime_error("Failed to write verifier Nef diff mesh: " + error);
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

std::string readExpectedKey(std::istream &input, const char *expected);
std::string readStringValue(std::istream &input, const char *key);
std::string readTokenValue(std::istream &input, const char *key);
std::size_t readSizeValue(std::istream &input, const char *key);

void replaceFile(const std::filesystem::path &tmpPath, const std::filesystem::path &path)
{
    std::error_code ec;
    std::filesystem::rename(tmpPath, path, ec);
    if (ec)
    {
        std::filesystem::remove(path, ec);
        ec.clear();
        std::filesystem::rename(tmpPath, path, ec);
        if (ec)
            throw std::runtime_error("Failed to replace cache file: " + path.string());
    }
}

void saveIndexedSurface(
    const IndexedExactMesh &surface,
    const std::filesystem::path &path,
    const std::string &key,
    const PreparedProblem &prepared,
    BoolOp operation)
{
    std::filesystem::create_directories(path.parent_path());
    const std::filesystem::path tmpPath = path.string() + ".tmp";
    {
        std::ofstream output(tmpPath, std::ios::trunc);
        if (!output)
            throw std::runtime_error("Failed to open oracle surface cache for writing: " + tmpPath.string());

        output << "schema " << kOracleSchema << '\n'
               << "key " << key << '\n'
               << "cgal_version " << std::quoted(std::string(CGAL_VERSION_STR)) << '\n'
               << "operation " << toString(operation) << '\n'
               << "shared_scale " << prepared.sharedScale << '\n'
               << "lhs_polygons " << prepared.lhsPolygons.size() << '\n'
               << "rhs_polygons " << prepared.rhsPolygons.size() << '\n'
               << "source_vertices " << surface.sourceVertexCount << '\n'
               << "points " << surface.points.size() << '\n';
        if (surface.pointKeys.size() != surface.points.size())
            throw std::runtime_error("Oracle surface cache received points without exact keys.");
        for (std::size_t pointIndex = 0; pointIndex < surface.points.size(); ++pointIndex)
        {
            output << "p " << std::quoted(surface.pointKeys[pointIndex]) << ' ';
            writeExactKernelPoint(output, surface.points[pointIndex]);
            output << '\n';
        }

        output << "faces " << surface.faces.size() << '\n';
        for (const std::vector<std::size_t> &face : surface.faces)
        {
            output << "f " << face.size();
            for (std::size_t index : face)
                output << ' ' << index;
            output << '\n';
        }
        if (!output)
            throw std::runtime_error("Failed to write oracle surface cache: " + tmpPath.string());
    }
    replaceFile(tmpPath, path);
}

IndexedExactMesh loadIndexedSurface(const std::filesystem::path &path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Failed to open oracle surface cache for reading: " + path.string());

    const std::string schema = readTokenValue(input, "schema");
    if (schema != kOracleSchema)
        throw std::runtime_error("Unsupported oracle surface cache schema: " + schema);
    (void)readTokenValue(input, "key");
    (void)readStringValue(input, "cgal_version");
    (void)readTokenValue(input, "operation");
    (void)readTokenValue(input, "shared_scale");
    (void)readSizeValue(input, "lhs_polygons");
    (void)readSizeValue(input, "rhs_polygons");

    IndexedExactMesh surface;
    surface.sourceVertexCount = readSizeValue(input, "source_vertices");
    const std::size_t pointCount = readSizeValue(input, "points");
    surface.points.reserve(pointCount);
    surface.pointKeys.reserve(pointCount);
    for (std::size_t i = 0; i < pointCount; ++i)
    {
        readExpectedKey(input, "p");
        std::string key;
        if (!(input >> std::quoted(key)))
            throw std::runtime_error("Oracle surface cache ended while reading a point key.");
        surface.pointKeys.push_back(std::move(key));
        surface.points.push_back(readExactKernelPoint(input));
    }

    const std::size_t faceCount = readSizeValue(input, "faces");
    surface.faces.reserve(faceCount);
    for (std::size_t i = 0; i < faceCount; ++i)
    {
        readExpectedKey(input, "f");
        std::size_t faceSize = 0;
        if (!(input >> faceSize))
            throw std::runtime_error("Oracle surface cache ended while reading face size.");
        std::vector<std::size_t> face;
        face.reserve(faceSize);
        for (std::size_t j = 0; j < faceSize; ++j)
        {
            std::size_t index = 0;
            if (!(input >> index))
                throw std::runtime_error("Oracle surface cache ended while reading face index.");
            if (index >= pointCount)
                throw std::runtime_error("Oracle surface cache face index is out of range.");
            face.push_back(index);
        }
        surface.faces.push_back(std::move(face));
    }
    return surface;
}

NefPolyhedron buildNefFromIndexedSurface(const IndexedExactMesh &surface, const char *label)
{
    if (surface.faces.empty())
        return NefPolyhedron(NefPolyhedron::EMPTY);

    ember::app::SurfaceMesh surfaceMesh;
    std::vector<ember::app::SurfaceMesh::Vertex_index> vertices;
    vertices.reserve(surface.points.size());
    for (const ember::app::ExactKernel::Point_3 &point : surface.points)
        vertices.push_back(surfaceMesh.add_vertex(point));

    for (const std::vector<std::size_t> &sourceFace : surface.faces)
    {
        if (sourceFace.size() < 3u)
            throw std::runtime_error(std::string("Oracle surface cache contains a short face for ") + label + ".");
        std::vector<ember::app::SurfaceMesh::Vertex_index> face;
        face.reserve(sourceFace.size());
        for (std::size_t index : sourceFace)
        {
            if (index >= vertices.size())
                throw std::runtime_error(std::string("Oracle surface cache contains an out-of-range face index for ") + label + ".");
            face.push_back(vertices[index]);
        }
        if (surfaceMesh.add_face(face) == ember::app::SurfaceMesh::null_face())
            throw std::runtime_error(std::string("Failed to rebuild oracle surface face for ") + label + ".");
    }

    CGAL::Polygon_mesh_processing::triangulate_faces(surfaceMesh);
    return NefPolyhedron(surfaceMesh).regularization();
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

std::shared_ptr<std::mutex> oracleMutexForKey(const std::string &key)
{
    static std::mutex mutexMapMutex;
    static std::map<std::string, std::weak_ptr<std::mutex>> mutexMap;

    std::lock_guard<std::mutex> guard(mutexMapMutex);
    std::shared_ptr<std::mutex> keyMutex = mutexMap[key].lock();
    if (!keyMutex)
    {
        keyMutex = std::make_shared<std::mutex>();
        mutexMap[key] = keyMutex;
    }
    return keyMutex;
}

bool shouldRefreshOracleForKey(const std::string &key, bool requested)
{
    if (!requested)
        return false;

    static std::mutex refreshedKeysMutex;
    static std::set<std::string> refreshedKeys;

    std::lock_guard<std::mutex> guard(refreshedKeysMutex);
    const auto inserted = refreshedKeys.insert(key);
    return inserted.second;
}

IndexedExactMesh loadOrBuildOracleSurface(
    const VerifyOptions &options,
    const PreparedProblem &prepared,
    const std::string &key,
    VerificationReport &report)
{
    const std::shared_ptr<std::mutex> keyMutex = oracleMutexForKey(key);
    std::lock_guard<std::mutex> keyGuard(*keyMutex);

    std::filesystem::create_directories(options.oracleCacheDir);
    const std::filesystem::path surfacePath = options.oracleCacheDir / (key + ".surface.txt");
    const std::filesystem::path metadataPath = options.oracleCacheDir / (key + ".json");
    report.oraclePath = surfacePath;

    const bool refreshThisKey = shouldRefreshOracleForKey(key, options.refreshOracle);
    if (!refreshThisKey && std::filesystem::exists(surfacePath))
    {
        report.cacheHit = true;
        return loadIndexedSurface(surfacePath);
    }

    report.cacheHit = false;
    const NefPolyhedron lhs = ember::app::makeNefFromPolygons(prepared.lhsPolygons, "lhs");
    const NefPolyhedron rhs = ember::app::makeNefFromPolygons(prepared.rhsPolygons, "rhs");
    const NefPolyhedron oracle = ember::app::applyBoolean(lhs, rhs, options.operation);
    const IndexedExactMesh surface = extractNefSurfaceOrEmpty(oracle);
    saveIndexedSurface(surface, surfacePath, key, prepared, options.operation);
    writeMetadata(metadataPath, key, prepared, options.operation);
    return surface;
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

ember::ExactMeshData buildCandidateExactMesh(
    const VerifyOptions &options,
    const std::vector<ember::Polygon256> &fragments)
{
    ember::ExactMeshData exactMesh;
    std::string error;
    switch (options.candidateMode)
    {
    case CandidateMode::FragmentsNef:
        if (!ember::buildExactMeshFromPolygonSoup(
                    fragments,
                    exactMesh,
                    error,
                    ember::PolygonSoupTopologyMode::Conforming))
        {
            throw std::runtime_error("Failed to build conforming fragments-nef candidate mesh: " + error);
        }
        return exactMesh;
    case CandidateMode::ExportConforming:
        if (!ember::buildExactMeshFromPolygonSoup(
                    fragments,
                    exactMesh,
                    error,
                    ember::PolygonSoupTopologyMode::Conforming))
        {
            throw std::runtime_error("Failed to build conforming candidate mesh: " + error);
        }
        return exactMesh;
    case CandidateMode::ExportNef:
        if (!ember::buildExactMeshFromPolygonSoup(
                    fragments,
                    exactMesh,
                    error,
                    ember::PolygonSoupTopologyMode::Conforming))
        {
            throw std::runtime_error("Failed to build Nef output-topology candidate mesh: " + error);
        }
        return exactMesh;
    }

    return exactMesh;
}

NefPolyhedron buildCandidateNefFromExactMesh(
    const VerifyOptions &options,
    const ember::ExactMeshData &exactMesh,
    std::size_t sourceFragmentCount)
{
    ember::app::NefBuildOptions nefOptions;
    nefOptions.refineEdgeInteriorPoints = false;
    if (options.candidateMode == CandidateMode::ExportNef)
        nefOptions.rejectEmptyRegularizedResult = sourceFragmentCount != 0u;
    return ember::app::makeNefFromExactMesh(exactMesh, "candidate", nefOptions);
}

NefPolyhedron buildCandidateNef(
    const VerifyOptions &options,
    const std::vector<ember::Polygon256> &fragments)
{
    const ember::ExactMeshData exactMesh = buildCandidateExactMesh(options, fragments);
    return buildCandidateNefFromExactMesh(options, exactMesh, fragments.size());
}

void writeCandidateCache(const std::filesystem::path &path, const CandidateCacheData &cache)
{
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());

    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open candidate cache for writing: " + path.string());

    output << "schema " << kCandidateCacheSchema << '\n'
           << "name " << std::quoted(cache.workload.name) << '\n'
           << "lhs " << std::quoted(cache.workload.lhsPath) << '\n'
           << "rhs " << std::quoted(cache.workload.rhsPath) << '\n'
           << "op " << toString(cache.workload.operation) << '\n'
           << "explicit_scale " << (cache.explicitScale ? std::to_string(*cache.explicitScale) : std::string("auto")) << '\n'
           << "shared_scale " << cache.sharedScale << '\n'
           << "candidate_mode " << toString(cache.candidateMode) << '\n'
           << "oracle_key " << cache.oracleKey << '\n'
           << "lhs_polygons " << cache.lhsPolygons << '\n'
           << "rhs_polygons " << cache.rhsPolygons << '\n'
           << "result_fragments " << cache.resultFragments << '\n'
           << std::fixed << std::setprecision(6)
           << "prepare_ms " << cache.prepareMs << '\n'
           << "solve_ms " << cache.solveMs << '\n'
           << "vertices " << cache.candidateMesh.vertices.size() << '\n';

    for (const ember::PlanePoint3i &vertex : cache.candidateMesh.vertices)
    {
        output << "v ";
        writePlaneToken(output, vertex.p);
        output << ' ';
        writePlaneToken(output, vertex.q);
        output << ' ';
        writePlaneToken(output, vertex.r);
        output << ' ';
        writeHomPointToken(output, vertex.x);
        output << '\n';
    }

    output << "faces " << cache.candidateMesh.faces.size() << '\n';
    for (const std::vector<std::size_t> &face : cache.candidateMesh.faces)
    {
        output << "f " << face.size();
        for (std::size_t index : face)
            output << ' ' << index;
        output << '\n';
    }
}

std::string readExpectedKey(std::istream &input, const char *expected)
{
    std::string key;
    if (!(input >> key))
        throw std::runtime_error("Candidate cache ended before key: " + std::string(expected));
    if (key != expected)
        throw std::runtime_error("Candidate cache expected key '" + std::string(expected) + "' but found '" + key + "'.");
    return key;
}

std::string readStringValue(std::istream &input, const char *key)
{
    readExpectedKey(input, key);
    std::string value;
    if (!(input >> std::quoted(value)))
        throw std::runtime_error("Candidate cache ended while reading string key: " + std::string(key));
    return value;
}

std::string readTokenValue(std::istream &input, const char *key)
{
    readExpectedKey(input, key);
    std::string value;
    if (!(input >> value))
        throw std::runtime_error("Candidate cache ended while reading key: " + std::string(key));
    return value;
}

std::size_t readSizeValue(std::istream &input, const char *key)
{
    const std::string value = readTokenValue(input, key);
    std::uint64_t parsed = 0;
    if (!parseNonNegativeUInt64(value, parsed))
        throw std::runtime_error("Candidate cache has invalid size for key: " + std::string(key));
    return static_cast<std::size_t>(parsed);
}

double readDoubleValue(std::istream &input, const char *key)
{
    const std::string value = readTokenValue(input, key);
    try
    {
        std::size_t consumed = 0;
        const double parsed = std::stod(value, &consumed);
        if (consumed != value.size())
            throw std::runtime_error("");
        return parsed;
    }
    catch (...)
    {
        throw std::runtime_error("Candidate cache has invalid floating value for key: " + std::string(key));
    }
}

CandidateCacheData readCandidateCache(const std::filesystem::path &path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("Failed to open candidate cache for reading: " + path.string());

    CandidateCacheData cache;
    const std::string schema = readTokenValue(input, "schema");
    if (schema != kCandidateCacheSchema)
        throw std::runtime_error("Unsupported candidate cache schema: " + schema);

    cache.workload.name = readStringValue(input, "name");
    cache.workload.lhsPath = readStringValue(input, "lhs");
    cache.workload.rhsPath = readStringValue(input, "rhs");
    const std::string opText = readTokenValue(input, "op");
    if (!parseBoolOp(opText, cache.workload.operation))
        throw std::runtime_error("Candidate cache has unsupported operation: " + opText);

    const std::string explicitScaleText = readTokenValue(input, "explicit_scale");
    if (explicitScaleText != "auto")
    {
        std::uint64_t scale = 0;
        if (!parsePositiveUInt64(explicitScaleText, scale))
            throw std::runtime_error("Candidate cache has invalid explicit scale.");
        cache.explicitScale = scale;
    }

    std::uint64_t sharedScale = 0;
    if (!parsePositiveUInt64(readTokenValue(input, "shared_scale"), sharedScale))
        throw std::runtime_error("Candidate cache has invalid shared scale.");
    cache.sharedScale = sharedScale;

    const std::string candidateModeText = readTokenValue(input, "candidate_mode");
    if (!parseCandidateMode(candidateModeText, cache.candidateMode))
        throw std::runtime_error("Candidate cache has unsupported candidate mode: " + candidateModeText);
    cache.oracleKey = readTokenValue(input, "oracle_key");
    cache.lhsPolygons = readSizeValue(input, "lhs_polygons");
    cache.rhsPolygons = readSizeValue(input, "rhs_polygons");
    cache.resultFragments = readSizeValue(input, "result_fragments");
    cache.prepareMs = readDoubleValue(input, "prepare_ms");
    cache.solveMs = readDoubleValue(input, "solve_ms");

    const std::size_t vertexCount = readSizeValue(input, "vertices");
    cache.candidateMesh.vertices.reserve(vertexCount);
    for (std::size_t i = 0; i < vertexCount; ++i)
    {
        readExpectedKey(input, "v");
        const ember::Plane3i p = readPlaneToken(input);
        const ember::Plane3i q = readPlaneToken(input);
        const ember::Plane3i r = readPlaneToken(input);
        const ember::HomPoint4i x = readHomPointToken(input);
        cache.candidateMesh.vertices.emplace_back(p, q, r, x);
    }

    const std::size_t faceCount = readSizeValue(input, "faces");
    cache.candidateMesh.faces.reserve(faceCount);
    for (std::size_t i = 0; i < faceCount; ++i)
    {
        readExpectedKey(input, "f");
        std::size_t faceSize = 0;
        if (!(input >> faceSize))
            throw std::runtime_error("Candidate cache ended while reading face size.");
        std::vector<std::size_t> face;
        face.reserve(faceSize);
        for (std::size_t j = 0; j < faceSize; ++j)
        {
            std::size_t index = 0;
            if (!(input >> index))
                throw std::runtime_error("Candidate cache ended while reading face indices.");
            if (index >= vertexCount)
                throw std::runtime_error("Candidate cache face index is out of range.");
            face.push_back(index);
        }
        cache.candidateMesh.faces.push_back(std::move(face));
    }

    return cache;
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

void writeInputPolygonDiagnostics(
    const std::filesystem::path &path,
    const std::vector<ember::Polygon256> &lhsPolygons,
    const std::vector<ember::Polygon256> &rhsPolygons)
{
    if (path.empty())
        return;

    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open input polygon diagnostics: " + path.string());

    output << "operand,polygon_index,wntv0,wntv1,edge_count,valid,vertices,"
           << "plane_a,plane_b,plane_c,plane_d,"
           << "aabb_x_min,aabb_x_max,aabb_y_min,aabb_y_max,aabb_z_min,aabb_z_max\n";

    auto writePolygons = [&](const char *operand, const std::vector<ember::Polygon256> &polygons)
    {
        for (std::size_t polygonIndex = 0; polygonIndex < polygons.size(); ++polygonIndex)
        {
            const ember::Polygon256 &polygon = polygons[polygonIndex];
            const ember::AABB3i &box = polygon.aabb();
            std::string vertices;
            const std::vector<ember::PlanePoint3i> &polygonVertices = polygon.vertices();
            for (std::size_t vertexIndex = 0; vertexIndex < polygonVertices.size(); ++vertexIndex)
            {
                if (vertexIndex != 0)
                    vertices.push_back(';');
                vertices += homPointKey(polygonVertices[vertexIndex].x);
            }
            output << operand << ','
                   << polygonIndex << ','
                   << (polygon.WNTV.size() > 0 ? polygon.WNTV[0] : 0) << ','
                   << (polygon.WNTV.size() > 1 ? polygon.WNTV[1] : 0) << ','
                   << polygon.edgeCount() << ','
                   << (polygon.isValid() ? 1 : 0) << ','
                   << vertices << ','
                   << ember::integerToString(polygon.plane.a) << ','
                   << ember::integerToString(polygon.plane.b) << ','
                   << ember::integerToString(polygon.plane.c) << ','
                   << ember::integerToString(polygon.plane.d) << ','
                   << ember::integerToString(box.xMin) << ','
                   << ember::integerToString(box.xMax) << ','
                   << ember::integerToString(box.yMin) << ','
                   << ember::integerToString(box.yMax) << ','
                   << ember::integerToString(box.zMin) << ','
                   << ember::integerToString(box.zMax) << '\n';
        }
    };

    writePolygons("lhs", lhsPolygons);
    writePolygons("rhs", rhsPolygons);
}

void writeResultChunkDiagnostics(const std::filesystem::path &path, const ember::BoolProblem &problem)
{
    if (path.empty())
        return;

    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open chunk diagnostics: " + path.string());

    const std::vector<std::vector<ember::Polygon256>> &chunks = problem.resultFragmentChunks();
    const std::vector<ember::BoolLeafSummary> &leaves = problem.leafSummaries();

    output << "leaf_index,chunk_index,first_fragment,chunk_fragment_count,leaf_result_fragment_count,"
           << "invalid_fragment_count,"
           << "depth,polygon_count,discarded,"
           << "aabb_x_min,aabb_x_max,aabb_y_min,aabb_y_max,aabb_z_min,aabb_z_max\n";

    std::size_t chunkIndex = 0;
    std::size_t firstFragment = 0;
    for (std::size_t leafIndex = 0; leafIndex < leaves.size(); ++leafIndex)
    {
        const ember::BoolLeafSummary &leaf = leaves[leafIndex];
        const bool hasChunk = leaf.resultFragmentCount > 0 && chunkIndex < chunks.size();
        const std::size_t chunkFragmentCount = hasChunk ? chunks[chunkIndex].size() : 0;
        std::size_t invalidFragmentCount = 0;
        if (hasChunk)
        {
            for (const ember::Polygon256 &fragment : chunks[chunkIndex])
            {
                if (!fragment.isValid())
                    ++invalidFragmentCount;
            }
        }

        output << leafIndex << ','
               << (hasChunk ? std::to_string(chunkIndex) : std::string()) << ','
               << (hasChunk ? std::to_string(firstFragment) : std::string()) << ','
               << chunkFragmentCount << ','
               << leaf.resultFragmentCount << ','
               << invalidFragmentCount << ','
               << leaf.depth << ','
               << leaf.polygonCount << ','
               << (leaf.discarded ? 1 : 0) << ','
               << ember::integerToString(leaf.aabb.xMin) << ','
               << ember::integerToString(leaf.aabb.xMax) << ','
               << ember::integerToString(leaf.aabb.yMin) << ','
               << ember::integerToString(leaf.aabb.yMax) << ','
               << ember::integerToString(leaf.aabb.zMin) << ','
               << ember::integerToString(leaf.aabb.zMax) << '\n';

        if (hasChunk)
        {
            firstFragment += chunkFragmentCount;
            ++chunkIndex;
        }
    }

    while (chunkIndex < chunks.size())
    {
        const std::size_t chunkFragmentCount = chunks[chunkIndex].size();
        output << ","
               << chunkIndex << ','
               << firstFragment << ','
               << chunkFragmentCount
               << ",0,0,,,,,,,,\n";
        firstFragment += chunkFragmentCount;
        ++chunkIndex;
    }
}

void writeResultFragmentDiagnostics(const std::filesystem::path &path, const ember::BoolProblem &problem)
{
    if (path.empty())
        return;

    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open fragment diagnostics: " + path.string());

    output << "fragment_index,chunk_index,chunk_offset,wntv0,wntv1,edge_count,valid,"
           << "plane_a,plane_b,plane_c,plane_d,"
           << "aabb_x_min,aabb_x_max,aabb_y_min,aabb_y_max,aabb_z_min,aabb_z_max\n";

    const std::vector<std::vector<ember::Polygon256>> &chunks = problem.resultFragmentChunks();
    std::size_t fragmentIndex = 0;
    for (std::size_t chunkIndex = 0; chunkIndex < chunks.size(); ++chunkIndex)
    {
        const std::vector<ember::Polygon256> &chunk = chunks[chunkIndex];
        for (std::size_t chunkOffset = 0; chunkOffset < chunk.size(); ++chunkOffset)
        {
            const ember::Polygon256 &fragment = chunk[chunkOffset];
            const ember::AABB3i &box = fragment.aabb();
            output << fragmentIndex << ','
                   << chunkIndex << ','
                   << chunkOffset << ','
                   << (fragment.WNTV.size() > 0 ? fragment.WNTV[0] : 0) << ','
                   << (fragment.WNTV.size() > 1 ? fragment.WNTV[1] : 0) << ','
                   << fragment.edgeCount() << ','
                   << (fragment.isValid() ? 1 : 0) << ','
                   << ember::integerToString(fragment.plane.a) << ','
                   << ember::integerToString(fragment.plane.b) << ','
                   << ember::integerToString(fragment.plane.c) << ','
                   << ember::integerToString(fragment.plane.d) << ','
                   << ember::integerToString(box.xMin) << ','
                   << ember::integerToString(box.xMax) << ','
                   << ember::integerToString(box.yMin) << ','
                   << ember::integerToString(box.yMax) << ','
                   << ember::integerToString(box.zMin) << ','
                   << ember::integerToString(box.zMax) << '\n';
            ++fragmentIndex;
        }
    }
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

VerifyOptions optionsForWorkload(const VerifyOptions &baseOptions, const BatchWorkload &workload)
{
    VerifyOptions options = baseOptions;
    options.lhsPath = workload.lhsPath;
    options.rhsPath = workload.rhsPath;
    options.operation = workload.operation;
    options.reportPath.clear();
    options.diffOutPath.clear();
    options.inputDumpPath.clear();
    options.chunkDumpPath.clear();
    options.fragmentDumpPath.clear();
    return options;
}

CandidateCacheData solveAndWriteCandidateCache(
    const VerifyOptions &baseOptions,
    const BatchWorkload &workload,
    const std::filesystem::path &cachePath)
{
    const VerifyOptions options = optionsForWorkload(baseOptions, workload);
    CandidateCacheData cache;
    cache.workload = workload;
    cache.candidateMode = options.candidateMode;
    cache.explicitScale = options.scale;

    const Clock::time_point prepareStart = Clock::now();
    const PreparedProblem prepared = prepareProblem(options);
    const Clock::time_point prepareEnd = Clock::now();
    cache.prepareMs = elapsedMilliseconds(prepareStart, prepareEnd);
    cache.sharedScale = prepared.sharedScale;
    cache.lhsPolygons = prepared.lhsPolygons.size();
    cache.rhsPolygons = prepared.rhsPolygons.size();
    cache.oracleKey = computeOracleKey(prepared, options.operation);

    const Clock::time_point solveStart = Clock::now();
    ember::BoolProblem problem = solveCandidate(options, prepared);
    const Clock::time_point solveEnd = Clock::now();
    cache.solveMs = elapsedMilliseconds(solveStart, solveEnd);
    cache.resultFragments = problem.resultFragmentCount();
    cache.candidateMesh = buildCandidateExactMesh(options, problem.resultFragments());

    writeCandidateCache(cachePath, cache);
    return cache;
}

void validateCandidateCacheAgainstPrepared(
    const CandidateCacheData &cache,
    const VerifyOptions &options,
    const PreparedProblem &prepared,
    const std::string &oracleKey)
{
    if (cache.workload.operation != options.operation)
        throw std::runtime_error("Candidate cache operation does not match workload operation.");
    if (cache.candidateMode != options.candidateMode)
        throw std::runtime_error("Candidate cache candidate mode does not match current options.");
    if (cache.explicitScale != options.scale)
        throw std::runtime_error("Candidate cache explicit scale does not match current options.");
    if (cache.sharedScale != prepared.sharedScale)
        throw std::runtime_error("Candidate cache shared scale does not match freshly prepared input.");
    if (cache.lhsPolygons != prepared.lhsPolygons.size() || cache.rhsPolygons != prepared.rhsPolygons.size())
        throw std::runtime_error("Candidate cache input polygon counts do not match freshly prepared input.");
    if (cache.oracleKey != oracleKey)
        throw std::runtime_error("Candidate cache oracle key does not match freshly prepared input.");
}

bool compareCandidateExactMesh(
    const VerifyOptions &options,
    const PreparedProblem &prepared,
    const ember::ExactMeshData &candidateMesh,
    std::size_t resultFragmentCount,
    VerificationReport &report)
{
    report.oracleKey = computeOracleKey(prepared, options.operation);
    const Clock::time_point oracleStart = Clock::now();
    const IndexedExactMesh oracleSurface = loadOrBuildOracleSurface(options, prepared, report.oracleKey, report);
    const Clock::time_point oracleEnd = Clock::now();
    report.oracleMs = elapsedMilliseconds(oracleStart, oracleEnd);
    if (options.diagnoseNef)
        printMeshDiagnostics("oracle_surface", computeMeshDiagnostics(oracleSurface));

    const Clock::time_point compareStart = Clock::now();
    bool equal = false;

    std::optional<NefPolyhedron> candidate;
    std::optional<NefPolyhedron> oracle;
    if (!options.disableSurfaceCompare)
    {
        candidate.emplace(buildCandidateNefFromExactMesh(options, candidateMesh, resultFragmentCount));
        const IndexedExactMesh regularizedCandidateSurface = extractNefSurfaceOrEmpty(*candidate);
        std::string surfaceReason;
        if (equivalentSurfaceMeshes(regularizedCandidateSurface, oracleSurface, surfaceReason))
        {
            equal = true;
            report.surfaceCompareUsed = true;
        }
        if (options.diagnoseNef)
            std::cerr << "[nef-diagnose] regularized_exact_surface_equal=" << (equal ? 1 : 0)
                      << " reason=\"" << surfaceReason << "\"" << std::endl;
    }
    if (!equal && options.nefCompareOp != NefCompareOp::Skip)
    {
        if (!candidate)
            candidate.emplace(buildCandidateNefFromExactMesh(options, candidateMesh, resultFragmentCount));
        if (options.diagnoseNef)
        {
            (void)diagnoseNef("candidate", *candidate);
            std::cerr << "[nef-diagnose] compare_begin op=" << toString(options.nefCompareOp) << std::endl;
        }
        oracle.emplace(buildNefFromIndexedSurface(oracleSurface, "oracle"));
        equal = runNefCompare(*candidate, *oracle, options.nefCompareOp);
    }
    if (!options.diffOutPath.empty())
    {
        if (!candidate)
            candidate.emplace(buildCandidateNefFromExactMesh(options, candidateMesh, resultFragmentCount));
        if (!oracle)
            oracle.emplace(buildNefFromIndexedSurface(oracleSurface, "oracle"));
        writeNefDifferenceMesh(
            options.diffOutPath,
            *candidate,
            *oracle,
            options.nefCompareOp,
            prepared.sharedScale);
    }
    if (options.diagnoseNef)
        std::cerr << "[nef-diagnose] compare_end op=" << toString(options.nefCompareOp)
                  << " empty=" << (equal ? 1 : 0) << std::endl;
    const Clock::time_point compareEnd = Clock::now();
    report.compareMs = elapsedMilliseconds(compareStart, compareEnd);
    report.passed = equal;
    return equal;
}

BatchVerificationRow compareCachedCandidate(
    const VerifyOptions &baseOptions,
    const std::filesystem::path &cachePath,
    const std::filesystem::path &reportPath)
{
    const CandidateCacheData cache = readCandidateCache(cachePath);
    const VerifyOptions options = optionsForWorkload(baseOptions, cache.workload);

    BatchVerificationRow row;
    row.workload = cache.workload.name;
    row.cachePath = cachePath;
    row.reportPath = reportPath;

    VerificationReport report;
    report.prepareMs = cache.prepareMs;
    report.solveMs = cache.solveMs;
    report.sharedScale = cache.sharedScale;
    report.lhsPolygons = cache.lhsPolygons;
    report.rhsPolygons = cache.rhsPolygons;
    report.resultFragments = cache.resultFragments;
    report.candidateMode = toString(cache.candidateMode);
    report.nefCompareOp = toString(options.nefCompareOp);

    if (options.nefCompareOp == NefCompareOp::Skip)
        throw std::runtime_error("Batch verifier cannot pass with --nef-compare-op skip.");

    const PreparedProblem prepared = prepareProblem(options);
    const std::string oracleKey = computeOracleKey(prepared, options.operation);
    validateCandidateCacheAgainstPrepared(cache, options, prepared, oracleKey);

    compareCandidateExactMesh(options, prepared, cache.candidateMesh, cache.resultFragments, report);
    writeReport(reportPath, report);

    row.passed = report.passed;
    row.oracleKey = report.oracleKey;
    row.oraclePath = report.oraclePath;
    row.cacheHit = report.cacheHit;
    row.sharedScale = report.sharedScale;
    row.lhsPolygons = report.lhsPolygons;
    row.rhsPolygons = report.rhsPolygons;
    row.resultFragments = report.resultFragments;
    row.prepareMs = report.prepareMs;
    row.solveMs = report.solveMs;
    row.oracleMs = report.oracleMs;
    row.compareMs = report.compareMs;
    row.surfaceCompareUsed = report.surfaceCompareUsed;
    return row;
}

BatchVerificationRow failureBatchRow(
    const BatchWorkload &workload,
    const std::filesystem::path &cachePath,
    const std::filesystem::path &reportPath,
    const std::string &message)
{
    writeFailureReport(reportPath, message);
    BatchVerificationRow row;
    row.workload = workload.name;
    row.passed = false;
    row.error = message;
    row.cachePath = cachePath;
    row.reportPath = reportPath;
    return row;
}

void writeBatchVerificationCsv(
    const std::filesystem::path &path,
    const std::vector<BatchVerificationRow> &rows)
{
    if (!path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open batch verification CSV: " + path.string());

    output << "workload,passed,error,cache_path,report_path,oracle_key,oracle_path,cache_hit,"
           << "shared_scale,lhs_polygons,rhs_polygons,result_fragments,prepare_ms,solve_ms,"
           << "oracle_ms,compare_ms,surface_compare_used\n";
    output << std::fixed << std::setprecision(6);
    for (const BatchVerificationRow &row : rows)
    {
        output << csvEscape(row.workload) << ','
               << (row.passed ? 1 : 0) << ','
               << csvEscape(row.error) << ','
               << csvEscape(row.cachePath.string()) << ','
               << csvEscape(row.reportPath.string()) << ','
               << csvEscape(row.oracleKey) << ','
               << csvEscape(row.oraclePath.string()) << ','
               << (row.cacheHit ? 1 : 0) << ','
               << row.sharedScale << ','
               << row.lhsPolygons << ','
               << row.rhsPolygons << ','
               << row.resultFragments << ','
               << row.prepareMs << ','
               << row.solveMs << ','
               << row.oracleMs << ','
               << row.compareMs << ','
               << (row.surfaceCompareUsed ? 1 : 0) << '\n';
    }
}

void writeBatchReport(
    const std::filesystem::path &path,
    const std::vector<BatchVerificationRow> &rows,
    std::size_t batchSize,
    std::size_t cpuThreads)
{
    std::ofstream output(path, std::ios::trunc);
    if (!output)
        throw std::runtime_error("Failed to open batch report: " + path.string());

    std::size_t passed = 0;
    for (const BatchVerificationRow &row : rows)
    {
        if (row.passed)
            ++passed;
    }

    output << "schema=re-EMBER-verify-batch-report-v1\n"
           << "workload_count=" << rows.size() << '\n'
           << "passed=" << passed << '\n'
           << "failed=" << (rows.size() - passed) << '\n'
           << "batch_size=" << batchSize << '\n'
           << "cpu_threads=" << cpuThreads << '\n'
           << "verification_csv=" << (path.parent_path() / "verification.csv").string() << '\n';

    for (const BatchVerificationRow &row : rows)
    {
        output << "workload=" << row.workload
               << " passed=" << (row.passed ? 1 : 0)
               << " oracle_key=" << row.oracleKey
               << " error=" << sanitizeReportValue(row.error)
               << " report=" << row.reportPath.string()
               << '\n';
    }
}

int runBatch(const VerifyOptions &options)
{
    const std::size_t cpuThreads = hardwareThreadCount();
    const std::size_t batchSize = options.batchSize == 0 ? cpuThreads : options.batchSize;
    if (batchSize == 0 || batchSize > cpuThreads)
        throw std::runtime_error("--batch-size must be in the range 1.." + std::to_string(cpuThreads) + ".");

    const std::vector<BatchWorkload> workloads = loadBatchWorkloads(options);
    std::filesystem::create_directories(options.batchOutDir);
    const std::filesystem::path cacheDir = options.batchOutDir / "cache";
    const std::filesystem::path reportsDir = options.batchOutDir / "reports";
    std::filesystem::create_directories(cacheDir);
    std::filesystem::create_directories(reportsDir);

    std::vector<BatchVerificationRow> rows;
    rows.reserve(workloads.size());
    for (std::size_t batchStart = 0; batchStart < workloads.size(); batchStart += batchSize)
    {
        const std::size_t batchEnd = std::min(batchStart + batchSize, workloads.size());
        std::vector<std::pair<BatchWorkload, std::filesystem::path>> cachedWorkloads;
        cachedWorkloads.reserve(batchEnd - batchStart);

        for (std::size_t index = batchStart; index < batchEnd; ++index)
        {
            const BatchWorkload &workload = workloads[index];
            const std::string stem = sanitizeFileStem(workload.name);
            const std::filesystem::path cachePath = cacheDir / (stem + ".candidate.txt");
            const std::filesystem::path reportPath = reportsDir / (stem + ".report.txt");
            std::cout << "[batch] solve_begin index=" << index << " workload=" << workload.name << std::endl;
            try
            {
                solveAndWriteCandidateCache(options, workload, cachePath);
                cachedWorkloads.push_back({workload, cachePath});
                std::cout << "[batch] solve_cached index=" << index
                          << " workload=" << workload.name
                          << " cache=" << cachePath.string() << std::endl;
            }
            catch (const std::exception &ex)
            {
                rows.push_back(failureBatchRow(workload, cachePath, reportPath, ex.what()));
                std::cerr << "[batch] solve_failed workload=" << workload.name
                          << " error=\"" << ex.what() << "\"" << std::endl;
            }
        }

        std::cout << "[batch] compare_begin first_index=" << batchStart
                  << " workload_count=" << cachedWorkloads.size() << std::endl;
        std::vector<std::future<BatchVerificationRow>> futures;
        futures.reserve(cachedWorkloads.size());
        for (const auto &cachedWorkload : cachedWorkloads)
        {
            const BatchWorkload workload = cachedWorkload.first;
            const std::filesystem::path cachePath = cachedWorkload.second;
            const std::filesystem::path reportPath = reportsDir / (sanitizeFileStem(workload.name) + ".report.txt");
            futures.push_back(std::async(std::launch::async, [&, workload, cachePath, reportPath]()
            {
                try
                {
                    std::cout << "[batch] compare_worker_begin workload=" << workload.name << std::endl;
                    BatchVerificationRow row = compareCachedCandidate(options, cachePath, reportPath);
                    std::cout << "[batch] compare_worker_end workload=" << workload.name
                              << " passed=" << (row.passed ? 1 : 0) << std::endl;
                    return row;
                }
                catch (const std::exception &ex)
                {
                    std::cerr << "[batch] compare_failed workload=" << workload.name
                              << " error=\"" << ex.what() << "\"" << std::endl;
                    return failureBatchRow(workload, cachePath, reportPath, ex.what());
                }
            }));
        }
        for (std::future<BatchVerificationRow> &future : futures)
            rows.push_back(future.get());
    }

    const std::filesystem::path csvPath = options.batchOutDir / "verification.csv";
    const std::filesystem::path reportPath = options.batchOutDir / "batch_report.txt";
    writeBatchVerificationCsv(csvPath, rows);
    writeBatchReport(reportPath, rows, batchSize, cpuThreads);

    const bool allPassed = std::all_of(rows.begin(), rows.end(), [](const BatchVerificationRow &row)
    {
        return row.passed;
    });
    std::cout << "batch_verification=" << (allPassed ? "pass" : "fail")
              << " workloads=" << rows.size()
              << " batch_size=" << batchSize
              << " verification_csv=" << csvPath.string()
              << " report=" << reportPath.string()
              << std::endl;
    return allPassed ? 0 : 2;
}

int runSingle(const VerifyOptions &options)
{
    VerificationReport report;

    const Clock::time_point prepareStart = Clock::now();
    const PreparedProblem prepared = prepareProblem(options);
    const Clock::time_point prepareEnd = Clock::now();
    report.prepareMs = elapsedMilliseconds(prepareStart, prepareEnd);
    report.sharedScale = prepared.sharedScale;
    report.lhsPolygons = prepared.lhsPolygons.size();
    report.rhsPolygons = prepared.rhsPolygons.size();
    writeInputPolygonDiagnostics(options.inputDumpPath, prepared.lhsPolygons, prepared.rhsPolygons);

    const Clock::time_point solveStart = Clock::now();
    ember::BoolProblem problem = solveCandidate(options, prepared);
    const Clock::time_point solveEnd = Clock::now();
    report.solveMs = elapsedMilliseconds(solveStart, solveEnd);
    report.resultFragments = problem.resultFragmentCount();
    report.candidateMode = toString(options.candidateMode);
    report.nefCompareOp = toString(options.nefCompareOp);
    writeResultChunkDiagnostics(options.chunkDumpPath, problem);
    writeResultFragmentDiagnostics(options.fragmentDumpPath, problem);

    if (options.diagnoseNef)
    {
        diagnosePolygonSoup("lhs_raw", prepared.lhsPolygons, ember::PolygonSoupTopologyMode::Raw);
        diagnosePolygonSoup("rhs_raw", prepared.rhsPolygons, ember::PolygonSoupTopologyMode::Raw);
        diagnosePolygonSoup("candidate_raw", problem.resultFragments(), ember::PolygonSoupTopologyMode::Raw);
        diagnosePolygonSoup("candidate_conforming", problem.resultFragments(), ember::PolygonSoupTopologyMode::Conforming);
    }

    if (options.nefCompareOp == NefCompareOp::Skip && options.diffOutPath.empty())
    {
        report.passed = false;
        writeReport(options.reportPath, report);
        std::cout << "verification=skip"
                  << " operation=" << toString(options.operation)
                  << " scale=" << report.sharedScale
                  << " lhs_polygons=" << report.lhsPolygons
                  << " rhs_polygons=" << report.rhsPolygons
                  << " result_fragments=" << report.resultFragments
                  << " candidate_mode=" << report.candidateMode
                  << " nef_compare_op=" << toString(options.nefCompareOp)
                  << " surface_compare_used=0"
                  << " cache_hit=0"
                  << " prepare_ms=" << std::fixed << std::setprecision(6) << report.prepareMs
                  << " solve_ms=" << report.solveMs
                  << " oracle_ms=0.000000"
                  << " compare_ms=0.000000"
                  << std::endl;
        return 2;
    }

    const Clock::time_point candidateMeshStart = Clock::now();
    const ember::ExactMeshData candidateMesh = buildCandidateExactMesh(options, problem.resultFragments());
    const Clock::time_point candidateMeshEnd = Clock::now();
    compareCandidateExactMesh(options, prepared, candidateMesh, problem.resultFragmentCount(), report);
    report.compareMs += elapsedMilliseconds(candidateMeshStart, candidateMeshEnd);
    writeReport(options.reportPath, report);

    std::cout << "verification=" << (report.passed ? "pass" : "fail")
              << " operation=" << toString(options.operation)
              << " scale=" << report.sharedScale
              << " lhs_polygons=" << report.lhsPolygons
              << " rhs_polygons=" << report.rhsPolygons
              << " result_fragments=" << report.resultFragments
              << " candidate_mode=" << report.candidateMode
              << " nef_compare_op=" << report.nefCompareOp
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
        if (!options.batchInputRoot.empty() || !options.batchManifest.empty())
            return runBatch(options);
        return runSingle(options);
    }
    catch (const std::exception &ex)
    {
        writeFailureReport(options.reportPath, ex.what());
        std::cerr << ex.what() << std::endl;
        return 1;
    }
}
