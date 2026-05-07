/**
 * @file main.cpp
 * @brief 实现基于 OBJ 的命令行布尔运算程序。
 */
#include "core/bool_problem.h"
#include "core/perf_tracing.h"
#include "io/io.h"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace
{
using ember::BoolOp;
using ember::BoolOperandAssumptions;
using Clock = std::chrono::steady_clock;

// 命令行入口只负责组装两输入布尔任务，不扩展到表达式树或多输入场景。
struct CliOptions
{
    std::string lhsPath;
    std::string rhsPath;
    std::string outputPath;
    std::string timingsOutputPath;
    BoolOp operation = BoolOp::Intersection;
    std::optional<std::uint64_t> scale;
    std::size_t leafThreshold = 25;
    BoolOperandAssumptions lhsAssumptions;
    BoolOperandAssumptions rhsAssumptions;
};

struct CliTimings
{
    double readMs = 0.0;
    double prepareMs = 0.0;
    double solveMs = 0.0;
    double exportMs = 0.0;
    std::size_t lhsInputFaces = 0;
    std::size_t rhsInputFaces = 0;
    std::uint64_t sharedScale = 0;
    std::size_t lhsPolygonCount = 0;
    std::size_t rhsPolygonCount = 0;
    std::size_t exportedFaces = 0;
    ember::BoolSolveMetrics solveMetrics;
};

void printUsage()
{
    std::cerr
            << "Usage: re-EMBER --lhs <file.obj> --rhs <file.obj> "
            << "--op union|intersection|difference --out <result.obj> "
            << "[--scale <positive_integer>] [--leaf-threshold <positive_integer>] "
            << "[--timings-out <metrics.txt>] "
            << "[--assume-lhs-nsi] [--assume-lhs-nnc] "
            << "[--assume-rhs-nsi] [--assume-rhs-nnc]"
            << std::endl;
    std::cerr
            << "Assumption flags are unchecked optimizations; wrong NSI/NNC declarations can make results invalid."
            << std::endl;
}

double elapsedMilliseconds(const Clock::time_point &start, const Clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

#if defined(REEMBER_ENABLE_TRACY)
// 仅供性能脚本使用：让 Tracy capture 在计时开始前有时间连上客户端。
std::uint64_t readTracyAttachWaitMilliseconds() noexcept
{
    const char *raw = std::getenv("REEMBER_TRACY_WAIT_MS");
    if (raw == nullptr || raw[0] == '\0')
        return 0;

    try
    {
        std::size_t consumed = 0;
        const unsigned long long parsed = std::stoull(raw, &consumed, 10);
        if (consumed != std::char_traits<char>::length(raw))
            return 0;

        return static_cast<std::uint64_t>(parsed);
    }
    catch (...)
    {
        return 0;
    }
}

bool tracyDiagnosticsEnabled() noexcept
{
    const char *raw = std::getenv("REEMBER_TRACY_DIAG");
    return raw != nullptr && raw[0] == '1';
}

void emitTracyDiagnostics(const char *phase)
{
    if (!tracyDiagnosticsEnabled())
        return;

    std::cerr
            << "[tracy] phase=" << phase
            << " profiler_available=" << (tracy::ProfilerAvailable() ? 1 : 0)
            << " connected=" << (TracyIsConnected ? 1 : 0)
            << std::endl;
}
#endif

bool writeTimingMetrics(const std::string &path, const CliTimings &timings, std::string &outError)
{
    outError.clear();

    std::ofstream output(path, std::ios::trunc);
    if (!output)
    {
        outError = "Failed to open timings output file: " + path;
        return false;
    }

    output << std::fixed << std::setprecision(6)
           << "read_ms=" << timings.readMs << '\n'
           << "prepare_ms=" << timings.prepareMs << '\n'
           << "solve_ms=" << timings.solveMs << '\n'
           << "export_ms=" << timings.exportMs << '\n'
           << "lhs_input_faces=" << timings.lhsInputFaces << '\n'
           << "rhs_input_faces=" << timings.rhsInputFaces << '\n'
           << "shared_scale=" << timings.sharedScale << '\n'
           << "lhs_polygons=" << timings.lhsPolygonCount << '\n'
           << "rhs_polygons=" << timings.rhsPolygonCount << '\n'
           << "exported_faces=" << timings.exportedFaces << '\n'
           << "input_polygons=" << timings.solveMetrics.inputPolygonCount << '\n'
           << "node_count=" << timings.solveMetrics.nodeCount << '\n'
           << "internal_node_count=" << timings.solveMetrics.internalNodeCount << '\n'
           << "leaf_node_count=" << timings.solveMetrics.leafNodeCount << '\n'
           << "discarded_node_count=" << timings.solveMetrics.discardedNodeCount << '\n'
           << "max_depth=" << timings.solveMetrics.maxDepth << '\n'
           << "total_polygon_count=" << timings.solveMetrics.totalPolygonCount << '\n'
           << "leaf_fragment_count=" << timings.solveMetrics.leafFragmentCount << '\n'
           << "classified_fragment_count=" << timings.solveMetrics.classifiedFragmentCount << '\n'
           << "result_fragment_count=" << timings.solveMetrics.resultFragmentCount << '\n'
           << "constant_discard_count=" << timings.solveMetrics.constantDiscardCount << '\n'
           << "invalid_or_empty_discard_count=" << timings.solveMetrics.invalidOrEmptyDiscardCount << '\n'
           << "leaf_threshold_stop_count=" << timings.solveMetrics.leafThresholdStopCount << '\n'
           << "aabb_not_splittable_stop_count=" << timings.solveMetrics.aabbNotSplittableStopCount << '\n'
           << "split_failure_stop_count=" << timings.solveMetrics.splitFailureStopCount << '\n'
           << "wntv_aware_split_count=" << timings.solveMetrics.wntvAwareSplitCount << '\n'
           << "center_range_split_count=" << timings.solveMetrics.centerRangeSplitCount << '\n'
           << "midpoint_split_count=" << timings.solveMetrics.midpointSplitCount << '\n'
           << "child_reference_reuse_count=" << timings.solveMetrics.childReferenceReuseCount << '\n'
           << "child_reference_trace_count=" << timings.solveMetrics.childReferenceTraceCount << '\n'
           << "child_reference_candidate_count=" << timings.solveMetrics.childReferenceCandidateCount << '\n'
           << "child_reference_fast_candidate_count=" << timings.solveMetrics.childReferenceFastCandidateCount << '\n'
           << "child_reference_exhaustive_candidate_count=" << timings.solveMetrics.childReferenceExhaustiveCandidateCount << '\n'
           << "child_reference_candidate_tried_count=" << timings.solveMetrics.childReferenceCandidateTriedCount << '\n'
           << "child_reference_fast_candidate_tried_count=" << timings.solveMetrics.childReferenceFastCandidateTriedCount << '\n'
           << "child_reference_exhaustive_candidate_tried_count=" << timings.solveMetrics.childReferenceExhaustiveCandidateTriedCount << '\n'
           << "single_operand_assumption_stop_count=" << timings.solveMetrics.singleOperandAssumptionStopCount << '\n'
           << "single_operand_assumption_fallback_count=" << timings.solveMetrics.singleOperandAssumptionFallbackCount << '\n'
           << "single_operand_leaf_bsp_skip_count=" << timings.solveMetrics.singleOperandLeafBspSkipCount << '\n'
           << "single_operand_classification_reuse_count=" << timings.solveMetrics.singleOperandClassificationReuseCount << '\n'
           << "leaf_bsp_build_count=" << timings.solveMetrics.leafBspBuildCount << '\n'
           << "leaf_classification_point_candidate_count=" << timings.solveMetrics.leafClassificationPointCandidateCount << '\n'
           << "leaf_classification_primary_point_candidate_count=" << timings.solveMetrics.leafClassificationPrimaryPointCandidateCount << '\n'
           << "leaf_classification_expanded_point_candidate_count=" << timings.solveMetrics.leafClassificationExpandedPointCandidateCount << '\n'
           << "leaf_classification_trace_attempt_count=" << timings.solveMetrics.leafClassificationTraceAttemptCount << '\n'
           << "leaf_classification_fast_candidate_count=" << timings.solveMetrics.leafClassificationFastCandidateCount << '\n'
           << "leaf_classification_fallback_candidate_count=" << timings.solveMetrics.leafClassificationFallbackCandidateCount << '\n'
           << "leaf_classification_normal_candidate_count=" << timings.solveMetrics.leafClassificationNormalCandidateCount << '\n'
           << "leaf_classification_interior_bridge_candidate_count=" << timings.solveMetrics.leafClassificationInteriorBridgeCandidateCount << '\n';
    if (!output)
    {
        outError = "Failed to write timings output file: " + path;
        return false;
    }

    return true;
}

// CLI 数值参数统一要求正整数，避免导入尺度和阈值出现 0 或负值。
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

// 第一版只开放当前内核已经支持的三种二元布尔运算。
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

// 解析最小外围 CLI：文件路径、布尔运算、共享量化尺度和叶子阈值。
bool parseArgs(int argc, char **argv, CliOptions &outOptions)
{
    outOptions = CliOptions();
    bool hasOperation = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--lhs" || arg == "--rhs" || arg == "--op" || arg == "--out" ||
                arg == "--scale" || arg == "--leaf-threshold" || arg == "--timings-out")
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
            else if (arg == "--out")
                outOptions.outputPath = value;
            else if (arg == "--timings-out")
                outOptions.timingsOutputPath = value;
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

    if (outOptions.lhsPath.empty() ||
            outOptions.rhsPath.empty() ||
            outOptions.outputPath.empty() ||
            !hasOperation)
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
}

int main(int argc, char **argv)
{
    REEMBER_PROFILE_ZONE("re-EMBER::main");
#if defined(REEMBER_ENABLE_TRACY)
    TracySetProgramName("re-EMBER");
    emitTracyDiagnostics("startup");

    const std::uint64_t tracyAttachWaitMs = readTracyAttachWaitMilliseconds();
    if (tracyAttachWaitMs > 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(tracyAttachWaitMs));
    emitTracyDiagnostics("post_attach_wait");
#endif

    CliOptions options;
    if (!parseArgs(argc, argv, options))
    {
        printUsage();
        return 1;
    }

    try
    {
        // 应用层只做三件事：OBJ 转多边形集合、驱动 BoolProblem、结果写回 OBJ。
        ember::ObjMeshData lhsMesh;
        ember::ObjMeshData rhsMesh;
        CliTimings timings;
        std::string error;

        const Clock::time_point readStart = Clock::now();
        {
            REEMBER_PROFILE_ZONE("re-EMBER::read_obj");
            if (!ember::readObjMesh(options.lhsPath, lhsMesh, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }
            if (!ember::readObjMesh(options.rhsPath, rhsMesh, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }
        }
        const Clock::time_point readEnd = Clock::now();
        timings.readMs = elapsedMilliseconds(readStart, readEnd);
        timings.lhsInputFaces = lhsMesh.faces.size();
        timings.rhsInputFaces = rhsMesh.faces.size();

        ember::QuantizeOptions quantizeOptions;
        quantizeOptions.explicitScale = options.scale;

        // 左右操作数必须进入同一个整数坐标系，否则后续精确谓词没有共同语义。
        const Clock::time_point prepareStart = Clock::now();
        std::uint64_t sharedScale = 0;
        std::vector<ember::Polygon256> lhsPolygons;
        std::vector<ember::Polygon256> rhsPolygons;
        ember::AABB3i lhsAABB;
        ember::AABB3i rhsAABB;
        ember::AABB3i sceneAABB;
        {
            REEMBER_PROFILE_ZONE("re-EMBER::prepare_polygons");
            if (!ember::chooseSharedScale({lhsMesh, rhsMesh}, quantizeOptions, sharedScale, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }

            if (!ember::computeScaledMeshAABB(lhsMesh, sharedScale, lhsAABB, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }
            if (!ember::computeScaledMeshAABB(rhsMesh, sharedScale, rhsAABB, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }
            ember::mergeAABB(sceneAABB, lhsAABB);
            ember::mergeAABB(sceneAABB, rhsAABB);
            ember::expandAABB(sceneAABB, 1);

            ember::PolygonSoupBuildOptions buildOptions;
            buildOptions.triangulateNonCoplanarFaces = true;
            if (!ember::buildPolygonSoup(lhsMesh, sharedScale, buildOptions, lhsPolygons, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }
            if (!ember::buildPolygonSoup(rhsMesh, sharedScale, buildOptions, rhsPolygons, error))
            {
                std::cerr << error << std::endl;
                return 1;
            }

            timings.sharedScale = sharedScale;
            timings.lhsPolygonCount = lhsPolygons.size();
            timings.rhsPolygonCount = rhsPolygons.size();
        }

        ember::BoolProblem problem(options.leafThreshold);
        {
            REEMBER_PROFILE_ZONE("re-EMBER::prepare_problem");
            problem.setOperation(options.operation);
            problem.setOperandAssumptions(options.lhsAssumptions, options.rhsAssumptions);
            problem.setOperands(lhsPolygons, rhsPolygons);
        }

        const Clock::time_point prepareEnd = Clock::now();
        timings.prepareMs = elapsedMilliseconds(prepareStart, prepareEnd);

        const Clock::time_point solveStart = Clock::now();
        {
            REEMBER_PROFILE_ZONE("re-EMBER::solve_bool_problem");
            problem.solve(sceneAABB);
        }
        const Clock::time_point solveEnd = Clock::now();
        timings.solveMs = elapsedMilliseconds(solveStart, solveEnd);
        timings.solveMetrics = problem.solveMetrics();

        // 默认直接导出 OBJ n 边面；三角化和拓扑恢复属于调用方后处理。
        const Clock::time_point exportStart = Clock::now();
        std::size_t exportedFaces = 0;
        {
            REEMBER_PROFILE_ZONE("re-EMBER::export_obj");
            if (!ember::writePolygonSoupObj(problem.resultFragments(), options.outputPath, exportedFaces, error, sharedScale))
            {
                std::cerr << error << std::endl;
                return 1;
            }
        }
        const Clock::time_point exportEnd = Clock::now();
        timings.exportMs = elapsedMilliseconds(exportStart, exportEnd);
        timings.exportedFaces = exportedFaces;

        if (!options.timingsOutputPath.empty() &&
                !writeTimingMetrics(options.timingsOutputPath, timings, error))
        {
            std::cerr << error << std::endl;
            return 1;
        }

        std::cout
                << "operation=" << toString(options.operation)
                << " lhs_input_faces=" << lhsMesh.faces.size()
                << " rhs_input_faces=" << rhsMesh.faces.size()
                << " scale=" << sharedScale
                << " lhs_polygons=" << lhsPolygons.size()
                << " rhs_polygons=" << rhsPolygons.size()
                << " node_count=" << timings.solveMetrics.nodeCount
                << " leaf_nodes=" << timings.solveMetrics.leafNodeCount
                << " discarded_nodes=" << timings.solveMetrics.discardedNodeCount
                << " max_depth=" << timings.solveMetrics.maxDepth
                << " constant_discards=" << timings.solveMetrics.constantDiscardCount
                << " single_operand_stops=" << timings.solveMetrics.singleOperandAssumptionStopCount
                << " single_operand_fallbacks=" << timings.solveMetrics.singleOperandAssumptionFallbackCount
                << " wntv_splits=" << timings.solveMetrics.wntvAwareSplitCount
                << " center_splits=" << timings.solveMetrics.centerRangeSplitCount
                << " midpoint_splits=" << timings.solveMetrics.midpointSplitCount
                << " child_ref_candidates=" << timings.solveMetrics.childReferenceCandidateCount
                << " child_ref_tried=" << timings.solveMetrics.childReferenceCandidateTriedCount
                << " leaf_trace_attempts=" << timings.solveMetrics.leafClassificationTraceAttemptCount
                << " result_fragments=" << problem.resultFragments().size()
                << " exported_faces=" << exportedFaces
                << std::endl;

        REEMBER_PROFILE_FRAME("re-EMBER workload");

        return 0;
    }
    catch (const std::exception &ex)
    {
        std::cerr << ex.what() << std::endl;
        return 1;
    }
}
