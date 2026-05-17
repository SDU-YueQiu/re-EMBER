/**
 * @file main.cpp
 * @brief 实现基于 OBJ/STL 的命令行布尔运算程序。
 */
#include "core/bool_problem.h"
#include "core/perf_tracing.h"
#include "io/io.h"

#include <oneapi/tbb/info.h>
#include <oneapi/tbb/task_arena.h>
#include <oneapi/tbb/task_group.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
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
    std::size_t threadCount = 0;
    ember::PolygonSoupTopologyMode outputTopologyMode = ember::PolygonSoupTopologyMode::Raw;
    BoolOperandAssumptions lhsAssumptions;
    BoolOperandAssumptions rhsAssumptions;
};

struct CliTimings
{
    double readMs = 0.0;
    double prepareMs = 0.0;
    double solveMs = 0.0;
    double exportMs = 0.0;
    double endToEndMs = 0.0;
    std::size_t lhsInputFaces = 0;
    std::size_t rhsInputFaces = 0;
    std::uint64_t sharedScale = 0;
    std::size_t lhsPolygonCount = 0;
    std::size_t rhsPolygonCount = 0;
    std::size_t exportedFaces = 0;
    ember::PolygonSoupTopologyMode outputTopologyMode = ember::PolygonSoupTopologyMode::Raw;
    ember::BoolSolveMetrics solveMetrics;
};

void printUsage()
{
    std::cerr
            << "Usage: re-EMBER --lhs <file.obj|file.stl> --rhs <file.obj|file.stl> "
            << "--op union|intersection|difference --out <result.obj|result.stl> "
            << "[--scale <positive_integer>] [--leaf-threshold <positive_integer>] "
            << "[--threads <positive_integer>] "
            << "[--output-topology raw|conforming|conforming-merge-convex] "
            << "[--timings-out <metrics.txt>] "
            << "[--assume-lhs-nsi] [--assume-lhs-nnc] "
            << "[--assume-rhs-nsi] [--assume-rhs-nnc]"
            << std::endl;
    std::cerr
            << "Assumption flags are unchecked optimizations; wrong NSI/NNC declarations can make results invalid."
            << std::endl;
    std::cerr
            << "Input supports .obj/.stl. Output .obj preserves n-gons; output .stl triangulates."
            << std::endl;
}

int topologyModeCode(ember::PolygonSoupTopologyMode mode) noexcept
{
    switch (mode)
    {
    case ember::PolygonSoupTopologyMode::Raw:
        return 0;
    case ember::PolygonSoupTopologyMode::Conforming:
        return 1;
    case ember::PolygonSoupTopologyMode::ConformingMergeConvex:
        return 2;
    }

    return 0;
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
           << "end_to_end_ms=" << timings.endToEndMs << '\n'
           << "lhs_input_faces=" << timings.lhsInputFaces << '\n'
           << "rhs_input_faces=" << timings.rhsInputFaces << '\n'
           << "shared_scale=" << timings.sharedScale << '\n'
           << "lhs_polygons=" << timings.lhsPolygonCount << '\n'
           << "rhs_polygons=" << timings.rhsPolygonCount << '\n'
           << "exported_faces=" << timings.exportedFaces << '\n'
           << "output_topology_mode=" << topologyModeCode(timings.outputTopologyMode) << '\n'
           << "input_polygons=" << timings.solveMetrics.inputPolygonCount << '\n'
           << "effective_thread_count=" << timings.solveMetrics.effectiveThreadCount << '\n'
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
           << "parallel_sibling_spawn_count=" << timings.solveMetrics.parallelSiblingSpawnCount << '\n'
           << "child_reference_reuse_count=" << timings.solveMetrics.childReferenceReuseCount << '\n'
           << "child_reference_trace_count=" << timings.solveMetrics.childReferenceTraceCount << '\n'
           << "child_reference_candidate_count=" << timings.solveMetrics.childReferenceCandidateCount << '\n'
           << "child_reference_fast_candidate_count=" << timings.solveMetrics.childReferenceFastCandidateCount << '\n'
           << "child_reference_exhaustive_candidate_count=" << timings.solveMetrics.childReferenceExhaustiveCandidateCount << '\n'
           << "child_reference_candidate_tried_count=" << timings.solveMetrics.childReferenceCandidateTriedCount << '\n'
           << "child_reference_fast_candidate_tried_count=" << timings.solveMetrics.childReferenceFastCandidateTriedCount << '\n'
           << "child_reference_exhaustive_candidate_tried_count=" << timings.solveMetrics.childReferenceExhaustiveCandidateTriedCount << '\n'
           << "trace_path_start_point_on_boundary_count=" << timings.solveMetrics.tracePathStartPointOnBoundaryCount << '\n'
           << "trace_path_end_point_on_boundary_count=" << timings.solveMetrics.tracePathEndPointOnBoundaryCount << '\n'
           << "trace_path_endpoint_on_boundary_contact_count=" << timings.solveMetrics.tracePathEndpointOnBoundaryContactCount << '\n'
           << "trace_path_edge_overlap_count=" << timings.solveMetrics.tracePathEdgeOverlapCount << '\n'
           << "trace_path_boundary_hit_rejected_regular_edge_count=" << timings.solveMetrics.tracePathBoundaryHitRejectedRegularEdgeCount << '\n'
           << "trace_path_boundary_hit_rejected_subdivision_clip_edge_count=" << timings.solveMetrics.tracePathBoundaryHitRejectedSubdivisionClipEdgeCount << '\n'
           << "trace_path_boundary_hit_rejected_mixed_edge_count=" << timings.solveMetrics.tracePathBoundaryHitRejectedMixedEdgeCount << '\n'
           << "trace_path_boundary_hit_rejected_unknown_count=" << timings.solveMetrics.tracePathBoundaryHitRejectedUnknownCount << '\n'
           << "trace_path_boundary_hit_allowed_subdivision_clip_edge_count=" << timings.solveMetrics.tracePathBoundaryHitAllowedSubdivisionClipEdgeCount << '\n'
           << "trace_path_non_strict_intersection_count=" << timings.solveMetrics.tracePathNonStrictIntersectionCount << '\n'
           << "trace_path_boundary_contact_without_intersection_count=" << timings.solveMetrics.tracePathBoundaryContactWithoutIntersectionCount << '\n'
           << "single_operand_assumption_stop_count=" << timings.solveMetrics.singleOperandAssumptionStopCount << '\n'
           << "single_operand_assumption_fallback_count=" << timings.solveMetrics.singleOperandAssumptionFallbackCount << '\n'
           << "single_operand_leaf_bsp_skip_count=" << timings.solveMetrics.singleOperandLeafBspSkipCount << '\n'
           << "single_operand_classification_reuse_count=" << timings.solveMetrics.singleOperandClassificationReuseCount << '\n'
           << "leaf_bsp_build_count=" << timings.solveMetrics.leafBspBuildCount << '\n'
           << "leaf_classification_centroid_point_count=" << timings.solveMetrics.leafClassificationCentroidPointCount << '\n'
           << "leaf_classification_inset_point_attempt_count=" << timings.solveMetrics.leafClassificationInsetPointAttemptCount << '\n'
           << "leaf_classification_trace_attempt_count=" << timings.solveMetrics.leafClassificationTraceAttemptCount << '\n'
           << "leaf_classification_axis_path_attempt_count=" << timings.solveMetrics.leafClassificationAxisPathAttemptCount << '\n'
           << "leaf_classification_plane_replacement_path_attempt_count=" << timings.solveMetrics.leafClassificationPlaneReplacementPathAttemptCount << '\n'
           << "leaf_classification_candidate_generated_count=" << timings.solveMetrics.leafClassificationCandidateGeneratedCount << '\n'
           << "leaf_classification_candidate_unique_count=" << timings.solveMetrics.leafClassificationCandidateUniqueCount << '\n'
           << "leaf_classification_candidate_duplicate_skip_count=" << timings.solveMetrics.leafClassificationCandidateDuplicateSkipCount << '\n'
           << "leaf_classification_candidate_rejected_count=" << timings.solveMetrics.leafClassificationCandidateRejectedCount << '\n'
           << "leaf_classification_candidate_repair_attempt_count=" << timings.solveMetrics.leafClassificationCandidateRepairAttemptCount << '\n'
           << "leaf_classification_candidate_repair_success_count=" << timings.solveMetrics.leafClassificationCandidateRepairSuccessCount << '\n'
           << "leaf_classification_centroid_axis_success_count=" << timings.solveMetrics.leafClassificationCentroidAxisSuccessCount << '\n'
           << "leaf_classification_centroid_axis_path_invalid_count=" << timings.solveMetrics.leafClassificationCentroidAxisPathInvalidCount << '\n'
           << "leaf_classification_centroid_axis_input_invalid_count=" << timings.solveMetrics.leafClassificationCentroidAxisInputInvalidCount << '\n'
           << "leaf_classification_centroid_axis_fail_count=" << timings.solveMetrics.leafClassificationCentroidAxisFailCount << '\n'
           << "leaf_classification_inset_replacement_success_count=" << timings.solveMetrics.leafClassificationInsetReplacementSuccessCount << '\n'
           << "leaf_classification_inset_replacement_path_invalid_count=" << timings.solveMetrics.leafClassificationInsetReplacementPathInvalidCount << '\n'
           << "leaf_classification_inset_replacement_input_invalid_count=" << timings.solveMetrics.leafClassificationInsetReplacementInputInvalidCount << '\n'
           << "leaf_classification_inset_replacement_fail_count=" << timings.solveMetrics.leafClassificationInsetReplacementFailCount << '\n'
           << "leaf_classification_bridge_rescue_success_count=" << timings.solveMetrics.leafClassificationBridgeRescueSuccessCount << '\n'
           << "leaf_classification_bridge_rescue_path_invalid_count=" << timings.solveMetrics.leafClassificationBridgeRescuePathInvalidCount << '\n'
           << "leaf_classification_bridge_rescue_input_invalid_count=" << timings.solveMetrics.leafClassificationBridgeRescueInputInvalidCount << '\n'
           << "leaf_classification_bridge_rescue_fail_count=" << timings.solveMetrics.leafClassificationBridgeRescueFailCount << '\n';
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

// 解析最小外围 CLI：文件路径、布尔运算、共享量化尺度、叶子阈值和线程数。
bool parseArgs(int argc, char **argv, CliOptions &outOptions)
{
    outOptions = CliOptions();
    bool hasOperation = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--lhs" || arg == "--rhs" || arg == "--op" || arg == "--out" ||
                arg == "--scale" || arg == "--leaf-threshold" || arg == "--threads" ||
                arg == "--output-topology" || arg == "--timings-out")
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
            else if (arg == "--output-topology")
            {
                if (!ember::parsePolygonSoupTopologyMode(value, outOptions.outputTopologyMode))
                {
                    std::cerr << "Unsupported output topology mode: " << value << std::endl;
                    return false;
                }
            }
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

int toTaskArenaConcurrency(std::size_t threadCount) noexcept
{
    if (threadCount == 0)
        return oneapi::tbb::info::default_concurrency();

    const std::size_t maxInt = static_cast<std::size_t>(std::numeric_limits<int>::max());
    return static_cast<int>(std::min(threadCount, maxInt));
}

template <typename LeftTask, typename RightTask>
void runTwoApplicationTasks(std::size_t threadCount, LeftTask &&leftTask, RightTask &&rightTask)
{
    oneapi::tbb::task_arena arena(toTaskArenaConcurrency(threadCount));
    arena.execute([&]()
    {
        if (threadCount == 1)
        {
            leftTask();
            rightTask();
            return;
        }

        oneapi::tbb::task_group tasks;
        tasks.run(std::forward<LeftTask>(leftTask));
        tasks.run(std::forward<RightTask>(rightTask));
        tasks.wait();
    });
}

template <typename Task>
void runApplicationTask(std::size_t threadCount, Task &&task)
{
    oneapi::tbb::task_arena arena(toTaskArenaConcurrency(threadCount));
    arena.execute(std::forward<Task>(task));
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
        // 应用层只做三件事：网格转多边形集合、驱动 BoolProblem、结果按扩展名写回 OBJ/STL。
        ember::ObjMeshData lhsMesh;
        ember::ObjMeshData rhsMesh;
        CliTimings timings;
        timings.outputTopologyMode = options.outputTopologyMode;
        std::string error;

        const Clock::time_point readStart = Clock::now();
        {
            REEMBER_PROFILE_ZONE("re-EMBER::read_obj");
            std::string lhsError;
            std::string rhsError;
            bool lhsRead = false;
            bool rhsRead = false;
            runTwoApplicationTasks(
                options.threadCount,
                [&]
            {
                lhsRead = ember::readMesh(options.lhsPath, lhsMesh, lhsError);
            },
                [&]
            {
                rhsRead = ember::readMesh(options.rhsPath, rhsMesh, rhsError);
            });
            if (!lhsRead)
            {
                std::cerr << lhsError << std::endl;
                return 1;
            }
            if (!rhsRead)
            {
                std::cerr << rhsError << std::endl;
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

            ember::PolygonSoupBuildOptions buildOptions;
            buildOptions.triangulateNonCoplanarFaces = true;

            std::string lhsPrepareError;
            std::string rhsPrepareError;
            bool lhsPrepared = false;
            bool rhsPrepared = false;
            runTwoApplicationTasks(
                options.threadCount,
                [&]
            {
                lhsPrepared =
                    ember::computeScaledMeshAABB(lhsMesh, sharedScale, lhsAABB, lhsPrepareError) &&
                    ember::buildPolygonSoup(lhsMesh, sharedScale, buildOptions, lhsPolygons, lhsPrepareError);
            },
                [&]
            {
                rhsPrepared =
                    ember::computeScaledMeshAABB(rhsMesh, sharedScale, rhsAABB, rhsPrepareError) &&
                    ember::buildPolygonSoup(rhsMesh, sharedScale, buildOptions, rhsPolygons, rhsPrepareError);
            });
            if (!lhsPrepared)
            {
                std::cerr << lhsPrepareError << std::endl;
                return 1;
            }
            if (!rhsPrepared)
            {
                std::cerr << rhsPrepareError << std::endl;
                return 1;
            }

            ember::mergeAABB(sceneAABB, lhsAABB);
            ember::mergeAABB(sceneAABB, rhsAABB);
            ember::expandAABB(sceneAABB, 1);

            timings.sharedScale = sharedScale;
            timings.lhsPolygonCount = lhsPolygons.size();
            timings.rhsPolygonCount = rhsPolygons.size();
        }

        ember::BoolProblem problem(options.leafThreshold);
        {
            REEMBER_PROFILE_ZONE("re-EMBER::prepare_problem");
            problem.setOperation(options.operation);
            problem.setOperandAssumptions(options.lhsAssumptions, options.rhsAssumptions);
            problem.setThreadCount(options.threadCount);
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

        // 输出扩展名决定导出格式：OBJ 保持 n 边面，STL 在边界层做扇形三角化。
        const Clock::time_point exportStart = Clock::now();
        std::size_t exportedFaces = 0;
        {
            REEMBER_PROFILE_ZONE("re-EMBER::export_obj");
            bool exported = false;
            runApplicationTask(options.threadCount, [&]
            {
                ember::PolygonSoupExportOptions exportOptions;
                exportOptions.coordinateScale = sharedScale;
                exportOptions.topologyMode = options.outputTopologyMode;
                exported = ember::writePolygonSoupMesh(
                    problem.resultFragments(),
                    options.outputPath,
                    exportedFaces,
                    error,
                    exportOptions);
            });
            if (!exported)
            {
                std::cerr << error << std::endl;
                return 1;
            }
        }
        const Clock::time_point exportEnd = Clock::now();
        timings.exportMs = elapsedMilliseconds(exportStart, exportEnd);
        timings.endToEndMs = elapsedMilliseconds(readStart, exportEnd);
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
                << " end_to_end_ms=" << timings.endToEndMs
                << " threads=" << timings.solveMetrics.effectiveThreadCount
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
                << " parallel_spawns=" << timings.solveMetrics.parallelSiblingSpawnCount
                << " child_ref_candidates=" << timings.solveMetrics.childReferenceCandidateCount
                << " child_ref_tried=" << timings.solveMetrics.childReferenceCandidateTriedCount
                << " leaf_trace_attempts=" << timings.solveMetrics.leafClassificationTraceAttemptCount
                << " result_fragments=" << problem.resultFragments().size()
                << " output_topology=" << ember::toString(options.outputTopologyMode)
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
