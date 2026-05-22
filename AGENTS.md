# AGENT 本地上下文

本文件是当前工作树的本地 AGENT 指南。全局 memory、旧 README、旧会话总结只能作为线索；遇到差异时，以当前源码、`CMakePresets.json`、测试、新鲜构建结果和性能产物为准。

## 当前事实

- 当前对外流水线是 `OBJ/STL -> 共享 scale + 浮点输入 AABB -> Polygon soup -> BoolProblem -> SubdivisionSolver -> resultFragmentChunks/resultFragments -> raw/conforming OBJ n 边面 / STL 三角面`。
- `main.cpp` 负责 CLI、输入读取、共享量化尺度、输入 AABB、polygon soup 构建、驱动 `BoolProblem` 和输出写回。
- `BoolProblem` 是公开门面，只保存输入、布尔配置和最终结果；不要把它当递归节点使用。诊断入口使用 `leafSummaries()`、`solveMetrics()`、`resultFragments()`、`resultFragmentChunks()`。
- `SubdivisionSolver` 独占递归树、AABB、局部参考点、叶片片段、分类片段和结果汇总；运行时细分状态属于 `SubdivisionSolver`。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部候选构造放在 `path_candidate_details.h`。
- CMake 使用 preset 和显式源文件列表。新增源码必须同步加入 `CMakeLists.txt`；不要回退到手写 configure/build 目录的旧工作流。
- `CMakePresets.json` 固定 Ninja，并提供独立构建树：`build/default`、`build/tests`、`build/verify`、`build/visual-test`。
- Tracy 插桩由 `REEMBER_ENABLE_TRACY` 控制；底层 `math256` 热点桩再由 `REEMBER_ENABLE_TRACY_MATH` 单独控制，二者默认关闭。
- 性能脚本入口是 `tools/profile-re-ember.ps1`；脚本统一把计时、Tracy 捕获、报告和结果写到 `build/performance/run_<timestamp>/`，并按 `-NoTracy/-EnableMathTracy` 自动切换 `build/profile_clang_notracy`、`build/profile_clang_tracy`、`build/profile_clang_tracy_math`。
- 如果性能脚本传入 `-ExecutablePath`，则直接复用已有 `re-EMBER.exe`，不重新配置 profiling 构建树。

## 工作规则

- 所有构建、测试、smoke、oracle cache 和性能产物都放在 `build/`。
- 仓库自有源码的文件头和解释性注释写中文；公开接口使用 Doxygen 结构，保留 `@brief`、`@param`、`@return`、`@retval`、`@note` 等标签。
- 不参考旧 README 作为架构事实；如果全局 memory 仍保留旧版递归入口或旧调用链描述，应视为过期。
- 默认保持 `.obj` 输出为 n 边面多边形集合；`.stl` 只在 I/O 边界三角化。
- `raw` 输出直接写 `resultFragmentChunks()` / `resultFragments()`；`conforming` 输出只作为精确 T-junction 修复后的调试/检查路径，不用于普通性能计时。
- 默认不启用共面凸合并或 Nef 正则化输出。
- 默认不编辑 `third_party/tracy`、`assets`、`reference`、`Doxyfile` 或构建产物。
- 每做一个有意义阶段的改动后要及时 commit，优先使用中文提交信息；不要把全局后端替换、solver 重构、性能优化、文档改写和无关清理混成一个提交。
- 更新功能后不仅要做代码工作，也要更新相关 README/docs；但文档不能替代代码、测试和性能证据。

## 代码约定

- 当前几何核心基于固定宽度整数运算；新增高阶代数、齐次点比较或新构造点前，必须先确认 256 位中间结果预算。
- 底层数学基础以 `reference/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs.md` 为准，尤其是 3.2、Table 1、4.1 和 4.5。
- 核心图元优先限制在 `plane_from_points`、`are_planes_parallel`、`intersect_3_planes`、`classify_vertex/signed_distance`、AABB 轴平面和已有平面裁剪。
- 优化顺序先参考论文已知方向，再做数学闭包分析，最后落到代码；不要凭热点直接引入未经论文证明的齐次点平均、中点、任意点差、坐标平面反推或由齐次输出点构造新平面。
- 论文指出 `classify_vertex` 应复用 4D 齐次交点做点积，`intersect_3_planes` 是四个 3x3 行列式，`plane_from_points` 的 gcd/规范化主要属于导入或可能越界的平面构造。
- 核心布尔分支不得依赖 `int512/cpp_int` 决策；它们只应用于测试 oracle、调试诊断或 I/O 后处理。
- 自定义 256 位定长算术先作为窄接口 backend 演进并接受 oracle 测试，不能一次性替换全局 `Integer` 或绕过论文允许图元边界。
- CLI 的 `--threads` 同时限制应用层 task arena 和求解线程数；`0` 表示自动并发度，`1` 表示全流程强制串行，`N>1` 表示总参与线程数为 `N`。

## 构建与测试

默认应用构建：

```powershell
cmake --preset default
cmake --build --preset default-app
```

测试构建和 CTest：

```powershell
cmake --preset tests
cmake --build --preset tests
ctest --preset default --timeout 120
```

smoke 运行：

```powershell
build\default\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 50
```

可选工具目标按需单独配置：

```powershell
cmake --preset verify
cmake --build --preset verify
cmake --preset visual-test
cmake --build --preset visual-test
```

`re-EMBER_tests` 会依赖并生成同一构建树中的 `re-EMBER.exe`。默认 CTest 还会运行 paper small 聚合测试，并通过 `-ExecutablePath` 复用 `build/tests/re-EMBER.exe`。

## Verifier

`re-EMBER_verify` 校验的是 re-EMBER 量化后的 `Polygon256` 输入上的精确布尔集合，不声明验证原始浮点 OBJ/STL 在 CAD 语义上的真实结果。

常用命令：

```powershell
build\verify\re-EMBER_verify.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --leaf-threshold 50 --oracle-cache-dir build\oracle_cache\nef
```

- 默认 oracle cache 位于 `build/oracle_cache/nef/`；需要强制重算时传 `--refresh-oracle`。
- 默认候选路径是 `--candidate-mode fragments-nef`，从 `BoolProblem::resultFragments()` 恢复 exact conforming mesh 后再转 Nef。
- `export-conforming` 和 `export-nef` 是 verifier 诊断模式，不代表应用层默认输出后处理已经启用。
- 如果 CGAL Nef overlay 卡住或崩溃，可用 `--diagnose-nef --nef-compare-op skip` 先跳过最终 overlay，查看 exact surface 统计和 `surface_compare_used`。

## 性能测试

只看端到端时间和 `BoolSolveMetrics` 时优先：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -NoTracy
```

Tracy 采样优先使用 `RelWithDebInfo`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo
```

只有确认热点落在 `determinant3x3`、`gcdMagnitude`、`primitiveHomPoint` 这类底层函数时，才额外打开底层 math zone：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -EnableMathTracy
```

优化阶段常用 paper 批量入口：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -UsePaperExperimentSet -NoTracy -Configuration Release -Iterations 1 `
  -LeafThreshold 50 -VerifyWithOracle
```

全量 100 组 paper workload：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -UsePaperExperimentSet -PaperSmallCount 34 -PaperMediumCount 33 -PaperLargeCount 33 `
  -NoTracy -Configuration Release -Iterations 1 -LeafThreshold 50
```

如果只比较固定工件和固定位姿下的不同布尔运算，保持 `-Lhs/-Rhs` 不变，只切换 `-Op`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 `
  -Lhs assets\visual_test\lhs.obj `
  -Rhs build\performance\run_<existing>\visual_test_overlap_pose_rhs.obj `
  -Op difference
```

性能产物优先看：

- `summary.txt`：每个 workload 的总体耗时摘要。
- `timings.csv`：逐次迭代的结构化结果。
- `timing_*.metrics.txt`：单次 workload 的详细求解统计。
- `report.md`：问题规模、pipeline inclusive 时间、策略交叉校验、WNV trace 失败原因和 self hot zones 汇总。
- `verification.csv`：`-VerifyWithOracle` 的校验结果。
- `tracy_zones.csv`：inclusive zone 进入次数和耗时统计。
- `tracy_zones_self.csv`：self-time zone 统计。
- `tracy_unwrap\*.csv`：按 `-UnwrapZoneFilter` 导出的逐事件明细。
- `tracy_traces\*.tracy`：Tracy 原始捕获。

当前最有用的高层字段：

- `solve_ms`：真正的布尔求解时间；先和 `read_ms/prepare_ms/export_ms` 分开看。
- `node_count`、`leaf_node_count`、`max_depth`：递归树规模。
- `total_polygon_count`、`leaf_fragment_count`、`result_fragment_count`：几何工作量。
- `constant_discard_count`：布尔指示函数提前剪枝命中数。
- `wntv_aware_split_count`、`center_range_split_count`、`midpoint_split_count`：切分策略命中数。
- `child_reference_candidate_count`、`child_reference_fast_candidate_count`、`child_reference_exhaustive_candidate_count`：子参考点传播的候选放大量。
- `child_reference_candidate_tried_count`、`child_reference_fast_candidate_tried_count`、`child_reference_exhaustive_candidate_tried_count`、`child_reference_trace_count`：子参考点传播的实际 trace 放大量。
- `leaf_classification_centroid_point_count`、`leaf_classification_inset_point_attempt_count`、`leaf_classification_trace_attempt_count`、`leaf_classification_axis_path_attempt_count`、`leaf_classification_plane_replacement_path_attempt_count`：叶片分类阶段的论文两阶段尝试量。
- `leaf_classification_candidate_generated_count`、`leaf_classification_candidate_unique_count`、`leaf_classification_candidate_duplicate_skip_count`、`leaf_classification_candidate_intermediate_endpoint_rejected_count`、`leaf_classification_candidate_repair_attempt_count`：候选枚举、去重、预检和修复放大量。
- `trace_path_*`、`trace_path_boundary_hit_allowed_subdivision_clip_edge_count`、`trace_path_boundary_hit_rejected_regular_edge_count`：排查 `PATH_INVALID` 几何来源时优先看。

`tracy_zones.csv` 看 inclusive 的总体阶段耗时；`tracy_zones_self.csv` 看真实 self hotspot。领域规模和正确性仍以 `timings.csv`、`metrics.txt`、`summary.txt`、`report.md`、`verification.csv` 为准。
