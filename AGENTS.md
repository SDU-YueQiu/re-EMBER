# AGENT 本地上下文

本文件是当前工作树的本地 AGENT 指南。全局 memory 只能作为线索，遇到差异时以当前代码、测试和新鲜构建结果为准。

## 当前事实

- 当前流水线是 `OBJ -> 多边形集合 -> BoolProblem -> SubdivisionSolver -> resultFragments -> OBJ n 边面`。
- `BoolProblem` 是公开门面，不再暴露递归子节点；诊断入口使用 `leafSummaries()` 和 `solveMetrics()`。
- `SubdivisionSolver` 独占递归节点、AABB、局部参考点、叶片片段、分类片段和结果汇总。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部候选构造在 `path_candidate_details.h`。
- CMake 使用显式源文件列表，新增源码需要同步加入 `CMakeLists.txt`。
- Tracy 性能插桩由 `REEMBER_ENABLE_TRACY` 控制；底层 `math256` 热点桩再由 `REEMBER_ENABLE_TRACY_MATH` 单独控制，二者默认都关闭，不影响普通发布构建。
- 性能脚本入口是 `tools/profile-re-ember.ps1`；统一把计时、Tracy 捕获、报告和结果 OBJ 写到 `build\perf\run_<timestamp>\`。

## 工作规则

- 所有构建、测试和 smoke 产物都放在 `build/`。
- 仓库自有源码的文件头和解释性注释写中文；公开接口使用 Doxygen 结构。
- 不参考旧 README 作为架构事实；如果全局 memory 仍提到 `BoolProblem::solveRecursive()` 调用链，应视为过期。
- 默认保持 OBJ 输出为 n 边面多边形集合，不主动三角化输出。
- 默认不编辑 `third_party/slimcpplib`、`third_party/tracy`、`assets`、`reference`、`Doxyfile` 或构建产物。
- 每做一个阶段的改动后要及时commit，同样优先使用中文提交

## 验证命令

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
cmake --build build --config Debug --target re-EMBER
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\codex_boolean_smoke.obj --leaf-threshold 25
```

## 性能测试

计时优先使用 `RelWithDebInfo`：

```powershell
vcpkg install tracy[cli-tools]:x64-windows
cmake -S . -B build -DREEMBER_ENABLE_TRACY=ON
cmake --build build --config RelWithDebInfo --target re-EMBER
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -SkipBuild
```

如果要抓 `math256` 这类底层代数热点，再额外打开：

```powershell
cmake -S . -B build -DREEMBER_ENABLE_TRACY=ON -DREEMBER_ENABLE_TRACY_MATH=ON
cmake --build build --config RelWithDebInfo --target re-EMBER
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -SkipBuild -EnableMathTracy
```

只有在确认热点落在 `determinant3x3`、`gcdMagnitude`、`primitiveHomPoint` 这类底层函数时才开 `REEMBER_ENABLE_TRACY_MATH` / `-EnableMathTracy`；平时保持关闭，避免给正常 profiling 和普通运行带来额外开销。

只要端到端时间和 `BoolSolveMetrics` 时可加 `-NoTracy`；普通构建仍保持 `REEMBER_ENABLE_TRACY=OFF`、`REEMBER_ENABLE_TRACY_MATH=OFF`。

如果只想比较固定工件和固定位姿下的不同布尔运算，保持 `-Lhs/-Rhs` 不变，只切换 `-Op`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 `
  -Lhs assets\visual_test\workpiece_block.obj `
  -Rhs build\perf\run_<existing>\visual_test_overlap_pose_tool.obj `
  -Op difference
```

优先查看这些产物：

- `summary.txt`：每个 workload 的总体耗时摘要。
- `timings.csv`：逐次迭代的结构化结果。
- `timing_*.metrics.txt`：单次 workload 的详细求解统计。
- `tracy_traces\*.tracy`：Tracy 原始捕获。
- `tracy_zones.csv`：inclusive zone 进入次数和耗时统计。
- `tracy_zones_self.csv`：self-time zone 统计。
- `tracy_unwrap\*.csv`：按 `-UnwrapZoneFilter` 导出的逐事件明细。
- `report.md`：问题规模、pipeline inclusive 时间、策略交叉校验和 self hot zones 汇总。

当前最有用的高层字段：

- `solve_ms`：真正的布尔求解时间；先和 `read_ms/prepare_ms/export_ms` 分开看。
- `node_count`、`leaf_node_count`、`max_depth`：递归树规模。
- `total_polygon_count`、`leaf_fragment_count`、`result_fragment_count`：几何工作量。
- `constant_discard_count`：布尔指示函数提前剪枝命中数。
- `wntv_aware_split_count`、`center_range_split_count`、`midpoint_split_count`：切分策略命中数。
- `child_reference_candidate_count`、`child_reference_fast_candidate_count`、`child_reference_exhaustive_candidate_count`：子参考点传播的候选放大量。
- `child_reference_candidate_tried_count`、`child_reference_fast_candidate_tried_count`、`child_reference_exhaustive_candidate_tried_count`、`child_reference_trace_count`：子参考点传播的实际 trace 放大量。
- `leaf_classification_primary_point_candidate_count`、`leaf_classification_expanded_point_candidate_count`、`leaf_classification_trace_attempt_count` 及各 layer candidate count：叶片分类阶段的路径尝试量。
- `tracy_zones.csv` 看 inclusive，总体阶段耗时；`tracy_zones_self.csv` 看 self，定位真实热点；领域规模仍以 `timings.csv` / `metrics.txt` 为准。
