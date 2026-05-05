# AGENT 本地上下文

本文件是当前工作树的本地 AGENT 指南。全局 memory 只能作为线索，遇到差异时以当前代码、测试和新鲜构建结果为准。

## 当前事实

- 当前流水线是 `OBJ -> 多边形集合 -> BoolProblem -> SubdivisionSolver -> resultFragments -> OBJ n 边面`。
- `BoolProblem` 是公开门面，不再暴露递归子节点；诊断入口使用 `leafSummaries()` 和 `solveMetrics()`。
- `SubdivisionSolver` 独占递归节点、AABB、局部参考点、叶片片段、分类片段和结果汇总。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部候选构造在 `path_candidate_details.h`。
- CMake 使用显式源文件列表，新增源码需要同步加入 `CMakeLists.txt`。
- 性能脚本入口是 `tools/profile-re-ember.ps1`；统一把计时、ETW 和结果 OBJ 写到 `build\perf\run_<timestamp>\`。

## 工作规则

- 所有构建、测试和 smoke 产物都放在 `build/`。
- 仓库自有源码的文件头和解释性注释写中文；公开接口使用 Doxygen 结构。
- 不参考旧 README 作为架构事实；如果全局 memory 仍提到 `BoolProblem::solveRecursive()` 调用链，应视为过期。
- 默认保持 OBJ 输出为 n 边面多边形集合，不主动三角化输出。
- 默认不编辑 `include/slimcpplib`、`assets`、`reference`、`Doxyfile` 或构建产物。
- 每做一个阶段的改动后要及时commit

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
cmake --build build --config RelWithDebInfo --target re-EMBER
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -NoEtw -Iterations 3
```

如果只想比较固定工件和固定位姿下的不同布尔运算，保持 `-Lhs/-Rhs` 不变，只切换 `-Op`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -NoEtw -SkipBuild -Iterations 1 `
  -Lhs assets\visual_test\workpiece_block.obj `
  -Rhs build\perf\run_<existing>\visual_test_overlap_pose_tool.obj `
  -Op difference
```

需要热点时，使用提升权限的 PowerShell，并在 ETW 前加 `-ForceStopExisting`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 -ForceStopExisting `
  -Lhs assets\visual_test\workpiece_block.obj `
  -Rhs build\perf\run_<existing>\visual_test_overlap_pose_tool.obj `
  -Op difference
```

优先查看这些产物：

- `summary.txt`：每个 workload 的总体耗时摘要。
- `timings.csv`：逐次迭代的结构化结果。
- `timing_*.metrics.txt` / `etw_*.metrics.txt`：单次 workload 的详细求解统计。
- `xperf_profile_detail.txt`：采样热点明细。
- `xperf_stack_butterfly.txt`：高层调用栈的 inclusive hot path。

当前最有用的高层字段：

- `solve_ms`：真正的布尔求解时间；先和 `read_ms/prepare_ms/export_ms` 分开看。
- `node_count`、`leaf_node_count`、`max_depth`：递归树规模。
- `total_polygon_count`、`leaf_fragment_count`、`result_fragment_count`：几何工作量。
- `constant_discard_count`：布尔指示函数提前剪枝命中数。
- `wntv_aware_split_count`、`center_range_split_count`、`midpoint_split_count`：切分策略命中数。
- `child_reference_candidate_count`、`child_reference_candidate_tried_count`、`child_reference_trace_count`：子参考点传播的候选放大量。
- `leaf_classification_trace_attempt_count` 及各 layer candidate count：叶片分类阶段的路径尝试量。

解释 ETW 时要注意：

- `xperf profile -detail` 里的 `Weight` 和 `stack butterfly` 里的 `hits` 都是采样权重，不是精确函数调用次数。
- 真正的 workload 次数优先看 `metrics.txt` 里的 candidate / trace 计数，而不是把 ETW 采样数当成调用数。
