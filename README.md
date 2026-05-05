# re-EMBER

`re-EMBER` 是一个围绕 EMBER（Exact Mesh Booleans via Efficient & Robust Local Arrangements）论文复现的 C++17 原型仓库。当前目标是验证精确整数几何、局部编排、WNV/WNTV 分类和二元网格布尔流水线，而不是提供完整建模软件或通用网格修复器。

## 当前状态

当前代码已经具备一条可运行的基础布尔流水线：

```text
OBJ -> 多边形集合 -> BoolProblem -> SubdivisionSolver -> resultFragments -> OBJ n 边面
```

重要边界如下：

- `src/application/main.cpp`：命令行入口，只负责解析参数、读写 OBJ、组装二元布尔任务。
- `src/io/io.h/.cpp`：OBJ 读取、共享量化尺度选择、`Polygon256` 多边形集合构建、OBJ n 边面导出。
- `src/core/bool_problem.h/.cpp`：公开布尔问题门面，接收输入、保存配置、调用内部求解器并缓存结果。
- `src/core/subdivision_solver.h/.cpp`：内部细分求解器，独占递归节点、AABB、参考点、叶片片段和结果汇总。
- `src/core/leaf_classifier.cpp`：叶片局部编排后的 WNV 分类与结果筛选。
- `src/algorithm/`：局部 BSP、叶子编排、WNV 路径追踪和分类路径候选。
- `src/geometry/`、`src/math/`：固定宽度整数几何 primitive、平面点表示、裁剪和基础代数。
- `src/tests/`：仓库自定义断言测试，不依赖第三方测试框架。
- `tools/profile-re-ember.ps1`：端到端性能测试与 ETW 采样入口。

`BoolProblem` 现在只暴露应用需要的门面接口：`setOperation`、`setLeafPolygonThreshold`、`addPolygon`、`setPolygons`、`setOperands`、`solve`、`isSolved`、`isDiscarded`、`resultFragments`、`leafSummaries` 和 `solveMetrics`。递归子问题状态属于 `SubdivisionSolver` 内部实现；测试或诊断需要看叶子结构时使用 `leafSummaries()`，需要看求解规模和剪枝/候选统计时使用 `solveMetrics()`。

## 构建与测试

所有构建都放在 `build/` 下：

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
cmake --build build --config Debug --target re-EMBER
```

基础 CLI smoke：

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\codex_boolean_smoke.obj --leaf-threshold 25
```

I/O 中较慢的回归用环境变量开启：

```powershell
$env:REEMBER_RUN_EXPENSIVE_IO_TESTS = '1'
build\Debug\re-EMBER_tests.exe
```

## 命令行用法

```powershell
build\Debug\re-EMBER.exe --lhs <left.obj> --rhs <right.obj> --op union|intersection|difference --out <result.obj> [--scale <positive_integer>] [--leaf-threshold <positive_integer>] [--timings-out <metrics.txt>] [--assume-lhs-nsi] [--assume-lhs-nnc] [--assume-rhs-nsi] [--assume-rhs-nnc]
```

参数说明：

- `--lhs`、`--rhs`：左右输入 OBJ。
- `--op`：二元布尔运算，支持 `union`、`intersection`、`difference`。
- `--out`：输出 OBJ 路径。
- `--scale`：可选的显式十进制量化尺度；不传时会为两输入选择共享尺度。
- `--leaf-threshold`：可选的叶子停止细分阈值，默认 `25`。
- `--timings-out`：把本次运行的时间与高层求解统计写到文本文件。
- `--assume-*-nsi`、`--assume-*-nnc`：声明输入操作数满足 NSI/NNC 假设，只用于性能优化；错误声明会破坏正确性。

导出默认保持多边形集合形态，直接写 OBJ n 边面。三角化、拓扑缝合和 T 形连接恢复属于调用方后处理。

## 性能测试

### 快速计时

推荐先构建 `RelWithDebInfo`：

```powershell
cmake --build build --config RelWithDebInfo --target re-EMBER
```

脚本默认会把结果写到 `build\perf\run_<timestamp>\`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -NoEtw -Iterations 3
```

默认 workload 包含：

- `icosphere80_toolbox_difference`
- `visual_block_toolbox_overlap_intersection`
- `visual_block_toolbox_visual_test_default_difference`

注意：默认 visual intersection 和 visual difference 使用的是不同刀具位姿，不能直接拿它们比较三种布尔运算的快慢。

如果要比较同一组工件、同一位姿下的 `union|intersection|difference`，请固定 `-Lhs/-Rhs`，只切换 `-Op`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -NoEtw -SkipBuild -Iterations 1 `
  -Lhs assets\visual_test\workpiece_block.obj `
  -Rhs build\perf\run_<existing>\visual_test_overlap_pose_tool.obj `
  -Op difference
```

### ETW 热点

需要 CPU hotspot 时，在提升权限的 PowerShell 中运行，并建议加 `-ForceStopExisting` 避免已有 kernel logger 冲突：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 -ForceStopExisting `
  -Lhs assets\visual_test\workpiece_block.obj `
  -Rhs build\perf\run_<existing>\visual_test_overlap_pose_tool.obj `
  -Op difference
```

如果 `RelWithDebInfo` 可执行文件已经存在，优先加 `-SkipBuild`，让脚本只做 workload 运行与采样。

### 看哪些文件

每次运行会生成一个 `build\perf\run_<timestamp>\` 目录。最常用的文件是：

- `summary.txt`：每个 workload 的总体摘要。
- `timings.csv`：逐次迭代的结构化表格。
- `timing_*.metrics.txt` / `etw_*.metrics.txt`：单个 workload 的时间和高层统计。
- `profile.log`：脚本实际执行的命令与日志。
- `xperf_profile_detail.txt`：采样热点明细。
- `xperf_stack_butterfly.txt`：高层调用栈的 inclusive hot path。
- `reember_cpu.etl`：后续用 WPA 深挖时的原始 ETW 数据。

### 怎么读结果

先看时间拆分：

- `read_ms`：OBJ 读取时间。
- `prepare_ms`：共享尺度选择和多边形集合构建时间。
- `solve_ms`：真正的布尔求解时间。
- `export_ms`：结果 OBJ 导出时间。

再看高层工作量：

- `node_count`、`internal_node_count`、`leaf_node_count`、`max_depth`：递归树规模。
- `total_polygon_count`：整棵细分树访问到的节点多边形数累计值；比 `input_polygons` 更接近真实 subdivision 工作量。
- `leaf_fragment_count`、`classified_fragment_count`、`result_fragment_count`：叶子阶段与最终输出规模。
- `constant_discard_count`：布尔指示函数提前剪枝命中数。
- `leaf_threshold_stop_count`、`aabb_not_splittable_stop_count`：递归停止原因。
- `wntv_aware_split_count`、`center_range_split_count`、`midpoint_split_count`：切分策略命中分布。

对子参考点传播和叶片分类，重点看候选放大量：

- `child_reference_candidate_count`：子参考点传播阶段生成的候选总数。
- `child_reference_candidate_tried_count`：真正进入 trace 的子参考点候选数。
- `child_reference_trace_count`：成功传播的子参考点数。
- `leaf_classification_point_candidate_count`：叶片分类阶段的目标点总数。
- `leaf_classification_trace_attempt_count`：叶片分类实际尝试的 path trace 总数。
- `leaf_classification_fast_candidate_count`、`leaf_classification_fallback_candidate_count`、`leaf_classification_normal_candidate_count`、`leaf_classification_interior_bridge_candidate_count`：各层路径候选贡献。

这些字段比单看 `node_count` 更能解释“为什么只有几十个节点却仍然很慢”。

### ETW 的正确读法

- `xperf profile -detail` 里的 `Weight` 不是精确函数调用次数。
- `xperf stack` / butterfly 里的 `inclusive hits`、`exclusive hits` 也是采样命中数，不是精确调用数。
- ETW 用来回答“时间主要烧在哪些函数/调用链上”；精确 workload 次数优先看 `metrics.txt` 中的 candidate / trace 计数。

## 代码约定

- 当前代码、测试和新鲜构建结果是事实来源；旧 README、旧会话和全局记忆只能作为线索。
- 仓库自有源码的文件头和解释性注释使用中文；公开接口使用 Doxygen 结构，保留 `@brief`、`@param`、`@return`、`@retval`、`@note` 等标签。
- 不把 `BoolProblem` 当递归节点使用；运行时细分状态属于 `SubdivisionSolver`。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部路径构造细节在 `path_candidate_details.h`。
- 默认不改 `include/slimcpplib`、`assets`、`reference`、`Doxyfile` 和构建产物。
- 当前几何核心基于固定宽度整数运算；新增高阶代数或齐次点比较前必须先确认 256 位中间结果预算。

## 已知限制

- 当前公开流水线面向二元布尔；多网格表达式和通用布尔表达式解析尚未实现。
- OBJ 导入只保留几何位置，不保留法线、UV、材质或拓扑连通关系。
- CLI 为了跑通真实 OBJ，会对量化后非共面的输入面启用扇形三角化；库 API 默认仍是严格构造策略。
- 输出是 OBJ n 边面多边形集合，不保证进行全局拓扑恢复。
- 当前已接入 4.5.1 的 NSI/NNC 快路径、4.5.2 的一部分常量 indicator 剪枝，以及 4.5.3 的 WNTV-aware / center-range 切分；但 4.5.4 并行化和 4.5.5 中“避免到处使用完整 256 位算术”的关键工程优化仍未完成。
