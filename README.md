# re-EMBER

`re-EMBER` 是一个围绕 EMBER（Exact Mesh Booleans via Efficient & Robust Local Arrangements）论文复现的 C++17 原型仓库。当前目标是验证精确整数几何、局部编排、WNV/WNTV 分类和二元网格布尔流水线，而不是提供完整建模软件或通用网格修复器。

## 当前状态

当前代码已经具备一条可运行的基础布尔流水线：

```text
OBJ/STL -> 共享 scale + 浮点输入AABB -> 多边形集合 -> BoolProblem(校验/懒顶点缓存) -> SubdivisionSolver -> resultFragments -> OBJ n 边面 / STL 三角面
```

重要边界如下：

- `src/application/main.cpp`：命令行入口，只负责解析参数、按扩展名读写 OBJ/STL、组装二元布尔任务。
- `src/io/io.h/.cpp`：OBJ 读取、基于 vendored `third_party/stl_reader/stl_reader.h` 的 ASCII/binary STL 读取、共享量化尺度选择、scale 后浮点输入 AABB 计算、`Polygon256` 多边形集合构建，以及 OBJ n 边面 / STL 三角面导出。
- `src/core/bool_problem.h/.cpp`：公开布尔问题门面，接收输入、保存配置、调用内部求解器并缓存结果。
- `src/core/subdivision_solver.h/.cpp`：内部细分求解器，独占递归节点、AABB、参考点、叶片片段和结果汇总。
- `src/core/leaf_classifier.cpp`：叶片局部编排后的 WNV 分类与结果筛选。
- `src/algorithm/`：局部 BSP、叶子编排、WNV 路径追踪和分类路径候选。
- `src/geometry/`、`src/math/`：固定宽度整数几何 primitive、平面点表示、裁剪和基础代数。
- `src/tests/`：仓库自定义断言测试，不依赖第三方测试框架。
- `tests/paper_experiments/`：从论文实验复制来的 oracle-success 端到端回归输入。
- `tools/profile-re-ember.ps1`：端到端性能测试、Tracy 捕获和报告入口。

`BoolProblem` 现在只暴露应用需要的二元门面接口：`setOperation`、`setOperandAssumptions`、`setThreadCount`、`setOperands`、`solve(sceneAABB)`、`isDiscarded`、`resultFragments`、`leafSummaries` 和 `solveMetrics`。`setOperands()` 会统一覆写输入多边形的 `WNTV`，强制收敛到 `lhs={1,0}`、`rhs={0,1}` 的二元约定，不再暴露“直接注入任意带标签 polygon 集合”的公开入口。根场景 AABB 在共享 `scale` 选定后直接由输入网格浮点顶点经 `floor(coord * scale)` / `ceil(coord * scale)` 生成，调用方合并左右输入后加 margin 并传给 `solve(sceneAABB)`；`SubdivisionSolver` 不再从 256 位多边形顶点反推根 AABB。`Polygon256` 顶点缓存改为按需构造，首次调用 `vertex()` / `vertices()` 时生成，后续复用。递归子问题状态属于 `SubdivisionSolver` 内部实现；它会在递归返回路径上一次性向上汇总 `resultFragments / leafSummaries / solveMetrics`，并尽早释放子树中间状态。当前并行实现使用 `oneTBB` 的 sibling-task work stealing：每次细分后当前线程继续一个 child，另一个 child 作为 task 提交给共享运行时，最终仍按 `left -> right` 固定顺序 merge。`BoolProblem` 仍然不是外部并发安全对象；一个实例不能被多个线程同时读写或复用。每个 `BoolProblem` 实例只允许执行一次 `solve()`；无论成功还是抛错，后续都不能再次 `solve()` 或修改配置。

## 构建与测试

所有构建都放在 `build/` 下：

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
cmake --build build --config Debug --target re-EMBER
```

`ctest` 现在包含 [论文实验测试](docs/paper-experiment-tests.md)。这些测试直接运行 `re-EMBER` CLI，输入来自 `tests/paper_experiments/`，产物写到 `build/paper_experiment_tests/`。它们是 oracle-success 样本，并启用论文实验使用的 NSI/NNC 假设；不设置 expected-fail，当前默认 10 个 small pair 都应通过，任一样本失败都表示当前流水线回归。CLI 回归默认使用 `REEMBER_CTEST_THREADS`，其默认值是当前构建机的逻辑处理器数；只有专门排查串行分支时才显式设为 `1`。

当前仓库默认通过 vcpkg toolchain 解析 `oneTBB`。如果当前机器还没有这个依赖，先执行：

```powershell
vcpkg install tbb:x64-windows
```

如果使用 MSVC，现在可以通过 `REEMBER_MSVC_ARCH` 显式控制 `/arch` 档位：

```powershell
cmake -S . -B build -DREEMBER_MSVC_ARCH=AUTO
cmake -S . -B build -DREEMBER_MSVC_ARCH=AVX2
cmake -S . -B build -DREEMBER_MSVC_ARCH=OFF
```

可选值为 `AUTO`、`OFF`、`SSE2`、`SSE4.2`、`AVX`、`AVX2`、`AVX512`、`AVX10.1`、`AVX10.2`。`AUTO` 会在当前构建机上同时检测“编译器是否接受该 `/arch` 开关”和“当前 CPU/OS 运行时是否支持该 ISA”，然后自动选择最高可用档位；在不支持请求 ISA 的机器上，显式指定更高档位会直接配置失败。

Tracy 性能插桩是编译期可选项，默认关闭；普通 Debug/Release/RelWithDebInfo 构建不会包含 Tracy 头、链接库或 profiler 线程。性能脚本会按输入参数自动选择专用 profiling 构建树：`build\profile_tracy\` 打开 `REEMBER_ENABLE_TRACY=ON`，`build\profile_tracy_math\` 额外打开 `REEMBER_ENABLE_TRACY_MATH=ON`，`build\profile_notracy\` 则保持两者都关闭，因此不会污染普通 `build\` 构建。

基础 CLI smoke：

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25 --threads $env:NUMBER_OF_PROCESSORS
```

也可以直接让 STL 走完整 CLI 边界：

```powershell
build\Debug\re-EMBER.exe --lhs build\test-output\lhs_box.stl --rhs build\test-output\rhs_box.stl --op difference --out build\boolean_smoke.stl --leaf-threshold 25 --threads $env:NUMBER_OF_PROCESSORS
```

I/O 中较慢的回归用环境变量开启：

```powershell
$env:REEMBER_RUN_EXPENSIVE_IO_TESTS = '1'
build\Debug\re-EMBER_tests.exe
```

## 命令行用法

```powershell
build\Debug\re-EMBER.exe --lhs <left.obj|left.stl> --rhs <right.obj|right.stl> --op union|intersection|difference --out <result.obj|result.stl> [--scale <positive_integer>] [--leaf-threshold <positive_integer>] [--threads <positive_integer>] [--timings-out <metrics.txt>] [--assume-lhs-nsi] [--assume-lhs-nnc] [--assume-rhs-nsi] [--assume-rhs-nnc]
```

参数说明：

- `--lhs`、`--rhs`：左右输入网格，当前支持 `.obj`、`.stl`。
- `--op`：二元布尔运算，支持 `union`、`intersection`、`difference`。
- `--out`：输出网格路径；`.obj` 保持 n 边面，`.stl` 会导出三角面。
- `--scale`：可选的显式十进制量化尺度；不传时会为两输入选择共享尺度。
- `--leaf-threshold`：可选的叶子停止细分阈值，默认 `25`。
- `--threads`：可选的总线程数；不传时自动选择并发度，`1` 表示强制串行。常规测试和性能测试应使用多线程，串行只用于专项排查。
- `--timings-out`：把本次运行的时间与高层求解统计写到文本文件。
- `--assume-*-nsi`、`--assume-*-nnc`：声明输入操作数满足 NSI/NNC 假设，只用于性能优化；错误声明会破坏正确性。

导出到 `.obj` 时默认保持多边形集合形态，直接写 n 边面；导出到 `.stl` 时会在 I/O 边界层按每个 `Polygon256` 的有序顶点做扇形三角化。更高层的拓扑缝合和 T 形连接恢复仍属于调用方后处理。

## visual-test

如果启用了 `REEMBER_BUILD_VISUAL_TEST`，可以单独构建交互式可视化程序：

```powershell
cmake --build build --config Debug --target visual-test
build\Debug\visual-test.exe
```

当前 visual-test 默认按 stem 查找 `assets\visual_test\workpiece_block` 和 `assets\visual_test\tool_box`，后缀接受 `.obj` 或 `.stl`，并优先命中 `.obj`。这套 UI 现在按当前两模型的 AABB 自动对齐中心，再把 `dx/dy/dz` 解释为相对偏移，因此像 `classic_fandisk` 这类原始坐标远离原点的模型也能直接拖回到可相交范围。`tool scale` 会围绕刀具自身 AABB 中心做统一缩放，适合在不同尺寸模型之间快速调到更容易观察和比较的比例。

交互规则也改成了“两阶段”，并保留可切换的连续求解：

- 调整 `tool scale`、`dx/dy/dz`、旋转、engine、operation 或 leaf threshold 时，只更新刀具预览，不再自动执行布尔求解。
- 点击 `Run Boolean` 后，才会基于当前预览位姿执行一次真正的布尔运算。
- 勾选 `continuous solve` 后，会恢复成旧模式：每次参数变动后自动重新求解；不勾选时仍然保持手动 `Run Boolean`。
- `Center` 和 `X/Y/Z min/max` 预设基于当前两模型的 AABB 生成，适合快速构造“居中重叠”或“边界接触”的初始姿态。

推荐的大模型测试集见 [docs/benchmark-models.md](docs/benchmark-models.md)。

## 性能测试

### 可选 Tracy 构建

首次使用 Tracy 报告前，先安装 vcpkg 的 Tracy CLI 工具：

```powershell
vcpkg install tracy[cli-tools]:x64-windows
```

性能脚本默认会配置 `build\profile_tracy\` 并构建 `re-EMBER`，未显式传 `-Threads` 时会使用当前机器的全部逻辑处理器：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo
```

如果要强制固定其它求解线程数，直接加 `-Threads`，脚本会透传给 `re-EMBER.exe` 并写进 profiling 产物：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -NoTracy -Threads $env:NUMBER_OF_PROCESSORS
```

如果对应模式的 profiling 构建树已经由脚本准备过，可以加 `-SkipBuild` 只运行 workload：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -SkipBuild
```

如果要追 `math256` 里的底层代数热点，例如 `determinant3x3`、`gcdMagnitude`、`primitiveHomPoint`，直接给脚本加 `-EnableMathTracy`；它会自动切到 `build\profile_tracy_math\`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -EnableMathTracy
```

`REEMBER_ENABLE_TRACY_MATH` / `-EnableMathTracy` 默认应该保持关闭。只有确认瓶颈已经落到低层数学工具时才打开，否则这些超高频 zone 本身会带来一定额外开销。

只想看端到端时间和 `BoolSolveMetrics`，不需要 Tracy zone 时使用 `-NoTracy`；脚本会自动改用 `build\profile_notracy\`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -NoTracy
```

如果后台负载让 timing 抖动太大，可以只对计时目标 `re-EMBER.exe` 固定调度策略。`-WorkloadPriority` 支持 `Normal|AboveNormal|High`；`-UsePCores` 会按 Windows 暴露的 `EfficiencyClass` 自动选择最高性能逻辑核；如果你已经知道本机的逻辑核 mask，也可以直接传 `-WorkloadAffinityMask`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -NoTracy `
  -WorkloadPriority High -UsePCores
```

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -NoTracy `
  -WorkloadPriority High -WorkloadAffinityMask 0xFFF
```

这些参数只作用在被计时的 `re-EMBER.exe` 进程，不会把 `cmake`、`tracy-capture`、`tracy-csvexport` 一起提权。脚本会把最终采用的优先级、affinity mask 和自动选中的逻辑核记录进 `profile.log`、`manifest.json` 和 `summary.txt`。如果当前机器不能可靠暴露 P/E 核信息，就改用显式 `-WorkloadAffinityMask`。

如果要对某几个热点 zone 导出逐事件明细，可以额外指定 `-UnwrapZoneFilter`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 `
  -UnwrapZoneFilter chooseSubdivisionSplit,appendSplitChildPolygons
```

默认 workload 包含：

- `icosphere80_toolbox_difference`
- `visual_block_toolbox_overlap_intersection`
- `visual_block_toolbox_visual_test_default_difference`

`tools\profile-re-ember.ps1` 现在会先把当前 `assets\visual_test\tool_box.obj` 按 AABB 中心自动对齐到 `workpiece_block.obj`，再生成 visual 默认位姿与 overlap 位姿的临时 OBJ，因此把 `assets\visual_test` 替换成更大模型后，不需要再手改脚本里的平移常量。

注意：默认 visual intersection 和 visual difference 使用的是不同刀具位姿，不能直接拿它们比较三种布尔运算的快慢。

如果要比较同一组工件、同一位姿下的 `union|intersection|difference`，请固定 `-Lhs/-Rhs`，只切换 `-Op`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 `
  -Lhs assets\visual_test\workpiece_block.obj `
  -Rhs build\perf\run_<existing>\visual_test_overlap_pose_tool.obj `
  -Op difference
```

### 看哪些文件

每次运行会生成一个 `build\perf\run_<timestamp>\` 目录。最常用的文件是：

- `summary.txt`：每个 workload 的总体摘要。
- `timings.csv`：逐次迭代的结构化表格。
- `timing_*.metrics.txt`：单个 workload 的时间和高层统计。
- `tracy_traces\*.tracy`：Tracy 原始捕获。
- `tracy_zones.csv`：`tracy-csvexport` 导出的 inclusive zone 进入次数和耗时统计。
- `tracy_zones_self.csv`：`tracy-csvexport -e` 导出的 self-time zone 统计。
- `tracy_unwrap\*.csv`：按 `-UnwrapZoneFilter` 选出的逐事件 zone 明细。
- `report.md`：问题规模、pipeline inclusive 时间、策略交叉校验和按 workload 的 self hot zones。
- `profile.log`：脚本实际执行的命令与日志。

### 怎么读结果

先看时间拆分：

- `read_ms`：输入网格读取时间。
- `prepare_ms`：共享尺度选择和多边形集合构建时间。
- `solve_ms`：真正的布尔求解时间。
- `export_ms`：结果网格导出时间。
- `effective_thread_count`：本次求解实际参与的总线程数。

再看高层工作量：

- `node_count`、`internal_node_count`、`leaf_node_count`、`max_depth`：递归树规模。
- `total_polygon_count`：整棵细分树访问到的节点多边形数累计值；比 `input_polygons` 更接近真实 subdivision 工作量。
- `leaf_fragment_count`、`classified_fragment_count`、`result_fragment_count`：叶子阶段与最终输出规模。
- `constant_discard_count`：子节点在创建前被常量布尔指示函数剪枝的次数。
- `leaf_threshold_stop_count`、`aabb_not_splittable_stop_count`：递归停止原因。
- `wntv_aware_split_count`、`center_range_split_count`、`midpoint_split_count`：切分策略命中分布。
- `parallel_sibling_spawn_count`：把 sibling 子树交给 `oneTBB` 并行执行的次数。

对子参考点传播和叶片分类，重点看候选放大量：

- `child_reference_candidate_count`：子参考点传播阶段生成的候选总数。
- `child_reference_fast_candidate_count`、`child_reference_exhaustive_candidate_count`：快速/穷举子参考点候选分布。
- `child_reference_candidate_tried_count`：真正进入 trace 的子参考点候选数。
- `child_reference_fast_candidate_tried_count`、`child_reference_exhaustive_candidate_tried_count`：快速/穷举子参考点实际 trace 数。
- `child_reference_trace_count`：成功传播的子参考点数。
- `leaf_classification_centroid_point_count`：叶片分类阶段命中的重心启发式目标点数。
- `leaf_classification_inset_point_attempt_count`：叶片分类阶段按论文 inset 规则尝试构点的总次数。
- `leaf_classification_trace_attempt_count`：叶片分类实际尝试的 path trace 总数。
- `leaf_classification_axis_path_attempt_count`、`leaf_classification_plane_replacement_path_attempt_count`：论文两阶段路径尝试分布。
- `leaf_classification_candidate_generated_count`、`leaf_classification_candidate_unique_count`：叶片分类候选枚举生成量和按路径端点签名去重后真正保留量。
- `leaf_classification_candidate_duplicate_skip_count`、`leaf_classification_candidate_rejected_count`：候选因 trace 级路径重复或结构非法且无法局部修复而跳过的数量；换平面构造级签名缓存发生在生成路径前，不单独计入该字段。
- `leaf_classification_candidate_repair_attempt_count`、`leaf_classification_candidate_repair_success_count`：候选路径终点/连续性不满足 surface trace 前置条件时，局部重建路径的尝试和成功次数。
- `leaf_classification_*_success_count`、`leaf_classification_*_path_invalid_count`、`leaf_classification_*_input_invalid_count`、`leaf_classification_*_fail_count`：按 `centroid_axis`、`inset_replacement`、`bridge_rescue` 三个阶段拆分的 trace 状态分布；`INPUT_INVALID` 应优先解释为候选构造问题，而不是普通几何退化路径。

这些字段比单看 `node_count` 更能解释“为什么只有几十个节点却仍然很慢”。

### Tracy 的正确读法

- `tracy_zones.csv` 是 inclusive time，适合看完整 pipeline 或父阶段总耗时。
- `tracy_zones_self.csv` 是 self time，适合找真正热点；`solveRecursive` 这类父 zone 要优先看 self 视角。
- `report.md` 默认先用 inclusive 看 pipeline，再用 self 看每个 workload 的热点。
- 领域规模仍以 `timings.csv` / `metrics.txt` 中的 `BoolSolveMetrics` 为准；`report.md` 会把关键策略的 `metric_count` 和 `zone_count` 做交叉校验。

## 已知限制

- 当前公开流水线面向二元布尔；多网格表达式和通用布尔表达式解析不在计划内。
- `BoolProblem` 公开输入固定为左右两个操作数；自定义混合 `WNTV` 标签或多输入聚合不再作为支持目标。
- OBJ/STL 导入都只保留几何位置，不保留法线、UV、材质或拓扑连通关系。
- 当前 STL 导入支持 ASCII STL 和 binary STL，并忽略 facet normal / attribute byte count。
- CLI 为了跑通真实网格，会对量化后非共面的输入面启用扇形三角化；库 API 默认仍是严格构造策略。
- 输出到 `.obj` 时保持 n 边面多边形集合；输出到 `.stl` 时会导出三角面，但都不保证进行全局拓扑恢复。
