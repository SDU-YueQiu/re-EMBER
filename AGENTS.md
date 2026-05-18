# AGENT 本地上下文

本文件是当前工作树的本地 AGENT 指南。全局 memory 只能作为线索，遇到差异时以当前代码、测试和新鲜构建结果为准。

## 当前事实

- 当前流水线是 `OBJ -> 多边形集合 -> BoolProblem -> SubdivisionSolver -> resultFragments -> OBJ n 边面`。
- `BoolProblem` 是公开门面，不再暴露递归子节点；诊断入口使用 `leafSummaries()` 和 `solveMetrics()`。
- `SubdivisionSolver` 独占递归节点、AABB、局部参考点、叶片片段、分类片段和结果汇总。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部候选构造在 `path_candidate_details.h`。
- CMake 使用显式源文件列表，新增源码需要同步加入 `CMakeLists.txt`。
- Tracy 性能插桩由 `REEMBER_ENABLE_TRACY` 控制；底层 `math256` 热点桩再由 `REEMBER_ENABLE_TRACY_MATH` 单独控制，二者默认都关闭，不影响普通发布构建。
- 性能脚本入口是 `tools/profile-re-ember.ps1`；统一把计时、Tracy 捕获、报告和结果 OBJ 写到 `build\performance\run_<timestamp>\`，并按 `-NoTracy/-EnableMathTracy` 自动切换 `build\profile_*` 专用构建树；如果传 `-ExecutablePath` 则直接复用已有 `re-EMBER.exe`。

## 工作规则

- 所有构建、测试和 smoke 产物都放在 `build/`。
- 仓库自有源码的文件头和解释性注释写中文；公开接口使用 Doxygen 结构。
- 不参考旧 README 作为架构事实；如果全局 memory 仍提到 `BoolProblem::solveRecursive()` 调用链，应视为过期。
- 默认保持 OBJ 输出为 n 边面多边形集合，不主动三角化输出。
- 默认不编辑 `third_party/tracy`、`assets`、`reference`、`Doxyfile` 或构建产物。
- 每做一个阶段的改动后要及时commit，同样优先使用中文提交
- 更新功能后不仅要做代码工作，也要更新readme（如果涉及）和docs里的文档相关内容

## 代码约定

- 当前代码、测试和新鲜构建结果是事实来源；旧 README、旧会话和全局记忆只能作为线索。
- 仓库自有源码的文件头和解释性注释使用中文；公开接口使用 Doxygen 结构，保留 `@brief`、`@param`、`@return`、`@retval`、`@note` 等标签。
- 不把 `BoolProblem` 当递归节点使用；运行时细分状态属于 `SubdivisionSolver`。
- `path_candidates.h` 保留公开候选类型和模板枚举入口；内部路径构造细节在 `path_candidate_details.h`。
- 默认不改 `third_party/tracy`、`assets`、`reference`、`Doxyfile` 和构建产物。
- 当前几何核心基于固定宽度整数运算；新增高阶代数或齐次点比较前必须先确认 256 位中间结果预算。
- 底层数学基础以 `reference/Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs.md` 为准，尤其是 3.2、Table 1、4.1 和 4.5：核心图元优先限制在 `plane_from_points`、`are_planes_parallel`、`intersect_3_planes`、`classify_vertex/signed_distance`、AABB 轴平面和已有平面裁剪。
- 优化顺序先参考论文已知方向，再做数学闭包分析，最后落到代码；不要凭热点直接引入未经论文证明的齐次点平均、中点、任意点差、坐标平面反推或由齐次输出点构造新平面。
- 论文指出 `classify_vertex` 应复用 4D 齐次交点做点积，`intersect_3_planes` 是四个 3x3 行列式，`plane_from_points` 的 gcd/规范化主要属于导入或可能越界的平面构造；核心布尔分支不得依赖 `int512/cpp_int` 决策。
- 自定义 256 位定长算术先作为窄接口 backend 演进并接受 oracle 测试，不能一次性替换全局 `Integer` 或绕过论文允许图元边界。

## 验证命令

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 120
cmake --build build --config Debug --target re-EMBER
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25
```

## 性能测试

Tracy 采样优先使用 `RelWithDebInfo`：

```powershell
vcpkg install tracy[cli-tools]:x64-windows
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo
```

如果要抓 `math256` 这类底层代数热点，再额外打开：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 -Configuration RelWithDebInfo -EnableMathTracy
```

只有在确认热点落在 `determinant3x3`、`gcdMagnitude`、`primitiveHomPoint` 这类底层函数时才开 `REEMBER_ENABLE_TRACY_MATH` / `-EnableMathTracy`；平时保持关闭，避免给正常 profiling 和普通运行带来额外开销。

只要端到端时间和 `BoolSolveMetrics` 时可加 `-NoTracy`；未显式传 `-Configuration` 时脚本默认使用 `Release` 并自动切到 `build\profile_notracy\`，普通构建仍保持 `REEMBER_ENABLE_TRACY=OFF`、`REEMBER_ENABLE_TRACY_MATH=OFF`。

如果只想比较固定工件和固定位姿下的不同布尔运算，保持 `-Lhs/-Rhs` 不变，只切换 `-Op`：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\profile-re-ember.ps1 `
  -Configuration RelWithDebInfo -SkipBuild -Iterations 1 `
  -Lhs assets\visual_test\lhs.obj `
  -Rhs build\performance\run_<existing>\visual_test_overlap_pose_rhs.obj `
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
- `leaf_classification_centroid_point_count`、`leaf_classification_inset_point_attempt_count`、`leaf_classification_trace_attempt_count`、`leaf_classification_axis_path_attempt_count`、`leaf_classification_plane_replacement_path_attempt_count`：叶片分类阶段的论文两阶段尝试量。
- `tracy_zones.csv` 看 inclusive，总体阶段耗时；`tracy_zones_self.csv` 看 self，定位真实热点；领域规模仍以 `timings.csv` / `metrics.txt` 为准。
