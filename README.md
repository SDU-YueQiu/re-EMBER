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

`BoolProblem` 现在只暴露应用需要的门面接口：`setOperation`、`setLeafPolygonThreshold`、`addPolygon`、`setPolygons`、`setOperands`、`solve`、`isSolved`、`isDiscarded`、`resultFragments` 和 `leafSummaries`。递归子问题状态属于 `SubdivisionSolver` 内部实现；测试或诊断需要看叶子结构时使用 `leafSummaries()`。

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
build\Debug\re-EMBER.exe --lhs <left.obj> --rhs <right.obj> --op union|intersection|difference --out <result.obj> [--scale <positive_integer>] [--leaf-threshold <positive_integer>]
```

参数说明：

- `--lhs`、`--rhs`：左右输入 OBJ。
- `--op`：二元布尔运算，支持 `union`、`intersection`、`difference`。
- `--out`：输出 OBJ 路径。
- `--scale`：可选的显式十进制量化尺度；不传时会为两输入选择共享尺度。
- `--leaf-threshold`：可选的叶子停止细分阈值，默认 `25`。

导出默认保持多边形集合形态，直接写 OBJ n 边面。三角化、拓扑缝合和 T 形连接恢复属于调用方后处理。

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
- 空间细分仍使用当前实现中的中点切分策略，并未实现论文 4.5 的完整优化策略。
