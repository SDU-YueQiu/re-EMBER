# re-EMBER

[English](README.md) | [中文](README.zh-CN.md)

`re-EMBER` 是一个围绕 EMBER 布尔网格流水线的 C++17 原型仓库，重点是精确整数几何、局部编排、WNV/WNTV 分类和稳健的二元网格布尔运算。

## 本仓库包含什么

- `src/application/main.cpp` 提供命令行入口。
- `src/io/` 负责 OBJ/STL 的导入和导出。
- `src/core/` 包含公开的 `BoolProblem` 门面和内部的 `SubdivisionSolver`。
- `src/algorithm/`、`src/geometry/` 和 `src/math/` 放的是布尔流水线、几何原语和固定宽度整数运算。
- `src/tests/` 与 `tests/paper_experiments/` 提供单元测试和论文风格回归输入。
- `tools/profile-re-ember.ps1` 用于计时运行和可选的 Tracy 采样。

## 构建

所有构建产物都放在 `build/` 下。

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 120
cmake --build build --config Debug --target re-EMBER
```

如果缺少 `TBB`，先安装：

```powershell
vcpkg install tbb:x64-windows
```

## 运行

最小布尔测试命令：

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25
```

`.obj` 输出默认保留 n 边面；`.stl` 输出会在 I/O 边界三角化。

## CLI 参数

- `--lhs <file.obj|file.stl>` 和 `--rhs <file.obj|file.stl>` 分别指定左右操作数。
- `--op union|intersection|difference` 选择布尔运算类型。
- `--out <result.obj|result.stl>` 指定输出文件。
- `--scale <positive_integer>` 手动覆盖共享量化尺度。
- `--leaf-threshold <positive_integer>` 控制细分到叶子时的停止阈值。
- `--threads <positive_integer>` 指定应用层 task arena 大小和求解线程数；设为 `1` 可强制串行。
- `--timings-out <metrics.txt>` 会把单次运行的计时和求解摘要写到文件里。
- `--assume-lhs-nsi`、`--assume-lhs-nnc`、`--assume-rhs-nsi`、`--assume-rhs-nnc` 用于声明输入假设以加速运行；同一侧的 `NNC` 依赖 `NSI`。

应用层并行与求解器共用 `--threads` 限制：外层并行调度左右操作数，内层对顶点 AABB、顶点量化、面到多边形构造和导出片段恢复做静态分块。

## 性能脚本

`tools/profile-re-ember.ps1` 负责计时运行、Tracy 采样和报告生成。常用参数如下：

- `-Lhs` / `-Rhs` 和 `-Op` 用于跑一个明确的布尔任务。
- `-InputRoot` 用于从目录树批量跑多个 case。
- `-Out` 指定单个任务的输出文件。
- `-ExecutablePath` 直接复用已有的 `re-EMBER.exe`，不重新构建。
- `-Configuration` 选择 profiling 构建类型。
- `-Iterations`、`-TimeoutSeconds`、`-BuildTimeoutSeconds`、`-ReportTimeoutSeconds` 控制运行超时。
- `-LeafThreshold` 会传给求解器；`-Threads` 同时设置应用层 task arena 大小和求解线程数。
- `-NoTracy` 跳过 Tracy 采样，使用 `build\profile_notracy\`。
- `-EnableMathTracy` 额外打开底层 `math256` Tracy 区间，并使用 `build\profile_tracy_math\`。
- `-SkipBuild` 复用已有的 profiling 构建树。
- `-UnwrapZoneFilter` 会导出指定热点 zone 的逐事件 CSV。
- `-WorkloadPriority`、`-UsePCores` 和 `-WorkloadAffinityMask` 控制被计时进程的调度方式。

脚本会在 `build\performance\run_<timestamp>\` 下生成 `summary.txt`、`timings.csv`、`manifest.json`、`profile.log`、`report.md`、`tracy_zones.csv`、`tracy_zones_self.csv`，以及可选的 `tracy_unwrap\*.csv`。

## 备注

- `build\Debug\re-EMBER_tests.exe` 可以运行仓库测试。
- `--threads 1` 可让应用层准备和求解都强制串行，方便排查问题。
- `--timings-out <file>` 会输出单次运行的计时摘要。
