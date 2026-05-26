# re-EMBER

[English](README.md) | [中文](README.zh-CN.md)

`re-EMBER` 是一个围绕 EMBER 布尔网格流水线的 C++17 原型仓库，重点是精确整数几何、局部编排、WNV/WNTV 分类和稳健的二元网格布尔运算。

## 几何 kernel 契约

核心布尔路径正在收敛到 EMBER 与 Nehring-Wirxel et al. 2021 使用的固定宽度齐次整数模型。核心几何代码只能通过论文图元集合构造和分类：整数平面、AABB 轴对齐平面、三平面齐次交点、齐次点对平面分类、整数顶点 signed-distance，以及针对已有合法平面的裁剪。

OBJ/STL 输入默认会量化到 signed 26-bit 整数坐标，除非显式传入 `--scale`。核心求解器内的 `int512_t` 和 `cpp_int` 不允许决定算法分支；它们只用于测试 oracle、调试诊断和 I/O 后处理。任意齐次点平均、中点/差分、从分数齐次点反推坐标平面、从输出齐次顶点构造新平面等不满足 256-bit 闭包预算的操作，必须显式失败，不能作为静默 fallback。

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
cmake --preset default
cmake --build --preset default-app
```

默认本地构建只要求 `reember_lib` 和 `re-EMBER` 所需的核心依赖。如果已经设置 `VCPKG_ROOT`，`CMakeLists.txt` 会自动接入 vcpkg toolchain。仓库内的 `CMakePresets.json` 会把 generator 固定为 Ninja，这样现有的 `clang-cl` 自动发现路径就能直接生效，不需要手动传编译器变量。

测试现在单独放在自己的 preset 和构建树里：

```powershell
cmake --preset tests
cmake --build --preset tests
ctest --preset default
```

默认构建先安装这些核心包：

```powershell
vcpkg install tinyobjloader tbb boost-multiprecision
scoop install llvm
```

`re-EMBER_verify` 和 `visual-test` 是额外工具目标，默认关闭，这样普通 configure 不会强制要求 `CGAL`、`Eigen3` 或 `libigl`。每个 preset 都会落到 `build/` 下独立的子目录，避免共用一个大而混杂的构建树。需要时再显式开启：

```powershell
cmake --preset verify
cmake --build --preset verify
cmake --preset visual-test
cmake --build --preset visual-test
```

这些可选目标额外需要：

```powershell
vcpkg install cgal eigen3 libigl
```

## 运行

最小布尔测试命令：

```powershell
build\default\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 50
```

`.obj` 输出保留 n 边面；`.stl` 输出会在 I/O 边界三角化。`--output-topology conforming` 会在导出前启用精确 T-junction 修复。该模式主要用于调试和 MeshLab 检查，不用于性能计时；它可能明显慢于 raw 输出。共面合并和 Nef 正则化输出仍在应用层 CLI 禁用。

## Oracle 校验工具

`re-EMBER_verify` 会把 `BoolProblem::resultFragments()` 候选结果和缓存的 CGAL Nef oracle 做集合相等校验：

```powershell
cmake --preset verify
cmake --build --preset verify
build\verify\re-EMBER_verify.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --leaf-threshold 50 --oracle-cache-dir build\oracle_cache\nef
```

oracle 的精确性边界是 re-EMBER 已经量化后的 `Polygon256` 输入；它不声明验证原始浮点 OBJ/STL 在 CAD 语义上的真实布尔结果。默认缓存目录是 `build\oracle_cache\nef\`；需要强制重算时传 `--refresh-oracle`。`--candidate-mode fragments-nef|export-conforming|export-nef` 可选择用原始结果片段或 verifier 内部诊断候选构造候选结果；这些模式不代表应用层输出后处理已经启用，也不改变 oracle cache key。

校验工具也支持独立于性能脚本的批处理模式：

```powershell
build\verify\re-EMBER_verify.exe --batch-input-root tests\paper_experiments\inputs\small --op difference --batch-out-dir build\verify_batch_small
build\verify\re-EMBER_verify.exe --batch-manifest tests\paper_experiments\manifest.csv --batch-out-dir build\verify_batch_manifest
```

`--batch-input-root` 会扫描子目录，每个 case 必须有且只有一个 `lhs.obj|stl` 和一个 `rhs.obj|stl`，布尔运算使用全局 `--op`。`--batch-manifest` 支持 `name,lhs,rhs,op`，也兼容论文 manifest 的 `pair_id,lhs_path,rhs_path,operation`。批处理输出包含 `verification.csv`、`batch_report.txt`、`cache/*.candidate.txt` 和 `reports/*.report.txt`。`--batch-size` 默认等于 CPU 逻辑线程数，有效范围是 `1..CPU 逻辑线程数`，超过上限会直接报错。每个 batch 内 solve/cache 阶段按 workload 顺序串行执行，但单个 workload 内部仍按 `--threads` 最大并行；候选缓存写完后，同一 batch 的 CGAL compare 阶段按 workload 并行。

## CLI 参数

- `--lhs <file.obj|file.stl>` 和 `--rhs <file.obj|file.stl>` 分别指定左右操作数。
- `--op union|intersection|difference` 选择布尔运算类型。
- `--out <result.obj|result.stl>` 指定输出文件。
- `--scale <positive_integer>` 手动覆盖共享量化尺度。
- `--leaf-threshold <positive_integer>` 控制细分到叶子时的停止阈值；默认值为 50。
- `--threads <positive_integer>` 指定应用层 task arena 大小和求解线程数；设为 `1` 可强制串行。
- `--output-topology raw|conforming` 选择应用层导出拓扑。`raw` 直接写 `resultFragments()`，并跳过只供 conforming 修复使用的拓扑元数据；`conforming` 会全局查找并插入落在其他面边上的已有顶点以消除 T-junction。`conforming` 是精确但较慢的调试/检查模式，不应用于性能测试。共面合并和 Nef 输出仍禁用。
- `--timings-out <metrics.txt>` 会把单次运行的计时和求解摘要写到文件里。
- `--assume-lhs-nsi`、`--assume-lhs-nnc`、`--assume-rhs-nsi`、`--assume-rhs-nnc` 用于声明输入假设以加速运行；同一侧的 `NNC` 依赖 `NSI`。

应用层并行与求解器共用 `--threads` 限制：外层并行调度左右操作数，内层对顶点 AABB、顶点量化、面到多边形构造和导出片段恢复做静态分块。

## 性能脚本

`tools/profile-re-ember.ps1` 负责计时运行、Tracy 采样和报告生成。常用参数如下：

- `-Lhs` / `-Rhs` 和 `-Op` 用于跑一个明确的布尔任务。
- `-InputRoot` 用于从目录树批量跑多个 case。
- `-UsePaperExperimentSet` 会按 manifest 在当前 run 目录下生成论文实验批量输入。当前纳入仓库的论文 corpus 共 100 个 workload：23 个 small、43 个 medium、34 个 large。默认快速批量仍选择 10 个 small、10 个 medium 和 2 个 large；全量运行使用 `-PaperSmallCount 23 -PaperMediumCount 43 -PaperLargeCount 34`。
- `-Out` 指定单个任务的输出文件。
- `-ExecutablePath` 直接复用已有的 `re-EMBER.exe`，不重新构建。
- `-Configuration` 选择 profiling 构建类型。只计时的 `-NoTracy` 默认使用 `Release`；Tracy 采样默认使用 `RelWithDebInfo`。
- `-Iterations`、`-TimeoutSeconds`、`-BuildTimeoutSeconds`、`-ReportTimeoutSeconds` 控制运行超时。
- `-LeafThreshold` 会传给求解器，默认值为 50；`-Threads` 同时设置应用层 task arena 大小和求解线程数。
- `-NoTracy` 跳过 Tracy 采样，使用 `build\profile_clang_notracy\`。
- `-EnableMathTracy` 额外打开底层 `math256` Tracy 区间，并使用 `build\profile_clang_tracy_math\`。
- `-SkipBuild` 复用已有的 profiling 构建树。
- `-UnwrapZoneFilter` 会导出指定热点 zone 的逐事件 CSV。
- `-WorkloadPriority`、`-UsePCores` 和 `-WorkloadAffinityMask` 控制被计时进程的调度方式。

性能脚本只负责计时和 profiling；oracle 校验请直接调用 `re-EMBER_verify`。脚本会在 `build\performance\run_<timestamp>\` 下生成 `summary.txt`、`timings.csv`、`manifest.json`、`profile.log`、`report.md`、`tracy_zones.csv`、`tracy_zones_self.csv`，以及可选的 `tracy_unwrap\*.csv`。

## 备注

- 默认 preset 现在会配置成 `Release`；每个 preset 会写入自己的子目录，例如 `build\default\`、`build\tests\`、`build\verify\`、`build\visual-test\`。
- `build\tests\re-EMBER_tests.exe` 可以运行仓库测试。
- `build\visual-test\visual-test.exe` 的 Ember 面板同样提供 `raw` / `conforming` 输出拓扑。
- `--threads 1` 可让应用层准备和求解都强制串行，方便排查问题。
- `--timings-out <file>` 会输出单次运行的计时摘要。
