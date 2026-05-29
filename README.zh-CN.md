# re-EMBER

[English](README.md) | [中文](README.zh-CN.md)

re-EMBER 是一个 C++17 开源**精确**网格布尔运算库，支持三角面片汤的并集、交集和差集运算，生成水密多边形输出，所有几何谓词均为精确整数运算。在所有 benchmark 中，端到端耗时平均仅为 **QuickCSG 的约 2 倍**——而后者不保证输出正确性。

## 安装

### FetchContent (CMake 3.24+)

```cmake
include(FetchContent)
FetchContent_Declare(
  re-EMBER
  GIT_REPOSITORY https://github.com/Yueq2003/Ember.git
  GIT_TAG v0.1.0
)
FetchContent_MakeAvailable(re-EMBER)
target_link_libraries(my_app PRIVATE reember::lib)
```

### 手动构建

```powershell
# 依赖
vcpkg install tinyobjloader tbb boost-multiprecision
scoop install llvm

cmake --preset default && cmake --build --preset default-app
cmake --install build/default --prefix <install-path>
```

然后在你的 CMakeLists.txt 中：

```cmake
find_package(re-EMBER REQUIRED)
target_link_libraries(my_app PRIVATE reember::lib)
```

## 快速开始

```powershell
# 构建并运行
cmake --preset default && cmake --build --preset default-app

# 运行
build\default\re-EMBER.exe --lhs A.obj --rhs B.obj --op difference --out result.obj
```

依赖安装及验证器、可视化等可选 preset 见[构建](#构建)章节。

## 性能

对比 CGAL Nef（精确基线）、Mesh Arrangement / libigl（精确）、QuickCSG / mesh_cs（快速但非精确）。硬件：Intel i7-12700H，64 GB DDR5，Windows 11。

### 通用 Benchmark — 100 对

从 1000 对候选模型中按面数分层抽样：小规模 23 对（1k–5k 面）、中规模 43 对（5k–20k 面）、大规模 34 对（20k–100k 面）。固定二元差运算，每对重复 3 次，叶阈值=25。

| 算法 | 精确 | 中位 | 几何平均 | 最大 | 峰值内存 |
|------|------|------|----------|------|----------|
| **re-EMBER** | 是 | **127 ms** | **142 ms** | 778 ms | 118 MiB |
| QuickCSG | 否 | 53 ms | 63 ms | 380 ms | 16 MiB |
| Mesh Arrangement | 是 | 801 ms | 830 ms | 4392 ms | 52 MiB |
| CGAL Nef | 是 | 1358 ms | 1603 ms | 33454 ms | 151 MiB |

全部 300 次运行（100 对 × 3 次）无失败、无超时。

### 高面数工件 + 低面数刀具 — 25 对

25 个 Thingi10K 模型（10–26 万面）为工件，各配 96 面圆柱体刀具。差运算。

| 算法 | 精确 | 中位 | 几何平均 | 最大 | 峰值内存 |
|------|------|------|----------|------|----------|
| **re-EMBER** | 是 | **661 ms** | **703 ms** | 1097 ms | 756 MiB |
| QuickCSG | 否 | 381 ms | 404 ms | 708 ms | 81 MiB |
| Mesh Arrangement | 是 | 2376 ms | 2439 ms | 4715 ms | 280 MiB |
| CGAL Nef | 是 | 16937 ms | 17971 ms | 33238 ms | 1427 MiB |

25/25 全部成功。re-EMBER 中位耗时为 CGAL Nef 的 3.9%、Mesh Arrangement 的 27.8%。

## 正确性

### 精确性 — Oracle 验证

通过 `re-EMBER_verify` 将输出与同批量化输入上的 CGAL Nef oracle 做集合等价校验。100 对 benchmark 样本。

| 已完成 | 通过 | 失败 |
|--------|------|------|
| 82 | **68** | **0** |

14 对在 Nef 比较阶段超时（CGAL Nef 在大模型上开销过高），18 对大规模样本未测。**所有完成验证的样本零失败。**

### 鲁棒性 — 缺陷输入

10 组含自交、共享面/边/顶点、嵌套/重合壳、薄壳和近共面切割的合成模型对。**关闭**输入假设。

**10/10 全部成功。** 中位 20 ms，最大 26 ms。

## 特性

- **精确算术** — 256 位定宽整数谓词，无浮点鲁棒性问题
- **保留多边形输出** — 输出 n-gon（三角形 60%、四边形 30%、n-gon 6%），不强制三角化
- **水密结果** — 可选 conforming T-junction 修复
- **并行** — 基于 TBB 的 sibling 并行细分，可配置线程数
- **OBJ + STL** — 支持 n-gon `.obj` 和三角化 `.stl` 的导入导出
- **输入假设** — 可声明 NSI/NNC 属性以加速运行

## 构建

全部 preset 使用 Ninja + clang-cl。产物在 `build/<preset>/` 下。

| Preset | 目标 | 额外依赖 |
|--------|------|----------|
| `default` | `re-EMBER.exe` CLI | — |
| `tests` | `re-EMBER_tests.exe` + CTest | — |
| `verify` | `re-EMBER_verify.exe`（CGAL Nef 验证器） | `cgal eigen3` |
| `visual-test` | `visual-test.exe`（libigl 交互查看） | `cgal eigen3 libigl` |

```powershell
# 核心依赖
vcpkg install tinyobjloader tbb boost-multiprecision
scoop install llvm

# 默认构建
cmake --preset default && cmake --build --preset default-app

# 测试
cmake --preset tests && cmake --build --preset tests && ctest --preset default

# 验证器（需 CGAL）
vcpkg install cgal eigen3
cmake --preset verify && cmake --build --preset verify
```

## CLI

```
re-EMBER.exe --lhs <file> --rhs <file> --op union|intersection|difference
             [--out <file>] [--scale <int>] [--leaf-threshold <int>]
             [--threads <int>] [--output-topology raw|conforming]
             [--timings-out <file>]
             [--assume-lhs-nsi] [--assume-lhs-nnc]
             [--assume-rhs-nsi] [--assume-rhs-nnc]
```

- `.obj` 输出保留 n-gon；`.stl` 在 I/O 边界三角化
- `--leaf-threshold` 控制细分深度（默认 50，越小树越深）
- `--output-topology conforming` 启用 T-junction 精确修复（慢，用于检查）
- `--threads 1` 全局串行

### Oracle 验证器

```powershell
# 单对
build\verify\re-EMBER_verify.exe --lhs A.obj --rhs B.obj --op difference

# 批量
build\verify\re-EMBER_verify.exe --batch-manifest manifest.csv --batch-out-dir results/
```

验证器在相同量化输入上将 re-EMBER 结果片段与 CGAL Nef 对照做集合等价校验。

## 补充实验

<details>
<summary><b>输出形态与网格质量</b></summary>

100 对，第 1 次重复（输出确定性）。

| 算法 | 输出数 | 面数中位数 | n-gon 占比 | P95 紧致度 |
|------|--------|-----------|-----------|-----------|
| **re-EMBER** | 100 | 13714 | 6.4% | **72.7** |
| CGAL Nef | 100 | 8583 | 0% | 168.6 |
| Mesh Arrangement | 100 | 8648 | 0% | 116.9 |
| QuickCSG | 100 | 8648 | 0% | 196.7 |

re-EMBER 保留局部排布产生的多边形面片，不强制三角化。P95 紧致度（72.7，越小越不细长）远优于三角化输出（168.6–196.7）。
</details>

<details>
<summary><b>Tracy 性能剖析</b></summary>

10 对（4 小、3 中、3 大），Tracy + RelWithDebInfo。

| 层级 | 样本数 | solve_ms 中位 |
|------|--------|---------------|
| 小规模 | 4 | 38.0 |
| 中规模 | 3 | 42.4 |
| 大规模 | 3 | 164.2 |

self-time 主要热点：`WNV trace`、`LeafClassification::insetPointAttempt`、`BSPTree::addSegmentRecursive`、`Polygon256::rebuildAABBCache`。无 fallback 切分或桥接兜底——高层剪枝已有效运作，剩余开销在底层整数算术和临时对象管理。
</details>

<details>
<summary><b>并行缩放</b></summary>

15 对，1–20 线程。

| 线程数 | 平均求解时间 | 加速比 |
|--------|-------------|--------|
| 1 | 567 ms | 1.00× |
| 4 | — | 2.45× |
| 20 | 184 ms | 3.08× |

4 线程后收益明显放缓。i7-12700H 大小核架构（6P + 8E）与当前粗粒度 sibling 并行策略共同限制。Tracy 事件展开显示任务之间存在负载不均。
</details>

## 后续计划

### 自定义定长整数底层

当前底层为 `bitint`。乘法性能优秀，但**除法极其慢**——AABB 构造等大量几何操作依赖除法，构成显著瓶颈。需编写自定义定宽有符号整数。

| 方案 | 问题 |
|------|------|
| Boost `int256_t` | 乘法太慢 |
| `wideinteger` | 各操作全面慢 |
| `fp256` | 仅支持无符号数；有符号封装后性能显著下降 |
| `intx` | 仅支持无符号数；有符号封装后性能显著下降 |

`bitint` 仍是当前综合性能最好的底层，除法是唯一瓶颈。

### 尚未实现的论文优化

- **递归细分时不扫全部 polygon** — 论文提及但未给出具体策略，缺乏明确思路未实现
- **中间结果低位长运算** — 论文指出中间结果可安全用低位。简单尝试未成功，自定义整数底层可能提供更灵活的高低混合运算
- **内存重用** — 临时结构生命周期、缓存和分配模式未深入优化

### 并行扩展

20 线程约 3.08× 加速，4 线程后收益锐减。需细化任务粒度、改善负载均衡，并关注多线程下的内存分配竞争。

## 项目结构

```
src/
  application/     命令行入口
  io/              OBJ/STL 导入导出
  core/            BoolProblem、SubdivisionSolver
  algorithm/       BSP、叶片编排、WNV 追踪
  geometry/        AABB、裁剪、平面/多边形图元
  math/            256 位定宽整数算术
  tests/           单元测试
tests/
  paper_experiments/   Benchmark 样本（100 对）及清单
tools/
  profile-re-ember.ps1  计时与 Tracy 性能剖析脚本
docs/
  geometry-kernel-contract.md  几何 kernel 契约（详细）
  core-logic-flow.md           核心逻辑流程图
  paper-to-code-audit.md       论文到实现对照审计
assets/models/  测试模型
```

## 许可

核心库 (`reember_lib`) 与 CLI (`re-EMBER`) 采用 **MIT** 许可（详见 [LICENSE](LICENSE)）。

可选工具 `re-EMBER_verify` 链接 **CGAL**（GPL/LGPL/商业许可），位于 `verify/` 子目录并附带独立许可说明，默认不构建，非使用核心库的必要条件。

第三方依赖：tinyobjloader (MIT)、Boost (BSL-1.0)、TBB (Apache-2.0)、Tracy (BSD-3-Clause，vendored 于 `third_party/tracy/`)。可选：CGAL (GPL/LGPL)、Eigen (MPL-2.0)、libigl (MPL-2.0)。

100 对 benchmark 元数据使用 Thingi10K 模型 ID（CC BY 4.0）和 EMBER 论文 supplemental 变换矩阵。仓库不直接分发测试 OBJ 文件——使用 `tests/paper_experiments/generate_inputs.py` 生成。

## 参考文献

Trettner, Nehring-Wirxel, and Kobbelt. "EMBER: Exact Mesh Booleans via Efficient & Robust Local Arrangements." *ACM Trans. Graph.* (SIGGRAPH), 2022.

Nehring-Wirxel, Trettner, and Kobbelt. "Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs." *Computer-Aided Design*, 2021.
