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

## 备注

- `build\Debug\re-EMBER_tests.exe` 可以运行仓库测试。
- `--threads 1` 可强制串行，方便排查问题。
- `--timings-out <file>` 会输出单次运行的计时摘要。
