# re-EMBER

[English](#english) | [中文](#chinese)

<a id="english"></a>
## English

`re-EMBER` is a C++17 prototype for the EMBER boolean-mesh pipeline. It focuses on exact integer geometry, local arrangements, WNV/WNTV classification, and robust binary mesh booleans.

### Build

All build artifacts go under `build/`.

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 120
cmake --build build --config Debug --target re-EMBER
```

If `TBB` is missing, install it first:

```powershell
vcpkg install tbb:x64-windows
```

### Run

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25
```

`OBJ` output keeps n-gon faces by default. `STL` output is triangulated at the I/O boundary.

<a id="chinese"></a>
## 中文

`re-EMBER` 是一个围绕 EMBER 布尔网格流水线的 C++17 原型仓库，重点是精确整数几何、局部编排、WNV/WNTV 分类和稳健的二元网格布尔运算。

### 构建

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

### 运行

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25
```

默认 `OBJ` 输出保持 n 边面；`STL` 输出会在 I/O 边界三角化。
