# re-EMBER

[English](README.md) | [中文](README.zh-CN.md)

`re-EMBER` is a C++17 prototype for the EMBER boolean-mesh pipeline. The repository focuses on exact integer geometry, local arrangements, WNV/WNTV classification, and robust binary mesh booleans.

## What is in this repo

- `src/application/main.cpp` provides the command-line entry point.
- `src/io/` handles OBJ/STL import and export.
- `src/core/` contains the public `BoolProblem` facade and the internal `SubdivisionSolver`.
- `src/algorithm/`, `src/geometry/`, and `src/math/` hold the boolean pipeline, geometry primitives, and fixed-width integer math.
- `src/tests/` and `tests/paper_experiments/` provide unit tests and paper-style regression inputs.
- `tools/profile-re-ember.ps1` runs timed workloads and optional Tracy captures.

## Build

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

## Run

Minimal boolean smoke test:

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25
```

`.obj` output keeps n-gon faces by default. `.stl` output is triangulated at the I/O boundary.

## Notes

- `build\Debug\re-EMBER_tests.exe` runs the repository tests.
- `--threads 1` forces a serial run when you need to debug.
- `--timings-out <file>` writes the timing summary for a single run.
