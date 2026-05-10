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

## CLI options

- `--lhs <file.obj|file.stl>` and `--rhs <file.obj|file.stl>` pick the left and right operands.
- `--op union|intersection|difference` selects the boolean operation.
- `--out <result.obj|result.stl>` sets the output file.
- `--scale <positive_integer>` overrides the shared quantization scale.
- `--leaf-threshold <positive_integer>` controls when subdivision stops at a leaf.
- `--threads <positive_integer>` sets the solver thread count; use `1` to force serial execution.
- `--timings-out <metrics.txt>` writes the timing and solve summary for a single run.
- `--assume-lhs-nsi`, `--assume-lhs-nnc`, `--assume-rhs-nsi`, and `--assume-rhs-nnc` declare input assumptions for faster runs. `NNC` requires `NSI` for the same side.

## Performance script

`tools/profile-re-ember.ps1` wraps timed runs, Tracy capture, and report generation. The most common parameters are:

- `-Lhs` / `-Rhs` and `-Op` run one explicit boolean workload.
- `-InputRoot` runs a batch of cases from a directory tree.
- `-Out` writes a single-workload result file.
- `-ExecutablePath` reuses an existing `re-EMBER.exe` instead of rebuilding.
- `-Configuration` chooses the profiling build type.
- `-Iterations`, `-TimeoutSeconds`, `-BuildTimeoutSeconds`, and `-ReportTimeoutSeconds` control runtime limits.
- `-LeafThreshold` and `-Threads` are passed through to the solver.
- `-NoTracy` skips Tracy capture and uses `build\profile_notracy\`.
- `-EnableMathTracy` also enables low-level `math256` Tracy zones and uses `build\profile_tracy_math\`.
- `-SkipBuild` reuses an already prepared profiling tree.
- `-UnwrapZoneFilter` exports per-event CSVs for selected hotspot zones.
- `-WorkloadPriority`, `-UsePCores`, and `-WorkloadAffinityMask` control workload scheduling.

The script writes `build\performance\run_<timestamp>\` with `summary.txt`, `timings.csv`, `manifest.json`, `profile.log`, `report.md`, `tracy_zones.csv`, `tracy_zones_self.csv`, and optional `tracy_unwrap\*.csv`.

## Notes

- `build\Debug\re-EMBER_tests.exe` runs the repository tests.
- `--threads 1` forces a serial run when you need to debug.
- `--timings-out <file>` writes the timing summary for a single run.
