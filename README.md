# re-EMBER

[English](README.md) | [中文](README.zh-CN.md)

`re-EMBER` is a C++17 prototype for the EMBER boolean-mesh pipeline. The repository focuses on exact integer geometry, local arrangements, WNV/WNTV classification, and robust binary mesh booleans.

## Geometry kernel contract

The core boolean path is being constrained to the fixed-width homogeneous integer model used by EMBER and by Nehring-Wirxel et al. 2021. Core geometry code may construct and classify only through the paper primitive set: integer planes, axis-aligned AABB planes, three-plane homogeneous intersections, vertex-vs-plane classification, integer vertex signed-distance tests, and clipping against already valid planes.

Input OBJ/STL coordinates are quantized to signed 26-bit integer coordinates unless `--scale` is explicitly supplied. Within the core solver, `int512_t` and `cpp_int` are not allowed to decide algorithmic branches; they are reserved for tests, debug oracles, diagnostics, and I/O post-processing. Operations that are not closed under the 256-bit budget, such as arbitrary homogeneous-point averaging, midpoint/difference construction, coordinate-plane reconstruction from fractional homogeneous points, and new planes derived from output homogeneous vertices, must fail explicitly instead of silently acting as fallback geometry.

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
$clang = "$env:USERPROFILE\scoop\apps\llvm\current\bin\clang-cl.exe"
$rc = "$env:USERPROFILE\scoop\apps\llvm\current\bin\llvm-rc.exe"
$ninja = "D:\Program Files\VisualStudio\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
cmake -S . -B build -G Ninja `
  -DCMAKE_MAKE_PROGRAM="$ninja" `
  -DCMAKE_CXX_COMPILER="$clang" `
  -DCMAKE_RC_COMPILER="$rc" `
  -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target re-EMBER_tests
ctest --test-dir build --output-on-failure --timeout 120
cmake --build build --target re-EMBER
```

The default supported local configuration is clang-cl plus Boost.Multiprecision. The optional CGAL oracle verifier is controlled by `REEMBER_BUILD_VERIFY` and is enabled in the normal local build. If `TBB`, LLVM, or CGAL is missing, install them first:

```powershell
vcpkg install tbb:x64-windows cgal:x64-windows
scoop install llvm
```

## Run

Minimal boolean smoke test:

```powershell
build\Debug\re-EMBER.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --out build\boolean_smoke.obj --leaf-threshold 25
```

`.obj` output keeps n-gon faces. `.stl` output is triangulated at the I/O boundary. `--output-topology conforming` enables the exact T-junction repair pass before export. This mode is intended for debugging and MeshLab inspection, not performance measurement; it can be much slower than raw output. Coplanar merging and Nef regularized output remain disabled in the application CLI.

## Oracle verifier

`re-EMBER_verify` checks a `BoolProblem::resultFragments()` candidate against a cached CGAL Nef oracle:

```powershell
cmake --build build --target re-EMBER_verify
build\Debug\re-EMBER_verify.exe --lhs assets\models\workpiece_block.obj --rhs assets\models\tool_box.obj --op difference --leaf-threshold 25 --oracle-cache-dir build\oracle_cache\nef
```

The oracle is exact over the quantized `Polygon256` input used by re-EMBER. It does not claim to validate the original floating OBJ/STL CAD intent before import and quantization. Oracle Nef files are cached under `build\oracle_cache\nef\` by default; pass `--refresh-oracle` to rebuild a cached entry. `--candidate-mode fragments-nef|export-conforming|export-nef` selects whether the candidate is compared from result fragments, from the conforming export topology, or from the Nef export topology path; this does not change the oracle cache key. The default `fragments-nef` candidate reuses the exact conforming mesh recovery before constructing CGAL Nef, avoiding the older quadratic Nef-side T-junction refinement. Before falling back to CGAL Nef overlay, the verifier first compares simple candidate/oracle Nef surfaces as exact vertices and face cycles; `surface_compare_used=1` in the report means this exact surface check proved equality without running the final overlay. Pass `--disable-surface-compare` to force the older overlay-only comparison.

For CGAL Nef failures, `--diagnose-nef` prints exact mesh topology statistics before and after Nef construction. Pair it with `--nef-compare-op skip` to avoid the final CGAL overlay, or with `equal` / `candidate-minus-oracle` / `oracle-minus-candidate` / `xor` to isolate which Nef comparison operation stalls or fails. This diagnostic path can be slow and is intended for correctness investigation, not performance timing.

## CLI options

- `--lhs <file.obj|file.stl>` and `--rhs <file.obj|file.stl>` pick the left and right operands.
- `--op union|intersection|difference` selects the boolean operation.
- `--out <result.obj|result.stl>` sets the output file.
- `--scale <positive_integer>` overrides the shared quantization scale.
- `--leaf-threshold <positive_integer>` controls when subdivision stops at a leaf.
- `--threads <positive_integer>` sets the application-layer task arena size and solver thread count; use `1` to force serial execution.
- `--output-topology raw|conforming` chooses application export topology. `raw` writes `resultFragments()` directly; `conforming` globally inserts existing vertices that lie on other face edges to remove T-junctions. `conforming` is exact but slow and should be used for debugging/inspection rather than performance runs. Coplanar merging and Nef output are disabled.
- `--timings-out <metrics.txt>` writes the timing and solve summary for a single run.
- `--assume-lhs-nsi`, `--assume-lhs-nnc`, `--assume-rhs-nsi`, and `--assume-rhs-nnc` declare input assumptions for faster runs. `NNC` requires `NSI` for the same side.

Application-layer parallelism uses the same `--threads` limit for coarse left/right operand work and fine-grained static partitions over vertex AABB building, vertex quantization, face-to-polygon construction, and export fragment recovery.

## Performance script

`tools/profile-re-ember.ps1` wraps timed runs, Tracy capture, and report generation. The most common parameters are:

- `-Lhs` / `-Rhs` and `-Op` run one explicit boolean workload.
- `-InputRoot` runs a batch of cases from a directory tree.
- `-Out` writes a single-workload result file.
- `-ExecutablePath` reuses an existing `re-EMBER.exe` instead of rebuilding.
- `-Configuration` chooses the profiling build type. Timing-only `-NoTracy` runs default to `Release`; Tracy runs default to `RelWithDebInfo`.
- `-Iterations`, `-TimeoutSeconds`, `-BuildTimeoutSeconds`, and `-ReportTimeoutSeconds` control runtime limits.
- `-LeafThreshold` is passed through to the solver; `-Threads` sets the application-layer task arena size and solver thread count.
- `-NoTracy` skips Tracy capture and uses `build\profile_clang_notracy\`.
- `-EnableMathTracy` also enables low-level `math256` Tracy zones and uses `build\profile_clang_tracy_math\`.
- `-SkipBuild` reuses an already prepared profiling tree.
- `-UnwrapZoneFilter` exports per-event CSVs for selected hotspot zones.
- `-VerifyWithOracle` runs `re-EMBER_verify` once per workload after timed iterations and writes `verification.csv`; verifier time is not included in `timings.csv`.
- `-WorkloadPriority`, `-UsePCores`, and `-WorkloadAffinityMask` control workload scheduling.

The script writes `build\performance\run_<timestamp>\` with `summary.txt`, `timings.csv`, `manifest.json`, `profile.log`, `report.md`, `tracy_zones.csv`, `tracy_zones_self.csv`, optional `verification.csv`, and optional `tracy_unwrap\*.csv`.

## Notes

- `build\Debug\re-EMBER_tests.exe` runs the repository tests.
- `build\Debug\visual-test.exe` exposes the same Ember output topology controls in the interactive panel, including `nef`.
- `--threads 1` forces a serial run across application-layer preparation and solving when you need to debug.
- `--timings-out <file>` writes the timing summary for a single run.
