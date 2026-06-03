# re-EMBER

[English](README.md) | [中文](README.zh-CN.md)

re-EMBER (redev-open-EMBER) is an **independent, community-driven open-source** (MIT) C++17 implementation of the EMBER paper (Trettner et al., SIGGRAPH 2022). It is **not** the commercially-licensed binary release from the original paper authors. EMBER is an **exact** mesh boolean algorithm using fixed-width integer geometry and local arrangements. It handles union, intersection, and difference on triangle soups, producing watertight polygon output. Across all benchmarks, its end-to-end wall-clock time averages **~2× QuickCSG** — while providing exact results that QuickCSG does not guarantee. Performance remains **several times slower** than the authors' original implementation.

## Installation

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

### Manual Build

```powershell
# Prerequisites
vcpkg install tinyobjloader tbb boost-multiprecision
scoop install llvm

cmake --preset default && cmake --build --preset default-app
cmake --install build/default --prefix <install-path>
```

Then in your CMakeLists.txt:

```cmake
find_package(re-EMBER REQUIRED)
target_link_libraries(my_app PRIVATE reember::lib)
```

## Quick Start

```powershell
# Build & run
cmake --preset default && cmake --build --preset default-app

# Run
build\default\re-EMBER.exe --lhs A.obj --rhs B.obj --op difference --out result.obj
```

See [Build](#build) for dependency installation and alternative presets (tests, verifier, visual viewer).

## Performance

Comparisons against CGAL Nef (exact baseline), Mesh Arrangement / libigl (exact), and QuickCSG / mesh_cs (fast, non-exact). Hardware: Intel i7-12700H, 64 GB DDR5, Windows 11.

### General Benchmark — 100 Pairs

100 model pairs (23 small / 43 medium / 34 large), stratified by face count. Difference operation, leaf threshold = 25.

| Algorithm | Exact | Median | Geom Mean | Max | Peak Mem |
|-----------|-------|--------|-----------|-----|----------|
| **re-EMBER** | yes | **102 ms** | **113 ms** | 668 ms | 118 MiB |
| QuickCSG | no | 53 ms | 63 ms | 380 ms | 16 MiB |
| Mesh Arrangement | yes | 801 ms | 830 ms | 4392 ms | 52 MiB |
| CGAL Nef | yes | 1358 ms | 1603 ms | 33454 ms | 151 MiB |

All 100 pairs completed. Zero failures.

### High-Face-Count Workpiece + Cylinder Tool — 25 Pairs

25 Thingi10K models (100k–260k faces) as workpieces, each cut by a 96-triangle cylinder.

| Algorithm | Exact | Median | Geom Mean | Max | Peak Mem |
|-----------|-------|--------|-----------|-----|----------|
| **re-EMBER** | yes | **661 ms** | **703 ms** | 1097 ms | 756 MiB |
| QuickCSG | no | 381 ms | 404 ms | 708 ms | 81 MiB |
| Mesh Arrangement | yes | 2376 ms | 2439 ms | 4715 ms | 280 MiB |
| CGAL Nef | yes | 16937 ms | 17971 ms | 33238 ms | 1427 MiB |

All 25/25 successes. re-EMBER is 3.9% of CGAL Nef, 27.8% of Mesh Arrangement.

## Correctness

### Accuracy — Oracle Verifier

Output validated against a CGAL Nef oracle constructed on the same quantized input. 100 benchmark pairs, `re-EMBER_verify`.

| Completed | Passed | Failed |
|-----------|--------|--------|
| 82 | **68** | **0** |

14 timed out during Nef comparison (CGAL Nef overhead on large models), 18 untested large-scale pairs. Among all completed verifications, zero correctness failures.

### Robustness — Defective Inputs

10 synthetic model pairs with self-intersections, shared faces/edges/vertices, nested/duplicate shells, thin shells, and near-coplanar cuts. Input-assumption flags **off**.

**10/10 successes.** Median 20 ms, max 26 ms.

## Features

- **Exact arithmetic** — 256-bit fixed-width integer predicates; no floating-point robustness issues
- **Polygon-preserving output** — n-gon output (60% triangles, 30% quads, 6% n-gons); no forced triangulation
- **Watertight results** — optional conforming T-junction repair pass
- **Parallel** — sibling-parallel subdivision via TBB; configurable thread count
- **OBJ + STL I/O** — n-gon `.obj` and triangulated `.stl` import/export
- **Input assumptions** — optionally declare NSI/NNC properties for speed

## Build

All presets use Ninja + clang-cl. Artifacts land in `build/<preset>/`.

| Preset | Target | Extra Deps |
|--------|--------|------------|
| `default` | `re-EMBER.exe` CLI | — |
| `tests` | `re-EMBER_tests.exe` + CTest | — |
| `verify` | `re-EMBER_verify.exe` (CGAL Nef oracle) | `cgal eigen3` |
| `visual-test` | `visual-test.exe` (libigl viewer) | `cgal eigen3 libigl` |

```powershell
# Core dependencies
vcpkg install tinyobjloader tbb boost-multiprecision
scoop install llvm

# Default
cmake --preset default && cmake --build --preset default-app

# Tests
cmake --preset tests && cmake --build --preset tests && ctest --preset default

# Verifier (needs CGAL)
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

- `.obj` output preserves n-gons; `.stl` triangulates at the boundary
- `--leaf-threshold` controls subdivision depth (default 50, lower = deeper tree)
- `--output-topology conforming` adds exact T-junction repair (slow; for inspection)
- `--threads 1` forces serial execution throughout

### Oracle Verifier

```powershell
# Single pair
build\verify\re-EMBER_verify.exe --lhs A.obj --rhs B.obj --op difference

# Batch
build\verify\re-EMBER_verify.exe --batch-manifest manifest.csv --batch-out-dir results/
```

The verifier compares re-EMBER result fragments against a CGAL Nef reference built from the same quantized `Polygon256` input.

## Additional Experiments

<details>
<summary><b>Output Morphology &amp; Mesh Quality</b></summary>

100 pairs, 1st repeat (deterministic output).

| Algorithm | Outputs | Median Faces | n-gon Ratio | P95 Compactness |
|-----------|---------|-------------|-------------|-----------------|
| **re-EMBER** | 100 | 13714 | 6.4% | **72.7** |
| CGAL Nef | 100 | 8583 | 0% | 168.6 |
| Mesh Arrangement | 100 | 8648 | 0% | 116.9 |
| QuickCSG | 100 | 8648 | 0% | 196.7 |

re-EMBER preserves polygon faces from local arrangements rather than force-triangulating. This results in dramatically better compactness (P95 = 72.7, vs 168.6–196.7 for triangulated output). Lower is less elongated.
</details>

<details>
<summary><b>Tracy Profiling Hotspots</b></summary>

10 pairs (4 small, 3 medium, 3 large), Tracy + RelWithDebInfo.

| Stratum | Samples | solve_ms Median |
|---------|---------|-----------------|
| Small | 4 | 38.0 |
| Medium | 3 | 42.4 |
| Large | 3 | 164.2 |

Top self-time consumers: `WNV trace`, `LeafClassification::insetPointAttempt`, `BSPTree::addSegmentRecursive`, `Polygon256::rebuildAABBCache`. Zero fallback splits/bridge rescues — high-level pruning is effective; remaining cost is in low-level integer arithmetic and temporary object management.
</details>

<details>
<summary><b>Parallel Scaling</b></summary>

15 pairs, 1–20 threads.

| Threads | Avg solve_ms | Speedup |
|---------|-------------|---------|
| 1 | 567 | 1.00× |
| 4 | — | 2.45× |
| 20 | 184 | 3.08× |

Plateaus after 4 threads. The i7-12700H hybrid architecture (6P + 8E cores) and coarse-grained sibling-parallel strategy both contribute. Tracy event traces show load imbalance once enough sibling tasks exist.
</details>

## Roadmap

### Custom fixed-length integer backend

The current backend is `bitint`. Multiplication is fast, but **division is extremely slow** — and AABB construction requires many divisions. A custom fixed-width signed integer is needed.

| Library | Issue |
|---------|-------|
| Boost `int256_t` | Multiplication too slow |
| `wideinteger` | Overall slow across all operations |
| `fp256` | Unsigned only; signed wrapper introduces significant overhead |
| `intx` | Unsigned only; signed wrapper introduces significant overhead |

`bitint` remains the best-performing backend; division is the sole bottleneck.

### Missing optimizations from the paper

- **Recursive subdivision polygon filtering** — the paper mentions not scanning all polygons during recursive subdivision, but provides no concrete strategy.
- **Lower-bit intermediate results** — the paper notes intermediates can safely use fewer bits. Attempts to exploit this were unsuccessful. A custom integer backend may enable mixed-precision operations.
- **Memory reuse** — temporary structure lifetimes, caching, and allocation patterns are adequate but not deeply optimized.

### Parallel scaling

~3.08× at 20 threads with diminishing returns after 4. Needs finer task granularity, better load balancing, and memory-allocation awareness under contention.

## Project Layout

```
src/
  application/     CLI entry point
  io/              OBJ/STL I/O
  core/            BoolProblem, SubdivisionSolver
  algorithm/       BSP, leaf arrangement, WNV tracing
  geometry/        AABB, clipping, plane/polygon primitives
  math/            256-bit integer arithmetic
  tests/           Unit tests
tests/
  paper_experiments/   Benchmark corpus (100 pairs) + manifests
tools/
  profile-re-ember.ps1  Timing + Tracy profiling harness
docs/
  geometry-kernel-contract.md  Kernel contract (detailed)
  core-logic-flow.md           Core logic flowcharts
  paper-to-code-audit.md       Paper-to-implementation audit
assets/models/  Test models
```

## License

The core library (`reember_lib`) and CLI (`re-EMBER`) are licensed under **MIT** (see [LICENSE](LICENSE)).

The optional `re-EMBER_verify` tool links against **CGAL** (GPL/LGPL/commercial) and lives in `verify/` with its own license notice. It is disabled by default and is not required to use the core boolean library.

Third-party dependencies: tinyobjloader (MIT), Boost (BSL-1.0), TBB (Apache-2.0), Tracy (BSD-3-Clause, vendored in `third_party/tracy/`). Optional: CGAL (GPL/LGPL), Eigen (MPL-2.0), libigl (MPL-2.0).

The 100-pair benchmark metadata uses Thingi10K model IDs (CC BY 4.0) and EMBER paper supplemental transforms. Test OBJ files are not distributed — generate them with `tests/paper_experiments/generate_inputs.py`.

## References

Trettner, Nehring-Wirxel, and Kobbelt. "EMBER: Exact Mesh Booleans via Efficient & Robust Local Arrangements." *ACM Trans. Graph.* (SIGGRAPH), 2022.

Nehring-Wirxel, Trettner, and Kobbelt. "Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs." *Computer-Aided Design*, 2021.
