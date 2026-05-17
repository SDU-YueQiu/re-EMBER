# 论文实验测试

当前仓库把论文 10% 实验中的 oracle-success pair 复制到 `tests/paper_experiments/`，并通过一个聚合 CTest 调用 `tools/profile-re-ember.ps1` 批量运行。它们不是独立的性能 benchmark，也不是已知失败的 expected-fail 测试；只要当前流水线不能成功完成，就应该让 CTest 失败。

## 默认样本和参数

默认纳入 `small_001` 到 `small_010` 共 10 个 small pair。CTest 会固定调用 `profile-re-ember.ps1 -InputRoot tests/paper_experiments/inputs/small -Op difference -NoTracy -Iterations 1`，并通过 `-ExecutablePath` 复用当前 CTest 构建树里的 CLI；普通手动 `-NoTracy` 运行在未显式传 `-Configuration` 时默认使用 `Release`。线程数默认取当前构建机的逻辑处理器数 `REEMBER_CTEST_THREADS`，并继续启用论文实验使用的四个输入假设。只有在专门排查串行分支时才应把 `REEMBER_CTEST_THREADS` 显式设为 `1`。

这些样本是当前代码的端到端回归集合，不在本文档固化某次历史通过率；修复算法时以最新 `ctest` 和对应 `build/performance/run_<timestamp>/timings.csv`、`summary.txt`、`report.md` 为准。多 workload 批量运行时，`summary.txt` 会额外给出 `overall_avg_*` 总平均时间，`report.md` 会给出 `Overall Average Timings` 表，方便先比较整批负载的平均耗时。其中 `end_to_end_ms` / `avg_end_to_end_ms` 表示 CLI 流水线从开始读输入到完成导出输出的墙钟时间；`process_elapsed_ms` / `avg_process_elapsed_ms` 表示整个 `re-EMBER.exe` 进程的墙钟时间，包含阶段外开销。`tests/paper_experiments/manifest.csv` 中的 `current_status` / `current_failure_category` 只记录本工作树上次刷新后的状态，不能替代重新运行 CTest。

默认 paper 聚合测试只证明 re-EMBER 能按当前指标和输入假设跑通，不等价于集合正确性证明。需要结果正确性校验时，在性能脚本上额外加 `-VerifyWithOracle`；脚本会在计时迭代结束后对每个 workload 调用一次 `re-EMBER_verify`，输出 `verification.csv`，并把 pass/fail、cache hit/miss 和 report 路径写进 `summary.txt` / `report.md`。这部分 oracle 基于 re-EMBER 量化后的 `Polygon256` 输入和 CGAL Nef 正则化集合运算，缓存默认位于 `build/oracle_cache/nef/`，校验耗时不计入 `timings.csv`。

## 运行方式

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 120
```

`re-EMBER_tests` 依赖 `re-EMBER`，因此上面的构建命令会同时生成 CLI。paper 聚合 test 会复用当前构建树里的 `re-EMBER.exe`，并把测试输出、逐 case metrics、OBJ 和汇总报告写入最新的 `build/performance/run_<timestamp>/`。当前默认 10 个 small pair 在本工作树中应全部通过；如果新增或修改算法导致聚合 test 失败，应先修复默认集合，再考虑继续从论文实验集追加更多 pair。

排查叶片分类性能时，优先查看 `timings.csv` 中逐 case 的 `leaf_classification_candidate_generated_count`、`leaf_classification_candidate_unique_count`、`leaf_classification_candidate_duplicate_skip_count`、`leaf_classification_candidate_repair_attempt_count` 和三个阶段的 `leaf_classification_*_input_invalid_count`。这些字段能区分候选枚举放大、重复路径、局部修复和真实 trace 失败。

如果要专门分析 `PATH_INVALID` 的几何来源，查看同批 `timing_*.metrics.txt` 里的 `trace_path_*` 计数即可；这些字段会区分起终点落边界、边重叠、边界命中以及原始边/`SubdivisionClip` 边等主要失败来源。统一放开 `SubdivisionClip` 边界横穿之后，重点看 `trace_path_boundary_hit_allowed_subdivision_clip_edge_count`、`trace_path_boundary_hit_rejected_regular_edge_count` 和 `leaf_classification_trace_attempt_count` 的联动变化。
