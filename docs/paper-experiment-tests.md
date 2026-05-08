# 论文实验测试

当前仓库把论文 10% 实验中的 oracle-success pair 复制到 `tests/paper_experiments/`，并通过 CTest 直接运行 `re-EMBER` CLI。它们不是性能 benchmark，也不是已知失败的 expected-fail 测试；只要当前流水线不能成功完成，就应该让 CTest 失败。

## 默认样本和参数

默认纳入 `small_001` 到 `small_010` 共 10 个 small pair。CTest 会固定使用 `difference`、`leaf-threshold=25`，线程数默认取当前构建机的逻辑处理器数 `REEMBER_CTEST_THREADS`，并启用论文实验使用的四个假设：`--assume-lhs-nsi --assume-lhs-nnc --assume-rhs-nsi --assume-rhs-nnc`。只有在专门排查串行分支时才应把 `REEMBER_CTEST_THREADS` 显式设为 `1`。

这些样本是当前代码的端到端回归集合，不在本文档固化某次历史通过率；修复算法时以最新 `ctest` 和对应 `build/paper_experiment_tests/*.metrics.txt` 为准。

## 运行方式

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
```

`re-EMBER_tests` 依赖 `re-EMBER`，因此上面的构建命令会同时生成 CLI。测试输出、metrics 和 OBJ 结果写入 `build/paper_experiment_tests/`。修复算法时应让这些测试从失败变为成功，再考虑继续从论文实验集追加更多 pair。

排查叶片分类性能时，优先查看 metrics 中的 `leaf_classification_candidate_generated_count`、`leaf_classification_candidate_unique_count`、`leaf_classification_candidate_duplicate_skip_count`、`leaf_classification_candidate_repair_attempt_count` 和三个阶段的 `leaf_classification_*_input_invalid_count`。这些字段能区分候选枚举放大、重复路径、局部修复和真实 trace 失败。
