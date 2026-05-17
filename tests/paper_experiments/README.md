# 论文实验回归测试

这里保存从论文实验工作区复制来的输入，用作当前仓库的端到端正确性和性能回归样本。每个样本都按 `lhs - rhs` 组织；当前 `re-EMBER` 必须至少成功完成 CLI 流水线并写出结果 OBJ。

当前默认 CTest 只纳入 10 个 small 样本，并在聚合测试中启用论文实验使用的四个输入假设：

- `--assume-lhs-nsi`
- `--assume-lhs-nnc`
- `--assume-rhs-nsi`
- `--assume-rhs-nnc`

当前 10 个默认 small 样本都会通过 `profile-re-ember.ps1 -InputRoot tests/paper_experiments/inputs/small` 作为一个聚合 CTest 批量运行，不设置 expected-fail；任一样本失败都表示当前流水线回归。medium / large 样本与 small 放在同一个 `inputs/` 根目录下，用于手动 verifier 或性能对比，不默认进入 120 秒 CTest。测试产物写入最新的 `build/performance/run_<timestamp>/`，不写回本目录。`manifest.csv` 记录 oracle 状态和本工作树上次刷新后的当前状态；面数只作为实验来源说明，不要求与当前 polygon-soup n-gon 输出一一相等。
