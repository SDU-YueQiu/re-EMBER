# 论文实验回归测试

这里保存从论文 10% 实验工作区复制来的 oracle-success 输入，用作当前仓库默认 CTest 的端到端正确性测试。每个样本都已经在 CGAL Nef oracle 中完成 `lhs - rhs` 并输出非空结果；当前 `re-EMBER` 必须至少成功完成 CLI 流水线并写出结果 OBJ。

当前默认纳入 10 个 small 样本，并在 CTest 中启用论文实验使用的四个输入假设：

- `--assume-lhs-nsi`
- `--assume-lhs-nnc`
- `--assume-rhs-nsi`
- `--assume-rhs-nnc`

当前 10 个默认 small 样本都作为普通 CTest 运行，不设置 expected-fail；任一样本失败都表示当前流水线回归。测试产物写入 `build/paper_experiment_tests/`，不写回本目录。`manifest.csv` 记录 oracle 结果面数和本工作树上次刷新后的当前状态；面数只作为实验来源说明，不要求与当前 polygon-soup n-gon 输出一一相等。
