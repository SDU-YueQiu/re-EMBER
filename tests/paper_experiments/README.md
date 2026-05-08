# 论文实验回归测试

这里保存从论文实验工作区复制来的 oracle-success 输入，用作当前仓库默认 CTest 的端到端正确性测试。每个样本都已经在 CGAL Nef oracle 中完成 `lhs - rhs` 并输出非空结果；当前 `re-EMBER` 必须至少成功完成 CLI 流水线并写出结果 OBJ。

当前默认纳入两个小型样本：

- `small_001_1291011_minus_153368`：覆盖局部 BSP 共面重叠去重时严格内点构造失败。
- `small_026_996810_minus_1582439`：覆盖叶片 WNV 分类路径构造失败。

测试产物写入 `build/paper_experiment_tests/`，不写回本目录。`manifest.csv` 记录 oracle 结果面数和当前已知失败类别；面数只作为实验来源说明，不要求与当前 polygon-soup n-gon 输出一一相等。
