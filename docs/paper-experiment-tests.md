# 论文实验测试

当前仓库把论文实验中的 oracle-success pair 复制到 `tests/paper_experiments/`，并通过 CTest 直接运行 `re-EMBER` CLI。它们不是性能 benchmark，也不是已知失败的 expected-fail 测试；只要当前流水线不能成功完成，就应该让 CTest 失败。

## 默认样本

- `small_001_1291011_minus_153368`：CGAL Nef oracle 成功，当前实现会触发 `BSPTree failed to find a strict interior point while disabling overlap leaves.`，对应局部 BSP 共面重叠叶片去重的严格内点构造缺口。
- `small_026_996810_minus_1582439`：CGAL Nef oracle 成功，当前实现会触发 `BoolProblem failed to classify leaf fragment ...`，对应叶片分类路径候选无法覆盖真实输入局部构型。

## 运行方式

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
```

`re-EMBER_tests` 依赖 `re-EMBER`，因此上面的构建命令会同时生成 CLI。测试输出、metrics 和 OBJ 结果写入 `build/paper_experiment_tests/`。修复算法时应让这些测试从失败变为成功，再考虑继续从论文实验集追加更多 pair。
