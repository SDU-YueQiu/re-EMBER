# 论文实验测试

当前仓库把论文 10% 实验中的 oracle-success pair 复制到 `tests/paper_experiments/`，并通过 CTest 直接运行 `re-EMBER` CLI。它们不是性能 benchmark，也不是已知失败的 expected-fail 测试；只要当前流水线不能成功完成，就应该让 CTest 失败。

## 默认样本和参数

默认纳入 `small_001` 到 `small_010` 共 10 个 small pair。CTest 会固定使用 `difference`、`leaf-threshold=25`、`threads=1`，并启用论文实验使用的四个假设：`--assume-lhs-nsi --assume-lhs-nnc --assume-rhs-nsi --assume-rhs-nnc`。

当前 10% 实验结果是 `re-EMBER` 30 次运行中 3 次成功，成功率 10%。按唯一 pair 看，`small_003_702407_minus_81309` 成功，其余 9 个样本都会触发 `BSPTree failed to find a strict interior point while disabling overlap leaves.`，对应局部 BSP 共面重叠叶片去重的严格内点构造缺口。CGAL Nef、libigl/CGAL 和 QuickCSG 在同一 10 个 pair 上均为 30/30 成功；Cork 当前未配置。

## 运行方式

```powershell
cmake -S . -B build
cmake --build build --config Debug --target re-EMBER_tests
ctest --test-dir build -C Debug --output-on-failure --timeout 60
```

`re-EMBER_tests` 依赖 `re-EMBER`，因此上面的构建命令会同时生成 CLI。测试输出、metrics 和 OBJ 结果写入 `build/paper_experiment_tests/`。修复算法时应让这些测试从失败变为成功，再考虑继续从论文实验集追加更多 pair。
