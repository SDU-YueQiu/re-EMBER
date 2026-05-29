# 论文实验回归测试

这里保存从 EMBER 论文 benchmark 列表中分层抽取的 100 组输入的元数据，用作当前仓库的端到端正确性和性能回归样本。

## 数据来源与许可

- **模型 ID 与变换矩阵**：来自 EMBER 论文 (Trettner et al. 2022) 公开的 supplemental benchmark case 列表。变换矩阵版权属 EMBER 论文作者。
- **原始 3D 模型**：来自 [Thingi10K](https://huggingface.co/datasets/Thingi10K/Thingi10K) 数据集（CC BY 4.0）。各模型有其独立 license 字段，请查阅 Thingi10K 元数据。**仓库不直接包含派生 OBJ 文件。**

## 文件说明

| 文件 | 说明 |
|------|------|
| `benchmark-cases.json` | 100 对的 Thingi10K ID + 变换矩阵（EMB ER paper supplemental 来源） |
| `manifest.csv` | 100 对的统计元数据（pair_id、stratum、面数等） |
| `generate_inputs.py` | 从 Thingi10K 下载 raw STL、应用变换、生成 OBJ |
| `build_case_list.py` | 从论文实验 CSV 重建 `benchmark-cases.json` |

## 生成测试输入

```powershell
pip install huggingface_hub numpy

# 按需下载
python generate_inputs.py --small
python generate_inputs.py --medium
python generate_inputs.py --large

# 全部
python generate_inputs.py --all
```

脚本会将 OBJ 写入 `inputs/<stratum>/<pair_id>/lhs.obj` 和 `rhs.obj`。

## 运行回归测试

```powershell
# 单层
build\default\re-EMBER.exe --lhs tests\paper_experiments\inputs\small\small_001_...\lhs.obj --rhs ... --op difference ...

# 批量（完整 corpus: 23 small + 43 medium + 34 large）
tools\profile-re-ember.ps1 -UsePaperExperimentSet -PaperSmallCount 23 -PaperMediumCount 43 -PaperLargeCount 34

# Oracle 验证
build\verify\re-EMBER_verify.exe --batch-manifest tests\paper_experiments\manifest.csv --batch-out-dir results/
```

## CTest 状态

论文 small 样本的 CTest 条目已暂时注释（CMakeLists.txt），等 OBJ 输入生成脚本就绪后恢复。当前仅 `re-EMBER_tests` 单元测试保留在 CTest 中。
