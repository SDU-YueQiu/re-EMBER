# 布尔 benchmark 模型建议

这页只记录当前这个工作区里更适合拿来做布尔和 visual-test 压测的模型档位，不把旧盒子 workload 当成主要基准。

## 当前推荐

| 档位 | 模型 | 面数 | 当前路径 | 说明 |
| --- | --- | ---: | --- | --- |
| 几千面 | `classic_spot` | 5,856 | `assets\models\classic_spot.obj` | 当前仓库内最稳定的经典模型，尺度接近原点，适合先验证 visual-test / smoke。 |
| 几万面 | Stanford Bunny | 69,451 | `build\benchmark_models\obj\stanford_bunny_69451.obj` | 已从 Stanford 官方 `bun_zipper.ply` 转成 OBJ，适合做中档 boolean 回归。 |
| 几十万面 | Stanford Armadillo | 345,944 | `build\benchmark_models\obj\stanford_armadillo_345944.obj` | 已从 Stanford 官方 `Armadillo.ply` 转成 OBJ，适合做大规模单线程性能压测。 |

## 当前不优先

- `assets\models\classic_cow.obj`
  当前观察里更容易出现破片，不适合先拿来判断 visual-test 交互或 profiling 工作流本身是否正常。
- `assets\models\classic_fandisk.obj`
  源坐标整体偏离原点很大；虽然新的 visual-test / profiling 默认位姿已经改成自动对齐中心，但它依然不适合当“第一批”回归模型。

## 来源

- Stanford Bunny 官方包：`https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz`
- Stanford Armadillo 官方包：`https://graphics.stanford.edu/pub/3Dscanrep/armadillo/Armadillo.ply.gz`

## 当前工作区生成结果

本次整理已经把两个 Stanford 模型下载并转成 OBJ，输出在：

- `build\benchmark_models\obj\stanford_bunny_69451.obj`
- `build\benchmark_models\obj\stanford_armadillo_345944.obj`

如果后续继续换 visual-test 的大模型，优先替换 `assets\visual_test\workpiece_block.obj` 和 `assets\visual_test\tool_box.obj`；新的 UI 和 `tools\profile-re-ember.ps1` 都会先按 AABB 自动居中，再把 `dx/dy/dz` 当作相对偏移处理。
