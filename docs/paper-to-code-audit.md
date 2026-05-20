# EMBER 论文机制对照审计

本文只记录当前 `refactor` 分支的工程事实和下一步优化靶点。旧 README
不作为架构事实来源；若本文与源码、测试或新 profile 冲突，以后者为准。

## 当前基线

- 最近可比性能基线：`d232e83 合并线段端点定向缓存构造`。
- 计时基线：`build/performance/run_20260521_054526/timings.csv`，Release
  NoTracy，论文实验集 `10 small / 10 medium / 2 large`，`--threads 20`。
- 最近 Tracy 归因：`build/performance/run_20260521_055010/`，单个 large
  workload，RelWithDebInfo，Tracy 和 math Tracy 开启。
- 当前流水线仍是 `OBJ/STL -> Polygon256 soup -> BoolProblem ->
  SubdivisionSolver -> resultFragments -> OBJ n-gon`。

阶段计时应优先比较 `solve_ms`。`read_ms`、`prepare_ms`、`export_ms`
只用于判断是否有 I/O 或导出噪声，不作为 solver 优化证据。

## 机制对照

| 论文机制 | 论文位置 | 当前实现 | 偏差 / 风险 | 下一步 |
| --- | --- | --- | --- | --- |
| 自适应递归 subdivision，子问题携带 polygon soup、AABB、local reference WNV | EMBER 4.1、4.2 | `SubdivisionSolver` 独占递归节点、AABB、参考点、叶片结果；`BoolProblem` 只做门面 | 基本对齐，但当前停止阈值仍是固定 polygon count，未直接以局部 BSP 代价或 trace 代价建模 | 用 `solveMetrics` 中的 `leaf_fragment_count`、`leaf_classification_trace_attempt_count`、`node_count` 反推更好的停止/切分信号 |
| 子参考点传播通过闭包安全路径追踪 WNV | EMBER 4.2.2、3.4 | `makeChildReference()` 使用 AABB 内候选和 `tracePathWNVAllowSubdivisionClipCrossingTrusted()` | 已有 fast candidate，但候选/trace 放大量仍需按 workload 量化；不能引入齐次点中点或点差 | 优先减少失败候选和重复 trace，而不是增加几何 fallback |
| 叶片内局部 BSP 解析面面交 | EMBER 4.3 | `buildLeafArrangement()` 对叶片 polygon soup 构造 face-local `BSPTree` | `BSPTree::addSegmentRecursive`、`appendSplitChildPolygons::clip` 仍是主要 self 热点；局部 BSP 产物会放大 `Polygon256` 缓存重建 | 先检查 BSP 插入是否重复插入等价 carrier，再看 clipping 是否可复用已分类顶点 |
| 叶片片段通过 segment tracing 得到 front/back WNV | EMBER 4.4 | `tracePathWNVToSurfacePointTrusted()` 对候选路径传播 WNV，并在目标支撑面两侧取 WNV；中间端点落面预检先用已有 AABB 缓存保守跳过 | 当前 `leaf_classification_trace_attempt_count` 仍高；候选生成、预筛和 trace 都在热点内 | 继续减少候选数量或无关多边形分类次数；禁止把无证明 fallback 变成默认路径 |
| 分类路径最多三段，由点的定义平面替换构造 | EMBER 3.2、4.4、Fig. 9 | `path_candidates.h` / `path_candidate_details.h` 提供 axis path、plane replacement、bridge rescue；centroid axis 目标会在四舍五入中心附近试少量整数轴探测线 | exhaustive plane replacement 是高 self time；邻近 centroid probe 已减少 inset/trace 放大量，但仍未解决所有 fallback | 下一步审计目标三平面排列和替换顺序是否存在语义重复，而不是只做局部分配优化 |
| operator indicator early-out | EMBER 4.5.2 | `constant_discard_count`、single-operand assumption、leaf BSP/classification reuse 已接入 | 早停仍依赖当前 reference WNV 和局部保守判定；错误早停会直接破坏结果 | 只在能证明 entire child indicator 常量时扩展；用 oracle 或 metrics 对照验证 |
| split strategy 减少热点工作量 | EMBER 4.5.3 | WNTV-aware split、center range split、midpoint fallback 已有 metrics | 当前 split 仍可能造成 polygon 放大，且未直接把 leaf trace/BSP 成本纳入策略 | 先用 profile 找“polygon 放大 -> leaf trace 放大”的 workload，再动 split 逻辑 |
| work-stealing parallel | EMBER 4.5.4、5.3 | 当前 child 子树级 oneTBB 任务，merge 固定 left -> right | 并行边界较粗；叶内 BSP 和分类串行。并行扩展前必须证明无共享状态和稳定聚合 | 不先动并行；先压低单任务工作量和共享状态复杂度 |
| 固定宽度齐次整数图元 | EMBER 3.2；BSP paper Table 1、4.1 | `PlanePoint3i` 已改为默认保留未约分三平面齐次交点；`classify_vertex` 暂用 512 位点积防止现有 `int256_t` 符号溢出 | 这是过渡层，不是最终 fixed 256 backend；导出阶段也因未约分点变重 | 下一步闭合平面系数预算或把 `classify_vertex` 接到自定义 fixed backend，并把 I/O canonical 化留在边界 |

## 当前 profile 结论

`run_20260521_030032` 的 single-large Tracy self hot zones 显示，下一阶段
最值得处理的不是 I/O 或导出，而是 solver 内部工作量：

- `Polygon256::rebuildVertexAndAABBCaches` / `rebuildVertexCache` 很高，但
  已测试过“trusted clipping 立即预计算顶点+AABB”，NoTracy 变慢；说明不能简单把
  懒缓存改成 eager 缓存。
- `enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints`
  和 `enumerateLeafClassificationAxisPathCandidatesFromPoints` 仍然重，优先检查候选
  枚举是否有语义重复或过度 fallback。
- `BSPTree::addSegmentRecursive` 与 `appendSplitChildPolygons::clip` 是局部 BSP
  和递归 clipping 的真实热点；适合做 carrier 去重、顶点分类复用或局部结构收缩。
- `tracePathWNVToSurfacePointImpl::classifyEndPoint` 已通过最后一段 AABB 相关性
  剪枝减少进入次数，但 `tracePathWNVToSurfacePointImpl::polygon` 和
  `intersectSupportPlane` 仍有明显成本。
- 当前二元 WNV/WNTV 已改为内联两个分量，避免 polygon tag、reference WNV、
  classified fragment 和 trace 累加的常见堆分配；这只收缩数据表示，不改变
  WNV 传播数学。

## 下一阶段优先队列

1. 审计 `enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints`
   的目标三平面排列和替换顺序：只删除可证明等价的构造，不降低闭包安全性。
2. 审计局部 BSP carrier 流：确认同一 leaf/base polygon 是否重复插入等价 split
   segment，以及是否能在不增加状态膨胀的情况下提前合并。
3. 审计 split strategy 的实际放大链：用 `node_count`、`leaf_fragment_count`、
   `leaf_classification_trace_attempt_count` 找造成 trace 放大的 split 模式。
4. 收敛当前 `DotInteger=int512_t` 过渡层：先量化哪些平面来源超过论文 256 位预算，
   再把 `classify_vertex` / 4D dot 和 `intersect_3_planes` 接到 fixed 256 窄接口。

## 已完成的接口收缩审计

- `clipPolygonToAABB()` 和内部 `clipPolygonToHalfSpace()` 在当前源码与测试中只有
  定义、没有调用；已从 `polygon_ops.h` 删除，并把真实裁剪依赖改为调用点显式包含
  `geometry/clipping.h`。
- `path_candidate_details.h` 中的 `std::vector<int>` / `std::vector<SplitAxis3i>`
  顺序包装重载没有生产调用；已删除，内部路径构造统一使用 `std::array + count`
  表达最多三步的论文路径。
- exhaustive plane replacement 的目标三平面排列和替换顺序当前不能直接合并：
  目标平面排列决定“哪张目标平面替换哪个定义槽位”，替换顺序又会改变中间点和
  AABB 内可达性；现有回归测试已经覆盖一个替换顺序失败、另一个顺序成功的场景。
  后续若要继续删候选，需要先建立“同一端点序列或同一 trace 签名”的等价判据，
  不能只按最终 target 点去重。
- centroid axis 第一阶段目标点本身就是由两张整数坐标平面和当前支撑平面构造的
  axis-probe 点；已把构造时的 `AxisProbeTarget` 传给候选枚举，避免后续再次识别
  三平面结构和重复构造目标交点。`run_20260521_030519` 对同一 10 small / 10 medium /
  2 large workload 做了 `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，结构计数
  与 `run_20260521_025314` 一致，聚合 `solve_ms` 从 2454.058ms 降到 2438.915ms。
- `clipLeafGeometryByPlaneTrusted()` 只在调用点没有现成顶点侧分类时自行计算侧别；
  小多边形侧别改用栈上缓冲，内部裁剪实现改成 `int* + count`，已有
  `SplitPolygonRoute::vertexSides` 调用仍沿用原公开面。`run_20260521_031822` 对同一
  10 small / 10 medium / 2 large workload 做了 `-NoTracy -VerifyWithOracle`，22 个 verifier
  全部通过且 oracle cache hit，结构计数与 `run_20260521_030519` 一致，聚合 `solve_ms`
  从 2438.915ms 降到 2423.462ms。
- `WNV` 从 `std::vector<int>` 别名收缩为二元内联小向量，`Polygon256::WNTV` 同步使用
  该类型；`WNV{...}`、`assign()`、`size()`、下标访问和相等比较保持兼容，超过两个
  分量时仍懒分配动态存储。`run_20260521_060849` 对同一 10 small / 10 medium /
  2 large workload 做了 `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过且
  oracle cache hit，聚合 `solve_ms` 从 `run_20260521_054526` 的 1990.55ms 降到
  1950.69ms，`end_to_end_ms` 从 5397.31ms 降到 5336.47ms。

## 已测但不保留的局部实验

- 叶片分类候选路径 view：结构和 trace 计数不变，但 NoTracy solve 变慢。
- `buildLeafArrangement()` 结果 vector 预留容量：结构计数不变，solve 信号为负。
- `buildLeafArrangement()` 删除 `polygonCount < 8` 小叶片旧路径，并改用统一
  pair-relation adjacency：结构计数不变，但 NoTracy solve 信号为负；进一步改成
  per-base 分桶、去掉 sort/offset 后仍为负。当前小叶片 `insertTrusted()` 路径虽然
  代码重复，但在现有 workload 上有性能意义，后续不应只为了统一控制流删除它。
- 中间端点预筛缓存：结构计数不变，但缓存维护成本超过收益。
- trusted clipped polygon eager 顶点/AABB 缓存：结构计数不变，solve 信号为负。
- BSP 插入端点齐次交点沿递归缓存：结构计数不变，22 个 verifier 全部通过，
  但 `run_20260521_032931` 相比 `run_20260521_031822` 的聚合 `solve_ms`
  从 2423.462ms 退化到 2455.315ms；额外状态传递和端点对象拷贝成本超过收益。
- centroid axis 的 local reference 整数起点按叶片缓存：结构计数不变，22 个
  verifier 全部通过，但 `run_20260521_033829` 相比 `run_20260521_031822`
  的聚合 `solve_ms` 从 2423.462ms 退化到 2431.565ms；减少重复解析不足以抵消
  context 状态和额外重载分派成本。
- `Polygon256::aabb()` 改成 AABB-only 重建、不顺带填充顶点缓存：结构计数不变，
  22 个 verifier 全部通过，但 `run_20260521_034830` 相比 `run_20260521_031822`
  的聚合 `solve_ms` 从 2423.462ms 退化到 2440.191ms；后续顶点访问的重复构造
  抵消了 AABB-only 的分配节省。
- `axisMinimum()` / `axisMaximum()` 改为返回 AABB 字段引用、减少 `Integer` 边界拷贝：
  结构计数不变，22 个 verifier 全部通过，但 `run_20260521_035707` 相比
  `run_20260521_031822` 的聚合 `solve_ms` 从 2423.462ms 退化到 2429.444ms；
  该热循环中的边界值拷贝不是当前主导成本。
- `addScaledWNTV()` 改为通过 `WNV::data()` 指针循环，22 个 verifier 全部通过，但
  `run_20260521_061452` 相比 `run_20260521_060849` 的聚合 `solve_ms` 从 1950.69ms
  退化到 1966.73ms；保留下标访问版本。

这些结论只用于避免近期重复试错；若 workload、算法边界或 profile 证据变化，可以重新评估。
