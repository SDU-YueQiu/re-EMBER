# EMBER 论文机制对照审计

本文只记录当前 `refactor` 分支的工程事实和下一步优化靶点。旧 README
不作为架构事实来源；若本文与源码、测试或新 profile 冲突，以后者为准。

## 当前基线

- 代码基线：`777c752 跳过无关末段终点分类`。
- 计时基线：`build/performance/run_20260520_174831/timings.csv`，Release
  NoTracy，`benchmark_plan_10s_5m_1l`，`--threads 20`。
- Tracy 归因：`build/performance/run_20260520_174915/`，单个 large
  workload，RelWithDebInfo，Tracy 开启。
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
| 叶片片段通过 segment tracing 得到 front/back WNV | EMBER 4.4 | `tracePathWNVToSurfacePointTrusted()` 对候选路径传播 WNV，并在目标支撑面两侧取 WNV | 当前 `leaf_classification_trace_attempt_count` 仍高；候选生成、预筛和 trace 都在热点内 | 继续减少候选数量或无关多边形分类次数；禁止把无证明 fallback 变成默认路径 |
| 分类路径最多三段，由点的定义平面替换构造 | EMBER 3.2、4.4、Fig. 9 | `path_candidates.h` / `path_candidate_details.h` 提供 axis path、plane replacement、bridge rescue | exhaustive plane replacement 是高 self time；之前简单 view 化和 AABB 预检没有稳定收益 | 下一步审计目标三平面排列和替换顺序是否存在语义重复，而不是只做局部分配优化 |
| operator indicator early-out | EMBER 4.5.2 | `constant_discard_count`、single-operand assumption、leaf BSP/classification reuse 已接入 | 早停仍依赖当前 reference WNV 和局部保守判定；错误早停会直接破坏结果 | 只在能证明 entire child indicator 常量时扩展；用 oracle 或 metrics 对照验证 |
| split strategy 减少热点工作量 | EMBER 4.5.3 | WNTV-aware split、center range split、midpoint fallback 已有 metrics | 当前 split 仍可能造成 polygon 放大，且未直接把 leaf trace/BSP 成本纳入策略 | 先用 profile 找“polygon 放大 -> leaf trace 放大”的 workload，再动 split 逻辑 |
| work-stealing parallel | EMBER 4.5.4、5.3 | 当前 child 子树级 oneTBB 任务，merge 固定 left -> right | 并行边界较粗；叶内 BSP 和分类串行。并行扩展前必须证明无共享状态和稳定聚合 | 不先动并行；先压低单任务工作量和共享状态复杂度 |
| 固定宽度齐次整数图元 | EMBER 3.2；BSP paper Table 1、4.1 | 核心仍以 plane、`PlanePoint3i`、4D homogeneous classify 为边界；实验 fixed backend 只作窄接口/oracle | `Integer` 仍不是最终定长 backend；但不能一次性替换全局类型 | 后续只在 `paper_kernel` / fixed backend 窄接口内推进，并保持 oracle 测试 |

## 当前 profile 结论

`run_20260520_174915` 的 single-large Tracy self hot zones 显示，下一阶段
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

## 下一阶段优先队列

1. 审计 `enumerateLeafClassificationExhaustivePlaneReplacementPathCandidatesFromPoints`
   的目标三平面排列和替换顺序：只删除可证明等价的构造，不降低闭包安全性。
2. 审计局部 BSP carrier 流：确认同一 leaf/base polygon 是否重复插入等价 split
   segment，以及是否能在不增加状态膨胀的情况下提前合并。
3. 审计 split strategy 的实际放大链：用 `node_count`、`leaf_fragment_count`、
   `leaf_classification_trace_attempt_count` 找造成 trace 放大的 split 模式。
4. 对 fixed 256 backend 只做窄接口实验：优先 `classify_vertex` / 4D dot 和
   `intersect_3_planes`，不得让 `cpp_int` 或 `int512_t` 参与核心分支决策。

## 已测但不保留的局部实验

- 叶片分类候选路径 view：结构和 trace 计数不变，但 NoTracy solve 变慢。
- `buildLeafArrangement()` 结果 vector 预留容量：结构计数不变，solve 信号为负。
- 中间端点预筛缓存：结构计数不变，但缓存维护成本超过收益。
- trusted clipped polygon eager 顶点/AABB 缓存：结构计数不变，solve 信号为负。

这些结论只用于避免近期重复试错；若 workload、算法边界或 profile 证据变化，可以重新评估。
