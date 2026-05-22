# EMBER 论文机制对照审计

本文只记录当前 `refactor` 分支的工程事实和下一步优化靶点。旧 README
不作为架构事实来源；若本文与源码、测试或新 profile 冲突，以后者为准。

## 当前基线

- 最近可比性能基线：`提前释放叶节点中间几何`（本文所在提交）。
- 计时基线：`build/performance/run_20260521_080941/timings.csv`，Release
  NoTracy，论文实验集 `10 small / 10 medium / 2 large`，`--threads 20`。
- 最近 Tracy 归因：`build/performance/run_20260521_072920/`，单个 large
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
| work-stealing parallel | EMBER 4.5.4、5.3 | 当前 child 子树级 oneTBB 任务，merge 固定 left -> right；sibling task 提交门槛已降到半个 leaf threshold | 并行边界仍较粗；叶内 BSP 和分类串行。继续扩展前必须证明无共享状态和稳定聚合 | 下一步看更细粒度并行是否会被 task 开销抵消，不能改动结果聚合顺序 |
| 固定宽度齐次整数图元 | EMBER 3.2；BSP paper Table 1、4.1 | `PlanePoint3i` 已改为默认保留未约分三平面齐次交点；`classify_vertex` 暂用 512 位点积防止现有 `int256_t` 符号溢出 | 这是过渡层，不是最终 fixed 256 backend；导出阶段也因未约分点变重 | 下一步闭合平面系数预算或把 `classify_vertex` 接到自定义 fixed backend，并把 I/O canonical 化留在边界 |

## 当前 profile 结论

`run_20260521_072920` 的 single-large Tracy self hot zones 显示，下一阶段
最值得处理的不是 I/O 或导出，而是 solver 内部工作量：

- 最新 self 热点仍集中在 `math256::gcdMagnitude`、`math256::floorCeilDiv`、
  `enumerateLeafClassificationAxisPathCandidateFromKnownAxisProbe`、
  `buildSubdivisionSplitStats::polygon`、`Polygon256` 缓存重建、
  `BSPTree::addSegmentRecursive` 和 `tracePathWNVToSurfacePointImpl::polygon`。
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
- 叶片分类 trace 已按 leaf 懒缓存 `localReference.point` 对各 polygon 的侧别；
  缓存只在 polygon 通过路径 AABB 预筛后填充，避免为被跳过的 polygon 提前分类。

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
- `tracePathWNVToSurfacePointTrustedWithStartSides()` 为叶片分类增加 local reference
  起点侧别懒缓存；同一 leaf 内多条候选路径复用同一 polygon 起点分类，且仍在路径
  AABB 预筛之后才填充缓存。`run_20260521_063440` 对同一 10 small / 10 medium /
  2 large workload 做了 `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过且
  oracle cache hit，聚合 `solve_ms` 从 `run_20260521_060849` 的 1950.69ms 降到
  1927.46ms，`end_to_end_ms` 从 5336.47ms 降到 5307.45ms。
- `buildSubdivisionSplitStats()` 的 WNTV 分组预留从当前节点 polygon 数量收缩到小常数；
  二元布尔通常只有 lhs/rhs 两个 WNTV 类，保留自动增长能力但避免每个 subdivision
  节点按 polygon 数量过度预留。`run_20260521_071448` 对同一 10 small /
  10 medium / 2 large workload 做了 `-NoTracy -VerifyWithOracle`，22 个 verifier
  全部通过，聚合 `solve_ms` 从 `run_20260521_063440` 的 1927.46ms 降到
  1923.92ms，`end_to_end_ms` 从 5307.45ms 降到 5282.89ms。
- `WntvSubdivisionGroups` 改为小数组优先存储：常见二元 lhs/rhs WNTV 分组不再进入
  vector 堆路径，超过 4 个 WNTV 类时才使用 overflow vector，仍保持多类输入兼容。
  `run_20260521_072311` 对同一 10 small / 10 medium / 2 large workload 做了
  `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，聚合 `solve_ms` 从
  `run_20260521_071448` 的 1923.92ms 降到 1893.49ms，`end_to_end_ms` 从
  5282.89ms 降到 5251.62ms。
- 内部 subdivision 节点在成功创建 child solver 后立即清空自己的 `polygons_`；
  该节点后续只需要 `polygonCount_`、子树结果和 metrics，不再访问父 polygon soup。
  `run_20260521_080040` 对同一 10 small / 10 medium / 2 large workload 做了
  `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，聚合 `solve_ms` 从
  `run_20260521_072311` 的 1893.49ms 降到 1879.84ms；`run_20260521_080523`
  额外做 3 次无 oracle 重复计时，按每轮 22 个 workload 聚合 `solve_ms` 为
  1868.40ms。端到端时间受导出和 I/O 噪声影响未同步下降，solver 指标保留为正收益。
- 叶节点完成 metrics 记录后立即清空 `polygons_`、`leafFragments_` 和
  `classifiedFragments_`；后续只需要 `resultFragments_`、`leafSummaries_` 和计数。
  `run_20260521_080941` 对同一 10 small / 10 medium / 2 large workload 做了
  `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，聚合 `solve_ms` 从
  `run_20260521_080040` 的 1879.84ms 降到 1852.88ms，`end_to_end_ms` 从
  5274.77ms 降到 5231.12ms。
- axis / plane replacement 分类路径复用已知端点缓存构造 `Segment256`：路径构造阶段
  已经持有当前点、下一整数点或目标 axis-probe 点时，直接走 trusted segment 入口，
  避免按同一组三平面重复恢复端点并重复做线段有效性分类；该改动仍只使用论文允许的
  坐标平面、已有支撑平面、三平面交点和 4D dot，不引入新平面。`run_20260521_083433`
  对同一 10 small / 10 medium / 2 large workload 做了 `-NoTracy -VerifyWithOracle`，
  22 个 verifier 全部通过，聚合 `solve_ms` 从 `run_20260521_080941` 的
  1852.88ms 降到 1777.65ms，`end_to_end_ms` 从 5231.12ms 降到 5196.95ms。
  `run_20260521_083920` 额外做 3 次无 oracle 重复计时，按每轮 22 个 workload
  聚合 `solve_ms` 为 1791.98ms、1800.61ms、1791.71ms，确认收益不是单次噪声。
- `buildSubdivisionSplitStats()` 的三轴 AABB 中心计算改为直接读取 `x/y/zMin/Max`
  字段，避免在每个 polygon 上反复通过 `axisMidpoint()` / `axisMinimum()` /
  `axisMaximum()` 做轴分派；公式仍是各轴 `floor((min + max) / 2)`，不改变切分语义。
  `run_20260521_084638` 对同一 10 small / 10 medium / 2 large workload 做了
  `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，单轮聚合 `solve_ms`
  为 1786.30ms；`run_20260521_085121` 额外做 3 次无 oracle 重复计时，按每轮
  22 个 workload 聚合 `solve_ms` 为 1781.70ms、1783.78ms、1791.77ms，
  相比上一阶段 3 次均值约 1794.77ms 继续下降。
- trusted clipping 的 release 路径只保留边数和 provenance 数量检查，把每条输出边的
  `hasUniqueIntersection()` / 支撑平面平行性结构验证和 `isValid()` 留在 Debug；
  依据是当前裁剪输入已经是有效凸多边形，边顺序和新增裁剪边由同一次遍历维护。
  `run_20260521_085436` 对同一 10 small / 10 medium / 2 large workload 做了
  `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，聚合 `solve_ms` 从
  1786.30ms 降到 1761.29ms，`end_to_end_ms` 从 5196.28ms 降到 5147.10ms。
  `run_20260521_085919` 额外做 3 次无 oracle 重复计时，按每轮 22 个 workload
  聚合 `solve_ms` 为 1745.69ms、1754.54ms、1762.66ms，收益稳定。
- plane replacement segment 增加“已定向端点缓存”内部构造入口：该入口允许端点
  边界平面保留非 primitive 比例，只要求方向线、两端齐次点缓存和外侧半空间约定
  已经由调用方验证。`buildPlaneReplacementSegment()` 因此不再把核心路径里的端点
  平面重新 `primitivePlane()`，更贴近 BSP 论文中 core solve 不依赖 gcd 的边界。
  `run_20260521_101713` 对同一 10 small / 10 medium / 2 large workload 做了
  `-NoTracy -VerifyWithOracle`，22 个 verifier 全部通过，单轮聚合 `solve_ms`
  为 1741.11ms；`run_20260521_102201` 三次无 oracle 重复计时为
  1752.32ms、1742.11ms、1744.67ms，均值 1746.37ms，略优于
  `run_20260521_085919` 的 1754.30ms。
- AABB 裁剪线段也改用同一“已定向端点缓存”入口：裁剪命中点已经由
  `intersect_3_planes` 得到，旧端点缓存也已存在；只需检查新旧端点分别落在对方
  边界平面的严格内侧，就能避免普通 `Segment256` 构造里的 endpoint plane
  primitive 化和重复端点恢复。`run_20260521_102636` 的 22 个 verifier 全部通过，
  单轮聚合 `solve_ms` 为 1745.46ms；`run_20260521_103114` 三次无 oracle 重复
  计时为 1744.71ms、1719.84ms、1721.30ms，均值 1728.62ms。
- 局部 BSP 递归插入在线段 carrier 的 `insertPlane` 与当前节点 `splitPlane`
  完全相同时直接返回，避免后续两个端点的三平面未约分交点分类；这是 `side0==0 &&
  side1==0` 的代数恒等短路，不改变 BSP 切分语义。Debug 测试通过，
  `run_20260521_114523` 的 22 个 verifier 全部通过且单轮 `solve_ms`
  为 1728.42ms；`run_20260521_114948` 三次无 oracle 重复计时为
  1735.36ms、1708.12ms、1719.65ms，均值 1721.04ms。
- 叶片曲面点 WNV trace 的 segment 循环先用已构造的 segment AABB 判断该段是否可能
  命中当前 polygon，只有相关段才分类起点和终点；若中间跳过了不相关段，则在下一段
  相关时重新分类该段起点，避免沿用过期侧别。该改动不改变路径候选、trace 次数或
  WNV 数学，只减少无关段的 `classify_vertex` 点积。Debug 测试通过，
  `run_20260521_120915` 的 22 个 verifier 全部通过且单轮 `solve_ms`
  为 1714.28ms；`run_20260521_121337` 三次无 oracle 重复计时为
  1709.71ms、1719.26ms、1716.58ms，均值 1715.18ms。
- 叶片曲面点 WNV trace 在计算线段与 polygon 支撑平面的交点时，改用
  `intersectHomogeneousUnnormalized()` 构造未规范化齐次点；后续只做边半空间符号分类，
  不需要 canonical 齐次点。该改动对应 BSP 论文中 `intersect_3_planes` 输出直接供
  `classify_vertex` 复用的数学边界，避免把核心求解热路径拉回点规范化。Debug 测试通过，
  `run_20260521_122941` 的 22 个 verifier 全部通过且单轮 `solve_ms`
  为 1660.13ms；`run_20260521_123402` 三次无 oracle 重复计时为
  1695.42ms、1674.61ms、1685.70ms，均值 1685.24ms。
- `Polygon256` 的派生顶点缓存改为保存 `intersectHomogeneousUnnormalized()` 的原始
  三平面交点。缓存顶点在求解核心里用于 `classify_vertex`、AABB 整数包围和比例等价
  比较，均不要求 canonical 齐次坐标；I/O 和去重边界仍显式做 primitive 化。Debug 测试通过，
  `run_20260521_124929` 的 22 个 verifier 全部通过且单轮 `solve_ms`
  为 1694.39ms；`run_20260521_125353` 三次无 oracle 重复计时为
  1688.01ms、1649.74ms、1692.92ms，均值 1676.89ms。
- sibling 子树并行提交门槛从 `2 * leafPolygonThreshold` 降到
  `leafPolygonThreshold`，让 25 到 50 个 polygon 的中等子树也进入 oneTBB
  work-stealing；该改动只改变调度，不改变几何决策和 merge 顺序。Debug 测试通过，
  `run_20260521_132017` 的 22 个 verifier 全部通过且单轮 `solve_ms`
  为 1650.381ms；`run_20260521_131933` 三次无 oracle 重复计时为
  1644.79ms、1658.35ms、1669.02ms，均值 1657.38ms，较
  `run_20260521_125353` 的 1676.89ms 继续降低约 1.16%。
- sibling 子树并行提交门槛继续降到 `leafPolygonThreshold / 2`（下限 8），
  进一步暴露中小子树给 oneTBB work-stealing；仍保持父节点固定先 merge
  left 再 merge right，因此只改变执行调度。Debug 测试通过，
  `run_20260521_134123` 的 22 个 verifier 全部通过且单轮 `solve_ms`
  为 1645.242ms；`run_20260521_134044` 三次无 oracle 重复计时为
  1649.68ms、1649.91ms、1643.52ms，均值 1647.70ms，较
  `run_20260521_131933` 的 1657.38ms 继续降低约 0.58%。
- 叶片分类候选目标在生成阶段已经通过严格内部点检查，Release 热路径在
  `prepareLeafClassificationCandidate()` 和 candidate repair 中不再重复执行
  `fragment.containsStrictly(targetPoint)`；Debug 仍保留检查来守住候选枚举器前置条件。
  该改动不改变候选路径、trace 次数或 WNV 传播数学，只减少每个候选准备阶段的重复
  边半空间分类。Debug 测试通过，`run_20260521_140155` 的 22 个 verifier
  全部通过；`run_20260521_140622` 三次无 oracle 重复计时为 1640.84ms、
  1625.48ms、1656.14ms，均值 1640.82ms，较 `run_20260521_134044`
  的 1647.70ms 继续降低约 0.42%。
- 叶片分类候选路径由枚举器逐段构造并在生成边界完成可 trace 闭包检查，Release
  热路径在 `prepareLeafClassificationCandidate()` 中不再重复调用
  `isTraceableSurfaceCandidatePath()` 和 repair fallback；Debug 仍保留完整校验与修复逻辑，
  用于捕获未来候选生成器破坏前置条件的回归。`run_20260521_140622` 的指标中
  `leaf_classification_candidate_repair_attempt_count` 和 repair success 均为 0，说明当前
  paper workload 没有实际依赖该 fallback。Debug 测试和 ctest 通过，
  `run_20260521_141316` 的 22 个 verifier 全部通过；`run_20260521_141226`
  三次无 oracle 重复计时为 1642.17ms、1637.29ms、1622.52ms，均值
  1633.99ms，较 `run_20260521_140622` 的 1640.82ms 继续降低约 0.42%。
- WNV trace 的 path AABB 预筛把每条候选路径的 segment box 临时表从
  `std::vector<AABB3i>` 换成 4 段内联的小缓冲。论文候选路径通常为 1-3 段，
  该改动不改变 path box、segment box、polygon AABB 剪枝或 WNV 传播，只减少
  高频 trace 的临时堆分配。Debug 测试和 ctest 通过，`run_20260521_142359`
  的 22 个 verifier 全部通过；`run_20260521_142321` 三次无 oracle 重复计时为
  1630.88ms、1625.60ms、1631.27ms，均值 1629.25ms，较
  `run_20260521_141226` 的 1633.99ms 继续降低约 0.29%。该类容器级优化收益已很小，
  后续应转向候选数量、leaf-local BSP 和 trace 扫描面的结构性重构。
- subdivision 的 center-range 切分选择从“中心范围最大”改为估计局部复制代价：
  优先减少会被复制到左右 child 的 polygon 数，其次压低最大 child polygon 数和
  不平衡度，最后才用中心范围作为 tie-breaker。WNTV-aware 仍保持论文 4.5.3 的
  分离距离优先；对照实验显示若在 WNTV 候选内部也按复制代价排序，虽然结构指标下降，
  但 large/medium workload 有更多回归，三次均值仅为 `run_20260521_143429`
  的 1614.88ms。当前保留版本只把复制代价用于无 WNTV 分离候选时的 center-range，
  对齐 BSP 论文 4.5 中按局部 BSP 复杂度控制 cell 的方向，同时保留 WNTV 语义分离权重。
  Debug 测试和 ctest 通过，`run_20260521_144257` 的 22 个 verifier 全部通过；
  `run_20260521_144122` 三次无 oracle 重复计时为 1586.80ms、1605.31ms、
  1607.50ms，均值 1599.87ms，较 `run_20260521_142321` 的 1629.25ms
  降低约 1.80%。结构指标同步下降：平均 `node_count` 25739 -> 24156，
  `total_polygon_count` 4137580 -> 4047214，`leaf_classification_trace_attempt_count`
  232002 -> 222678。
- `Polygon256::aabb()` 拆分 AABB 缓存和顶点缓存：纯 broad-phase AABB 请求不再顺带
  填充 `cachedVertices_`，而是直接用未约分 `intersect_3_planes` 齐次点扩展整数包围盒；
  若顶点缓存已经存在，则仍从顶点缓存重建 AABB。该版本是在未约分顶点和新版 split
  基线之后重测旧 AABB-only 方向，语义不变、结构指标与 `run_20260521_144122` 一致：
  `node_count` 24156、`leaf_node_count` 10317、`total_polygon_count` 4047214、
  `leaf_fragment_count` 456721、`leaf_classification_trace_attempt_count` 222678。
  Debug 测试和 ctest 通过，`run_20260521_152314` 的 22 个 verifier 全部通过；
  `run_20260521_152227` 三次无 oracle 重复计时为 1601.40ms、1595.39ms、
  1593.52ms，均值 1596.77ms，略优于 `run_20260521_144122` 的 1599.87ms。
- OBJ raw 导出阶段从逐项 `operator<<` 改为先用 `std::to_chars` 组装单个文本缓冲，
  再一次性写入文件；顶点恢复、齐次点 primitive 去重、face 顺序和输出拓扑语义不变。
  该改动针对 100 组论文样本里 `export_ms` 已超过 `solve_ms` 的端到端瓶颈，不改变
  solver 结构指标。`ctest --preset default --output-on-failure --timeout 120` 通过；
  `run_20260521_233848` 对 34 small / 33 medium / 33 large 做 `-NoTracy`、不跑 verifier，
  平均 `export_ms` 从 `run_20260521_231745` 的 140.127ms 降到 91.831ms，
  `end_to_end_ms` 从 394.360ms 降到 344.953ms；slowest `large_021_1396886_minus_551020`
  的 `export_ms` 从 710.786ms 降到 479.708ms，`result_fragment_count` 仍为 116537。
- OBJ raw 导出的最终文本生成按固定块并行生成顶点行和面行，再按块顺序拼接回同一个
  输出缓冲；小输出仍走原单线程路径。该改动只改变文本组装调度，不改变顶点去重、
  face 顺序或 solver 指标。Debug 构建和 `ctest --preset default --output-on-failure --timeout 120`
  通过；两轮 100 组论文样本 `-NoTracy`、不跑 verifier 的结构计数与
  `run_20260522_000710` 完全一致。`run_20260522_003433` 平均 `export_ms`
  从 91.532ms 降到 90.378ms，`end_to_end_ms` 从 340.942ms 到 339.160ms；
  `run_20260522_003601` 平均 `export_ms` 降到 90.185ms，但端到端受 solve/prepare
  噪声影响为 341.594ms。只并行顶点行的 `run_20260522_003820` 降幅较弱
  （`export_ms` 90.876ms），因此保留顶点行和面行同时分块的版本。
- `buildPolygonSoup()` 的 face 级构建结果从“每个 face 一个 `std::vector<Polygon256>`”
  改为单 polygon 内联保存，只有非共面三角化 fallback 才使用 overflow vector。
  常规 OBJ 面仍按原有顺序生成同一个 `Polygon256`，三角化 fallback 语义不变。
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 通过；两轮
  100 组论文样本 `-NoTracy`、不跑 verifier 的 polygon/solver 结构计数均与
  `run_20260521_233848` 一致。`run_20260522_000556` 平均 `prepare_ms`
  从 116.628ms 降到 112.873ms，`end_to_end_ms` 从 344.953ms 降到 341.895ms；
  `run_20260522_000710` 平均 `prepare_ms` 为 112.997ms，`end_to_end_ms`
  为 340.942ms，确认 prepare 阶段收益稳定。
- center-range fallback 的三轴候选成本估计从“每个候选一次完整 polygon AABB
  扫描”改成收集候选后单次扫描同时累计三组成本。候选集合、tie-break 顺序和
  切分结果不变，只减少固定平均中心策略的重复 broad-phase 统计工作。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；100 组论文样本
  `run_20260522_004433` 与 `run_20260522_000710` 的 `node_count`、
  `total_polygon_count`、`leaf_fragment_count`、trace 次数和 split 策略计数完全一致。
  平均 `solve_ms` 从 120.266ms 降到 119.261ms，`end_to_end_ms` 从
  340.942ms 到 339.122ms。
- WNV surface-point trace 在已经算出 segment 与 polygon 支撑面交点、且该交点被分类为
  polygon 边界命中时，改用已知命中点直接收集边界边来源，不再调用通用
  `classifySegmentPolygonBoundaryContactUnchecked()` 重复求支撑面交点和面内位置。
  该改动只影响允许 subdivision clip 边界穿越的判定前置分类，不改变 WNV 更新、
  候选路径或边界策略。Debug 构建和 `ctest --preset default --output-on-failure --timeout 120`
  通过；两轮 100 组论文样本 `-NoTracy`、不跑 verifier 与 `run_20260522_004433`
  的结构计数和 trace 状态计数完全一致。`run_20260522_005611` 平均 `solve_ms`
  为 118.241ms、`end_to_end_ms` 为 337.422ms；`run_20260522_005732`
  平均 `solve_ms` 为 119.231ms、`end_to_end_ms` 为 338.561ms。
- plane replacement fallback 路径为最多三段的 `rawPath` / `outPath` 预留容量，
  避免 AABB 内替换路径失败后进入 raw path + clip/bridge 时发生短 vector 扩容。
  该改动不改变候选点、替换顺序、trace 次数或 WNV 传播数学。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；两轮 100 组论文样本
  `run_20260522_021842` / `run_20260522_021956` 与 `run_20260522_005732`
  的结构计数、result 计数、`leaf_classification_trace_attempt_count` 和
  `leaf_classification_plane_replacement_path_attempt_count` 完全一致。两轮新结果均值
  `solve_ms` 约 118.253ms、`end_to_end_ms` 约 337.346ms，优于
  `run_20260522_005611` / `run_20260522_005732` 两轮保留基线均值
  118.736ms / 337.992ms。
- raw OBJ 导出在 `validateFragments=false` 且不需要 topology metadata 时绕过
  `RecoveredPolygonBuildResult::orderedVertices` 中间复制，先并行触发 fragment 顶点缓存
  和有限性检查，再按原顺序直接从 `Polygon256::vertices()` 做全局齐次顶点去重与
  face index 恢复。公开 API 默认验证路径、conforming/STL 拓扑恢复路径不变。
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 通过；两轮
  100 组论文样本 `run_20260522_023707` / `run_20260522_023820` 与
  `run_20260522_021956` 的结构计数、result 计数、exported face 数和 trace 次数完全一致，
  且第二轮 100 个 raw OBJ 与基线逐文件 SHA256 完全一致。相对
  `run_20260522_021842` / `run_20260522_021956` 当前保留基线，两轮新结果均值
  `export_ms` 从 89.545ms 降到 84.127ms，`end_to_end_ms` 从 337.345ms 降到
  332.007ms，`process_elapsed_ms` 从 387.247ms 降到 376.655ms。
- WNV trace 的 polygon 级 AABB 预筛改为每个 polygon 只读取一次缓存 AABB，
  并把同一 `polygonBox` 传给整条 path 预筛和逐 segment 相关性预筛，避免在
  `tracePathWNVImpl::polygon` / `tracePathWNVToSurfacePointImpl::polygon` 内对同一
  polygon 反复走 `poly.aabb()` 访问路径。该改动不改变 path、候选、分类侧别或
  WNV/WNTV 累加数学。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；两轮 100 组论文样本
  `run_20260522_030701` / `run_20260522_030809` 与 `run_20260522_023820`
  的结构计数完全一致，100 个 raw OBJ 与基线逐文件 SHA256 完全一致。相对
  raw 导出 fast path 保留基线 `run_20260522_023707` / `run_20260522_023820`，
  两轮新结果均值 `solve_ms` 从 118.826ms 降到 118.311ms，`export_ms` 从
  84.127ms 降到 83.857ms，`end_to_end_ms` 从 332.007ms 降到 331.715ms。
- WNV trace 的整条 path 预筛移除 `doesPlaneIntersectAABB(polygonPlane, pathBox)`
  粗判断，只保留 path AABB 与 polygon AABB 的重叠判断；逐 segment 相关性预筛中
  的 `doesPlaneIntersectAABB(poly.plane, segmentBox)` 仍保留。原因是整条 path box
  对 polygon plane 的粗测试在当前 100 组论文样本中不减少 trace 次数，却会给每个
  polygon/path 组合增加一次 plane-box 分类成本。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；两轮 100 组论文样本
  `run_20260522_033808` / `run_20260522_033921` 与当前保留基线
  `run_20260522_030701` / `run_20260522_030809` 的结构计数和
  `leaf_classification_trace_attempt_count` 完全一致，第二轮 100 个 raw OBJ 与
  `run_20260522_030809` 逐文件 SHA256 完全一致。两轮新结果均值 `solve_ms`
  从 118.311ms 降到 117.948ms，`end_to_end_ms` 从 331.715ms 降到
  331.085ms，`process_elapsed_ms` 从 376.917ms 降到 376.034ms。
- 叶片分类 axis-probe 路径中的纯整数网格段直接用已知两端 `PlanePoint3i`
  构造 `Segment256`，不再回到通用 coordinate-plane 段构造入口重复恢复端点、
  定向边界平面并做端点互侧分类。该路径只用于已确认起点和下一整数 AABB 网格点的
  轴对齐段，最后通向支撑平面的非整数段仍走原通用构造；候选点、trace 次数和
  WNV 传播数学不变。Debug 构建和 `ctest --preset default --output-on-failure --timeout 120`
  通过；两轮 100 组论文样本 `run_20260522_040610` / `run_20260522_040952`
  与当前保留基线 `run_20260522_033921` 的结构计数和
  `leaf_classification_trace_attempt_count` 完全一致，100 个 raw OBJ 与基线逐文件
  SHA256 完全一致。两轮新结果均值 `solve_ms` 从 117.948ms 降到 117.172ms，
  `end_to_end_ms` 从 331.085ms 降到 330.307ms，`process_elapsed_ms`
  从 376.034ms 降到 375.520ms。
- 叶片局部 BSP pair relation 已在 `buildLeafPairRelation()` 入口确认两个 polygon
  AABB 重叠，`computeBidirectionalPolygonIntersectionCarriersTrusted()` 不再重复做同一
  AABB 检查，只保留平面平行和双向交线 carrier 构造。当前该 trusted 入口只有这一处
  调用，因此前置条件闭合在 leaf arrangement 内部，不改变 pair 枚举、carrier 语义或
  WNV 分类。Debug 构建和 `ctest --preset default --output-on-failure --timeout 120`
  通过；两轮 100 组论文样本 `run_20260522_042422` / `run_20260522_042538`
  与当前保留基线 `run_20260522_040952` 的结构计数和 trace 次数完全一致，100 个
  raw OBJ 与基线逐文件 SHA256 完全一致。两轮新结果均值 `solve_ms`
  从 117.172ms 降到 117.035ms。
- WNV surface-point trace 的末段如果已经通过 `poly.classify(endPoint)` 确认终点在
  当前 polygon 内/边界，且起点侧别是严格 front/back（`pcs == ±1`），则支撑面交点
  必定就是该终点；此时直接复用 `endPoint` 做面内位置分类，跳过
  `intersectLinePlaneUnnormalized()`。共面外侧贴边路径仍走原非唯一交点和边界接触
  逻辑，避免把 `Polygon256::classify()` 的外部同面返回值误当成穿越方向。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；两轮 100 组论文样本
  `run_20260522_045009` / `run_20260522_045123` 与当前保留基线
  `run_20260522_042538` 的结构计数、trace 诊断计数和 100 个 raw OBJ SHA256
  完全一致。两轮新结果均值 `solve_ms` 从 117.035ms 降到 116.842ms。
- 叶片局部 BSP 在 `polygonCount >= 8` 的预计算 pair-relation 路径中不再保存完整
  `basePolygon` 副本；该路径已经提前得到双向 carrier 或共面关系，后续只需要
  root leaf geometry、base 支撑平面、稳定 order key 和插入多边形本身。small-case
  `insertTrusted()` 路径仍保留完整 base polygon，避免破坏公开 BSP 插入前置条件。
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 通过；两轮
  100 组论文样本 `run_20260522_053232` / `run_20260522_053346` 与当前保留基线
  `run_20260522_045123` 的结构计数、trace 诊断计数和 100 个 raw OBJ SHA256
  完全一致。两轮新结果均值 `solve_ms` 从 116.842ms 降到 116.541ms；
  `export_ms` / `end_to_end_ms` 基本处于导出和调度噪声内。
- 叶片局部 BSP 抽片与叶片分类融合：为 `BSPTree` 和 `leaf_arrangement` 增加逐片
  visitor 入口，普通叶片不再先把全部启用片段批量写入 `leafFragments_` 后再第二轮分类，
  而是在局部 BSP 抽出一个启用 leaf geometry 后立即调用原有 `classifyLeafFragment()`
  并写入结果。单操作数跳过 leaf BSP 的 bulk fast path 仍保留原 `polygons_` 别名语义；
  `buildLeafArrangement()` 的 vector 返回入口也保留给测试和旧调用方。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；三轮 100 组论文样本
  `run_20260522_061654` / `run_20260522_061813` / `run_20260522_062118`
  与上一保留基线的结构计数、trace 诊断计数和 100 个 raw OBJ SHA256 完全一致。
  三轮新结果均值 `solve_ms` 从 116.541ms 降到 115.888ms；`end_to_end_ms`
  为 332.787ms，`process_elapsed_ms` 为 376.636ms。
- streaming 叶片分类路径不再保存 `ClassifiedFragment` 中间数组：普通叶片局部 BSP
  visitor 已经逐片分类并立刻决定是否输出，因此成功 trace 得到的 `front/back WNV`
  直接保存在当前 `LeafClassificationContext`，并立即用布尔指示函数写入
  `resultFragments_`；`(OUT, IN)` 直接输出时移动当前 fragment，`(IN, OUT)` 仍按原逻辑
  生成反向片段。旧 `leafFragments_` 别名路径和 `buildLeafArrangement()` vector 返回入口
  仍保留 `ClassifiedFragment` 行为。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；两轮 100 组论文样本
  `run_20260522_062546` / `run_20260522_062700` 与上一保留基线的结构计数、trace 诊断计数
  和 100 个 raw OBJ SHA256 完全一致。两轮新结果均值 `solve_ms` 从 115.888ms 降到
  114.508ms；`end_to_end_ms` 为 330.745ms，`process_elapsed_ms` 为 374.304ms。
- 叶片编排 pair-relation adjacency 改为按 base polygon 链式追加：`polygonCount >= 8`
  的预计算路径不再把所有 adjacency 写入全局数组后排序并构造 offsets，而是为每个
  base 保留 head/tail 链表。pair relation 仍只计算一次，且每个 base 的插入顺序
  仍保持 polygonIndex 递增。`ctest --test-dir build\tests --output-on-failure --timeout 120`
  通过；`run_20260522_063615` 对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。
  `run_20260522_063924` 的 100 组 NoTracy 与 `run_20260522_062700` 的 key structural
  counters、leaf classification trace/candidate 计数完全一致，`solve_ms` 从 114.895ms
  到 113.992ms，`end_to_end_ms` 从 330.789ms 到 330.468ms。该收益很小，主要价值是
  去掉局部 BSP 编排热路径中的排序和 offsets 构造。
- `BoolProblem::setOperands()` 增加 rvalue 输入入口，并让 CLI 在 polygon soup 构建后
  直接把左右输入移交给 `BoolProblem`。公开 const 引用入口仍保留并继续复制调用方输入，
  但内部合并改为 move，避免 prepare 阶段把左右 `Polygon256` 集合再整表复制一遍。
  `ctest --test-dir build\tests --output-on-failure --timeout 120` 通过；
  `run_20260522_064450` 对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。
  `run_20260522_064654` 的 100 组 NoTracy 与 `run_20260522_063924` 的 key structural
  counters、leaf classification trace/candidate 计数完全一致，`prepare_ms` 从
  117.092ms 降到 51.839ms，`end_to_end_ms` 从 330.468ms 降到 265.735ms；
  `solve_ms` / `export_ms` 基本只剩计时噪声。
- raw trusted OBJ 导出增加紧凑 face index 恢复路径：当 CLI 使用
  `topologyMode=Raw` 且 `validateFragments=false` 时，导出阶段仍按原顺序做齐次顶点
  primitive 去重，但 face 不再表示为每面一个小 `std::vector<std::size_t>`，而是用
  扁平 vertex-index 数组加 offsets 写出 OBJ。公开验证路径、conforming 拓扑恢复和 STL
  导出仍走原 `RecoveredPolygonSoupData`。`ctest --test-dir build\tests --output-on-failure --timeout 120`
  通过；`run_20260522_065244` 对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。
  `run_20260522_065445` 的 100 组 NoTracy 与 `run_20260522_064654` 的 key structural
  counters、leaf classification trace/candidate 计数完全一致，100 个 raw OBJ SHA256
  完全一致；平均 `export_ms` 从 83.672ms 降到 81.736ms，`end_to_end_ms`
  从 265.735ms 到 264.485ms。后续把该紧凑路径的每 fragment 错误字符串槽改为
  原子记录最小失败下标，成功导出时不再构造整批空字符串；`run_20260522_065912`
  的 34 个 small verifier 全部通过，`run_20260522_065758` 相比 `run_20260522_065445`
  的结构计数和 100 个 raw OBJ SHA256 完全一致，平均 `export_ms` 从 81.736ms
  到 81.431ms，`end_to_end_ms` 从 264.485ms 到 263.519ms。
- `BoolProblem` 保留 `SubdivisionSolver` 的结果块，默认 CLI raw OBJ 导出直接顺序扫描
  `resultFragmentChunks()`，不再在 `solve()` 末尾把所有子树结果移动到一个大
  `resultFragments_` vector；公开 `resultFragments()` 仍保留并在首次读取时按需物化，
  verifier 和测试语义不变。`ctest --test-dir build\tests --output-on-failure --timeout 120`
  通过；`run_20260522_070826` 对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。
  `run_20260522_070657` 的 100 组 NoTracy 与 `run_20260522_065758` 的 key structural
  counters、leaf classification trace/candidate 计数完全一致，100 个 raw OBJ SHA256
  完全一致；平均 `solve_ms` 从 114.365ms 降到 109.153ms，`export_ms` 基本持平
  （81.431ms 到 81.898ms），`end_to_end_ms` 从 263.519ms 降到 258.271ms。
- 默认 leaf threshold 从 25 调到 75，并同步 CLI、verifier、visual-test、CTest
  paper small 批量入口和 `tools/profile-re-ember.ps1` 默认参数；显式传入的阈值仍覆盖默认值。
  该调整不改变几何判定，只改变递归停止点和 leaf 局部 BSP 规模。100 组 NoTracy
  参数扫描显示：`run_20260522_070657`（25）平均 `solve_ms=109.153ms`、
  `export_ms=81.898ms`、`end_to_end_ms=258.271ms`；`run_20260522_072029`（50）
  为 `112.015ms / 73.514ms / 252.890ms`；`run_20260522_072122`（75）
  为 `114.069ms / 68.916ms / 250.008ms`；`run_20260522_072214`（125）
  为 `124.081ms / 64.799ms / 256.101ms`。75 在当前 raw OBJ 默认流水线中端到端最优，
  同时相比 25 将平均 `node_count` 从 1485.36 降到 547.12、`leaf_bsp_build_count`
  从 423.04 降到 166.34、`result_fragment_count` 从 29698.00 降到 27036.72。
  改默认值后的保留重跑 `run_20260522_072754` 确认脚本默认传入 75，结构计数与扫描一致，
  平均 `solve_ms=115.096ms`、`export_ms=69.116ms`、`end_to_end_ms=252.100ms`。
  `run_20260522_072559` 对 34 个 small 样本使用默认 75 和 CGAL oracle 校验，全部通过。
- 二元应用入口选择共享 scale 时改走 `chooseSharedScale(lhs, rhs, ...)` 引用入口，
  CLI、verifier 和 visual-test 不再用 `{lhsMesh, rhsMesh}` 为接口临时复制完整
  `ObjMeshData`；保留向量入口给需要扫描任意 mesh 集合的公开调用方。该改动只减少
  prepare 数据搬运，不改变共享 scale 选择规则。`ctest --test-dir build\tests
  --output-on-failure --timeout 120` 通过；`run_20260522_073247` 对 34 个 small
  样本做 `-VerifyWithOracle` 全部通过。`run_20260522_073439` 的 100 组 NoTracy
  与 `run_20260522_072754` 的 key structural counters、leaf classification
  trace/candidate 计数一致，平均 `prepare_ms` 从 52.091ms 降到 48.236ms，
  `end_to_end_ms` 从 252.100ms 降到 248.175ms；`solve_ms` / `export_ms`
  基本只剩计时噪声。
- 应用层输入准备改用 `buildPolygonSoupWithAABB()`：每个 mesh 的浮点顶点
  `floor/ceil` AABB 区间和四舍五入量化顶点在同一轮静态并行扫描里生成，
  后续面构造仍复用原来的严格 `Polygon256` 构建逻辑；公开的
  `computeScaledMeshAABB()` / `buildPolygonSoup()` 仍保留给拆分调用方。`ctest
  --test-dir build\tests --output-on-failure --timeout 120` 通过；`run_20260522_073835`
  对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。`run_20260522_074030`
  的 100 组 NoTracy 与 `run_20260522_073439` 的 key structural counters、
  leaf classification trace/candidate 计数一致，100 个 raw OBJ SHA256 完全一致；
  平均 `prepare_ms` 从 48.236ms 降到 47.382ms，`end_to_end_ms` 从 248.175ms
  降到 247.759ms。
- raw trusted OBJ 导出的全局顶点去重从 `std::unordered_map` 节点表改为局部
  flat hash 链表：仍对每个齐次点先做 `primitiveHomPoint()`，仍按首次出现顺序给
  顶点编号并生成同一 face index 序列，但桶、key 和 next 链表都存放在连续数组中，
  避免每个唯一顶点一次节点分配。`ctest --test-dir build\tests --output-on-failure
  --timeout 120` 通过；`run_20260522_075555` 对 34 个 small 样本做
  `-VerifyWithOracle` 全部通过。`run_20260522_075742` 的 100 组 NoTracy
  与保留基线 `run_20260522_074030` 的 key structural counters、
  leaf classification trace/candidate 计数一致，100 个 raw OBJ SHA256 完全一致；
  平均 `export_ms` 从 69.227ms 降到 66.575ms，`end_to_end_ms` 从
  247.759ms 降到 245.732ms。
- raw trusted OBJ 导出在 `resultFragmentChunks()` 粒度先做局部齐次顶点去重，
  再按 chunk 顺序合并到全局 flat hash：每个 worker 只扫描本 chunk 的
  fragment，生成局部唯一顶点、紧凑 face index 和 face offset；全局阶段只对
  chunk 内唯一点做一次 `primitiveHomPoint()`/flat hash 插入，然后把局部 index
  remap 回原 face 槽位顺序。该改动保持全局首次出现顺序、face 顺序和 raw OBJ
  文本完全不变，但把全局去重调用从所有 face slot 缩到每个 chunk 的局部唯一点。
  `ctest --test-dir build\tests --output-on-failure --timeout 120` 通过；
  `run_20260522_081603` 对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。
  `run_20260522_081436` 的 100 组 NoTracy 与保留基线 `run_20260522_075742`
  的结构计数、result/exported face 数和 leaf classification trace 次数一致，
  100 个 raw OBJ SHA256 完全一致；平均 `export_ms` 从 66.575ms 降到
  49.571ms，`end_to_end_ms` 从 245.732ms 降到 228.397ms。
- raw 导出成本下降后重新扫描默认 leaf threshold，把默认值从 75 回调到 50：
  50 会比 75 产生更多叶片和 result fragment，但明显减少叶片内局部 BSP/trace
  压力，新的导出路径能覆盖额外输出片段成本。100 组 NoTracy 扫描中，
  `run_20260522_082017`（25）平均 `solve_ms/export_ms/end_to_end_ms` 为
  `110.347ms / 60.647ms / 234.200ms`；`run_20260522_082057`（50）
  为 `112.089ms / 52.425ms / 227.776ms`；保留基线
  `run_20260522_081436`（75）为 `115.558ms / 49.571ms / 228.397ms`。
  复跑 `run_20260522_082153`（50）为 `111.781ms / 52.769ms / 227.946ms`，
  说明 50 对 `solve_ms` 的收益稳定，端到端也略优于 75；显式传入的阈值仍覆盖默认值。
- raw trusted OBJ 导出的 chunk 局部去重结果保留 primitive 齐次 key，顺序合并到
  全局 flat hash 时直接复用该 key，不再对每个 chunk 局部唯一点重复调用
  `primitiveHomPoint()`；局部和全局首次出现顺序、face index 序列和 raw OBJ 文本不变。
  `ctest --test-dir build\tests --output-on-failure --timeout 120` 通过；
  `run_20260522_083036` 对 34 个 small 样本做 `-VerifyWithOracle` 全部通过。
  `run_20260522_082933` 的 100 组 NoTracy 与 50 阈值保留基线
  `run_20260522_082153` 结构计数完全一致，100 个 raw OBJ SHA256 完全一致；
  平均 `export_ms` 从约 52.6ms 降到 31.542ms，`end_to_end_ms` 从约
  227.9ms 降到 206.473ms。

## 已测但不保留的局部实验

- raw OBJ 导出按文本块直接写 `ofstream`：尝试保留现有并行顶点/面文本块生成，
  但不再把所有块拼接成一个完整 `objText` 后单次写出，而是按 header、顶点块、
  面块顺序直接写文件，目标是减少最终大缓冲复制和峰值内存。`ctest --test-dir
  build\tests --output-on-failure --timeout 120` 通过，`run_20260522_074347`
  对 34 个 small 样本做 `-VerifyWithOracle` 全部通过；`run_20260522_074534`
  与保留基线 `run_20260522_074030` 的 100 个 raw OBJ SHA256 完全一致，但平均
  `export_ms` 仅从 69.227ms 到 69.220ms，属于噪声级，源码不保留。
- 合并输入准备扫描后重新试探 leaf threshold 90/100：更高阈值继续减少递归节点和
  输出片段，但 leaf 局部 BSP/分类规模变大，`solve_ms` 上升抵消了 export 下降。
  保留基线 75 的 `run_20260522_074030` 平均 `solve_ms/export_ms/end_to_end_ms`
  为 `115.327ms / 69.227ms / 247.759ms`；90 的 `run_20260522_074750`
  为 `118.033ms / 66.976ms / 247.945ms`；100 的 `run_20260522_074832`
  为 `119.843ms / 65.758ms / 248.326ms`。该阶段默认阈值仍保持 75。
- sibling task 提交门槛从半个 leaf threshold 降到四分之一个 leaf threshold：
  目标是给 oneTBB 暴露更多中等规模子树任务。`ctest --test-dir build\tests
  --output-on-failure --timeout 120` 通过；100 组 NoTracy `run_20260522_075032`
  相比保留基线 `run_20260522_074030` 平均 `parallel_sibling_spawn_count`
  从 159.69 增至 168.35，`solve_ms` 仅从 115.327ms 到 115.139ms，
  但 `end_to_end_ms` 从 247.759ms 退化到 247.851ms，属于噪声级且进程耗时
  也退化。当前仍保留半阈值门槛。
- raw 顶点 key 的整数 hash 改为直接扫描 `UnsignedInteger` limb：尝试避免
  `hashIntegerForKey()` 内部的 signed `_BitInt` mask 和临时 chunk。`ctest --test-dir
  build\tests --output-on-failure --timeout 120` 通过，raw OBJ SHA256 与 flat hash
  保留基线一致；但两轮 100 组 NoTracy `run_20260522_075950` /
  `run_20260522_080045` 的平均 `export_ms` 为 66.810ms / 66.833ms，
  均慢于保留基线 `run_20260522_075742` 的 66.575ms。源码不保留。
- raw flat hash 桶数从 vertex slot 的两倍降到一倍：尝试减少桶数组写入和缓存压力。
  `ctest --test-dir build\tests --output-on-failure --timeout 120` 通过，
  `run_20260522_080454` 与保留基线 `run_20260522_075742` 的结构计数和
  100 个 raw OBJ SHA256 完全一致；但平均 `export_ms` 从 66.575ms 退到
  66.701ms。当前仍保留两倍 slot 的低负载桶表。
- raw OBJ face index 预生成字符串 token：100 组输出平均每个唯一顶点索引在 face
  中出现约 4.5 次，因此尝试为 `1..maxIndex` 预生成带前导空格的索引 token，
  再由 face 写出阶段直接拼接。`ctest --test-dir build\tests --output-on-failure
  --timeout 120` 通过，`run_20260522_080726` 与保留基线 `run_20260522_075742`
  的结构计数和 100 个 raw OBJ SHA256 完全一致；但平均 `export_ms` 从
  66.575ms 退到 67.421ms。额外 token 构造和字符串拼接成本超过重复
  `to_chars` 节省，源码不保留。
- Release 叶片分类候选不再做 path signature 去重：平均 duplicate skip 只有
  10.61，因此尝试省掉每个候选的齐次端点 signature 构造和历史路径比较。`ctest
  --test-dir build\tests --output-on-failure --timeout 120` 通过，
  `run_20260522_081027` 与保留基线 `run_20260522_075742` 的结构计数和
  100 个 raw OBJ SHA256 完全一致；但 `leaf_classification_trace_attempt_count`
  从 13882.31 增到 13892.92，平均 `end_to_end_ms` 从 245.732ms 退到
  245.962ms，进程耗时也退化。源码不保留。
- CLI 默认路径关闭 `BoolProblem::leafSummaries()` 收集：为 `BoolProblem` 增加
  `setCollectLeafSummaries(false)` 并把该开关传入 `SubdivisionSolver`，尝试避免每个
  叶子摘要在子树合并时向上搬运；公开默认仍收集 leaf summaries。`ctest --test-dir build\tests --output-on-failure --timeout 120`
  通过，`run_20260522_071428` 与保留基线 `run_20260522_070657` 的结构计数和
  100 个 raw OBJ SHA256 完全一致，但平均 `solve_ms` 从 109.153ms 退化到
  109.941ms，`end_to_end_ms` 从 258.271ms 退化到 259.184ms。leaf summary 聚合
  不是当前默认 CLI 热点，源码不保留。
- 叶片分类 WNV surface-point trace 增加叶片局部 polygon AABB BVH：尝试在
  `LeafClassificationContext` 中为当前 `polygons_` 构造保序 broad-phase 索引，
  让 `tracePathWNVToSurfacePointTrustedWithStartSides()` 先按 path AABB 查询可能
  相交的 polygon 原始下标，再按原始顺序进入原有精判循环。第一版对 `polygonCount >= 16`
  建索引，第二版改为 `polygonCount >= 128` 懒构建且查询结果超过半数时回落全表扫描。
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 均通过；
  两轮 100 组论文样本 `run_20260522_060650` / `run_20260522_060919` 的 100 个
  raw OBJ SHA256 互相完全一致，但平均 `solve_ms` 分别为 120.454ms / 120.082ms，
  明显差于当前保留优化 `run_20260522_053232` / `run_20260522_053346` 的两轮均值
  116.541ms。说明当前 leaf trace 的多边形集合规模和 path-AABB 选择性不足以抵消
  BVH 构建、查询和结果排序成本，源码不保留；下一步应转向减少 leaf trace 次数、
  懒物化 leaf polygon 或共享更高层结构，而不是给每个 leaf 单独建索引。
- 叶片分类候选路径签名省略起点：尝试让 `makePathSignature()` 只记录每段终点，
  不再记录首段起点。进入 trace 前候选路径已经由 debug 前置条件约束为从当前
  local reference 出发，同一 fragment 内起点为常量，因此该改动不改变 path
  duplicate 判定。Debug 构建和 `ctest --preset default --output-on-failure --timeout 120`
  通过；100 组论文样本 `run_20260522_055309` 与当前保留基线
  `run_20260522_045123` 的结构计数、candidate/trace 诊断计数和 100 个 raw OBJ
  SHA256 完全一致，但平均 `solve_ms` 为 117.238ms，差于当前保留优化
  `run_20260522_053232` / `run_20260522_053346` 的两轮均值 116.541ms。
  说明少复制一个齐次端点没有换回稳定收益，源码不保留。
- 叶片分类中间端点预检循环反转：尝试把
  `hasIntermediateEndpointOnInputSurface()` 从 polygon 外层 / path 中间端点内层改成
  中间端点外层 / polygon 内层，希望减少每个 polygon 上重复读取 path 端点和分支。
  该改动只调整“是否存在中间端点落在输入面上”的遍历顺序，不改变候选拒绝语义。
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 通过；
  100 组论文样本 `run_20260522_054900` 与当前保留基线 `run_20260522_045123`
  的结构计数、候选/trace 诊断计数和 100 个 raw OBJ SHA256 完全一致，但平均
  `solve_ms` 为 117.174ms，差于当前保留优化 `run_20260522_053232` /
  `run_20260522_053346` 的两轮均值 116.541ms。说明原有 polygon 外层顺序更适合
  当前缓存访问局部性，源码不保留。
- 局部 BSP 递归插入端点约束平面零侧短路：尝试在
  `BSPTree::addSegmentRecursive()` 计算端点相对当前 `node.splitPlane` 的侧别前，
  若 carrier 端点约束平面 `v0` / `v1` 与 `node.splitPlane` 完全相同则直接返回
  0，避免一次三平面交点和 4D dot 分类。该改动是代数恒等短路，不改变递归走向；
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 通过；
  100 组论文样本 `run_20260522_054428` 与当前保留基线 `run_20260522_045123`
  的结构计数、trace 诊断计数和 100 个 raw OBJ SHA256 完全一致，但平均
  `solve_ms` 退化到 122.712ms，明显差于当前保留优化
  `run_20260522_053232` / `run_20260522_053346` 的两轮均值 116.541ms。
  说明端点平面等价命中率不足以覆盖每次递归多出的两次平面等价比较，源码不保留。
- 叶片编排 adjacency 计数前移和 fragments 预留：尝试在
  `buildLeafArrangement()` 枚举 pair relation 并写入 adjacency 时同步累计
  `adjacencyOffsets`，删除排序后的第二次 adjacency 扫描；同时对输出
  `fragments` 做 `polygonCount` 级别的最低预留。该改动不改变 pair 枚举、
  排序键、每个 base polygon 的 BSP 插入顺序或 raw OBJ 输出语义。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；100 组论文样本
  `run_20260522_054036` 与当前保留基线 `run_20260522_045123` 的结构计数、
  trace 诊断计数和 100 个 raw OBJ SHA256 完全一致，但平均 `solve_ms`
  为 116.550ms，未优于当前保留优化 `run_20260522_053232` /
  `run_20260522_053346` 的两轮均值 116.541ms，`export_ms` 和端到端也没有改善。
  说明这次少一次 adjacency 线性计数和初始 reserve 的收益落在噪声内，源码不保留。
- raw trusted OBJ 导出合并有限性检查：尝试删除
  `recoverRawTrustedPolygonSoupData()` 开头的并行 fragment 顶点缓存/有限性检查，
  改为在后续顺序全局齐次顶点去重循环里同时检查 `hasUniqueIntersection()` 和
  `w != 0`，以减少 raw 导出前的第二遍顶点扫描。该改动不改变 canonical
  顶点 key、face 顺序、OBJ 文本生成或拓扑语义。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；100 组论文样本
  `run_20260522_052849` 与当前保留基线 `run_20260522_045123` 的结构计数、
  exported face 数和 100 个 raw OBJ SHA256 完全一致，但平均 `export_ms`
  从两轮保留基线约 83.8ms 退化到 92.163ms，`end_to_end_ms` 退到
  340.561ms。说明前置并行扫描不只是检查，它还并行触发顶点缓存构建；合并到
  顺序去重循环会把这部分成本串行化，源码不保留。
- split route 顶点侧别扫描使用栈上小缓冲：尝试让
  `classifySplitChildPolygonRoute()` 先把常见小边数 polygon 的 `vertexSides`
  写入栈上数组，只有确认 route 真正需要 split 时才填充 `route.vertexSides`，
  避免纯左/纯右扫描路径也提前 `vector::resize()`。该改动不改变 child 路由、
  裁剪侧别或 polygon materialization 语义。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；100 组论文样本
  `run_20260522_052435` 与当前保留基线 `run_20260522_045123` 的结构计数、
  trace 诊断计数和 100 个 raw OBJ SHA256 完全一致，但平均 `solve_ms`
  为 117.659ms，慢于两轮保留基线 `run_20260522_045009` /
  `run_20260522_045123` 的 116.842ms。说明新增分支和 split 时的
  `assign()` 没有换回足够的非 split route 分配节省，源码不保留。
- WNV 普通 trace 先做 segment/polygon 相关性筛选：尝试让
  `tracePathWNVImpl()` 像 surface-point trace 一样，先用段 AABB 与 polygon AABB /
  支撑平面相关性跳过无关段，再按需分类起点和终点，避免不会参与交叉的段触发
  `poly.classify()`。该改动不改变 WNV 累加、boundary policy、leaf trace 次数或
  子参考点 trace 次数。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；100 组论文样本
  `run_20260522_052041` 与当前保留基线 `run_20260522_045123` 的结构计数、
  trace 诊断计数和 100 个 raw OBJ SHA256 完全一致，但平均 `solve_ms`
  从两轮保留基线 `run_20260522_045009` / `run_20260522_045123` 的
  116.842ms 退化到 120.372ms，`end_to_end_ms` 也退到 338.464ms。
  说明普通 trace 中提前相关性分支和重新分类起点的成本高于跳过无关端点分类的收益，
  源码不保留。
- WNV surface-point trace 复用末段终点面内分类：尝试在最后一段
  `classifyEndPoint` 时顺带返回 `StrictInterior/Boundary`，当已确认
  `surfaceHit == &endPoint` 时跳过后续 `classifyPolygonSurfacePointUnchecked()`
  的第二次边半空间扫描。该改动不改变 trace 次数、boundary policy 或 WNV/WNTV
  累加语义。Debug 构建和 `ctest --preset default --output-on-failure --timeout 120`
  通过；两轮 100 组论文样本 `run_20260522_051128` / `run_20260522_051243`
  与当前保留基线 `run_20260522_045123` 的结构计数、trace 诊断计数和 100 个
  raw OBJ SHA256 完全一致。第一轮 `solve_ms` 为 116.655ms，但第二轮退到
  117.071ms，两轮均值 116.863ms，略差于末段曲面命中终点复用后两轮保留基线
  `run_20260522_045009` / `run_20260522_045123` 的 116.842ms。
  说明额外分类状态维护没有稳定覆盖重复面内扫描成本，源码不保留。
- WNTV-aware split 候选按 polygon 放大成本择优：尝试在仍只选择可分离 WNTV 类的
  候选切面前提下，复用 `SplitCostEstimate` 先最小化 `splitCount`、child 最大
  polygon 数和不平衡度，再用原先的分离距离打平。该方向希望减少后续 leaf BSP
  和 trace 工作量，但会给每次 WNTV-aware split 增加候选成本估计扫描。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 通过；100 组论文样本
  `run_20260522_050620` 相对当前保留基线 `run_20260522_045123` 改变全部
  workload 的递归结构，`leaf_fragment_count` 从 3600346 降到 3583793，
  `leaf_classification_trace_attempt_count` 从 1328146 降到 1322574，但
  `node_count` 从 148536 增到 150160，`total_polygon_count` 从 32701981
  增到 33056340；平均 `solve_ms` 为 117.566ms，慢于末段曲面命中终点复用后
  两轮保留基线 `run_20260522_045009` / `run_20260522_045123` 的 116.842ms。
  说明候选估计扫描和 polygon 放大超过了 trace/fragment 下降收益，源码不保留。
- WNV surface-point trace 的末段端点边界命中类型复用：尝试在
  `surfaceHit == &endPoint` 时跳过
  `classifyKnownSegmentPolygonBoundaryPointHitUnchecked()` 内部的起终点平面点相等检查，
  只保留 `collectBoundaryEdgesAtPointUnchecked()` 收集边界边，并直接标记为
  `EndpointOnBoundary`。该实验建立在“末段曲面命中终点复用”已确认命中点就是
  `endPoint` 的前提上，不改变 boundary policy 的后续判断。Debug 构建和
  `ctest --preset default --output-on-failure --timeout 120` 已在实验分支通过；
  `run_20260522_050106` 与当前保留基线 `run_20260522_045123` 的结构计数、
  trace 诊断计数和 100 个 raw OBJ SHA256 完全一致；但单轮 `solve_ms`
  为 117.077ms，慢于末段曲面命中终点复用后两轮保留基线
  `run_20260522_045009` / `run_20260522_045123` 的 116.842ms 均值。
  说明分支和 contact 状态手动维护没有覆盖掉两次端点相等检查成本，源码不保留。
- WNV surface hit 面内分类与边界来源合并扫描：尝试把
  `classifyPolygonSurfacePointUnchecked()` 和随后边界命中时的
  `classifyKnownSegmentPolygonBoundaryPointHitUnchecked()` 合并成一次边半空间扫描，
  在确认 Boundary 的同时收集 `side==0` 的边来源，减少边界命中路径的重复边分类。
  Debug 构建和 `ctest --preset default --output-on-failure --timeout 120` 通过；
  `run_20260522_045540` 与当前保留基线 `run_20260522_045123` 的结构计数、trace
  诊断计数和 100 个 raw OBJ SHA256 完全一致；但单轮 `solve_ms` 为 117.519ms，
  明显差于末段曲面命中终点复用后两轮保留基线
  `run_20260522_045009` / `run_20260522_045123` 的 116.842ms 均值。
  说明额外返回对象和边界 contact 状态维护超过了少扫一次边的收益，源码不保留。
- 叶片 centroid axis-probe 起点缓存：尝试在 `LeafClassificationContext` 中缓存
  local reference 的整数坐标、整数坐标平面和对应 `PlanePoint3i`，让已知
  axis-probe 目标枚举跳过每个 fragment 重复 `tryExtractExactIntegerPoint()` 和
  `makeIntegerCoordinatePlanes()`。该改动不改变候选目标、路径线段、trace 次数或
  WNV 传播数学。Debug 构建、`ctest --preset default --output-on-failure --timeout 120`
  通过；`run_20260522_044011` 与当前保留基线 `run_20260522_042538` 的结构计数和
  trace 次数完全一致，100 个 raw OBJ 与基线逐文件 SHA256 完全一致；但单轮
  `solve_ms` 为 117.102ms，略差于当前保留基线
  `run_20260522_042422` / `run_20260522_042538` 的 117.035ms 均值。
  说明这部分重复起点解析不是当前主导成本，源码不保留。
- 单向 `computePolygonIntersectionCarrierTrusted()` 去重 AABB：尝试把该 trusted
  入口的 `target.aabb()` / `incoming.aabb()` 重叠检查前移到公开 wrapper，并依赖
  `BSPTree::insertTrusted()` 入口已有的 base/insert polygon AABB 重叠判断，减少叶片
  局部 BSP 插入路径上的重复 broad-phase。Debug 构建和 ctest 已在实验分支通过；
  `run_20260522_042925` 与当前保留基线 `run_20260522_042538` 的结构计数和 trace
  次数完全一致，100 个 raw OBJ 与基线逐文件 SHA256 完全一致；但单轮
  `solve_ms` 为 117.164ms，差于双向交线去重后的保留基线
  `run_20260522_042422` / `run_20260522_042538` 的 117.035ms 均值。
  说明这个单向入口的 AABB 检查不是当前主导成本，前移检查还会让公开 wrapper 多承担
  一次 broad-phase；源码不保留。
- WNV trace 单段路径 AABB 预筛内联缓存：尝试在 `PathAABBPrecheck` 中为
  `path.size()==1` 的候选路径单独保存 `singleSegmentBox`，避免初始化和填充
  `small_vector<AABB3i, 4>`；多段路径仍走原小缓冲。该改动不改变 path AABB、
  segment AABB、polygon AABB 剪枝或 WNV 传播数学。Debug 构建和 ctest 通过；
  `run_20260522_042002` / `run_20260522_042117` 与当前保留基线
  `run_20260522_040952` 的结构计数和 trace 次数完全一致，100 个 raw OBJ 与基线
  逐文件 SHA256 完全一致；但两轮候选均值 `solve_ms` 为 117.550ms，差于当前保留
  基线 `run_20260522_040610` / `run_20260522_040952` 的 117.172ms 均值。
  说明这层 small-vector 维护不是当前主导成本，新增分支反而拖慢；不保留。
- 通用 exact-integer axis path 直构：在非 axis-probe 的整数参考点到整数目标点
  1-3 段路径中，尝试复用已知起止整数坐标和 `PlanePoint3i`，绕过
  `buildAxisAlignedCoordinatePath()` 内每段通用 coordinate-plane 端点恢复与互侧分类。
  该改动接入 axis path 单点/多点枚举和 repair path，候选数量、axis path 尝试数、
  trace 次数和输出均不变。Debug 构建和 ctest 通过；`run_20260522_041527`
  与当前保留基线 `run_20260522_040952` 的结构计数完全一致，100 个 raw OBJ 与基线
  逐文件 SHA256 完全一致；但单轮 `solve_ms` 为 117.882ms，差于当前保留基线
  `run_20260522_040610` / `run_20260522_040952` 的 117.172ms 均值。
  这说明该通用整数路径不是当前主导成本，新增分支和端点选择没有稳定收益；不保留。
- `computePolygonPlaneIntersection()` 顶点侧别直接分类：尝试给 `Polygon256`
  增加按顶点下标分类到目标平面的入口；已有 vertex cache 时复用缓存，否则直接用
  未约分三平面交点做 4D dot，避免 `computePolygonPlaneIntersection()` 为
  classify-only 扫描强制构造整条 `source.vertices()` 缓存。Debug 构建和 ctest
  通过；`run_20260522_040102` 与当前保留基线 `run_20260522_033921` 的结构计数、
  child-reference/leaf trace 计数完全一致，100 个 raw OBJ 与基线逐文件 SHA256
  完全一致；但单轮 `solve_ms` 为 119.321ms，明显差于当前保留基线
  `run_20260522_033808` / `run_20260522_033921` 的 117.948ms 均值。
  该改动可能破坏了后续 vertex cache 复用，或把同一顶点交点计算从一次缓存构造
  变成更多局部重复计算；不保留。
- 子参考点 fast AABB 候选短容器内联缓冲：在
  `visitFastAABBPathCandidateSeeds()` 中把候选坐标去重表和三轴 inset 选择表从
  短 `std::vector` 改为 `boost::container::small_vector`，尝试减少每个需要传播
  子参考点的内部节点在 fast candidate 枚举阶段的堆分配。Debug 构建和 ctest 通过；
  `run_20260522_035608` 与当前保留基线 `run_20260522_033921` 的结构计数、
  child-reference candidate/trace 计数和 leaf trace 计数完全一致，100 个 raw OBJ
  与基线逐文件 SHA256 完全一致；但单轮 `solve_ms` 为 118.090ms，差于当前保留
  基线 `run_20260522_033808` / `run_20260522_033921` 的 117.948ms 均值，
  `export_ms` 与端到端也未改善。该短容器替换不保留。
- WNV 普通 trace 的 segment/polygon trusted 交点旁路：在
  `tracePathWNVImpl()` 中，尝试用已完成的 segment AABB 与 polygon AABB/支撑平面
  相关性预筛作为前置条件，绕过 `intersectionSegmentPolygon()` 内部重复 AABB 构造
  和 plane-box 判断，并直接用未约分 `intersect_3_planes` 交点做面内/线段内判定。
  Debug 构建和 ctest 通过；`run_20260522_034616` 与当前保留基线
  `run_20260522_033921` 的结构计数和 trace 计数完全一致，100 个 raw OBJ 与基线
  逐文件 SHA256 完全一致；但单轮 `solve_ms` 为 118.156ms，差于当前保留基线
  `run_20260522_033808` / `run_20260522_033921` 的 117.948ms 均值。
  该路径节省的重复 AABB/规范化工作没有抵消额外局部 helper 和分类路径变化成本；
  不保留。
- 局部 BSP 叶片裁剪前 AABB-平面剪枝：尝试在
  `BSPTree::addSegmentRecursive()` 的 leaf 分支进入
  `clipLeafGeometryByPlaneTrusted()` 前先读取 `node.leafGeometry.aabb()`，用
  `doesPlaneIntersectAABB(insertPlane, leafBox)` 保守跳过不可能被当前 split plane
  切开的叶片。Debug 构建和 ctest 通过；`run_20260522_033530` 与当前保留基线
  `run_20260522_030809` 的结构计数完全一致，100 个 raw OBJ 与基线逐文件
  SHA256 完全一致；但单轮 `solve_ms` 为 119.128ms，差于 WNV polygon AABB 复用
  基线 `run_20260522_030701` / `run_20260522_030809` 的 118.311ms 均值。
  额外 leaf AABB 构造和 plane-box 测试没有换回足够的 clipping 节省；不保留。
- 叶片分类候选路径签名内联缓冲：在保留候选 path signature 去重语义的前提下，
  把 `uniquePathSignatures` 的单条签名从 `std::vector<HomPoint4i>` 改成
  `boost::container::small_vector<HomPoint4i, 4>`，让常见的 1-3 段候选路径签名
  不单独申请堆缓冲。Debug 构建和 ctest 通过；
  `run_20260522_032957` / `run_20260522_033113` 与当前保留基线
  `run_20260522_030809` 的结构计数、trace 尝试数和 duplicate skip 数完全一致，
  100 个 raw OBJ 与基线逐文件 SHA256 完全一致；但两轮均值 `solve_ms`
  为 118.319ms，略差于 WNV polygon AABB 复用基线
  `run_20260522_030701` / `run_20260522_030809` 的 118.311ms。端到端均值下降
  更像导出和调度噪声，solver 热路径没有稳定收益；不保留。
- 叶片分类候选路径去重旁路：当前 100 组论文样本中
  `leaf_classification_candidate_generated_count` 为 1411882，而
  `leaf_classification_candidate_duplicate_skip_count` 只有 923。尝试让
  `registerUniqueLeafClassificationCandidatePath()` 只检查空路径，不再为每条候选构造
  `HomPoint4i` 路径签名并线性查重，允许重复候选直接进入 trace。Debug 构建和 ctest
  通过；`run_20260522_032346` / `run_20260522_032456` 与当前保留基线
  `run_20260522_030809` 的结构计数完全一致，100 个 raw OBJ 与基线逐文件
  SHA256 完全一致；但重复 trace 从 1328146 增至 1329069 后，两轮均值
  `solve_ms` 为 118.338ms，差于 WNV polygon AABB 复用基线
  `run_20260522_030701` / `run_20260522_030809` 的 118.311ms，端到端均值也从
  331.715ms 退化到 332.183ms。少做签名查重不足以抵消额外 trace 和噪声；不保留。
- 交线载体构造复用已读 polygon AABB：尝试把
  `computePolygonPlaneIntersection()` 拆出接收 `AABB3i` 的内部入口，让
  `computePolygonIntersectionCarrierTrusted()` 和
  `computeBidirectionalPolygonIntersectionCarriersTrusted()` 在 pair AABB 预筛后复用
  `targetBox/incomingBox` 或 `lhsBox/rhsBox`，避免交线端点载体构造内重复读取
  `poly.aabb()`。Debug 构建和 ctest 通过；`run_20260522_031533` 与当前保留基线
  `run_20260522_030809` 的结构计数完全一致，100 个 raw OBJ 与基线逐文件
  SHA256 完全一致；但相对 WNV polygon AABB 复用后的两轮保留基线
  `run_20260522_030701` / `run_20260522_030809` 均值，`solve_ms` 从
  118.311ms 退化到 118.704ms，`export_ms` 从 83.857ms 退化到 84.124ms。
  额外入口和引用传递没有换回足够的 AABB 访问节省；不保留。
- 局部 BSP 递归插入端点齐次点缓存：尝试把
  `intersectHomogeneousUnnormalized(basePlane, endpointPlane, insertPlane)`
  的结果沿 `BSPTree::addSegmentRecursive()` 递归传递，减少内部节点重复构造端点
  齐次交点。直接 eager 版本 `run_20260522_025715` 相对 raw 导出 fast path
  保留基线 `run_20260522_023707` / `run_20260522_023820`，`solve_ms` 从
  118.826ms 退化到 118.998ms；改成只在遇到内部 BSP 节点时懒构造缺失端点后，
  `run_20260522_025937` / `run_20260522_030050` 的结构计数与
  `run_20260522_023820` 完全一致，100 个 raw OBJ 与基线逐文件 SHA256 完全一致，
  两轮 `solve_ms` 均值为 118.724ms，仅比基线低 0.103ms，但 workload paired
  统计为 54 个变快、46 个变慢，端到端均值从 332.007ms 退化到 332.782ms。
  收益落在噪声内且增加递归接口状态复杂度；不保留。
- raw OBJ 导出 exact raw 齐次点缓存：在 raw trusted 导出路径里增加
  raw `HomPoint4i` 完全相等到 OBJ 顶点下标的旁路缓存，raw miss 时仍走 primitive
  canonical map，尝试减少重复 `primitiveHomPoint()`。Debug 构建和 ctest 通过，
  `run_20260522_025056` 的结构计数、result/exported face 数与
  `run_20260522_023820` 完全一致，100 个 raw OBJ 与基线逐文件 SHA256 完全一致；
  但相对 raw 导出 fast path 保留基线 `run_20260522_023707` /
  `run_20260522_023820`，单轮 `export_ms` 从 84.127ms 退化到 85.660ms，
  `end_to_end_ms` 从 332.007ms 到 332.720ms。额外 hash map 查询和内存压力超过
  raw 精确重复命中节省；不保留。
- 删除 `ClassifiedFragment` 中间数组，改为在单片叶片分类成功后直接用当前
  front/back WNV 追加 `resultFragments_`，尝试避免每个成功分类片段额外复制一次
  `Polygon256`。结构计数与 `run_20260521_233848` 完全一致，但
  `run_20260521_234558` 的 100 组论文样本单轮 NoTracy 计时整体变慢：
  平均 `solve_ms` 从 120.335ms 到 120.715ms，`end_to_end_ms` 从 344.953ms
  到 346.368ms。small/medium 分别有小幅收益，但 large 33 组中 21 组 solve
  退化，large 平均 `solve_ms` 从 246.313ms 到 249.679ms；不保留。
- `BSPTree` 尝试把完整 `basePolygon` 副本改成 `basePlane/baseAABB` 加
  `insertTrusted()` 懒 materialize，减少大叶片局部 BSP 构造时的缓存/vector
  复制。Debug 构建和 ctest 通过，结构计数与 `run_20260521_233848` 完全一致；
  但 100 组论文样本重复 NoTracy 结果不稳定：`run_20260521_235526`
  平均 `solve_ms` 小幅下降 0.5%，`run_20260521_235640` 又退化 0.4%，端到端
  同步偏负；收益落在噪声内且增加了 `insertTrusted()` 的状态分支，不保留。
- `buildRoundedCentroidPoint()` 尝试把欧氏顶点 `vector` 改成流式三角扇重心计算，
  避免每个 centroid axis 尝试临时分配并保存全部 `Vec3d`。Debug 构建和 ctest
  通过，100 组论文样本结构计数、centroid 点计数、axis path 尝试数均与
  `run_20260521_233848` 一致；但 `run_20260522_000027` 平均 `solve_ms`
  从 120.335ms 退化到 120.837ms，large 平均从 246.313ms 到 247.601ms；
  不保留。
- OBJ polygon soup 构建尝试让正常共面 face 直接按索引访问 `quantizedVertices`，
  避免每个 face 先拷贝到临时 `faceVertices` vector；三角化 fallback 改用 3
  个索引的小数组。Debug 构建和 ctest 通过，100 组论文样本结构计数与
  `run_20260522_000710` 完全一致；但 `run_20260522_001255` 平均
  `prepare_ms` 基本持平（112.997ms 到 113.084ms），`end_to_end_ms`
  从 340.942ms 到 342.630ms。导入阶段主要瓶颈不在该 face 顶点拷贝上，不保留。
- `orientPolygonEdgesOutward()` 尝试把临时顶点数组从每次 `std::vector`
  分配改成栈上小缓冲，以减少高频 `Polygon256` 构造的局部分配。直接
  `std::array<PlanePoint3i, 16>` 会默认构造大量大整数对象，`run_20260522_001621`
  和 `run_20260522_001723` 没有稳定收益；改成 `std::optional<PlanePoint3i>`
  延迟构造后，`run_20260522_001907` 一轮显示 `prepare_ms` 下降 1.06%，
  但 `run_20260522_002014` 又回归到 `prepare_ms` 上升 0.43%、端到端上升
  0.48%。结构计数均与 `run_20260522_000710` 一致，收益落在噪声内；不保留。
- center-range fallback 全量扫描每轴 polygon AABB 端点作为候选切面：该方向试图把
  固定平均中心切面改成更接近“最小局部复制代价”的轴平面选择。第一版按
  `splitCount` 优先会选择几乎不推进递归的一侧空切，`re-EMBER_paper_small_batch`
  栈溢出；改为优先降低最大 child polygon 数并过滤无进展切面后，Debug/CTest 通过，
  但 `run_20260521_145526` 单轮聚合 `solve_ms` 从 `run_20260521_144122`
  三次均值 1599.87ms 退化到 2343.96ms。加上 `polygonCount >= 96`
  的大节点门槛后，`run_20260521_145709` 仍退化到 2325.65ms。结构指标虽有下降，
  但候选扫描成本远大于省下的 leaf BSP/trace 工作量；不保留。
- center-range fallback 只有一个可用轴候选时跳过 `estimateSplitCostsFromPolygons()`：
  该分支不改变切分面，尝试避免无竞争候选时重复扫描 polygon AABB。Debug 构建和
  ctest 通过，三轮 100 组论文样本 `run_20260522_024243` /
  `run_20260522_024357` / `run_20260522_024516` 与 `run_20260522_023820`
  的结构计数、center-range split 次数、result 计数和 trace 次数完全一致；但相对
  raw 导出 fast path 保留基线 `run_20260522_023707` / `run_20260522_023820`，
  三轮均值 `solve_ms` 基本持平略退 0.03%，`end_to_end_ms` 退 0.19%。
  单候选场景不够主导，新增分支没有稳定收益；不保留。
- `appendPointToAABB()` 尝试从 `PlanePoint3i` 的三张定义平面里识别单位坐标平面，
  对已知整数坐标跳过对应轴的 `floorCeilDiv()`。该改动不改变 AABB 语义，Debug/CTest
  通过，但 `run_20260521_150724` 三次无 oracle 重复计时为 1610.62ms、
  1616.98ms、1604.68ms，均值 1610.76ms，差于 `run_20260521_144122`
  的 1599.87ms。额外平面识别分支没有换回足够的 256 位除法节省；不保留。
- `clipLeafGeometryByPlaneTrusted()` 尝试绕过 `source.vertex(i)`，直接用
  `intersectHomogeneousUnnormalized()` 加 4D dot 分类裁剪源 polygon 顶点，目标是让
  叶片 BSP clipping 更贴近 BSP 论文的 `classify_vertex` 热路径。Debug 构建和 ctest
  通过，100 组论文样本 `run_20260522_005043` 的结构计数与
  `run_20260522_004433` 完全一致，但平均 `solve_ms` 从 119.261ms 退化到
  120.397ms，`end_to_end_ms` 从 339.122ms 到 341.185ms。当前缓存顶点仍会被
  后续有效性、AABB 或分类路径复用，单独绕过缓存没有收益；不保留。
- leaf 停止条件尝试从固定 `polygonCount <= 25` 扩展为“低于阈值但 AABB 重叠对很密时
  继续细分”，以减少局部 BSP pair 和分类 trace。Debug 构建通过，paper small batch
  通过，但 `re-EMBER_tests` 的 IO 断言 `problem.resultFragments().size() == 12u`
  失败；该策略会改变当前 API/测试可见的结果片段分块数量。即使几何可能等价，也不能
  在当前输出契约下直接保留，后续若要做自适应停止，必须先定义并验证片段稳定性边界。
- 路径候选中的 `makePointFromPlanes()` 和 axis segment 临时端点改用
  `intersectHomogeneousUnnormalized()`，尝试把 plane replacement / axis path
  的中间点也推到 BSP 论文的未约分 `intersect_3_planes` 边界；Debug 测试和
  `run_20260521_131131` 的 22 个 verifier 均通过，但
  `run_20260521_131614` 三次无 oracle 重复计时为 1678.75ms、1681.76ms、
  1704.47ms，均值 1688.33ms，差于 `run_20260521_125353` 的 1676.89ms。
  该路径上的中间点会反复参与 AABB 和半空间分类，未约分齐次代表元放大了后续
  256 位乘法成本；暂时保留 normalized 路径点。
- sibling 子树并行提交门槛继续降到 `leafPolygonThreshold / 4`（下限 4），
  Debug 测试通过，但 `run_20260521_134738` 三次无 oracle 重复计时为
  1644.60ms、1655.29ms、1653.80ms，均值 1651.23ms，差于半阈值版本
  `run_20260521_134044` 的 1647.70ms；更细任务已开始被调度开销抵消，
  保留 `leafPolygonThreshold / 2`（下限 8）。
- split 路由顶点侧别扫描改用栈缓冲，只在确实需要裁剪时再 materialize 到
  `SplitPolygonRoute::vertexSides`，尝试减少未切分 polygon 的临时 `vector`
  分配；Debug 测试和 `run_20260521_135340` 的 22 个 verifier 均通过，但
  `run_20260521_135821` 三次无 oracle 重复计时为 1664.73ms、1632.99ms、
  1667.70ms，均值 1655.14ms，差于半阈值保留基线 `run_20260521_134044`
  的 1647.70ms。额外分支和 split 时的 `assign()` 没有换回足够的分配成本，
  不保留该改动。
- 叶片分类前按 `sourceFragmentCount` 对 `classifiedFragments_` 和
  `resultFragments_` 预留容量，尝试减少每片 leaf 内结果收集的 vector 增长；Debug
  测试通过，但 `run_20260521_140932` 三次无 oracle 重复计时为 1661.97ms、
  1660.56ms、1657.33ms，均值 1659.95ms，差于当前保留版本
  `run_20260521_140622` 的 1640.82ms。该上界预留会放大内存占用和缓存压力，
  不保留。
- 叶片分类候选路径 view：结构和 trace 计数不变，但 NoTracy solve 变慢。
- `buildLeafArrangement()` 结果 vector 预留容量：结构计数不变，solve 信号为负。
- `buildLeafArrangement()` 删除 `polygonCount < 8` 小叶片旧路径，并改用统一
  pair-relation adjacency：结构计数不变，但 NoTracy solve 信号为负；进一步改成
  per-base 分桶、去掉 sort/offset 后仍为负。当前小叶片 `insertTrusted()` 路径虽然
  代码重复，但在现有 workload 上有性能意义，后续不应只为了统一控制流删除它。
- 中间端点预筛缓存：尝试在单片 leaf fragment 分类过程中按 `HomPoint4i` 缓存
  `hasIntermediateEndpointOnInputSurface()` 的 surface-membership 结果，避免多个候选路径
  共享中间端点时重复扫描所有输入 polygon。Debug 构建和 ctest 通过，100 组论文样本
  结构计数、candidate 计数和 `leaf_classification_trace_attempt_count` 均与
  `run_20260522_000710` 完全一致；但 `run_20260522_002535` 平均 `solve_ms`
  从 120.266ms 退化到 121.071ms，`end_to_end_ms` 从 340.942ms 到 342.737ms。
  缓存线性查找、写入和额外状态维护成本超过重复 surface 预检节省；不保留。
- 叶片分类 inset 候选点 vector 搬运优化：尝试把
  `visitLeafClassificationInsetPointCandidates()` 返回的候选点移动给 plane replacement，
  并把 bridge rescue 目标延后到 plane replacement 失败后再保存，减少 `PlanePoint3i`
  大对象复制。Debug 构建和 ctest 通过，100 组论文样本 `run_20260522_010137`
  的结构、candidate 和 trace 计数与 `run_20260522_005732` 完全一致；但相对
  `run_20260522_005611` / `run_20260522_005732` 两轮保留基线均值，`solve_ms`
  基本持平略退，`end_to_end_ms` 略退。候选点数量较小，额外分支和 move 路径没有
  换回稳定收益；不保留。
- 叶片 pair relation 前置 AABB/平行/共面判定：尝试在
  `buildLeafPairRelation()` 已知 AABB overlap 后先判断支撑平面是否平行，非平行时调用
  新的 overlapping/non-parallel trusted carrier 入口，平行时用已知平行的共面检查，
  避免 `computeBidirectionalPolygonIntersectionCarriersTrusted()` 与 `areCoplanarPolygons()`
  重复做 AABB 和法向平行判定。ctest 通过，两轮 100 组 NoTracy
  `run_20260522_021124` / `run_20260522_021237` 与 `run_20260522_005732`
  的结构计数、result 计数和 `leaf_classification_trace_attempt_count` 完全一致；
  但两轮新结果均值 `solve_ms` 约 118.956ms、`end_to_end_ms` 约 338.719ms，
  不优于 `run_20260522_005611` / `run_20260522_005732` 两轮保留基线均值
  118.736ms / 337.992ms。重复判定本身不是当前主导成本，新增分支和额外入口没有
  换回稳定收益；不保留。
- axis-aligned corner path 底层构造预留 `axisCount` 容量：尝试给
  `buildAxisAlignedCornerPathFromIntegers()` 的最多三段路径提前 `reserve()`，
  减少 axis path 和 AABB bridge path 的短 vector 扩容。Debug 构建和 ctest 通过，
  两轮 100 组论文样本 `run_20260522_022424` / `run_20260522_022541` 与
  `run_20260522_021956` 的结构计数、result 计数、axis/plane replacement 尝试数和
  trace 次数完全一致；但两轮新结果均值 `solve_ms` 约 118.605ms，差于
  `run_20260522_021842` / `run_20260522_021956` 当前保留基线均值 118.252ms，
  `end_to_end_ms` 基本持平。通用 corner path 的短 vector 扩容不是当前主导成本；
  不保留。
- fast AABB 子参考候选去重表延迟分配：尝试在
  `visitFastAABBPathCandidateSeeds()` 中先访问候选路径，只有 visitor 继续枚举时才
  `reserve(20)` 并记录已发射目标，目标是让常见“首个 fast candidate 成功”路径不分配
  去重表。Debug 构建和 ctest 已在实验前通过，100 组论文样本
  `run_20260522_023020` 与 `run_20260522_021956` 的结构计数、child reference
  candidate/trace 计数和 leaf classification trace 次数完全一致；但单轮
  `solve_ms` 从 118.531ms 到 118.631ms，且差于
  `run_20260522_021842` / `run_20260522_021956` 当前保留基线均值 118.252ms。
  端到端小幅改善更像准备/导出噪声，核心 solve 没有收益；不保留。
- trusted clipped polygon eager 顶点/AABB 缓存：结构计数不变，solve 信号为负。
- BSP 插入端点齐次交点沿递归缓存：结构计数不变，22 个 verifier 全部通过，
  但 `run_20260521_032931` 相比 `run_20260521_031822` 的聚合 `solve_ms`
  从 2423.462ms 退化到 2455.315ms；额外状态传递和端点对象拷贝成本超过收益。
- 共面凸多边形在局部 BSP 插入前增加严格边半空间分离剪枝，22 个 verifier 全部通过，
  但 `run_20260521_092840` 相比当前保留基线 `run_20260521_085436` 的单轮
  `solve_ms` 从 1761.29ms 退化到 1767.55ms；额外顶点恢复和半空间分类成本
  没有换回足够的无效 coplanar edge 插入减少。
- 深层 `tryReuseChildReference()` 基于“父节点 reference 已避开当前支撑平面，
  child 支撑平面是其子集”的不变量，跳过 child support planes 线性扫描：
  `run_20260521_094847` 的 22 个 verifier 全部通过且单轮 `solve_ms` 为
  1753.95ms，但 `run_20260521_095320` 三次无 oracle 重复计时为
  1762.23ms、1756.16ms、1763.34ms，均值 1760.58ms，差于
  `run_20260521_085919` 的三次均值约 1754.30ms；不保留该不变量剪枝。
- leaf 停止阈值从默认 25 调到 50：`run_20260521_095518` 的 22 个 verifier
  全部通过，但单轮聚合 `solve_ms` 为 1830.47ms，明显差于当前保留基线；
  说明更粗叶片减少节点数后，会把更多代价转移到叶片 BSP/trace。
- leaf 停止阈值从默认 25 调到 15：`run_20260521_095924` 的 22 个 verifier
  全部通过，单轮聚合 `solve_ms` 为 1748.85ms；`run_20260521_100459`
  三次无 oracle 重复计时为 1729.03ms、1743.30ms、1758.29ms，均值
  1743.54ms，只比 `run_20260521_085919` 的默认阈值均值 1754.30ms
  快约 0.6%，且端到端时间更差。该方向可能提示停止策略要建模 leaf 代价，
  但固定改默认阈值不是数量级收益点，暂不保留。
- AABB 裁剪线段时，对单位坐标边界尝试复用已算出的交点缓存并走 trusted
  `Segment256` 构造，避免普通构造里的 primitive 化和端点重算；Debug 测试通过，
  `run_20260521_100834` 的 22 个 verifier 全部通过，但单轮 `solve_ms` 为
  1770.47ms，差于 `run_20260521_085436` 的 1761.29ms。该分支增加的判断和
  更复杂控制流没有换回足够的 gcd/端点重算节省，不保留；后续保留的是去掉
  单位平面判断、直接使用有向端点缓存入口的版本。
- WNV path AABB precheck 复用相邻线段共享端点的 point AABB：22 个 verifier
  全部通过，但 `run_20260521_104801` 单轮 `solve_ms` 为 1748.16ms，差于当前
  保留版本 `run_20260521_103114` 的三次均值 1728.62ms；少算的
  `floorCeilDiv()` 不足以抵消额外 point-box 状态、组件比较和 merge 成本。
- `appendPointToAABB()` 在 `w > 0` 时改走正分母专用
  `floorCeilDivPositiveDen()`，尝试避开通用 `floorCeilDiv()` 的分母符号处理；
  Debug 测试通过，`run_20260521_105729` 的 22 个 verifier 全部通过，但单轮
  `solve_ms` 为 1753.76ms，差于当前保留版本 `run_20260521_103114` 的三次均值
  1728.62ms。说明当前 AABB 热点瓶颈不只是分母符号分支，继续在单次除法入口做
  小特化不是有效方向。
- plane-replacement build 去重从 `unordered_set` 改为线性签名表，避免每个候选
  hash 多个 256 位平面系数；Debug 测试通过，`run_20260521_110637` 的 22 个
  verifier 全部通过且单轮 `solve_ms` 为 1727.53ms，但
  `run_20260521_111117` 三次无 oracle 重复计时为 1724.09ms、1727.23ms、
  1735.83ms，均值 1729.05ms，略差于当前保留版本 `run_20260521_103114`
  的三次均值 1728.62ms；不保留该容器替换。
- WNV tracing 的 path/segment AABB 预筛中，把 `doesPlaneIntersectAABB()`
  前置到 `polygon.aabb()` 之前，尝试在不恢复 polygon 顶点/AABB 的情况下按支撑平面
  早跳过；Debug 测试通过，`run_20260521_111554` 的 22 个 verifier 全部通过，
  但单轮 `solve_ms` 为 1744.72ms，差于当前保留版本 `run_20260521_103114`
  的三次均值 1728.62ms。该平面-盒测试的额外代价或命中率不足，不保留。
- `buildRoundedCentroidPoint()` 改为单遍转换顶点并累计三角扇面积重心，尝试避免
  每个 leaf fragment 构造 `std::vector<Vec3d>`；Debug 测试通过，
  `run_20260521_112355` 的 22 个 verifier 全部通过，但单轮 `solve_ms`
  为 1733.38ms，差于当前保留版本 `run_20260521_103114` 的三次均值
  1728.62ms；不保留该重心构造改写。
- 删除 `hasIntermediateEndpointOnInputSurface()` 预筛，让 WNV trace 统一处理
  非最后段端点落面并返回 `PATH_INVALID`；Debug 测试通过，
  `run_20260521_113104` 的 22 个 verifier 全部通过，但 trace 尝试数从
  当前保留版本的 232002 增至 245780，单轮 `solve_ms` 为 1734.45ms，
  差于 `run_20260521_103114` 的三次均值 1728.62ms；预筛仍然比多跑无效 trace 便宜。
- `hasIntermediateEndpointOnInputSurface()` 改为先判断中间点是否落在 polygon
  支撑平面，再查已有 AABB 和边半空间，尝试减少完整 `polygon.classify()`；
  Debug 测试通过，`run_20260521_113741` 的 22 个 verifier 全部通过且计数不变，
  但单轮 `solve_ms` 为 1808.25ms，明显差于当前保留版本；现有 AABB-first
  快速拒绝顺序更适合当前 workload。
- centroid axis 的 local reference 整数起点按叶片缓存：结构计数不变，22 个
  verifier 全部通过，但 `run_20260521_033829` 相比 `run_20260521_031822`
  的聚合 `solve_ms` 从 2423.462ms 退化到 2431.565ms；减少重复解析不足以抵消
  context 状态和额外重载分派成本。该假设在后续基线又复测一次：
  `run_20260521_091932` 的 22 个 verifier 全部通过且单轮 `solve_ms` 为
  1741.94ms，但 `run_20260521_092401` 三次无 oracle 重复计时为
  1767.29ms、1766.60ms、1773.86ms，均值 1769.25ms，仍差于上一保留基线
  `run_20260521_085919` 的三次均值约 1754.30ms；不保留该缓存。
- `Polygon256::aabb()` 改成 AABB-only 重建、不顺带填充顶点缓存：结构计数不变，
  22 个 verifier 全部通过，但 `run_20260521_034830` 相比 `run_20260521_031822`
  的聚合 `solve_ms` 从 2423.462ms 退化到 2440.191ms；后续顶点访问的重复构造
  抵消了 AABB-only 的分配节省。该旧负实验不等同于后续保留的拆分版本：后者已在
  未约分顶点缓存和新版 split 基线之后重测，且 AABB-only 入口直接消费未规范化齐次点。
- `axisMinimum()` / `axisMaximum()` 改为返回 AABB 字段引用、减少 `Integer` 边界拷贝：
  结构计数不变，22 个 verifier 全部通过，但 `run_20260521_035707` 相比
  `run_20260521_031822` 的聚合 `solve_ms` 从 2423.462ms 退化到 2429.444ms；
  该热循环中的边界值拷贝不是当前主导成本。
- `addScaledWNTV()` 改为通过 `WNV::data()` 指针循环，22 个 verifier 全部通过，但
  `run_20260521_061452` 相比 `run_20260521_060849` 的聚合 `solve_ms` 从 1950.69ms
  退化到 1966.73ms；保留下标访问版本。
- `appendPointToAABB()` 为同一齐次点三个坐标共用一次分母符号规范化，22 个 verifier
  全部通过，但 `run_20260521_062619` 相比 `run_20260521_060849` 的聚合 `solve_ms`
  从 1950.69ms 退化到 1971.96ms；保留原 `floorCeilDiv()` 调用形态。
- `BSPTree` 节点从 `unique_ptr` 子节点改成 `deque` 节点池、裸指针连接，22 个 verifier
  全部通过，但 `run_20260521_064400` 相比 `run_20260521_063440` 的聚合
  `solve_ms` 从 1927.46ms 退化到 1932.49ms；deque 分块和指针间接没有抵消分配收益。
- 已知 axis-probe 目标的 AABB 检查改为只对自由轴做齐次比较、固定轴直接整数比较，
  22 个 verifier 全部通过，但 `run_20260521_065528` 相比 `run_20260521_063440`
  的聚合 `solve_ms` 从 1927.46ms 退化到 1929.04ms；保留通用 AABB 检查。
- `gcdMagnitude()` 改为欧几里得 `%` 版本，22 个 verifier 全部通过，但
  `run_20260521_070128` 相比 `run_20260521_063440` 的聚合 `solve_ms` 从
  1927.46ms 退化到 1987.54ms，`end_to_end_ms` 从 5307.45ms 退化到 6072.29ms；
  该 `_BitInt(256)` 后端上 256 位取模成本高于现有二进制 gcd。
- Raw OBJ/STL 导出的顶点去重 key 改为未 gcd 的符号规范化齐次四元组，22 个 verifier
  全部通过且导出面数不变，但 `run_20260521_070837` 相比 `run_20260521_063440`
  的聚合 `export_ms` 从 1933.74ms 退化到 2171.55ms；未约分 key 降低顶点合并率，
  反而放大后续导出工作。
- 叶片分类 centroid axis-probe 改为绕过 `LeafClassificationPathCandidate` 临时对象，
  直接复用 `LeafClassificationContext` 中的路径缓冲；22 个 verifier 全部通过，但
  `run_20260521_073652` 相比 `run_20260521_072311` 的聚合 `solve_ms` 从
  1893.49ms 退化到 1922.97ms，`end_to_end_ms` 从 5251.62ms 退化到 5365.28ms；
  保留原候选对象路径，说明当前瓶颈不是这层 vector 生命周期。
- split plan 的 child support planes 改为保存 `Plane3i*` 视图、避免复制 256 位平面；
  22 个 verifier 全部通过，但 `run_20260521_074607` 相比 `run_20260521_072311`
  的聚合 `solve_ms` 从 1893.49ms 退化到 1914.10ms，`end_to_end_ms` 从 5251.62ms
  退化到 5266.75ms；保留平面值拷贝，当前路径更受连续存储和直接访问影响。
- `floorDivByTwo()` 的负奇数判断从 `quotient * 2 != value` 改为最低位测试；
  22 个 verifier 全部通过，但 `run_20260521_075303` 相比 `run_20260521_072311`
  的聚合 `solve_ms` 从 1893.49ms 退化到 1906.74ms，`end_to_end_ms` 从 5251.62ms
  退化到 5272.82ms；保留乘法回判版本，当前 `_BitInt(256)` 下位运算分支不占优。
- split plan 的 `vertexSides` 从每条 split route 独立 vector 改为 plan 级连续缓冲，
  并给 trusted clipping 增加 `int* + count` 入口；22 个 verifier 全部通过，但
  `run_20260521_082208` 相比当前基线 `run_20260521_080941` 只有单轮
  `solve_ms` 1852.88ms -> 1850.92ms 的微弱信号，`end_to_end_ms` 5231.12ms ->
  5264.79ms 退化；随后 `run_20260521_082736` 做 3 次无 oracle 重复计时，每轮
  聚合 `solve_ms` 为 1882.86ms、1858.12ms、1852.29ms，平均不优于基线。
  该路径不保留，说明 route 局部 vector 的分配不是当前 split materialization 主因。

这些结论只用于避免近期重复试错；若 workload、算法边界或 profile 证据变化，可以重新评估。
