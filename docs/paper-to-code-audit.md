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
| work-stealing parallel | EMBER 4.5.4、5.3 | 当前 child 子树级 oneTBB 任务，merge 固定 left -> right；sibling task 提交门槛已降到 leaf threshold | 并行边界仍较粗；叶内 BSP 和分类串行。继续扩展前必须证明无共享状态和稳定聚合 | 下一步看更细粒度并行是否会被 task 开销抵消，不能改动结果聚合顺序 |
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

## 已测但不保留的局部实验

- 路径候选中的 `makePointFromPlanes()` 和 axis segment 临时端点改用
  `intersectHomogeneousUnnormalized()`，尝试把 plane replacement / axis path
  的中间点也推到 BSP 论文的未约分 `intersect_3_planes` 边界；Debug 测试和
  `run_20260521_131131` 的 22 个 verifier 均通过，但
  `run_20260521_131614` 三次无 oracle 重复计时为 1678.75ms、1681.76ms、
  1704.47ms，均值 1688.33ms，差于 `run_20260521_125353` 的 1676.89ms。
  该路径上的中间点会反复参与 AABB 和半空间分类，未约分齐次代表元放大了后续
  256 位乘法成本；暂时保留 normalized 路径点。
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
  抵消了 AABB-only 的分配节省。
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
