# kEmber API 文档

本文档按“便于查阅”的方式整理当前代码中的主要接口，重点关注：

- 接口属于哪个模块
- 方法做什么
- 参数和返回值含义
- 调用前需要满足什么约束
- 还有哪些容易踩坑的补充说明


## 1. 目录索引

| 模块 | 主要文件 | 主要内容 | 备注 |
| --- | --- | --- | --- |
| 数学基础 | `src/math/math256.h` | 256 位整数、向量、行列式、符号判断 | 所有几何算法的基础 |
| 平面几何基础 | `src/geometry/plane_geometry256.h` | 平面、齐次点、三平面交点 | 点和线的核心表示基础 |
| 布尔状态辅助 | `src/math/winding_number_f.h` | `WNV`、`BoolStatus`、简单布尔状态函数 | 目前只是辅助，不是完整求解器 |
| 几何对象 | `src/geometry/geometry256.h` | `Line256`、`Segment256`、`Polygon256` | 当前最重要的数据抽象层 |
| 裁剪/求交 | `src/algorithm/clipping.h` | 多边形-平面求交、交线载体、叶子裁剪 | BSP 的关键下游依赖 |
| BSP | `src/algorithm/bsp.h` | `BSPNode`、`BSPTree` | 单个基底多边形的切分结构 |
| 问题级组装 | `src/core/bool_problem.h` | `BoolProblem` | 批量组织输入多边形并建立树 |

## 2. 类型总览

| 类型 | 所在文件 | 作用 | 关键说明 |
| --- | --- | --- | --- |
| `Integer` | `src/math/math256.h` | 256 位整数 | 实际类型为 `slim::int256_t` |
| `Vec2i` | `src/math/math256.h` | 二维整数向量 | 用于 2D 行列式和方向判断 |
| `Vec3i` | `src/math/math256.h` | 三维整数向量 | 用于平面法向与空间行列式 |
| `Plane3i` | `src/geometry/plane_geometry256.h` | 平面 `a*x+b*y+c*z+d=0` | 整个几何系统的基础实体 |
| `HomPoint4i` | `src/geometry/plane_geometry256.h` | 齐次点 | 用于精确平面关系判断 |
| `PlanePoint3i` | `src/geometry/plane_geometry256.h` | 三平面交点 | 当前代码中“点”的主表示方式 |
| `Line256` | `src/geometry/geometry256.h` | 两平面交线 | 不是端点式线段表示 |
| `Segment256` | `src/geometry/geometry256.h` | 直线 + 两个端点约束平面 | 构造时会自动调整端点平面方向 |
| `Polygon256` | `src/geometry/geometry256.h` | 支撑平面 + 边界平面序列 | 默认针对凸多边形 |
| `BSPNode` | `src/algorithm/bsp.h` | BSP 树节点 | 更偏内部结构，不建议上层直接依赖 |
| `BSPTree` | `src/algorithm/bsp.h` | 单个基底多边形的切分树 | 当前算法层的核心输出对象 |
| `BoolProblem` | `src/core/bool_problem.h` | 问题级容器 | 负责组织多边形和批量建树 |
| `WNV` | `src/math/winding_number_f.h` | 绕组数向量 | 当前只作为状态载体 |
| `BoolStatus` | `src/math/winding_number_f.h` | `IN/OUT` 状态 | 用于简单布尔状态判断 |

## 3. 数学基础接口

### 3.1 基础数值函数

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `signum(const Integer& value)` | 返回数值符号 | `value`：目标整数 | `-1 / 0 / 1` | 后续大量分类函数依赖这个约定 |
| `isZero(const Integer& value)` | 判断是否为 0 | `value`：目标整数 | `bool` | 常用于平面平行、向量退化判断 |
| `isPositive(const Integer& value)` | 判断是否大于 0 | `value`：目标整数 | `bool` | - |
| `isNegative(const Integer& value)` | 判断是否小于 0 | `value`：目标整数 | `bool` | - |
| `floorDiv(const Integer& a, const Integer& b)` | 数学意义下向下整除 | `a`：被除数；`b`：除数 | `Integer` | 与 C++ 默认截断除法不同 |
| `ceilDiv(const Integer& a, const Integer& b)` | 数学意义下向上整除 | `a`：被除数；`b`：除数 | `Integer` | `computeAABBPlanes()` 会用到 |

### 3.2 `Vec2i`

| 方法/运算 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `operator+ / - / * / /` | 向量四则运算 | 向量或标量 | `Vec2i` | 均为整数运算 |
| `lengthSquared()` | 返回长度平方 | 无 | `Integer` | 不做开方，保持精确整数 |
| `dot(a, b)` | 点积 | `a`、`b`：二维向量 | `Integer` | - |
| `cross(a, b)` | 二维叉积 | `a`、`b`：二维向量 | `Integer` | 常用于方向判断 |
| `determinant2x2(...)` | 2x2 行列式 | 四个标量 | `Integer` | - |
| `determinant(row1, row2)` | 2x2 行列式快捷写法 | 两个 `Vec2i` | `Integer` | - |
| `crossSign(a, b)` | 二维叉积符号 | `a`、`b`：二维向量 | `int` | 内部等价 `signum(cross(...))` |
| `orient2d(a, b, c)` | 2D 方向行列式 | 三个点 | `Integer` | 用于左转/右转判定 |
| `orient2dSign(a, b, c)` | 2D 方向符号 | 三个点 | `int` | 返回 `-1/0/1` |

### 3.3 `Vec3i`

| 方法/运算 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `operator+ / - / * / /` | 向量四则运算 | 向量或标量 | `Vec3i` | - |
| `lengthSquared()` | 返回长度平方 | 无 | `Integer` | - |
| `dot(a, b)` | 点积 | `a`、`b`：三维向量 | `Integer` | - |
| `cross(a, b)` | 叉积 | `a`、`b`：三维向量 | `Vec3i` | `Line256::directionVector()` 用它求方向 |
| `scalarTriple(a, b, c)` | 标量三重积 | 三个向量 | `Integer` | 用于三维方向关系 |
| `determinant3x3(...)` | 3x3 行列式 | 九个标量 | `Integer` | - |
| `determinant4x4(...)` | 4x4 行列式 | 十六个标量 | `Integer` | 平面分类底层会用到 |
| `determinant(row1, row2, row3)` | 3x3 行列式快捷写法 | 三个 `Vec3i` | `Integer` | - |
| `orient3dSign(a, b, c)` | 三维方向符号 | 三个向量 | `int` | 返回 `-1/0/1` |

## 4. 平面几何接口

### 4.1 `Plane3i`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `normal() const` | 返回平面法向量 | 无 | `Vec3i` | 即 `(a, b, c)` |
| `Plane3i::fromPointNormal(point, normal)` | 由点和法向构造平面 | `point`：平面上一点；`normal`：法向量 | `Plane3i` | 当前创建平面的首选方式 |

字段说明：

| 字段 | 含义 | 补充 |
| --- | --- | --- |
| `a, b, c, d` | 平面方程系数 | 满足 `a*x + b*y + c*z + d = 0` |

### 4.2 `HomPoint4i`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `dotPlane(const Plane3i& s) const` | 计算点与平面方程的代数值 | `s`：目标平面 | `Integer` | 常用于符号分类 |
| `classify(const Plane3i& s) const` | 判断点相对平面的位置 | `s`：目标平面 | `int` | 返回 `-1/0/1`，且考虑 `w` 的符号 |
| `hasSameComponents(const HomPoint4i& rhs) const` | 判断四个分量是否完全相等 | `rhs`：比较对象 | `bool` | 这是“逐分量相等”，不是射影意义下等价 |

### 4.3 `plane_geometry256.h` 中的辅助函数

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `areSameHomPoint(lhs, rhs)` | 射影意义下判断两个齐次点是否等价 | `lhs`、`rhs`：两个齐次点 | `bool` | 会做按分量比例比较 |
| `determinant4x4(row1, row2, row3, row4)` | 由四个平面构造 4x4 行列式 | 四个 `Plane3i` | `Integer` | 底层分类函数使用 |
| `arePlaneNormalsParallel(p, q)` | 判断两个平面法向是否平行 | `p`、`q`：两个平面 | `bool` | 只看法向，不看是否重合 |
| `normalDeterminant(p, q, r)` | 三个平面法向的 3x3 行列式 | 三个平面 | `Integer` | 用于判断三平面是否唯一相交 |
| `hasUniqueIntersection(p, q, r)` | 判断三个平面是否有唯一交点 | 三个平面 | `bool` | 当前很多对象合法性的基础 |
| `intersectHomogeneous(p, q, r)` | 求三个平面的齐次交点 | 三个平面 | `HomPoint4i` | 调用方需要自行确保唯一交点存在 |
| `classifyPointAgainstPlane(x, s)` | 判断齐次点相对平面的位置 | `x`：齐次点；`s`：平面 | `int` | 是 `HomPoint4i::classify` 的函数版本 |
| `classifyByDeterminants(p, q, r, s)` | 基于行列式判断三平面交点相对第四个平面的位置 | 四个平面 | `int` | 测试中用于交叉验证分类结果 |

### 4.4 `PlanePoint3i`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `PlanePoint3i(const Plane3i& pVal, const Plane3i& qVal, const Plane3i& rVal)` | 由三个平面构造一个点 | 三个平面 | 构造对象 | 不会主动拒绝退化输入 |
| `hasUniqueIntersection() const` | 判断该点是否真的对应唯一交点 | 无 | `bool` | 使用前通常应该先检查 |
| `classify(const Plane3i& s) const` | 判断该点相对另一个平面的位置 | `s`：目标平面 | `int` | 返回 `-1/0/1` |

字段说明：

| 字段 | 含义 | 补充 |
| --- | --- | --- |
| `p, q, r` | 定义该点的三个平面 | 保留了点的“来源” |
| `x` | 三平面交点的齐次坐标 | 用于各种分类计算 |

## 5. 布尔状态辅助接口

### 5.1 `winding_number_f.h`

| 名称 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `using WNV = std::vector<int>` | 绕组数向量类型别名 | - | - | 当前主要作为状态载体 |
| `enum class BoolStatus { OUT, IN }` | 简单布尔状态 | - | - | 只有 `IN/OUT` 两种 |
| `f_diff(WNV& wnv, int subed, int subor)` | 差运算状态判定 | `wnv`：绕组向量；`subed`：被减对象索引；`subor`：减对象索引 | `BoolStatus` | 仅支持双对象语义 |
| `f_intersection(WNV& wnv, int subed, int subor)` | 交运算状态判定 | 同上 | `BoolStatus` | 当前不是完整布尔求解流程 |
| `f_union(WNV& wnv, int subed, int subor)` | 并运算状态判定 | 同上 | `BoolStatus` | 后续若支持表达式，需重构 |

## 6. 几何对象接口

### 6.1 `Line256`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `Line256(const Plane3i& first, const Plane3i& second)` | 由两个平面构造交线 | `first`、`second`：两个平面 | 构造对象 | 两平面不能平行，否则是退化线 |
| `directionVector() const` | 求交线方向向量 | 无 | `Vec3i` | 通过两个法向量叉积得到 |
| `isValid() const` | 判断这条线是否有效 | 无 | `bool` | 本质上检查两个平面是否平行 |
| `contains(const HomPoint4i& x) const` | 判断齐次点是否在线上 | `x`：目标点 | `bool` | 要求点同时在两个平面上 |
| `contains(const PlanePoint3i& x) const` | 判断三平面交点是否在线上 | `x`：目标点 | `bool` | 内部转到齐次点版本 |

辅助函数：

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `makeLine(const Plane3i& p1, const Plane3i& p2)` | 构造 `Line256` 的便捷函数 | 两个平面 | `Line256` | 只是包装构造函数 |
| `intersect(const Line256& line, const Plane3i& plane)` | 求直线与平面的交点 | `line`：直线；`plane`：平面 | `PlanePoint3i` | 调用方需自行检查唯一交点 |
| `areParallel(const Line256& lhs, const Line256& rhs)` | 判断两条线是否平行 | 两条线 | `bool` | 基于方向向量叉积 |

### 6.2 `Segment256`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `Segment256(const Plane3i& startPlane, const Plane3i& endPlane)` | 用两个端点约束平面构造线段 | `startPlane`、`endPlane`：端点平面 | 构造对象 | 头文件里声明，具体行为依赖实现 |
| `Segment256(const Plane3i& startPlane, const Plane3i& endPlane, const Line256& directionLine)` | 用方向线和端点平面构造线段 | `startPlane`、`endPlane`、`directionLine` | 构造对象 | 会自动调整端点平面的朝向 |
| `isDegenerate() const` | 判断线段是否退化 | 无 | `bool` | 当前会检查：方向线是否有效、端点是否唯一、端点方向关系是否合理 |

字段说明：

| 字段 | 含义 | 补充 |
| --- | --- | --- |
| `start` | 起点约束平面 | 线段内部应位于其负侧 |
| `end` | 终点约束平面 | 线段内部应位于其负侧 |
| `direction` | 线段所在直线 | 由两个平面定义 |

### 6.3 `Polygon256`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `Polygon256()` | 默认构造 | 无 | 构造对象 | 默认是空对象，不代表有效多边形 |
| `Polygon256(const Plane3i& supportPlane, std::vector<Plane3i> edges)` | 构造凸多边形 | `supportPlane`：支撑平面；`edges`：边界平面序列 | 构造对象 | 构造时会自动尝试把边界法向修正为朝外 |
| `addEdgePlane(const Plane3i& edge)` | 追加一条边界平面 | `edge`：边界平面 | `void` | 不会自动校验顺序和合法性 |
| `edgeCount() const` | 返回边界条数 | 无 | `std::size_t` | 小于 3 一定无效 |
| `isValid() const` | 判断多边形表示是否合法 | 无 | `bool` | 会做较重的几何一致性检查 |
| `classify(const PlanePoint3i& point) const` | 判断点相对多边形的状态 | `point`：目标点 | `int` | `0` 可能表示外部、非共面或退化；`1/-1` 表示内部或边界，符号取决于边法向朝向 |
| `containsStrictly(const PlanePoint3i& point) const` | 判断点是否严格在内部 | `point`：目标点 | `bool` | 点落在边界上会返回 `false` |
| `containsOrOnBoundary(const PlanePoint3i& point) const` | 判断点是否在内部或边界上 | `point`：目标点 | `bool` | 当前内部直接依赖 `classify() != 0` |
| `findStrictInteriorPoint(PlanePoint3i& outPoint) const` | 尝试构造一个严格内部点 | `outPoint`：输出参数 | `bool` | BSP 共面重叠消解依赖它 |

字段说明：

| 字段 | 含义 | 补充 |
| --- | --- | --- |
| `plane` | 支撑平面 | 多边形所在平面 |
| `edgePlanes` | 边界平面序列 | 顺序很重要，当前默认针对凸多边形 |
| `WNTV` | 顶点相关绕组/状态字段 | 当前语义未完全闭合 |
| `WNVF` | 面正向绕组状态 | 当前未形成完整上层流程 |
| `WNVB` | 面背向绕组状态 | 当前未形成完整上层流程 |

辅助函数：

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `intersectionSegmentPolygon(Segment256& seg, Polygon256& poly, PlanePoint3i& outPoint)` | 求线段与多边形的交点 | `seg`：线段；`poly`：多边形；`outPoint`：输出交点 | `bool` | 只在交点唯一、交点严格位于多边形内部且在线段内部时返回 `true` |

使用约束：

| 约束 | 说明 |
| --- | --- |
| 凸性约束 | 当前实现默认输入是凸多边形 |
| 顺序约束 | `edgePlanes` 必须按边循环顺序提供 |
| 朝向约束 | 边界平面法向默认应朝外 |
| 唯一交点约束 | 相邻边界平面与支撑平面应能定义唯一顶点 |

## 7. 裁剪与求交接口

### 7.1 `computePolygonPlaneIntersection`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `computePolygonPlaneIntersection(const Polygon256& source, const Plane3i& target, Plane3i& p0, Plane3i& p1)` | 计算多边形与平面的交线端点载体 | `source`：源多边形；`target`：目标平面；`p0/p1`：输出端点所对应的边界平面 | `bool` | `p0/p1` 不是笛卡尔点，而是端点来源边界平面 |

返回 `true` 的前提：

| 条件 | 说明 |
| --- | --- |
| 恰好两个交点 | 当前实现要求得到 2 个有效交点 |
| 无整边重合退化 | 如果出现整条边落在裁剪平面上，当前实现通常直接失败 |

### 7.2 `computePolygonIntersectionCarrier`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `computePolygonIntersectionCarrier(const Polygon256& target, const Polygon256& incoming, Plane3i& outSplitPlane, Plane3i& outV0, Plane3i& outV1)` | 计算两个不共面多边形的交线载体 | `target`：当前目标多边形；`incoming`：插入多边形；`outSplitPlane`：输出切分平面；`outV0/outV1`：输出交线端点约束平面 | `bool` | 是 `BSPTree::insert()` 的核心下游接口 |

补充说明：

| 项目 | 说明 |
| --- | --- |
| `outSplitPlane` | 当前实现固定使用 `incoming.plane` |
| `outV0/outV1` | 表示交线端点“来源边界”，不是显式坐标点 |
| 失败条件 | 输入多边形无效、支撑平面平行、无有效交线、符号一致性检查失败 |

### 7.3 `clipLeafGeometryByPlane`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `clipLeafGeometryByPlane(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped)` | 用一个平面把叶子多边形切成前后两部分 | `source`：源多边形；`clipPlane`：切分平面；`frontClipped/backClipped`：输出子多边形 | `bool` | `front` 表示 `clipPlane` 非负侧，`back` 表示负侧 |

补充说明：

| 项目 | 说明 |
| --- | --- |
| 输出平面 | 两个输出多边形会继承 `source.plane` |
| 输出状态 | 两个输出多边形会复制 `source.WNTV` |
| 常见失败 | `source` 无效、切分平面与支撑平面平行、出现当前实现不接受的退化边 |

## 8. BSP 接口

### 8.1 `BSPNode`

| 字段/方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `BSPNode()` | 默认构造叶子节点 | 无 | 构造对象 | 默认 `isLeaf=true`、`disabled=false` |
| `BSPNode(const Polygon256& polygon)` | 用一个叶子几何初始化节点 | `polygon`：叶子多边形 | 构造对象 | 当前主要由 `BSPTree` 内部使用 |
| `isLeaf` | 是否是叶子节点 | - | - | `true` 时 `leafGeometry` 有意义 |
| `disabled` | 是否禁用该叶子 | - | - | 共面重叠消解时会被置为 `true` |
| `splitPlane` | 当前节点切分平面 | - | - | 非叶子节点使用 |
| `leafGeometry` | 叶子几何 | - | - | 叶子节点使用 |
| `front/back` | 前后子树 | - | - | 非叶子节点使用 |

说明：

| 项目 | 说明 |
| --- | --- |
| 对外建议 | 不建议上层直接依赖 `BSPNode` |
| 当前角色 | 更像 `BSPTree` 的内部实现细节 |

### 8.2 `BSPTree`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `setBasePolygon(const Polygon256& polygon, std::size_t orderKey = 0)` | 设置基底多边形并重置树 | `polygon`：基底多边形；`orderKey`：优先级键 | `void` | 共面重叠处理会用到 `orderKey` |
| `insert(const Polygon256& polygon, std::size_t incomingOrder = 0)` | 向当前树插入一个多边形 | `polygon`：待插入多边形；`incomingOrder`：插入对象顺序键 | `void` | 会按“非共面/共面”两条路径处理 |
| `addSegment(const Plane3i& v0, const Plane3i& v1, const Plane3i& splitPlane)` | 直接插入一条切分线段 | `v0/v1`：端点约束平面；`splitPlane`：切分平面 | `void` | 更底层，适合测试或特殊构造 |
| `contains(const PlanePoint3i& point) const` | 判断点是否位于当前树表示的区域内 | `point`：目标点 | `bool` | 点落在切分平面上时会同时检查两侧 |
| `collectLeafGeometries() const` | 收集所有未禁用的叶子几何 | 无 | `std::vector<Polygon256>` | 当前读取 BSP 结果的主要接口 |

内部关键方法：

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `addSegmentRecursive(BSPNode& node, const Plane3i& v0, const Plane3i& v1, const Plane3i& insertPlane)` | 递归将切分线段压入树 | 当前节点、交线端点约束、切分平面 | `void` | 命中叶子时会调用 `clipLeafGeometryByPlane()` |
| `insertCoplanarPolygonEdges(const Polygon256& polygon)` | 将共面多边形的边界逐条插入树 | `polygon`：共面插入对象 | `void` | 共面路径使用 |
| `disableOverlapLeaves(const Polygon256& polygon)` | 禁用与共面插入对象重叠的叶子 | `polygon`：共面对象 | `void` | 仅在 `baseOrderKey > incomingOrder` 时调用 |

使用约束：

| 约束 | 说明 |
| --- | --- |
| 先设基底 | `insert()` 之前应先调用 `setBasePolygon()` |
| 输入有效 | `basePolygon` 和 `incoming polygon` 都应满足 `isValid()` |
| 当前范围 | 当前树表示的是“单个支撑平面内的切分结果”，不是全局 3D BSP |

## 9. 问题级接口

### 9.1 `BoolProblem`

| 方法 | 介绍 | 参数 | 返回值 | 补充 |
| --- | --- | --- | --- | --- |
| `clear() noexcept` | 清空输入多边形和树结果 | 无 | `void` | 会同时清空 `polygons_` 和 `trees_` |
| `addPolygon(const Polygon256& polygon)` | 追加一个输入多边形 | `polygon`：待加入多边形 | `void` | 不会自动建树 |
| `setPolygons(const std::vector<Polygon256>& polygons)` | 直接设置输入多边形列表 | `polygons`：输入集合 | `void` | 会清空已有树结果 |
| `polygonCount() const noexcept` | 返回输入多边形数量 | 无 | `std::size_t` | - |
| `buildTrees()` | 为每个输入多边形建立一棵 `BSPTree` | 无 | `void` | 当前系统的主流程入口 |
| `computeAABBPlanes() const` | 计算所有输入多边形的整体 AABB 平面 | 无 | `std::vector<Plane3i>` | 返回 6 个平面，并额外扩 1 单位 margin |
| `polygons() const noexcept` | 读取输入多边形列表 | 无 | `const std::vector<Polygon256>&` | 只读访问 |
| `trees() const noexcept` | 读取构建出的树列表 | 无 | `const std::vector<BSPTree>&` | 只读访问 |

主流程说明：

| 阶段 | 说明 |
| --- | --- |
| 初始化 | `trees_` 被清空并按输入多边形数量扩容 |
| 外层循环 | 对每个 `polygons_[i]` 建立一棵树 |
| 内层循环 | 将其余所有 `polygons_[j]` 插入 `trees_[i]` |
| 最终结果 | `trees().size() == polygonCount()` |

## 10. 典型调用方式

### 10.1 低层几何与裁剪

| 步骤 | 调用 | 说明 |
| --- | --- | --- |
| 1 | 构造 `Polygon256 base` 和 `Polygon256 cutter` | 两者都应先满足 `isValid()` |
| 2 | `computePolygonIntersectionCarrier(base, cutter, splitPlane, v0, v1)` | 先求交线载体 |
| 3 | `clipLeafGeometryByPlane(base, splitPlane, frontPart, backPart)` | 再按切分平面裁开基底多边形 |

### 10.2 单棵 BSP 树

| 步骤 | 调用 | 说明 |
| --- | --- | --- |
| 1 | `tree.setBasePolygon(basePolygon, 0)` | 设置基底多边形 |
| 2 | `tree.insert(poly)` | 逐个插入其他多边形 |
| 3 | `tree.collectLeafGeometries()` | 读取切分后的叶子结果 |

### 10.3 问题级处理

| 步骤 | 调用 | 说明 |
| --- | --- | --- |
| 1 | `problem.setPolygons(polygons)` | 设置输入集合 |
| 2 | `problem.buildTrees()` | 批量建树 |
| 3 | `problem.trees()` | 读取每个基底多边形的树 |
| 4 | `problem.computeAABBPlanes()` | 计算整体 AABB |

## 11. 使用注意事项

| 主题 | 说明 |
| --- | --- |
| 多边形类型 | 当前默认是凸多边形，不要直接拿非凸多边形使用 |
| 边界顺序 | `edgePlanes` 顺序错误会直接影响顶点拓扑和合法性 |
| 分类语义 | `Polygon256::classify()` 的 `0` 含义较混合，不能简单理解成“在边界上” |
| 退化情况 | 整边重合、平面平行、非唯一交点等情况，当前很多接口会直接返回 `false` |
| 共面裁决 | `BSPTree` 中共面重叠依赖 `orderKey` 进行优先级处理 |
| 稳定性 | `WNTV/WNVF/WNVB` 相关语义还没有在系统层闭合，不建议过度依赖 |
