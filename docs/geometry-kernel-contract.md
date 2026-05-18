# 几何 Kernel 契约

本文档记录当前核心布尔路径使用的底层数学边界。它是实现约束，不是性能建议。

## 允许的核心图元

- `plane_from_points` / point-normal 平面构造：输入必须来自量化整数顶点、AABB 轴平面，或已有证明安全的平面。
- `are_planes_parallel`：只比较平面法向。
- `intersect_3_planes`：用四个 3x3 行列式构造齐次整数点。
- `classify_vertex`：用 `sign(x^T s) * sign(x.w)` 分类齐次点。
- `classify_integer_vertex`：整数顶点对平面的 signed-distance 符号。
- clipping / BSP / tracing 只能复用已有 support plane、edge plane、split plane、AABB plane 和它们的三平面交点。

## 禁止进入核心路径的操作

- 任意齐次点加减、平均、中点或方向向量构造。
- 从分数齐次点反推 `w*x-p=0` 这类坐标平面作为路径或分类 fallback。
- 从输出齐次顶点构造新的 polygon 平面或重三角化平面。
- 用 `int512_t` 或 `cpp_int` 的结果决定核心算法分支。
- 在分类失败时使用未经论文闭包证明的几何 fallback 猜测结果。

## 当前实现状态

- `src/math/paper_kernel.h` 是论文 primitive 的受控入口。
- `src/math/fixed_int256.h` 是实验性的自定义 256 位定长算术后端，只覆盖 checked 加减乘、3x3 行列式和 4D dot；当前先由测试 oracle 验证，尚未替换核心 `Integer`。
- `math256_tests` 使用 `cpp_int` 只作为 oracle，验证 `classify_vertex` 与 4x4 行列式符号一致。
- leaf classification 已移除 homogeneous average、equalized edge 和 free-coordinate path fallback。
- axis path 只允许端点都能精确提取为整数坐标；分数齐次点必须走平面替换路径或显式失败。
- polygon-plane intersection 的 carrier 去重不再依赖 `areSameHomPoint`，而按 carrier plane 身份去重。

## 迁移原则

新增几何代码必须先说明它归属于允许图元之一。若需要新图元，先给出 256-bit 中间结果预算和失败策略，再接入核心求解器。`int512_t` / `cpp_int` 可以作为测试参考或 I/O 后处理，但不能把核心算法从固定宽度闭包里带出去。自定义定长 backend 必须先以可切换、可 oracle 校验的窄接口接入，不能一次性替换全局数值类型。
