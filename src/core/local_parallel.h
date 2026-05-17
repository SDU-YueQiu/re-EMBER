/**
 * @file local_parallel.h
 * @brief 声明求解器内部局部并行 kernel 共用的小工具。
 */
#pragma once

#include "core/parallel_solve_context.h"
#include "geometry/geometry256.h"

#include <cstddef>
#include <vector>

namespace ember
{
namespace detail
{
/**
 * @brief 缓存行对齐的局部并行 block 输出槽。
 *
 * 局部并行循环只写各自 block 的 `value`，结束后再按 block 顺序串行归并，
 * 避免共享 `std::vector::push_back` 和相邻 block header 的伪共享。
 */
template <typename T>
struct alignas(64) LocalParallelBlock
{
    T value;
};

/**
 * @brief 判断局部并行 kernel 是否满足上下文和工作量门槛。
 *
 * @param context 根求解器创建的内部并行上下文。
 * @param workItemCount 当前局部 kernel 的逻辑任务数量。
 * @param minimumWorkItemCount 启动局部并行前要求的最小任务数量。
 * @return 可以安全提交局部并行任务时返回 `true`。
 */
inline bool shouldRunLocalParallel(
    const ParallelSolveContext *context,
    std::size_t workItemCount,
    std::size_t minimumWorkItemCount) noexcept
{
    return context != nullptr && context->canRunLocalParallel(workItemCount, minimumWorkItemCount);
}

/**
 * @brief 串行预热多边形的顶点和 AABB 派生缓存。
 *
 * `Polygon256::vertices()` / `Polygon256::aabb()` 是 const 接口，但内部会写
 * mutable cache。任何局部并行读共享 polygon 的入口都应先串行调用该函数，
 * 避免多个线程同时初始化同一份缓存。
 */
inline void prewarmPolygonDerivedCaches(const std::vector<Polygon256> &polygons)
{
    for (const Polygon256 &polygon : polygons)
    {
        polygon.vertices();
        polygon.aabb();
    }
}
}
}
