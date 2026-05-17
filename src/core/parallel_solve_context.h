/**
 * @file parallel_solve_context.h
 * @brief 声明内部细分并行执行上下文。
 */
#pragma once

#include <oneapi/tbb/task_group.h>

#include <cstddef>

namespace ember
{
/**
 * @brief 根求解阶段共享的轻量并行执行上下文。
 *
 * 当前版本只保存 sibling task 所需的最小信息，不把 TBB 运行时细节泄露到
 * `BoolProblem` 的公开接口中。
 */
struct ParallelSolveContext
{
    bool enabled = false;                                ///< 是否允许当前子树继续提交并行 sibling task。
    std::size_t effectiveThreadCount = 1;               ///< 本次求解实际参与的总线程数。
    oneapi::tbb::task_group_context *taskGroupContext = nullptr; ///< 根求解共享的 TBB task group context。

    /**
     * @brief 返回当前上下文是否允许提交并行 sibling task。
     */
    bool canSpawnSiblingTasks() const noexcept
    {
        return enabled && effectiveThreadCount > 1 && taskGroupContext != nullptr;
    }
};
}
