/**
 * @file solver_shared.h
 * @brief 定义 BoolProblem 与 SubdivisionSolver 共享的内部辅助函数。
 */
#pragma once

#include "algorithm/WNV_tracing.h"

#include <vector>

namespace ember
{
    namespace detail
    {
        inline std::size_t computeWNVSize(const std::vector<Polygon256> &polygons) noexcept
        {
            //TODO:当前只打算实现二元运算

            // std::size_t dimension = 0;
            // for (const Polygon256 &polygon : polygons)
            // {
            //     dimension = std::max(dimension, polygon.WNTV.size());
            // }

            return 2;
        }
    }
}
