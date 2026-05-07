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
inline constexpr std::size_t kBinaryWnvDimension = 2u;
inline constexpr std::size_t kLhsOperandIndex = 0u;
inline constexpr std::size_t kRhsOperandIndex = 1u;

inline bool isLhsOperandWNTV(const WNV &wntv) noexcept
{
    return wntv.size() == kBinaryWnvDimension &&
           wntv[kLhsOperandIndex] == 1 &&
           wntv[kRhsOperandIndex] == 0;
}

inline bool isRhsOperandWNTV(const WNV &wntv) noexcept
{
    return wntv.size() == kBinaryWnvDimension &&
           wntv[kLhsOperandIndex] == 0 &&
           wntv[kRhsOperandIndex] == 1;
}

inline bool isCanonicalBinaryOperandWNTV(const WNV &wntv) noexcept
{
    return isLhsOperandWNTV(wntv) || isRhsOperandWNTV(wntv);
}
}
}
