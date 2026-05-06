/**
 * @file winding_number_f.h
 * @brief 定义 WNV 存储和二元布尔指示函数。
 */
// 环绕数和面分类时的指示函数。

#pragma once

#include <cstddef>
#include <vector>

namespace ember
{
    // TODO：当前仅实现两个网格的单一布尔运算，多网格、表达式计算需要写语法解析。

    using WNV = std::vector<int>;

    enum BoolStatus
    {
        OUT,
        IN
    };

    // 两网格布尔差运算指示函数，使用 int 指定被减网格和减网格；如 A-B，A 为被减网格。
    inline BoolStatus f_diff(const WNV &wnv, int subed, int subor)
    {
        return (wnv[subed] != 0 && wnv[subor] == 0) ? IN : OUT;
    }

    inline BoolStatus f_intersection(const WNV &wnv, int A, int B)
    {
        return (wnv[A] != 0 && wnv[B] != 0) ? IN : OUT;
    }

    inline BoolStatus f_union(const WNV &wnv, int A, int B)
    {
        return (wnv[A] != 0 || wnv[B] != 0) ? IN : OUT;
    }
}
