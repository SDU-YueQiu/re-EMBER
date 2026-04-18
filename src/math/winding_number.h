#pragma once

#include <vector>

namespace ember
{
    // TODO:当前仅实现两个网格的单一布尔运算，多网格、表达式计算需要写语法解析

    typedef std::vector<int> WNV;

    enum class BoolStatus
    {
        OUT,
        IN
    };

    // 两网格布尔差运算指示函数，使用int指定被减网格和减网格，如A-B，A为被减网格
    inline BoolStatus f_diff(WNV &wnv, int subed, int subor)
    {
        return (wnv[subed] != 0 && wnv[subor] == 0) ? BoolStatus::IN : BoolStatus::OUT;
    }

    inline BoolStatus f_intersection(WNV &wnv, int A, int B)
    {
        return (wnv[A] != 0 && wnv[B] != 0) ? BoolStatus::IN : BoolStatus::OUT;
    }

    inline BoolStatus f_union(WNV &wnv, int A, int B)
    {
        return (wnv[A] != 0 || wnv[B] != 0) ? BoolStatus::IN : BoolStatus::OUT;
    }
}