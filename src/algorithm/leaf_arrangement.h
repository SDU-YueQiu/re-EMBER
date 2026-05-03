#pragma once

#include "geometry/geometry256.h"

#include <vector>

namespace ember
{
    /**
     * @brief 为一个局部多边形集合构造全部启用的叶片片段。
     *
     * @throws std::runtime_error 当局部 BSP 切分流程破坏几何不变量时抛出。
     */
    std::vector<Polygon256> buildLeafArrangement(const std::vector<Polygon256> &polygons);
}
