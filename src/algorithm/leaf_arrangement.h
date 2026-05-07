/**
 * @file leaf_arrangement.h
 * @brief 声明细分叶子节点的局部编排构造入口。
 */
#pragma once

#include "geometry/geometry256.h"

#include <vector>

namespace ember
{
/**
 * @brief 为一个局部多边形集合构造全部启用的叶片片段。
 *
 * @throws std::runtime_error 当局部 BSP 切分流程破坏几何不变量时抛出。
 * @pre `polygons` 来自 `BoolProblem::solve(sceneAABB)` 已验证输入或其有效裁剪结果。
 */
std::vector<Polygon256> buildLeafArrangement(const std::vector<Polygon256> &polygons);
}
