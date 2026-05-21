/**
 * @file leaf_arrangement.h
 * @brief 声明细分叶子节点的局部编排构造入口。
 */
#pragma once

#include "geometry/geometry256.h"

#include <cstddef>
#include <vector>

namespace ember
{
using LeafArrangementFragmentVisitor = void (*)(void *userData, Polygon256 &&fragment);

/**
 * @brief 为一个局部多边形集合构造全部启用的叶片片段。
 *
 * @throws std::runtime_error 当局部 BSP 切分流程破坏几何不变量时抛出。
 * @pre `polygons` 来自 `BoolProblem::solve(sceneAABB)` 已验证输入或其有效裁剪结果。
 */
std::vector<Polygon256> buildLeafArrangement(const std::vector<Polygon256> &polygons);

/**
 * @brief 逐片访问一个局部多边形集合编排出的启用叶片片段。
 *
 * @param[in] polygons 当前叶节点内的输入多边形集合。
 * @param[in,out] userData 透传给 `visitor` 的调用方状态。
 * @param[in] visitor 接收每个启用片段的回调。
 * @return 产生的启用叶片片段数量。
 * @throws std::runtime_error 当局部 BSP 切分流程破坏几何不变量时抛出。
 * @pre `polygons` 来自 `BoolProblem::solve(sceneAABB)` 已验证输入或其有效裁剪结果。
 */
std::size_t visitLeafArrangementFragments(
    const std::vector<Polygon256> &polygons,
    void *userData,
    LeafArrangementFragmentVisitor visitor);
}
