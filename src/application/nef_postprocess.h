/**
 * @file nef_postprocess.h
 * @brief 声明应用层 CGAL Nef 后处理和 oracle 转换工具。
 */
#pragma once

#include "core/bool_problem.h"
#include "io/io.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Surface_mesh.h>

#include <cstdint>
#include <string>
#include <vector>

namespace ember::app
{
using ExactKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using SurfaceMesh = CGAL::Surface_mesh<ExactKernel::Point_3>;
using NefPolyhedron = CGAL::Nef_polyhedron_3<ExactKernel>;

/**
 * @brief 将精确面网格转换为正则化 Nef 多面体。
 */
NefPolyhedron makeNefFromExactMesh(const ExactMeshData &mesh, const char *label);

/**
 * @brief 将 `Polygon256` 集合转换为正则化 Nef 多面体。
 */
NefPolyhedron makeNefFromPolygons(const std::vector<Polygon256> &polygons, const char *label);

/**
 * @brief 对两个 Nef 多面体执行布尔运算。
 */
NefPolyhedron applyBoolean(const NefPolyhedron &lhs, const NefPolyhedron &rhs, BoolOp op);

/**
 * @brief 将 Nef 多面体转回轻量面网格。
 */
ObjMeshData makeObjMeshData(const NefPolyhedron &nef, std::uint64_t coordinateScale);

/**
 * @brief 对布尔结果片段执行 Nef 正则化并导出为轻量面网格。
 */
bool buildNefPostprocessedMeshFromPolygons(
    const std::vector<Polygon256> &fragments,
    PolygonSoupTopologyMode topologyMode,
    std::uint64_t coordinateScale,
    ObjMeshData &outMesh,
    std::string &outError);

/**
 * @brief 对布尔结果片段执行 Nef 正则化并按扩展名写出。
 */
bool writeNefPostprocessedMesh(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    PolygonSoupTopologyMode topologyMode,
    std::uint64_t coordinateScale,
    std::size_t &outFaceCount,
    std::string &outError);
}
