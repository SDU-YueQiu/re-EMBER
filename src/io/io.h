/**
 * @file io.h
 * @brief 声明 OBJ/STL 网格数据和多边形集合 I/O API。
 */
#pragma once

#include "geometry/aabb.h"
#include "geometry/geometry256.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ember
{
/**
 * @brief OBJ 顶点位置。
 *
 * 第一版 I/O 只关心几何位置，不保留法线、UV 或材质属性。
 */
struct ObjVertex
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * @brief 轻量面网格数据。
 *
 * 历史命名沿用 `ObjMeshData`，但该结构也复用于 STL 等“仅几何位置 + 面索引”
 * 的外层 I/O 边界。`faces` 中每个元素保存一个面按输入顺序给出的顶点位置下标，
 * 下标均为 0-based，且仅对应 `vertices`。
 */
struct ObjMeshData
{
    std::vector<ObjVertex> vertices;
    std::vector<std::vector<std::size_t>> faces;
};

/**
 * @brief 浮点 OBJ 量化到整数域时的配置。
 *
 * 若 `explicitScale` 为空，则导入器会为一组输入网格自动选择共享的十进制缩放因子。
 */
struct QuantizeOptions
{
    std::optional<std::uint64_t> explicitScale;
};

/**
 * @brief 面片集合构建策略。
 *
 * 默认保持严格导入：每个 OBJ 面必须在量化后直接构造成一个合法 `Polygon256`。
 */
struct PolygonSoupBuildOptions
{
    /**
     * @brief 将量化后非共面的 n 边面按输入顶点顺序扇形三角化。
     *
     * 该选项只处理“量化后非共面”这一类失败；重复点、退化、非严格凸等错误仍会失败。
     * 已经共面的 n 边面会原样保留为单个 `Polygon256`，不会被三角化。
     */
    bool triangulateNonCoplanarFaces = false;
};

/**
 * @brief 多边形集合导出前的可选拓扑恢复策略。
 */
enum class PolygonSoupTopologyMode
{
    /**
     * @brief 原样导出 `BoolProblem::resultFragments()` 恢复出的 polygon soup。
     */
    Raw,

    /**
     * @brief 只补齐落在其他面边界上的已有顶点，消除导出网格中的 T 形连接。
     */
    Conforming,

    /**
     * @brief 先补齐 T 形连接，再保守合并仍为凸多边形的同向共面相邻面。
     */
    ConformingMergeConvex
};

/**
 * @brief 多边形集合导出配置。
 */
struct PolygonSoupExportOptions
{
    /**
     * @brief 将整数坐标反量化回输出坐标的共享尺度。
     */
    std::uint64_t coordinateScale = 1;

    /**
     * @brief 导出前执行的拓扑恢复策略。
     */
    PolygonSoupTopologyMode topologyMode = PolygonSoupTopologyMode::Raw;
};

/**
 * @brief 返回导出拓扑策略的 CLI/日志名称。
 */
const char *toString(PolygonSoupTopologyMode mode) noexcept;

/**
 * @brief 从 CLI 字符串解析导出拓扑策略。
 */
bool parsePolygonSoupTopologyMode(const std::string &token, PolygonSoupTopologyMode &outMode) noexcept;

/**
 * @brief 读取仅包含几何位置的 OBJ 数据。
 *
 * @param[in] path OBJ 文件路径。
 * @param[out] outMesh 成功时写入顶点和面索引。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 读取成功。
 * @retval false 文件无法打开，或 `v` / `f` 语法不满足当前导入约束。
 * @note 当前接受 `f v`、`f v/vt`、`f v//vn`、`f v/vt/vn`，但只使用位置索引。
 */
bool readObjMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError);

/**
 * @brief 读取仅包含几何位置的 STL 数据。
 *
 * @param[in] path STL 文件路径。
 * @param[out] outMesh 成功时写入顶点和三角面索引。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 读取成功。
 * @retval false 文件无法打开，或 ASCII / binary STL 语法不满足当前导入约束。
 * @note 当前导入器接受 ASCII STL 和 binary STL，并忽略 facet normal / attribute byte count。
 */
bool readStlMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError);

/**
 * @brief 按文件扩展名读取几何网格数据。
 *
 * @param[in] path 输入文件路径。
 * @param[out] outMesh 成功时写入顶点和面索引。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 读取成功。
 * @retval false 扩展名不受支持，或对应格式解析失败。
 * @note 当前仅支持 `.obj` 和 `.stl`。
 */
bool readMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError);

/**
 * @brief 为一组输入网格选择统一的量化尺度。
 *
 * @param[in] meshes 需要共享整数坐标系的输入网格集合。
 * @param[in] options 显式或自动量化配置。
 * @param[out] outScale 成功时写入共享缩放因子。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 成功选出共享缩放因子。
 * @retval false 显式缩放非法，或自动缩放下输入坐标超出当前 26-bit 约束。
 */
bool chooseSharedScale(
    const std::vector<ObjMeshData> &meshes,
    const QuantizeOptions &options,
    std::uint64_t &outScale,
    std::string &outError);

/**
 * @brief 按共享 scale 把输入顶点范围转换为覆盖输入的整数 AABB。
 *
 * @param[in] mesh 输入网格数据。
 * @param[in] sharedScale 调用方为全部输入统一选定的量化尺度。
 * @param[out] outAABB 成功时写入 `floor(coord * scale)` / `ceil(coord * scale)` 覆盖的输入 AABB。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 成功得到覆盖输入顶点的整数 AABB。
 * @retval false `sharedScale` 非法、输入顶点为空，或顶点坐标非法。
 * @pre `sharedScale > 0`。
 */
bool computeScaledMeshAABB(
    const ObjMeshData &mesh,
    std::uint64_t sharedScale,
    AABB3i &outAABB,
    std::string &outError);

/**
 * @brief 将一个输入网格转换为 `Polygon256` 多边形集合。
 *
 * @param[in] mesh 输入网格数据。
 * @param[in] sharedScale 调用方为全部输入统一选定的量化尺度。
 * @param[out] outPolygons 成功时写入转换后的凸多边形集合。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 所有面都成功量化并构造成合法 `Polygon256`。
 * @retval false 任一面在量化后退化、非共面、非严格凸或不满足当前几何约束。
 * @pre `sharedScale > 0`。
 */
bool buildPolygonSoup(
    const ObjMeshData &mesh,
    std::uint64_t sharedScale,
    std::vector<Polygon256> &outPolygons,
    std::string &outError);

/**
 * @brief 按指定策略将一个输入网格转换为 `Polygon256` 多边形集合。
 *
 * @param[in] mesh 输入网格数据。
 * @param[in] sharedScale 调用方为全部输入统一选定的量化尺度。
 * @param[in] options 多边形集合构建策略。
 * @param[out] outPolygons 成功时写入转换后的凸多边形集合。
 * @param[out] outError 失败时写入可读错误信息。
 * @retval true 所有面都成功量化并构造成合法 `Polygon256`。
 * @retval false 任一面在量化后不满足当前几何约束，且不能被所选策略处理。
 * @pre `sharedScale > 0`。
 */
bool buildPolygonSoup(
    const ObjMeshData &mesh,
    std::uint64_t sharedScale,
    const PolygonSoupBuildOptions &options,
    std::vector<Polygon256> &outPolygons,
    std::string &outError);

/**
 * @brief 将布尔结果多边形集合直接导出为 OBJ n 边面。
 *
 * @param[in] fragments 待导出的结果面集合。
 * @param[in] path 输出 OBJ 路径。
 * @param[out] outFaceCount 成功时写入导出的面数。
 * @param[out] outError 失败时写入可读错误信息。
 * @param[in] coordinateScale 将整数坐标反量化回 OBJ 坐标的共享尺度。
 * @retval true 导出成功。
 * @retval false 结果面无法恢复有序顶点，或输出文件无法写入。
 * @note 当前导出保持多边形集合形态，不做三角化、拓扑缝合或 T 形连接恢复。
 */
bool writePolygonSoupObj(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    std::uint64_t coordinateScale = 1);

/**
 * @brief 将布尔结果多边形集合按指定拓扑策略导出为 OBJ n 边面。
 */
bool writePolygonSoupObj(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    const PolygonSoupExportOptions &options);

/**
 * @brief 将布尔结果多边形集合导出为 STL 三角面。
 *
 * @param[in] fragments 待导出的结果面集合。
 * @param[in] path 输出 STL 路径。
 * @param[out] outFaceCount 成功时写入导出的三角面数。
 * @param[out] outError 失败时写入可读错误信息。
 * @param[in] coordinateScale 将整数坐标反量化回 STL 坐标的共享尺度。
 * @retval true 导出成功。
 * @retval false 结果面无法恢复有序顶点，或输出文件无法写入。
 * @note STL 不支持 n 边面；当前导出器会按每个 `Polygon256` 的边顺序做扇形三角化。
 */
bool writePolygonSoupStl(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    std::uint64_t coordinateScale = 1);

/**
 * @brief 将布尔结果多边形集合按指定拓扑策略导出为 STL 三角面。
 */
bool writePolygonSoupStl(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    const PolygonSoupExportOptions &options);

/**
 * @brief 按输出扩展名导出多边形集合。
 *
 * @param[in] fragments 待导出的结果面集合。
 * @param[in] path 输出路径。
 * @param[out] outFaceCount 成功时写入导出面数。
 * @param[out] outError 失败时写入可读错误信息。
 * @param[in] coordinateScale 将整数坐标反量化回输出坐标的共享尺度。
 * @retval true 导出成功。
 * @retval false 扩展名不受支持，或对应格式导出失败。
 * @note `.obj` 保持 n 边面，`.stl` 会三角化导出。
 */
bool writePolygonSoupMesh(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    std::uint64_t coordinateScale = 1);

/**
 * @brief 按输出扩展名和指定拓扑策略导出多边形集合。
 */
bool writePolygonSoupMesh(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    const PolygonSoupExportOptions &options);

/**
 * @brief 将 `Polygon256` 多边形集合转换为 OBJ 风格的 n 边面网格。
 *
 * 该函数按 `Polygon256` 的边平面顺序恢复每个面的有序顶点，并保留原始
 * 多边形集合形态。
 *
 * @param[in] fragments 待转换的结果面集合。
 * @param[out] outMesh 成功时写入顶点和 n 边面索引。
 * @param[out] outError 失败时写入可读错误信息。
 * @param[in] coordinateScale 将整数坐标反量化回 OBJ 坐标的共享尺度。
 * @retval true 转换成功。
 * @retval false 任一结果面无法恢复唯一有限顶点。
 * @note 输出坐标会按齐次点的 `x/w`、`y/w`、`z/w` 近似恢复为十进制双精度。
 */
bool buildObjMeshFromPolygonSoup(
    const std::vector<Polygon256> &fragments,
    ObjMeshData &outMesh,
    std::string &outError,
    std::uint64_t coordinateScale = 1);

/**
 * @brief 按指定拓扑策略将 `Polygon256` 多边形集合转换为 OBJ 风格网格。
 */
bool buildObjMeshFromPolygonSoup(
    const std::vector<Polygon256> &fragments,
    ObjMeshData &outMesh,
    std::string &outError,
    const PolygonSoupExportOptions &options);
}
