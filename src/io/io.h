#pragma once

#include "geometry/geometry256.h"

#include <array>
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
     * @brief OBJ 原始网格数据。
     *
     * `faces` 中每个元素保存一个面按 OBJ 顺序给出的顶点位置下标。
     * 下标均为 0-based，且仅对应 `vertices`。
     */
    struct ObjMeshData
    {
        std::vector<ObjVertex> vertices;
        std::vector<std::vector<std::size_t>> faces;
    };

    /**
     * @brief 可直接用于显示或三角网格算法的三角面网格数据。
     *
     * `triangles` 中每个元素保存一个三角形的 0-based 顶点下标，
     * 且仅对应 `vertices`。
     */
    struct TriangleMeshData
    {
        std::vector<ObjVertex> vertices;
        std::vector<std::array<std::size_t, 3>> triangles;
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
     * @brief 读取几何-only OBJ 数据。
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
     * @brief 为一组输入 OBJ 选择统一的量化尺度。
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
     * @brief 将一个 OBJ 网格转换为 `Polygon256` polygon soup。
     *
     * @param[in] mesh 输入 OBJ 数据。
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
     * @brief 将布尔结果 polygon soup 直接导出为 OBJ `n-gon`。
     *
     * @param[in] fragments 待导出的结果面集合。
     * @param[in] path 输出 OBJ 路径。
     * @param[out] outFaceCount 成功时写入导出的面数。
     * @param[out] outError 失败时写入可读错误信息。
     * @retval true 导出成功。
     * @retval false 结果面无法恢复有序顶点，或输出文件无法写入。
     * @note 当前导出保持 polygon soup 形态，不做三角化、拓扑缝合或 T-junction 恢复。
     */
    bool writePolygonSoupObj(
        const std::vector<Polygon256> &fragments,
        const std::string &path,
        std::size_t &outFaceCount,
        std::string &outError);

    /**
     * @brief 将 `Polygon256` polygon soup 转换为可显示的三角网格。
     *
     * 该函数先按 `Polygon256` 的边平面顺序恢复每个面的有序顶点，
     * 再按凸扇方式将每个 `n-gon` 三角化。
     *
     * @param[in] fragments 待转换的结果面集合。
     * @param[out] outMesh 成功时写入顶点和三角面索引。
     * @param[out] outError 失败时写入可读错误信息。
     * @retval true 转换成功。
     * @retval false 任一结果面无法恢复唯一有限顶点，或三角化前不满足当前几何约束。
     * @note 输出坐标会按齐次点的 `x/w`、`y/w`、`z/w` 近似恢复为十进制双精度。
     */
    bool buildTriangleMeshFromPolygonSoup(
        const std::vector<Polygon256> &fragments,
        TriangleMeshData &outMesh,
        std::string &outError);
}
