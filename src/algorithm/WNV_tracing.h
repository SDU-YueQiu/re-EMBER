/**
 * @file WNV_tracing.h
 * @brief 声明 WNV 追踪 API 与可信内部追踪入口。
 */
#pragma once

#include "geometry/geometry256.h"
#include <vector>
#include "math/winding_number_f.h"

namespace ember
{
struct BoolSolveMetrics;

/// 由若干 `Segment256` 组成的传播路径。
typedef std::vector<Segment256> Path;

/// WNV 传播的参考点及其已知环绕数向量。
struct refPoint
{
    PlanePoint3i point;
    WNV wnv;

    refPoint(const PlanePoint3i &p, const WNV &w) : point(p), wnv(w) {}
};

/// 路径追踪结果状态。
enum traceStatus
{
    SUCCESS,          ///< 成功完成传播，结果已写入 `targetWNV`。
    PATH_INVALID,     ///< 路径无效，无法完成传播。
    INPUT_INVALID,    ///< 输入多边形或路径无效，包括多边形、线段无效或路径不连续。
    FAIL              ///< 未预期失败情况
};

/**
 * @brief 按给定路径从参考点传播 WNV。
 *
 * 该过程需要对路径中的每一段与所有 `polygons` 做相交测试，开销相对较大。
 * 计算同一平面前后两侧的 WNV 时，建议只调用一次本函数，再结合平面法向区分
 * `Wf` 和 `Wb`，而不是调用两遍路径重复追踪。
 *
 * 注意因为状态过于复杂难以调试且新建路径开销可接受，数学层面可解的点落在多边形
 * 内部或者线段落一部分落在多边形内部本函数不处理。
 *
 *
 * @param[in] refpoint 已知 WNV 的参考点。
 * @param[in] path 用于传播的路径，需要按顺序的前后连接的线段数组。
 * @param[in] polygons 参与更新 WNV 的多边形集合。
 * @param[out] targetWNV 成功时写入传播后的 WNV；仅在返回 `SUCCESS` 时有效。
 *
 * @retval SUCCESS 成功完成传播，结果已写入 `targetWNV`。
 * @retval PATH_INVALID 路径无法完成传播，需要重新选择路径。
 * @retval INPUT_INVALID 输入多边形或路径中线段不合法或路径不连续、起点不是参考点等不合法问题。
 * @retval FAIL 未预期失败情况
 *
 */
traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV);

/**
 * @brief 按给定路径传播到位于曲面内部的目标点，并计算其前后两侧 WNV。
 *
 * @param[in] refpoint 已知 WNV 的参考点。
 * @param[in] path 从 `refpoint.point` 到目标曲面点的传播路径。
 * @param[in] polygons 当前子问题中的输入多边形集合。
 * @param[in] referencePlane 目标曲面点所属参考曲面的支撑平面。
 * @param[out] frontWNV 成功时写入目标点法向前侧的 WNV。
 * @param[out] backWNV 成功时写入目标点法向后侧的 WNV。
 *
 * @retval SUCCESS 成功计算出 `frontWNV` 与 `backWNV`。
 * @retval PATH_INVALID 路径在传播意义上无效，需要重新选择路径。
 * @retval INPUT_INVALID 输入路径、多边形或参考平面不合法。
 * @retval FAIL 未预期失败情况。
 *
 * @note 该接口实现论文 3.4 节中“终点 `y` 落在曲面上”的修正规则。
 */
traceStatus tracePathWNVToSurfacePoint(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV);

namespace detail
{
/**
 * @brief 在调用方已验证多边形集合和路径的前提下传播 WNV。
 *
 * @pre `polygons` 均为有效 polygon，且其 `WNTV` 维度与 `refpoint.wnv` 一致。
 * @pre `path` 为空，或由有效线段组成并从 `refpoint.point` 连续连接到目标点。
 */
traceStatus tracePathWNVTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV,
    BoolSolveMetrics *solveMetrics = nullptr);

/**
 * @brief 在调用方已验证输入的前提下，允许 subdivision 裁剪边的单点横穿继续传播 WNV。
 *
 * @pre `polygons` 均为有效 polygon，且其 `WNTV` 维度与 `refpoint.wnv` 一致。
 * @pre `path` 为空，或由有效线段组成并从 `refpoint.point` 连续连接到目标点。
 */
traceStatus tracePathWNVAllowSubdivisionClipCrossingTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    WNV &targetWNV,
    BoolSolveMetrics *solveMetrics = nullptr);

/**
 * @brief 在调用方已验证多边形集合和路径的前提下传播到曲面目标点。
 *
 * @pre `polygons` 均为有效 polygon，且其 `WNTV` 维度与 `refpoint.wnv` 一致。
 * @pre `path` 非空、线段有效、从 `refpoint.point` 连续连接到曲面目标点。
 */
traceStatus tracePathWNVToSurfacePointTrusted(
    const refPoint &refpoint,
    const Path &path,
    const std::vector<Polygon256> &polygons,
    const Plane3i &referencePlane,
    WNV &frontWNV,
    WNV &backWNV,
    BoolSolveMetrics *solveMetrics = nullptr);
}
}

