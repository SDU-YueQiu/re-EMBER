#pragma once

#include "geometry/geometry256.h"
#include <vector>
#include "math/winding_number_f.h"

namespace ember
{
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
        BOUNDARY_HIT,     ///< 路径与某多边形边界相交，无法唯一确定穿越事件。
        END_AT_POLYGON,   ///< 路径终点落在某多边形内部。
        INVALID_SEGMENT,  ///< 路径中的某段退化或不满足 `Segment256` 约束。
        INVALID_POLYGON,  ///< 输入多边形退化或不满足 `Polygon256` 约束。
        START_AT_POLYGON, ///< 路径起点落在某多边形内部。
        FAIL              ///< 未预期失败情况
    };

    /**
     * @brief 按给定路径从参考点传播 WNV。
     *
     * 该过程需要对路径中的每一段与所有 `polygons` 做相交测试，开销相对较大。
     * 计算同一平面前后两侧的 WNV 时，建议只调用一次本函数，再结合平面法向区分
     * `Wf` 和 `Wb`，而不是调用两遍路径重复追踪。
     *
     * 注意因为底层屎山，这里有些返回值还没发判断，需要在底层重构之后才能返回
     *
     * @param[in] refpoint 已知 WNV 的参考点。
     * @param[in] path 用于传播的路径。
     * @param[in] polygons 参与更新 WNV 的多边形集合。
     * @param[out] targetWNV 成功时写入传播后的 WNV；仅在返回 `SUCCESS` 时有效。
     *
     * @retval SUCCESS 成功完成传播，结果已写入 `targetWNV`。
     * @retval BOUNDARY_HIT 路径与某多边形边界相交，无法唯一确定穿越事件。
     * @retval END_AT_POLYGON 路径终点落在某多边形内部。
     * @retval INVALID_SEGMENT 路径中的某段退化或不满足 `Segment256` 约束。
     * @retval INVALID_POLYGON 输入多边形退化或不满足 `Polygon256` 约束。
     * @retval START_AT_POLYGON 路径起点落在某多边形内部。
     * @retval FAIL 未预期失败情况
     *
     * @todo 重构几何底层后能正常返回失败原因
     */
    traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV);
}
