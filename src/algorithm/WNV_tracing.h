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
     * @retval PATH_INVALID 路径无效，无法完成传播。
     * @retval INPUT_INVALID 输入多边形或路径中线段无效。
     * @retval FAIL 未预期失败情况
     *
     */
    traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV);
}
