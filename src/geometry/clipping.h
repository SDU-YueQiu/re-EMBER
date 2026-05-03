#pragma once

#include "geometry256.h"

namespace ember
{
    //计算多边形source和裁剪平面clipPlane的交线
    bool computePolygonPlaneIntersection(const Polygon256& source, const Plane3i& target, Plane3i& p0, Plane3i& p1);

    // 计算不共面多边形 target 与 incoming 的交线载体：
    // - outSplitPlane: 作为 BSP 切分平面（使用 incoming.plane）
    // - outV0/outV1: 保留“线段端点来源”信息（当前 BSP addSegment 接口仍是 Plane3i）
    bool computePolygonIntersectionCarrier(
        const Polygon256& target,
        const Polygon256& incoming,
        Plane3i& outSplitPlane,
        Plane3i& outV0,
        Plane3i& outV1);

    namespace detail
    {
        /**
         * @brief 在调用方已验证两个 polygon 的前提下计算交线载体。
         *
         * @pre `target` 与 `incoming` 均满足 `Polygon256::isValid()`。
         */
        bool computePolygonIntersectionCarrierTrusted(
            const Polygon256& target,
            const Polygon256& incoming,
            Plane3i& outSplitPlane,
            Plane3i& outV0,
            Plane3i& outV1);
    }

    bool clipLeafGeometryByPlane(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped);
}
