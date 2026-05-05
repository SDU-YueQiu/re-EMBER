/**
 * @file clipping.h
 * @brief 声明多边形交线载体和裁剪操作。
 */
#pragma once

#include "geometry256.h"

#include <vector>

namespace ember
{
    // 计算源多边形和裁剪平面的交线。
    bool computePolygonPlaneIntersection(const Polygon256& source, const Plane3i& target, Plane3i& p0, Plane3i& p1);

    // 计算不共面目标多边形与新插入多边形的交线载体：
    // - outSplitPlane: 作为 BSP 切分平面（使用新插入多边形的支撑平面）
    // - outV0/outV1: 保留“线段端点来源”信息（当前 BSP addSegment 接口仍使用 Plane3i）
    bool computePolygonIntersectionCarrier(
        const Polygon256& target,
        const Polygon256& incoming,
        Plane3i& outSplitPlane,
        Plane3i& outV0,
        Plane3i& outV1);

    namespace detail
    {
        /**
         * @brief 描述单侧 BSP 插入所需的交线载体。
         */
        struct IntersectionCarrier
        {
            Plane3i splitPlane;
            Plane3i v0;
            Plane3i v1;
        };

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

        /**
         * @brief 一次计算一对多边形在两个方向上的交线载体。
         *
         * @pre `lhs` 与 `rhs` 均满足 `Polygon256::isValid()`。
         */
        bool computeBidirectionalPolygonIntersectionCarriersTrusted(
            const Polygon256& lhs,
            const Polygon256& rhs,
            IntersectionCarrier& outLhsCarrier,
            IntersectionCarrier& outRhsCarrier);

        /**
         * @brief 在调用方已验证 source polygon 的前提下按平面切分叶片几何。
         *
         * @pre `source` 满足 `Polygon256::isValid()`。
         */
        bool clipLeafGeometryByPlaneTrusted(
            const Polygon256& source,
            const Plane3i& clipPlane,
            Polygon256& frontClipped,
            Polygon256& backClipped,
            PolygonEdgeProvenance insertedEdgeProvenance = PolygonEdgeProvenance::Regular);

        /**
         * @brief 使用调用方已缓存的顶点侧分类按平面切分叶片几何。
         *
         * @pre `source` 满足 `Polygon256::isValid()`。
         * @pre `vertexSides[i]` 是第 i 个顶点相对 `clipPlane` 的分类。
         */
        bool clipLeafGeometryByPlaneTrustedWithSides(
            const Polygon256& source,
            const Plane3i& clipPlane,
            const std::vector<int>& vertexSides,
            Polygon256& frontClipped,
            Polygon256& backClipped,
            PolygonEdgeProvenance insertedEdgeProvenance = PolygonEdgeProvenance::Regular);
    }

    bool clipLeafGeometryByPlane(const Polygon256& source, const Plane3i& clipPlane, Polygon256& frontClipped, Polygon256& backClipped);
}
