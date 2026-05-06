/**
 * @file geometry256.h
 * @brief 声明精确 256 位线、线段和凸多边形 primitive。
 */
#pragma once

#include "math/math256.h"
#include "geometry/plane_geometry256.h"
#include "math/winding_number_f.h"

#include <cstddef>
#include <vector>

namespace ember
{
    enum class PolygonEdgeProvenance
    {
        Regular,
        SubdivisionClip
    };

    struct Line256
    {
        Plane3i p1, p2;

        Line256() noexcept = default;
        Line256(const Plane3i &first, const Plane3i &second) noexcept : p1(first), p2(second) {}

        Vec3i directionVector() const noexcept
        {
            return cross(p1.normal(), p2.normal());
        }

        bool isValid() const noexcept
        {
            const Vec3i d = directionVector();
            return !isZero(d.x) || !isZero(d.y) || !isZero(d.z);
        }

        bool contains(const HomPoint4i &x) const noexcept
        {
            return x.classify(p1) == 0 && x.classify(p2) == 0;
        }

        bool contains(const PlanePoint3i &x) const noexcept
        {
            return contains(x.x);
        }
    };

    // 法向要求：线段内侧为负，也即起点和终点平面法向分别向外。
    struct Segment256
    {
        Plane3i start;
        Plane3i end;

        Line256 direction;

        Segment256() noexcept = default;
        Segment256(const Plane3i &startPlane, const Plane3i &endPlane, const Line256 &directionLine) noexcept;

        const PlanePoint3i &getStartPointRef() const noexcept
        {
            return startPointCache_;
        }

        const PlanePoint3i &getEndPointRef() const noexcept
        {
            return endPointCache_;
        }

        bool isValid() const noexcept
        {
            const PlanePoint3i &s = startPointCache_;
            const PlanePoint3i &e = endPointCache_;
            if (!s.hasUniqueIntersection() || !e.hasUniqueIntersection())
                return false;

            // 注意：此处把法向错误和起终点共面一起判断。
            if ((s.classify(end) != -1) || (e.classify(start) != -1))
                return false;

            return true;
        }

    private:
        void refreshEndpointCache() noexcept
        {
            startPointCache_ = PlanePoint3i(direction.p1, direction.p2, start);
            endPointCache_ = PlanePoint3i(direction.p1, direction.p2, end);
        }

        PlanePoint3i startPointCache_;
        PlanePoint3i endPointCache_;
    };

    struct Polygon256
    {
        Plane3i plane; // 支撑平面

        // 裁剪要求边平面法向必须指向多边形外侧
        std::vector<Plane3i> edgePlanes;
        std::vector<PolygonEdgeProvenance> edgeProvenances;

        // 环绕数转换向量
        std::vector<int> WNTV;

        Polygon256() = default;
        Polygon256(const Plane3i &supportPlane, std::vector<Plane3i> edges);
        Polygon256(
            const Plane3i &supportPlane,
            std::vector<Plane3i> edges,
            std::vector<PolygonEdgeProvenance> provenances);

        // 该方法对法向不封闭
        void addEdgePlane(
            const Plane3i &edge,
            PolygonEdgeProvenance provenance = PolygonEdgeProvenance::Regular);

        /**
         * @brief 标记当前顶点缓存失效。
         *
         * @note 只有在调用方直接改写 `plane` 或 `edgePlanes` 时才需要手动调用；
         *       常规构造函数和 `addEdgePlane()` 会自己维护缓存状态。
         */
        void invalidateVertexCache() noexcept;

        /**
         * @brief 立即重建一次按边顺序排列的顶点缓存。
         *
         * @return 所有缓存顶点都有唯一交点时返回 `true`。
         */
        bool precomputeVertices() const noexcept;

        /**
         * @brief 读取指定下标的缓存顶点。
         *
         * @param[in] vertexIndex 0-based 顶点下标。
         * @return 顶点越界时返回一个无唯一交点的空点引用。
         */
        const PlanePoint3i &vertex(std::size_t vertexIndex) const noexcept;

        /**
         * @brief 读取当前多边形按边顺序缓存的全部顶点。
         *
         * @return 顶点顺序满足 `v_i = (plane, edge_i, edge_{i-1})`。
         */
        const std::vector<PlanePoint3i> &vertices() const noexcept;
        std::size_t edgeCount() const noexcept;
        PolygonEdgeProvenance edgeProvenance(std::size_t edgeIndex) const noexcept;

        bool isValid() const noexcept;

        // 临时返回约定：
        // -2: 输入点无唯一交点，属于数据错误
        // -1/+1: 点不在多边形内，且分别位于支撑平面负侧/正侧
        //  0: 点在多边形内或边界上
        //  2: 点在支撑平面上，但不在多边形内
        int classify(const PlanePoint3i &point) const noexcept;

        bool containsStrictly(const PlanePoint3i &point) const noexcept;
        bool containsOrOnBoundary(const PlanePoint3i &point) const noexcept;
        bool findStrictInteriorPoint(PlanePoint3i &outPoint) const noexcept;

    private:
        void rebuildVertexCache(bool force) const noexcept;

        mutable std::vector<PlanePoint3i> vertexCache_;
        mutable bool vertexCacheDirty_ = true;
    };

    inline Line256 makeLine(const Plane3i &p1, const Plane3i &p2) noexcept
    {
        return Line256(p1, p2);
    }

    // 返回直线与平面的交点
    inline PlanePoint3i intersect(const Line256 &line, const Plane3i &plane) noexcept
    {
        return PlanePoint3i(line.p1, line.p2, plane);
    }

    bool areParallel(const Line256 &lhs, const Line256 &rhs) noexcept;

    /**
     * @brief 求线段和多边形的交点
     * 
     * 并不区分严格内部，交在边上也行，但只能有一个交点；若存在一段重叠则返回 `false`。
     * 
     * @param seg 输入线段
     * @param poly 输入多边形
     * @param[out] outPoint 输出交点坐标（如果有交点）
     * @return 如果有交点返回 `true`，否则返回 `false`。
     * 
     * @todo 添加交在不同地方的不同返回状态
     */
    bool intersectionSegmentPolygon(const Segment256 &seg, const Polygon256 &poly, PlanePoint3i &outPoint);
}

