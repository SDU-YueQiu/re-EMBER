/**
 * @file leaf_arrangement.cpp
 * @brief 为多边形集合构建叶子局部 BSP 编排。
 */
#include "leaf_arrangement.h"

#include "algorithm/bsp.h"
#include "core/perf_tracing.h"

#include <iterator>

namespace ember
{
    namespace
    {
        enum class LeafPairRelationKind
        {
            None,
            Segment,
            Coplanar
        };

        struct LeafPairRelation
        {
            LeafPairRelationKind kind = LeafPairRelationKind::None;
            detail::IntersectionCarrier lhsCarrier;
            detail::IntersectionCarrier rhsCarrier;
        };

        bool areCoplanarPolygonsForPairCache(const Polygon256 &lhs, const Polygon256 &rhs) noexcept
        {
            if (!arePlaneNormalsParallel(lhs.plane, rhs.plane))
            {
                return false;
            }

            const PlanePoint3i vertex(lhs.plane, lhs.edgePlanes[0], lhs.edgePlanes[1]);
            return vertex.hasUniqueIntersection() && vertex.classify(rhs.plane) == 0;
        }

        LeafPairRelation buildLeafPairRelation(const Polygon256 &lhs, const Polygon256 &rhs)
        {
            LeafPairRelation relation;
            if (detail::computeBidirectionalPolygonIntersectionCarriersTrusted(
                    lhs,
                    rhs,
                    relation.lhsCarrier,
                    relation.rhsCarrier))
            {
                relation.kind = LeafPairRelationKind::Segment;
                return relation;
            }

            Plane3i lhsSplitPlane;
            Plane3i lhsV0;
            Plane3i lhsV1;
            Plane3i rhsSplitPlane;
            Plane3i rhsV0;
            Plane3i rhsV1;
            if (detail::computePolygonIntersectionCarrierTrusted(lhs, rhs, lhsSplitPlane, lhsV0, lhsV1) &&
                detail::computePolygonIntersectionCarrierTrusted(rhs, lhs, rhsSplitPlane, rhsV0, rhsV1))
            {
                relation.kind = LeafPairRelationKind::Segment;
                relation.lhsCarrier.splitPlane = lhsSplitPlane;
                relation.lhsCarrier.v0 = lhsV0;
                relation.lhsCarrier.v1 = lhsV1;
                relation.rhsCarrier.splitPlane = rhsSplitPlane;
                relation.rhsCarrier.v0 = rhsV0;
                relation.rhsCarrier.v1 = rhsV1;
                return relation;
            }

            if (areCoplanarPolygonsForPairCache(lhs, rhs))
            {
                relation.kind = LeafPairRelationKind::Coplanar;
            }

            return relation;
        }
    }

    std::vector<Polygon256> buildLeafArrangement(const std::vector<Polygon256> &polygons)
    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement");

        std::vector<Polygon256> fragments;
        const std::size_t polygonCount = polygons.size();
        if (polygonCount < 8u)
        {
            for (std::size_t i = 0; i < polygonCount; ++i)
            {
                BSPTree tree;
                tree.setBasePolygon(polygons[i], i);
                for (std::size_t j = 0; j < polygonCount; ++j)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    tree.insertTrusted(polygons[j], j);
                }

                std::vector<Polygon256> localFragments = tree.collectLeafGeometries();
                fragments.insert(
                    fragments.end(),
                    std::make_move_iterator(localFragments.begin()),
                    std::make_move_iterator(localFragments.end()));
            }
            return fragments;
        }

        std::vector<std::vector<LeafPairRelation>> pairRelations(
            polygonCount,
            std::vector<LeafPairRelation>(polygonCount));
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            for (std::size_t j = i + 1; j < polygonCount; ++j)
            {
                pairRelations[i][j] = buildLeafPairRelation(polygons[i], polygons[j]);
            }
        }

        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            BSPTree tree;
            tree.setBasePolygon(polygons[i], i);
            for (std::size_t j = 0; j < polygonCount; ++j)
            {
                if (i == j)
                {
                    continue;
                }

                const LeafPairRelation &relation = (i < j) ? pairRelations[i][j] : pairRelations[j][i];
                if (relation.kind == LeafPairRelationKind::Segment)
                {
                    const detail::IntersectionCarrier &carrier = (i < j) ? relation.lhsCarrier : relation.rhsCarrier;
                    tree.addSegment(carrier.v0, carrier.v1, carrier.splitPlane);
                    continue;
                }

                if (relation.kind == LeafPairRelationKind::Coplanar)
                {
                    tree.insertCoplanarPolygonTrusted(polygons[j], j);
                }
            }

            std::vector<Polygon256> localFragments = tree.collectLeafGeometries();
            fragments.insert(
                fragments.end(),
                std::make_move_iterator(localFragments.begin()),
                std::make_move_iterator(localFragments.end()));
        }
        return fragments;
    }
}
