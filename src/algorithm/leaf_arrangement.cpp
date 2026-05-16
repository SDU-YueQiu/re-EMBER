/**
 * @file leaf_arrangement.cpp
 * @brief 为多边形集合构建叶子局部 BSP 编排。
 */
#include "leaf_arrangement.h"

#include "algorithm/bsp.h"
#include "core/perf_tracing.h"
#include "geometry/polygon_ops.h"

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

LeafPairRelation buildLeafPairRelation(const Polygon256 &lhs, const Polygon256 &rhs)
{
    REEMBER_PROFILE_ZONE("buildLeafPairRelation");

    LeafPairRelation relation;
    if (!doAABBsOverlap(lhs.aabb(), rhs.aabb()))
    {
        REEMBER_PROFILE_ZONE("buildLeafPairRelation::aabbReject");
        return relation;
    }

    {
        REEMBER_PROFILE_ZONE("buildLeafPairRelation::bidirectionalCarrier");
        if (detail::computeBidirectionalPolygonIntersectionCarriersTrusted(
                    lhs,
                    rhs,
                    relation.lhsCarrier,
                    relation.rhsCarrier))
        {
            relation.kind = LeafPairRelationKind::Segment;
            return relation;
        }
    }

    {
        REEMBER_PROFILE_ZONE("buildLeafPairRelation::coplanarCheck");
        if (areCoplanarPolygons(lhs, rhs))
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
        REEMBER_PROFILE_ZONE("buildLeafArrangement::smallCase");
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            REEMBER_PROFILE_ZONE("buildLeafArrangement::smallCaseBasePolygon");
            BSPTree tree;
            tree.setBasePolygon(polygons[i], i);
            for (std::size_t j = 0; j < polygonCount; ++j)
            {
                if (i == j)
                    continue;

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
    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement::pairRelationMatrix");
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            for (std::size_t j = i + 1; j < polygonCount; ++j)
                pairRelations[i][j] = buildLeafPairRelation(polygons[i], polygons[j]);
        }
    }

    for (std::size_t i = 0; i < polygonCount; ++i)
    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement::basePolygon");
        BSPTree tree;
        tree.setBasePolygon(polygons[i], i);
        for (std::size_t j = 0; j < polygonCount; ++j)
        {
            if (i == j)
                continue;

            const LeafPairRelation &relation = (i < j) ? pairRelations[i][j] : pairRelations[j][i];
            if (relation.kind == LeafPairRelationKind::Segment)
            {
                const detail::IntersectionCarrier &carrier = (i < j) ? relation.lhsCarrier : relation.rhsCarrier;
                tree.addSegment(carrier.v0, carrier.v1, carrier.splitPlane);
                continue;
            }

            if (relation.kind == LeafPairRelationKind::Coplanar)
                tree.insertCoplanarPolygonTrusted(polygons[j], j);
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
