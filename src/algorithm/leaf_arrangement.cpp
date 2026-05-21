/**
 * @file leaf_arrangement.cpp
 * @brief 为多边形集合构建叶子局部 BSP 编排。
 */
#include "leaf_arrangement.h"

#include "algorithm/bsp.h"
#include "core/perf_tracing.h"
#include "geometry/polygon_ops.h"

#include <algorithm>
#include <stdexcept>

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

struct LeafArrangementInsertion
{
    std::size_t baseIndex = 0;
    LeafPairRelationKind kind = LeafPairRelationKind::None;
    std::size_t polygonIndex = 0;
    detail::IntersectionCarrier carrier;
};

struct VectorAppendVisitorState
{
    std::vector<Polygon256> *fragments = nullptr;
};

void appendFragmentToVector(void *userData, Polygon256 &&fragment)
{
    auto *state = static_cast<VectorAppendVisitorState *>(userData);
    state->fragments->push_back(std::move(fragment));
}

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
    std::vector<Polygon256> fragments;
    VectorAppendVisitorState state{&fragments};
    visitLeafArrangementFragments(polygons, &state, appendFragmentToVector);
    return fragments;
}

std::size_t visitLeafArrangementFragments(
    const std::vector<Polygon256> &polygons,
    void *userData,
    LeafArrangementFragmentVisitor visitor)
{
    REEMBER_PROFILE_ZONE("buildLeafArrangement");

    if (visitor == nullptr)
        throw std::runtime_error("visitLeafArrangementFragments received a null visitor.");

    const std::size_t polygonCount = polygons.size();
    std::size_t fragmentCount = 0u;
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

            fragmentCount += tree.visitLeafGeometries(userData, visitor);
        }
        return fragmentCount;
    }

    std::vector<LeafArrangementInsertion> adjacency;
    adjacency.reserve(polygonCount * 4u);
    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement::pairRelationAdjacency");
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            for (std::size_t j = i + 1; j < polygonCount; ++j)
            {
                const LeafPairRelation relation = buildLeafPairRelation(polygons[i], polygons[j]);
                if (relation.kind == LeafPairRelationKind::Segment)
                {
                    adjacency.push_back(LeafArrangementInsertion{
                        i,
                        LeafPairRelationKind::Segment,
                        j,
                        relation.lhsCarrier});
                    adjacency.push_back(LeafArrangementInsertion{
                        j,
                        LeafPairRelationKind::Segment,
                        i,
                        relation.rhsCarrier});
                }
                else if (relation.kind == LeafPairRelationKind::Coplanar)
                {
                    adjacency.push_back(LeafArrangementInsertion{
                        i,
                        LeafPairRelationKind::Coplanar,
                        j,
                        detail::IntersectionCarrier{}});
                    adjacency.push_back(LeafArrangementInsertion{
                        j,
                        LeafPairRelationKind::Coplanar,
                        i,
                        detail::IntersectionCarrier{}});
                }
            }
        }
    }
    std::sort(
        adjacency.begin(),
        adjacency.end(),
        [](const LeafArrangementInsertion &lhs, const LeafArrangementInsertion &rhs)
        {
            if (lhs.baseIndex != rhs.baseIndex)
                return lhs.baseIndex < rhs.baseIndex;
            return lhs.polygonIndex < rhs.polygonIndex;
        });

    std::vector<std::size_t> adjacencyOffsets(polygonCount + 1u, 0u);
    for (const LeafArrangementInsertion &insertion : adjacency)
        ++adjacencyOffsets[insertion.baseIndex + 1u];
    for (std::size_t i = 1; i < adjacencyOffsets.size(); ++i)
        adjacencyOffsets[i] += adjacencyOffsets[i - 1u];

    for (std::size_t i = 0; i < polygonCount; ++i)
    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement::basePolygon");
        const std::size_t insertionBegin = adjacencyOffsets[i];
        const std::size_t insertionEnd = adjacencyOffsets[i + 1u];
        if (insertionBegin == insertionEnd)
        {
            visitor(userData, Polygon256(polygons[i]));
            ++fragmentCount;
            continue;
        }

        BSPTree tree;
        tree.setBasePolygonForPrecomputedRelations(polygons[i], i);
        for (std::size_t insertionIndex = insertionBegin; insertionIndex < insertionEnd; ++insertionIndex)
        {
            const LeafArrangementInsertion &insertion = adjacency[insertionIndex];
            if (insertion.kind == LeafPairRelationKind::Segment)
            {
                tree.addSegment(insertion.carrier.v0, insertion.carrier.v1, insertion.carrier.splitPlane);
                continue;
            }

            if (insertion.kind == LeafPairRelationKind::Coplanar)
                tree.insertCoplanarPolygonTrusted(polygons[insertion.polygonIndex], insertion.polygonIndex);
        }

        fragmentCount += tree.visitLeafGeometries(userData, visitor);
    }
    return fragmentCount;
}
}
