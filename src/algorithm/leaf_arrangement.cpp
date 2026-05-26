/**
 * @file leaf_arrangement.cpp
 * @brief 为多边形集合构建叶子局部 BSP 编排。
 */
#include "leaf_arrangement.h"

#include "algorithm/bsp.h"
#include "core/perf_tracing.h"
#include "geometry/polygon_ops.h"

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

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
    PolygonHandle baseIndex;
    LeafPairRelationKind kind = LeafPairRelationKind::None;
    PolygonHandle polygonIndex;
    PlaneHandle splitPlane;
    PlaneHandle v0;
    PlaneHandle v1;
    std::size_t next = 0;
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

PolygonHandle makePolygonHandle(std::size_t index)
{
    if (index >= static_cast<std::size_t>(kInvalidPolygonHandleIndex))
        throw std::runtime_error("visitLeafArrangementFragments polygon handle range exhausted.");
    return PolygonHandle(static_cast<std::uint32_t>(index));
}

std::size_t polygonHandleIndex(PolygonHandle handle)
{
    if (!handle.isValid())
        throw std::runtime_error("visitLeafArrangementFragments received an invalid polygon handle.");
    return static_cast<std::size_t>(handle.index);
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

    constexpr std::size_t kNoInsertion = std::numeric_limits<std::size_t>::max();
    std::vector<LeafArrangementInsertion> adjacency;
    std::vector<std::size_t> adjacencyHeads(polygonCount, kNoInsertion);
    std::vector<std::size_t> adjacencyTails(polygonCount, kNoInsertion);
    PlanePalette carrierPlanes;
    adjacency.reserve(polygonCount * 4u);
    auto appendAdjacency = [&](LeafArrangementInsertion insertion)
    {
        const std::size_t baseIndex = polygonHandleIndex(insertion.baseIndex);
        insertion.next = kNoInsertion;
        const std::size_t insertionIndex = adjacency.size();
        adjacency.push_back(std::move(insertion));
        if (adjacencyTails[baseIndex] == kNoInsertion)
            adjacencyHeads[baseIndex] = insertionIndex;
        else
            adjacency[adjacencyTails[baseIndex]].next = insertionIndex;
        adjacencyTails[baseIndex] = insertionIndex;
    };

    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement::pairRelationAdjacency");
        for (std::size_t i = 0; i < polygonCount; ++i)
        {
            for (std::size_t j = i + 1; j < polygonCount; ++j)
            {
                LeafPairRelation relation = buildLeafPairRelation(polygons[i], polygons[j]);
                if (relation.kind == LeafPairRelationKind::Segment)
                {
                    appendAdjacency(LeafArrangementInsertion{
                        makePolygonHandle(i),
                        LeafPairRelationKind::Segment,
                        makePolygonHandle(j),
                        carrierPlanes.intern(relation.lhsCarrier.splitPlane),
                        carrierPlanes.intern(relation.lhsCarrier.v0),
                        carrierPlanes.intern(relation.lhsCarrier.v1),
                        kNoInsertion});
                    appendAdjacency(LeafArrangementInsertion{
                        makePolygonHandle(j),
                        LeafPairRelationKind::Segment,
                        makePolygonHandle(i),
                        carrierPlanes.intern(relation.rhsCarrier.splitPlane),
                        carrierPlanes.intern(relation.rhsCarrier.v0),
                        carrierPlanes.intern(relation.rhsCarrier.v1),
                        kNoInsertion});
                }
                else if (relation.kind == LeafPairRelationKind::Coplanar)
                {
                    appendAdjacency(LeafArrangementInsertion{
                        makePolygonHandle(i),
                        LeafPairRelationKind::Coplanar,
                        makePolygonHandle(j),
                        PlaneHandle(),
                        PlaneHandle(),
                        PlaneHandle(),
                        kNoInsertion});
                    appendAdjacency(LeafArrangementInsertion{
                        makePolygonHandle(j),
                        LeafPairRelationKind::Coplanar,
                        makePolygonHandle(i),
                        PlaneHandle(),
                        PlaneHandle(),
                        PlaneHandle(),
                        kNoInsertion});
                }
            }
        }
    }

    for (std::size_t i = 0; i < polygonCount; ++i)
    {
        REEMBER_PROFILE_ZONE("buildLeafArrangement::basePolygon");
        if (adjacencyHeads[i] == kNoInsertion)
        {
            visitor(userData, Polygon256(polygons[i]));
            ++fragmentCount;
            continue;
        }

        BSPTree tree;
        tree.setBasePolygonForPrecomputedRelations(polygons[i], i);
        for (std::size_t insertionIndex = adjacencyHeads[i];
             insertionIndex != kNoInsertion;
             insertionIndex = adjacency[insertionIndex].next)
        {
            const LeafArrangementInsertion &insertion = adjacency[insertionIndex];
            if (insertion.kind == LeafPairRelationKind::Segment)
            {
                tree.addSegment(carrierPlanes, insertion.v0, insertion.v1, insertion.splitPlane);
                continue;
            }

            if (insertion.kind == LeafPairRelationKind::Coplanar)
            {
                const std::size_t incomingIndex = polygonHandleIndex(insertion.polygonIndex);
                tree.insertCoplanarPolygonTrusted(polygons[incomingIndex], incomingIndex);
            }
        }

        fragmentCount += tree.visitLeafGeometries(userData, visitor);
    }
    return fragmentCount;
}
}
