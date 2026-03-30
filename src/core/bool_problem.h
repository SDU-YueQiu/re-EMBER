#pragma once

#include "algorithm/bsp.h"

#include <cstddef>
#include <vector>

namespace ember
{
    class BoolProblem
    {
    public:
        BoolProblem() noexcept = default;
        ~BoolProblem() noexcept = default;

        void clear() noexcept;
        void addPolygon(const Polygon256& polygon);
        void setPolygons(const std::vector<Polygon256>& polygons);

        std::size_t polygonCount() const noexcept;
        void buildTrees();

        std::vector<Plane3i> computeAABBPlanes() const;

        const std::vector<Polygon256>& polygons() const noexcept;
        const std::vector<BSPTree>& trees() const noexcept;

    private:
        std::vector<Polygon256> polygons_;
        std::vector<BSPTree> trees_;
    };
}

