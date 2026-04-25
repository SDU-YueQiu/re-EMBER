#pragma once

#include "algorithm/bsp.h"

#include <cstddef>
#include <vector>

namespace ember
{
    enum BoolOp
    {
        Union,
        Intersection,
        Difference
    };

    // 布尔问题求解
    class BoolProblem
    {
    public:
        BoolProblem() noexcept = default;
        ~BoolProblem() noexcept = default;

        void addPolygon(const Polygon256 &polygon);
        void setPolygons(const std::vector<Polygon256> &polygons);

        void buildTrees();
    private:
        bool isLeaf = false;
        BoolOp op;
        std::vector<Polygon256> polygons;
        std::vector<BSPTree> trees;
    };

}
