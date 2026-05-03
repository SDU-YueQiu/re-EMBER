#include "leaf_arrangement.h"

#include "algorithm/bsp.h"

#include <iterator>
#include <sstream>
#include <stdexcept>

namespace ember
{
    namespace
    {
        void validateLeafArrangementInput(const std::vector<Polygon256> &polygons)
        {
            for (std::size_t polygonIndex = 0; polygonIndex < polygons.size(); ++polygonIndex)
            {
                if (!polygons[polygonIndex].isValid())
                {
                    std::ostringstream message;
                    message << "Leaf arrangement received an invalid polygon at index "
                            << polygonIndex
                            << ".";
                    throw std::runtime_error(message.str());
                }
            }
        }
    }

    std::vector<Polygon256> buildLeafArrangement(const std::vector<Polygon256> &polygons)
    {
        validateLeafArrangementInput(polygons);

        std::vector<Polygon256> fragments;
        const std::size_t polygonCount = polygons.size();
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
}
