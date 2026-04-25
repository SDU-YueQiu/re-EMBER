#include "bool_problem.h"

namespace ember
{
    void BoolProblem::addPolygon(const Polygon256 &polygon)
    {
        polygons.push_back(polygon);
    }

    void BoolProblem::setPolygons(const std::vector<Polygon256> &inputPoly)
    {
        polygons = inputPoly;
        trees.clear();
    }

    void BoolProblem::buildTrees()
    {
        // 仅空间划分后的叶节点执行
        const std::size_t n = polygons.size();
        trees.clear();
        trees.resize(n);

        for (std::size_t i = 0; i < n; ++i)
        {
            BSPTree &tree = trees[i];
            tree.setBasePolygon(polygons[i], i);

            for (std::size_t j = 0; j < n; ++j)
            {
                if (j == i)
                    continue;
                
                //插入多边形进行相交裁剪
                tree.insert(polygons[j], j);
            }
        }
    }
}
