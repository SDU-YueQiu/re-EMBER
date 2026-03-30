#include "bool_problem.h"

namespace ember
{
    void BoolProblem::clear() noexcept
    {
        polygons_.clear();
        trees_.clear();
    }

    void BoolProblem::addPolygon(const Polygon256& polygon)
    {
        polygons_.push_back(polygon);
    }

    void BoolProblem::setPolygons(const std::vector<Polygon256>& polygons)
    {
        polygons_ = polygons;
        trees_.clear();
    }

    std::size_t BoolProblem::polygonCount() const noexcept
    {
        return polygons_.size();
    }

    void BoolProblem::buildTrees()
    {
        const std::size_t n = polygons_.size();
        trees_.clear();
        trees_.resize(n);

        for (std::size_t i = 0; i < n; ++i)
        {
            BSPTree& tree = trees_[i];
            tree.setBasePolygon(polygons_[i]);

            for (std::size_t j = 0; j < n; ++j)
            {
                if (j == i)
                {
                    continue;
                }

                tree.insert(polygons_[j]);
            }
        }
    }

    std::vector<Plane3i> BoolProblem::computeAABBPlanes() const
    {
        if (polygons_.empty())
        {
            return {};
        }

        bool initialized = false;
        Integer xMin, xMax, yMin, yMax, zMin, zMax;

        for (const auto& poly : polygons_)
        {
            const std::size_t n = poly.edgePlanes.size();
            for (std::size_t i = 0; i < n; ++i)
            {
                const std::size_t prev = (i == 0) ? (n - 1) : (i - 1);
                const HomPoint4i v = intersectHomogeneous(poly.plane, poly.edgePlanes[i], poly.edgePlanes[prev]);

                if (isZero(v.w))
                {
                    continue;
                }

                const Integer fx = floorDiv(v.x, v.w);
                const Integer cx = ceilDiv(v.x, v.w);
                const Integer fy = floorDiv(v.y, v.w);
                const Integer cy = ceilDiv(v.y, v.w);
                const Integer fz = floorDiv(v.z, v.w);
                const Integer cz = ceilDiv(v.z, v.w);

                if (!initialized)
                {
                    xMin = fx; xMax = cx;
                    yMin = fy; yMax = cy;
                    zMin = fz; zMax = cz;
                    initialized = true;
                }
                else
                {
                    if (fx < xMin) xMin = fx;
                    if (cx > xMax) xMax = cx;
                    if (fy < yMin) yMin = fy;
                    if (cy > yMax) yMax = cy;
                    if (fz < zMin) zMin = fz;
                    if (cz > zMax) zMax = cz;
                }
            }
        }

        if (!initialized)
        {
            return {};
        }

        const Integer margin = 1;
        xMin -= margin;
        xMax += margin;
        yMin -= margin;
        yMax += margin;
        zMin -= margin;
        zMax += margin;

        return {
            Plane3i(-1,  0,  0,  xMin),
            Plane3i( 1,  0,  0, -xMax),
            Plane3i( 0, -1,  0,  yMin),
            Plane3i( 0,  1,  0, -yMax),
            Plane3i( 0,  0, -1,  zMin),
            Plane3i( 0,  0,  1, -zMax),
        };
    }

    const std::vector<Polygon256>& BoolProblem::polygons() const noexcept
    {
        return polygons_;
    }

    const std::vector<BSPTree>& BoolProblem::trees() const noexcept
    {
        return trees_;
    }
}
