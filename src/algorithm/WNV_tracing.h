#include "geometry/geometry256.h"

namespace ember
{
    struct refPoint
    {
        PlanePoint3i point;
        WNV wnv;

        refPoint(const PlanePoint3i &p, const WNV &w) : point(p), wnv(w) {}
    };

}