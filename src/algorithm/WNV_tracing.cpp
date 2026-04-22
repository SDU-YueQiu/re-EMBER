#include "WNV_tracing.h"

namespace ember
{

    traceStatus tracePathWNV(const refPoint &refpoint, const Path &path, const std::vector<Polygon256> &polygons, WNV &targetWNV)
    {
        // 实现为对path里的每条线段遍历所有多边形求交，然后累加wnvt到wnv上

        auto tmpwnv = refpoint.wnv;

        bool prePointInPolygon = false;

        for (int i = 0; i < path.size(); i++)
        {
            const auto &seg = path[i];

            // TODO：测试中若发现性能需要优化，应为调用前明确约束参数有效性然后关闭该函数的有效性检查
            if (!seg.isValid())
                return INVALID_SEGMENT;

            for (int j = 0; j < polygons.size(); j++)
            {
                const auto &poly = polygons[j];
                if (!poly.isValid())
                    return INVALID_POLYGON;

                PlanePoint3i intersectPoint;
                // 线段与多边形存在内部交点（仅一个）
                if (intersectionSegmentPolygon(seg, poly, intersectPoint))
                {
                    if (!poly.containsStrictly(intersectPoint))
                        return BOUNDARY_HIT;

                    PlanePoint3i startPoint = seg.getStartPoint();
                    PlanePoint3i endPoint = seg.getEndPoint();

                    int scp = startPoint.classify(poly.plane); // start point classify polygon
                    int ecp = endPoint.classify(poly.plane);

                    // 路径起点落在多边形内部时无法处理
                    // 其他情况该线段终点会和前面一条线段起点并做一次计算不会进这条分支
                    if (scp == 0)
                    {
                        if (i == 0)
                            return START_AT_POLYGON;
                        else
                            return FAIL; // 正常情况下不会到这儿
                    }

                    if (ecp == 0)
                    {
                        if (i == path.size() - 1)
                            return END_AT_POLYGON;
                        else // 末端在多边形外部，只看当前线段起点和下一条线段终点
                        {
                            const auto &nextEndPoint = path[i + 1].getEndPoint();
                            int necp = nextEndPoint.classify(poly.plane);//next ecp
                            if(necp)
                        }
                    }

                    // clean case
                    // 点对面分类时已经包含了面的法向量
                    int sigma = (scp - ecp) / 2;
                    for (int k = 0; k < tmpwnv.size(); ++k)
                        tmpwnv[k] += sigma * poly.WNTV[k];
                }
                else
                    continue;
            }
        }

        // 成功计算wnv
        targetWNV = tmpwnv;

        return SUCCESS;
    }
}