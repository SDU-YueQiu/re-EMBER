#pragma once

#include <array>
#include <string>
#include <vector>

namespace ember
{
    struct TriMesh
    {
        std::vector<std::array<double, 3>> vertices;
        std::vector<std::array<int, 3>>    faces;
    };

    // 从 OBJ 文件加载三角网格（忽略非三角面）
    bool loadOBJ(const std::string& path, TriMesh& out);

    // 将三角网格写入 OBJ 文件
    bool saveOBJ(const std::string& path, const TriMesh& mesh);
}
