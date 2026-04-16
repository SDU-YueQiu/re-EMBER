#include "mesh_io.h"

#include <fstream>
#include <sstream>

namespace ember
{
    bool loadOBJ(const std::string& path, TriMesh& out)
    {
        std::ifstream f(path);
        if (!f.is_open()) return false;

        out.vertices.clear();
        out.faces.clear();

        std::string line;
        while (std::getline(f, line))
        {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ss(line);
            std::string token;
            ss >> token;

            if (token == "v")
            {
                double x, y, z;
                ss >> x >> y >> z;
                out.vertices.push_back({x, y, z});
            }
            else if (token == "f")
            {
                // 支持 "f v1 v2 v3" 和 "f v1/vt1/vn1 ..." 格式
                std::vector<int> idx;
                std::string s;
                while (ss >> s)
                {
                    int vi = std::stoi(s.substr(0, s.find('/')));
                    // OBJ 索引从 1 开始
                    idx.push_back(vi > 0 ? vi - 1 : (int)out.vertices.size() + vi);
                }
                // 扇形三角化
                for (int i = 1; i + 1 < (int)idx.size(); ++i)
                    out.faces.push_back({{idx[0], idx[i], idx[i + 1]}});
            }
        }
        return true;
    }

    bool saveOBJ(const std::string& path, const TriMesh& mesh)
    {
        std::ofstream f(path);
        if (!f.is_open()) return false;

        for (const auto& v : mesh.vertices)
            f << "v " << v[0] << ' ' << v[1] << ' ' << v[2] << '\n';

        for (const auto& face : mesh.faces)
            f << "f " << face[0]+1 << ' ' << face[1]+1 << ' ' << face[2]+1 << '\n';

        return true;
    }
}
