#include "io.h"

#include "algorithm/shit_wrapper.h"
#include "geometry/plane_geometry256.h"
#include "math/math256.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>

namespace ember
{
    namespace
    {
        // 论文中给出的输入整数坐标安全范围：每个笛卡尔坐标使用 26-bit 有符号整数。
        constexpr long long kInputCoordinateLimit = (1LL << 25) - 1;

        // 严格解析一个完整浮点 token，避免默默接受尾随垃圾字符。
        bool parseDoubleToken(const std::string &token, double &outValue)
        {
            try
            {
                std::size_t consumed = 0;
                outValue = std::stod(token, &consumed);
                return consumed == token.size();
            }
            catch (...)
            {
                return false;
            }
        }

        // 仅提取 OBJ face token 中的位置索引部分，并要求使用正向 1-based 索引。
        bool parsePositiveIndexToken(const std::string &token, std::size_t &outIndex)
        {
            const std::size_t slash = token.find('/');
            const std::string indexToken = token.substr(0, slash);
            if (indexToken.empty())
            {
                return false;
            }

            try
            {
                std::size_t consumed = 0;
                const unsigned long long parsed = std::stoull(indexToken, &consumed, 10);
                if (consumed != indexToken.size() || parsed == 0)
                {
                    return false;
                }

                outIndex = static_cast<std::size_t>(parsed - 1ULL);
                return true;
            }
            catch (...)
            {
                return false;
            }
        }

        // 按调用方给定的共享 scale 执行四舍五入量化，并同时检查 26-bit 输入界限。
        bool quantizeCoordinate(double value, std::uint64_t scale, Integer &outValue, std::string &outError)
        {
            if (!std::isfinite(value))
            {
                outError = "OBJ vertex coordinate is not finite.";
                return false;
            }

            const long double scaled = static_cast<long double>(value) * static_cast<long double>(scale);
            const long double rounded = std::round(scaled);
            if (rounded < -static_cast<long double>(kInputCoordinateLimit) ||
                rounded > static_cast<long double>(kInputCoordinateLimit))
            {
                outError = "Quantized OBJ vertex coordinate exceeds the 26-bit signed input bound.";
                return false;
            }

            outValue = Integer(static_cast<int>(rounded));
            return true;
        }

        // 顶点量化按坐标独立执行，任何一个分量失败都视为整个顶点失败。
        bool quantizeVertex(const ObjVertex &vertex, std::uint64_t scale, Vec3i &outVertex, std::string &outError)
        {
            Integer x;
            Integer y;
            Integer z;
            if (!quantizeCoordinate(vertex.x, scale, x, outError) ||
                !quantizeCoordinate(vertex.y, scale, y, outError) ||
                !quantizeCoordinate(vertex.z, scale, z, outError))
            {
                return false;
            }

            outVertex = Vec3i(x, y, z);
            return true;
        }

        // 量化后重复顶点会让某些边退化成零长度，先在面级别提前拦截。
        bool hasDuplicateFaceVertex(const std::vector<Vec3i> &faceVertices) noexcept
        {
            for (std::size_t i = 0; i < faceVertices.size(); ++i)
            {
                for (std::size_t j = i + 1; j < faceVertices.size(); ++j)
                {
                    if (faceVertices[i] == faceVertices[j])
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        // 按输入顶点顺序寻找第一个非共线三元组，用它确定该面的支撑平面。
        bool findSupportPlane(const std::vector<Vec3i> &faceVertices, Plane3i &outPlane)
        {
            for (std::size_t i = 0; i < faceVertices.size(); ++i)
            {
                for (std::size_t j = i + 1; j < faceVertices.size(); ++j)
                {
                    for (std::size_t k = j + 1; k < faceVertices.size(); ++k)
                    {
                        const Vec3i e0 = faceVertices[j] - faceVertices[i];
                        const Vec3i e1 = faceVertices[k] - faceVertices[i];
                        const Vec3i normal = cross(e0, e1);
                        if (!isZero(normal.x) || !isZero(normal.y) || !isZero(normal.z))
                        {
                            outPlane = Plane3i::fromPointNormal(faceVertices[i], normal);
                            return true;
                        }
                    }
                }
            }

            return false;
        }

        // 量化后仍需保证所有顶点严格共面，否则不能安全构造 plane-based polygon。
        bool isVertexOnPlane(const Vec3i &vertex, const Plane3i &plane) noexcept
        {
            return isZero(dot(vertex, plane.normal()) + plane.d);
        }

        // 导出阶段需要把 int256_t 转为文本，再由 long double 近似输出十进制坐标。
        std::string integerToString(const Integer &value)
        {
            std::ostringstream stream;
            stream << value;
            return stream.str();
        }

        long double integerToLongDouble(const Integer &value)
        {
            return std::stold(integerToString(value));
        }

        ObjVertex homogeneousPointToObjVertex(const PlanePoint3i &vertex)
        {
            const long double w = integerToLongDouble(vertex.x.w);
            const long double x = integerToLongDouble(vertex.x.x) / w;
            const long double y = integerToLongDouble(vertex.x.y) / w;
            const long double z = integerToLongDouble(vertex.x.z) / w;

            ObjVertex point;
            point.x = static_cast<double>(x);
            point.y = static_cast<double>(y);
            point.z = static_cast<double>(z);
            return point;
        }

        // 以齐次四元组全文本作为顶点键，避免浮点近似导致的错误去重。
        std::string makeHomogeneousPointKey(const HomPoint4i &point)
        {
            std::ostringstream stream;
            stream << point.x << "|" << point.y << "|" << point.z << "|" << point.w;
            return stream.str();
        }

        // OBJ 导出要求有稳定的顶点顺序，这里直接复用 Polygon256 的边平面顺序恢复顶点。
        bool recoverOrderedPolygonVertices(
            const Polygon256 &polygon,
            std::vector<PlanePoint3i> &outVertices,
            std::string &outError)
        {
            outVertices.clear();
            if (!polygon.isValid())
            {
                outError = "Attempted to export an invalid Polygon256.";
                return false;
            }

            outVertices.reserve(polygon.edgeCount());
            for (std::size_t i = 0; i < polygon.edgeCount(); ++i)
            {
                const PlanePoint3i vertex = getPolygonVertex(polygon, i);
                if (!vertex.hasUniqueIntersection() || isZero(vertex.x.w))
                {
                    outError = "Failed to recover an ordered polygon vertex with a unique finite homogeneous point.";
                    return false;
                }

                outVertices.push_back(vertex);
            }

            return true;
        }

        struct RecoveredPolygonSoupData
        {
            std::vector<PlanePoint3i> uniqueVertices;
            std::vector<std::vector<std::size_t>> faces;
        };

        bool recoverPolygonSoupData(
            const std::vector<Polygon256> &fragments,
            RecoveredPolygonSoupData &outData,
            std::string &outError)
        {
            outData.uniqueVertices.clear();
            outData.faces.clear();
            outError.clear();

            std::vector<std::pair<std::string, std::size_t>> vertexIndexTable;
            outData.faces.reserve(fragments.size());
            vertexIndexTable.reserve(fragments.size() * 4);

            for (std::size_t polygonIndex = 0; polygonIndex < fragments.size(); ++polygonIndex)
            {
                std::vector<PlanePoint3i> orderedVertices;
                if (!recoverOrderedPolygonVertices(fragments[polygonIndex], orderedVertices, outError))
                {
                    outError = "Failed to recover polygon " + std::to_string(polygonIndex) + ": " + outError;
                    return false;
                }

                std::vector<std::size_t> face;
                face.reserve(orderedVertices.size());
                for (const PlanePoint3i &vertex : orderedVertices)
                {
                    const std::string key = makeHomogeneousPointKey(vertex.x);
                    std::size_t slot = 0;
                    bool found = false;
                    for (const auto &entry : vertexIndexTable)
                    {
                        if (entry.first == key)
                        {
                            slot = entry.second;
                            found = true;
                            break;
                        }
                    }

                    if (!found)
                    {
                        slot = outData.uniqueVertices.size();
                        outData.uniqueVertices.push_back(vertex);
                        vertexIndexTable.push_back(std::make_pair(key, slot));
                    }

                    face.push_back(slot);
                }

                outData.faces.push_back(std::move(face));
            }

            return true;
        }

        // 导出 OBJ 时只写几何坐标；这里把齐次点按 x/w, y/w, z/w 近似恢复为十进制。
        void writeObjVertexLine(std::ostream &stream, const PlanePoint3i &vertex)
        {
            const ObjVertex point = homogeneousPointToObjVertex(vertex);

            stream << "v "
                   << std::setprecision(std::numeric_limits<long double>::digits10 + 1)
                   << point.x << " "
                   << point.y << " "
                   << point.z << "\n";
        }
    }

    bool readObjMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError)
    {
        outMesh = ObjMeshData();
        outError.clear();

        std::ifstream input(path);
        if (!input)
        {
            outError = "Failed to open OBJ file: " + path;
            return false;
        }

        std::string line;
        std::size_t lineNumber = 0;
        while (std::getline(input, line))
        {
            ++lineNumber;

            std::istringstream lineStream(line);
            std::string keyword;
            if (!(lineStream >> keyword))
            {
                continue;
            }

            if (keyword == "#")
            {
                continue;
            }

            if (keyword == "v")
            {
                // 第一版只读取位置顶点；若顶点坐标不完整或不是合法浮点数则立即报错。
                std::string xs;
                std::string ys;
                std::string zs;
                if (!(lineStream >> xs >> ys >> zs))
                {
                    outError = "OBJ vertex line is missing coordinates at line " + std::to_string(lineNumber) + ".";
                    return false;
                }

                ObjVertex vertex;
                if (!parseDoubleToken(xs, vertex.x) ||
                    !parseDoubleToken(ys, vertex.y) ||
                    !parseDoubleToken(zs, vertex.z))
                {
                    outError = "OBJ vertex line contains an invalid floating-point coordinate at line " + std::to_string(lineNumber) + ".";
                    return false;
                }

                outMesh.vertices.push_back(vertex);
                continue;
            }

            if (keyword == "f")
            {
                // face token 允许携带 vt/vn，但当前位置索引必须合法且面至少有三个顶点。
                std::vector<std::size_t> face;
                std::string token;
                while (lineStream >> token)
                {
                    if (!token.empty() && token[0] == '#')
                    {
                        break;
                    }

                    std::size_t index = 0;
                    if (!parsePositiveIndexToken(token, index))
                    {
                        outError = "OBJ face contains an invalid position index token at line " + std::to_string(lineNumber) + ".";
                        return false;
                    }
                    if (index >= outMesh.vertices.size())
                    {
                        outError = "OBJ face references a vertex that does not exist at line " + std::to_string(lineNumber) + ".";
                        return false;
                    }

                    face.push_back(index);
                }

                if (face.size() < 3)
                {
                    outError = "OBJ face has fewer than three vertices at line " + std::to_string(lineNumber) + ".";
                    return false;
                }

                outMesh.faces.push_back(std::move(face));
            }
        }

        return true;
    }

    bool chooseSharedScale(
        const std::vector<ObjMeshData> &meshes,
        const QuantizeOptions &options,
        std::uint64_t &outScale,
        std::string &outError)
    {
        outError.clear();
        outScale = 0;

        if (options.explicitScale.has_value())
        {
            if (*options.explicitScale == 0)
            {
                outError = "Explicit scale must be a positive integer.";
                return false;
            }

            outScale = *options.explicitScale;
            return true;
        }

        // 自动模式下，为全部输入挑选同一个 10^k，使量化后所有坐标仍落在安全范围内。
        long double maxAbsCoordinate = 0.0L;
        for (const ObjMeshData &mesh : meshes)
        {
            for (const ObjVertex &vertex : mesh.vertices)
            {
                maxAbsCoordinate = std::max(maxAbsCoordinate, std::fabs(static_cast<long double>(vertex.x)));
                maxAbsCoordinate = std::max(maxAbsCoordinate, std::fabs(static_cast<long double>(vertex.y)));
                maxAbsCoordinate = std::max(maxAbsCoordinate, std::fabs(static_cast<long double>(vertex.z)));
            }
        }

        if (maxAbsCoordinate == 0.0L)
        {
            outScale = 1;
            return true;
        }

        if (maxAbsCoordinate > static_cast<long double>(kInputCoordinateLimit))
        {
            outError = "OBJ coordinates exceed the 26-bit signed input bound even at scale 1.";
            return false;
        }

        const long double upperBound = static_cast<long double>(kInputCoordinateLimit) / maxAbsCoordinate;
        std::uint64_t scale = 1;
        while (scale <= (std::numeric_limits<std::uint64_t>::max() / 10ULL) &&
               static_cast<long double>(scale * 10ULL) <= upperBound)
        {
            scale *= 10ULL;
        }

        outScale = scale;
        return true;
    }

    bool buildPolygonSoup(
        const ObjMeshData &mesh,
        std::uint64_t sharedScale,
        std::vector<Polygon256> &outPolygons,
        std::string &outError)
    {
        outPolygons.clear();
        outError.clear();

        if (sharedScale == 0)
        {
            outError = "Shared scale must be a positive integer.";
            return false;
        }

        std::vector<Vec3i> quantizedVertices;
        quantizedVertices.reserve(mesh.vertices.size());
        for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
        {
            // 顶点先整体量化，后续所有面都复用同一批整数顶点。
            Vec3i quantizedVertex;
            if (!quantizeVertex(mesh.vertices[i], sharedScale, quantizedVertex, outError))
            {
                outError = "Failed to quantize OBJ vertex " + std::to_string(i) + ": " + outError;
                return false;
            }

            quantizedVertices.push_back(quantizedVertex);
        }

        outPolygons.reserve(mesh.faces.size());
        for (std::size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex)
        {
            // 每个 OBJ face 都必须在量化后独立满足：无重复点、可定支撑平面、全体共面、严格凸。
            const std::vector<std::size_t> &face = mesh.faces[faceIndex];

            std::vector<Vec3i> faceVertices;
            faceVertices.reserve(face.size());
            for (const std::size_t vertexIndex : face)
            {
                if (vertexIndex >= quantizedVertices.size())
                {
                    outError = "OBJ face " + std::to_string(faceIndex) + " references an out-of-range quantized vertex.";
                    return false;
                }

                faceVertices.push_back(quantizedVertices[vertexIndex]);
            }

            if (hasDuplicateFaceVertex(faceVertices))
            {
                outError = "OBJ face " + std::to_string(faceIndex) + " contains duplicate vertices after quantization.";
                return false;
            }

            Plane3i supportPlane;
            if (!findSupportPlane(faceVertices, supportPlane))
            {
                outError = "OBJ face " + std::to_string(faceIndex) + " degenerates to a collinear or point set after quantization.";
                return false;
            }

            for (const Vec3i &vertex : faceVertices)
            {
                if (!isVertexOnPlane(vertex, supportPlane))
                {
                    outError = "OBJ face " + std::to_string(faceIndex) + " is not coplanar after quantization.";
                    return false;
                }
            }

            std::vector<Plane3i> edgePlanes;
            edgePlanes.reserve(faceVertices.size());
            for (std::size_t i = 0; i < faceVertices.size(); ++i)
            {
                // 每条边平面由“边向量 x 支撑平面法向”给出，实际朝向交给 Polygon256 自动外翻校正。
                const Vec3i &start = faceVertices[i];
                const Vec3i &end = faceVertices[(i + 1) % faceVertices.size()];
                const Vec3i edgeVector = end - start;
                if (isZero(edgeVector.x) && isZero(edgeVector.y) && isZero(edgeVector.z))
                {
                    outError = "OBJ face " + std::to_string(faceIndex) + " contains a zero-length edge after quantization.";
                    return false;
                }

                const Vec3i edgeNormal = cross(edgeVector, supportPlane.normal());
                if (isZero(edgeNormal.x) && isZero(edgeNormal.y) && isZero(edgeNormal.z))
                {
                    outError = "OBJ face " + std::to_string(faceIndex) + " produced an invalid edge plane.";
                    return false;
                }

                edgePlanes.push_back(Plane3i::fromPointNormal(start, edgeNormal));
            }

            Polygon256 polygon(supportPlane, std::move(edgePlanes));
            if (!polygon.isValid())
            {
                outError = "OBJ face " + std::to_string(faceIndex) + " is not a valid strictly convex polygon under the current exact geometry contract.";
                return false;
            }

            outPolygons.push_back(std::move(polygon));
        }

        return true;
    }

    bool writePolygonSoupObj(
        const std::vector<Polygon256> &fragments,
        const std::string &path,
        std::size_t &outFaceCount,
        std::string &outError)
    {
        outFaceCount = 0;
        outError.clear();

        const std::filesystem::path outputPath(path);
        if (outputPath.has_parent_path())
        {
            std::error_code ec;
            std::filesystem::create_directories(outputPath.parent_path(), ec);
            if (ec)
            {
                outError = "Failed to create OBJ output directory: " + outputPath.parent_path().string();
                return false;
            }
        }

        RecoveredPolygonSoupData recovered;
        if (!recoverPolygonSoupData(fragments, recovered, outError))
        {
            outError = "Failed to prepare polygon soup OBJ export: " + outError;
            return false;
        }

        std::ofstream output(path, std::ios::trunc);
        if (!output)
        {
            outError = "Failed to open OBJ output file for writing: " + path;
            return false;
        }

        // 默认保持 polygon soup 形态：写顶点，再逐面写 OBJ n-gon。
        output << "# Ember exact polygon soup export\n";
        for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        {
            writeObjVertexLine(output, vertex);
        }

        for (const std::vector<std::size_t> &face : recovered.faces)
        {
            output << "f";
            for (const std::size_t vertexIndex : face)
            {
                output << " " << (vertexIndex + 1);
            }
            output << "\n";
        }

        outFaceCount = recovered.faces.size();
        return true;
    }

    bool buildTriangleMeshFromPolygonSoup(
        const std::vector<Polygon256> &fragments,
        TriangleMeshData &outMesh,
        std::string &outError)
    {
        outMesh = TriangleMeshData();
        outError.clear();

        RecoveredPolygonSoupData recovered;
        if (!recoverPolygonSoupData(fragments, recovered, outError))
        {
            outError = "Failed to prepare triangle mesh from polygon soup: " + outError;
            return false;
        }

        outMesh.vertices.reserve(recovered.uniqueVertices.size());
        for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        {
            outMesh.vertices.push_back(homogeneousPointToObjVertex(vertex));
        }

        for (std::size_t faceIndex = 0; faceIndex < recovered.faces.size(); ++faceIndex)
        {
            const std::vector<std::size_t> &face = recovered.faces[faceIndex];
            if (face.size() < 3)
            {
                outError = "Recovered polygon face " + std::to_string(faceIndex) + " has fewer than three vertices.";
                return false;
            }

            for (std::size_t i = 1; i + 1 < face.size(); ++i)
            {
                outMesh.triangles.push_back({face[0], face[i], face[i + 1]});
            }
        }

        return true;
    }
}
