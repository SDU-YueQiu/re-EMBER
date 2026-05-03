#include "io.h"

#include "core/logging.h"
#include "algorithm/shit_wrapper.h"
#include "geometry/plane_geometry256.h"
#include "math/math256.h"

#include <tiny_obj_loader.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace ember
{
    namespace
    {
        constexpr const char *kReadObjMeshScope = "io::readObjMesh";
        constexpr const char *kChooseSharedScaleScope = "io::chooseSharedScale";
        constexpr const char *kBuildPolygonSoupScope = "io::buildPolygonSoup";
        constexpr const char *kWritePolygonSoupObjScope = "io::writePolygonSoupObj";
        constexpr const char *kBuildObjMeshScope = "io::buildObjMeshFromPolygonSoup";

        bool failIo(const char *scope, std::string &outError, const std::string &message)
        {
            outError = message;
            emitLog(LogLevel::Error, LogCategory::Io, scope, outError);
            return false;
        }

        // 论文中给出的输入整数坐标安全范围：每个笛卡尔坐标使用 26-bit 有符号整数。
        constexpr long long kInputCoordinateLimit = (1LL << 25) - 1;

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

        ObjVertex homogeneousPointToObjVertex(const PlanePoint3i &vertex, std::uint64_t coordinateScale)
        {
            const long double w = integerToLongDouble(vertex.x.w);
            const long double scale = static_cast<long double>(coordinateScale);
            const long double x = integerToLongDouble(vertex.x.x) / w / scale;
            const long double y = integerToLongDouble(vertex.x.y) / w / scale;
            const long double z = integerToLongDouble(vertex.x.z) / w / scale;

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

            std::unordered_map<std::string, std::size_t> vertexIndexByKey;
            outData.faces.reserve(fragments.size());
            vertexIndexByKey.reserve(fragments.size() * 4);

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
                    const auto [entry, inserted] = vertexIndexByKey.emplace(key, outData.uniqueVertices.size());
                    if (inserted)
                    {
                        outData.uniqueVertices.push_back(vertex);
                    }

                    face.push_back(entry->second);
                }

                outData.faces.push_back(std::move(face));
            }

            return true;
        }

        // 导出 OBJ 时只写几何坐标；这里把齐次点按 x/w, y/w, z/w 近似恢复为十进制。
        void writeObjVertexLine(std::ostream &stream, const PlanePoint3i &vertex, std::uint64_t coordinateScale)
        {
            const ObjVertex point = homogeneousPointToObjVertex(vertex, coordinateScale);

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

        emitLogLazy(
            LogLevel::Info,
            LogCategory::Io,
            kReadObjMeshScope,
            [&path]()
            {
                return "Reading OBJ path=" + path + ".";
            });

        tinyobj::ObjReaderConfig config;
        config.triangulate = false;

        tinyobj::ObjReader reader;
        if (!reader.ParseFromFile(path, config))
        {
            std::string message = "Failed to parse OBJ file: " + path;
            if (!reader.Error().empty())
            {
                message += ": " + reader.Error();
            }
            else
            {
                message += ".";
            }
            return failIo(kReadObjMeshScope, outError, message);
        }

        if (!reader.Warning().empty())
        {
            emitLog(LogLevel::Debug, LogCategory::Io, kReadObjMeshScope, reader.Warning());
        }

        const tinyobj::attrib_t &attrib = reader.GetAttrib();
        if ((attrib.vertices.size() % 3u) != 0u)
        {
            return failIo(kReadObjMeshScope, outError, "OBJ vertex attribute array is not divisible by three.");
        }

        outMesh.vertices.reserve(attrib.vertices.size() / 3u);
        for (std::size_t i = 0; i < attrib.vertices.size(); i += 3u)
        {
            ObjVertex vertex;
            vertex.x = static_cast<double>(attrib.vertices[i]);
            vertex.y = static_cast<double>(attrib.vertices[i + 1u]);
            vertex.z = static_cast<double>(attrib.vertices[i + 2u]);
            outMesh.vertices.push_back(vertex);
        }

        const std::vector<tinyobj::shape_t> &shapes = reader.GetShapes();
        for (std::size_t shapeIndex = 0; shapeIndex < shapes.size(); ++shapeIndex)
        {
            const tinyobj::mesh_t &mesh = shapes[shapeIndex].mesh;
            std::size_t indexOffset = 0;
            for (std::size_t faceIndex = 0; faceIndex < mesh.num_face_vertices.size(); ++faceIndex)
            {
                const std::size_t faceVertexCount = static_cast<std::size_t>(mesh.num_face_vertices[faceIndex]);
                if ((indexOffset + faceVertexCount) > mesh.indices.size())
                {
                    return failIo(
                        kReadObjMeshScope,
                        outError,
                        "OBJ face index data is incomplete in shape " + std::to_string(shapeIndex) + ".");
                }

                if (faceVertexCount < 3u)
                {
                    return failIo(
                        kReadObjMeshScope,
                        outError,
                        "OBJ face has fewer than three vertices in shape " + std::to_string(shapeIndex) + ".");
                }

                // tinyobjloader handles standard OBJ index forms, including relative indices;
                // this layer keeps only geometry position indices for the project boundary.
                std::vector<std::size_t> face;
                face.reserve(faceVertexCount);
                for (std::size_t i = 0; i < faceVertexCount; ++i)
                {
                    const tinyobj::index_t &index = mesh.indices[indexOffset + i];
                    if (index.vertex_index < 0)
                    {
                        return failIo(
                            kReadObjMeshScope,
                            outError,
                            "OBJ face contains a missing position index in shape " + std::to_string(shapeIndex) + ".");
                    }

                    const std::size_t vertexIndex = static_cast<std::size_t>(index.vertex_index);
                    if (vertexIndex >= outMesh.vertices.size())
                    {
                        return failIo(
                            kReadObjMeshScope,
                            outError,
                            "OBJ face references a vertex that does not exist in shape " +
                                std::to_string(shapeIndex) + ".");
                    }

                    face.push_back(vertexIndex);
                }

                outMesh.faces.push_back(std::move(face));
                indexOffset += faceVertexCount;
            }
        }

        emitLogLazy(
            LogLevel::Info,
            LogCategory::Io,
            kReadObjMeshScope,
            [&path, &outMesh]()
            {
                std::ostringstream message;
                message << "Read OBJ path=" << path
                        << " vertices=" << outMesh.vertices.size()
                        << " faces=" << outMesh.faces.size()
                        << ".";
                return message.str();
            });

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
                return failIo(kChooseSharedScaleScope, outError, "Explicit scale must be a positive integer.");
            }

            outScale = *options.explicitScale;
            emitLogLazy(
                LogLevel::Info,
                LogCategory::Io,
                kChooseSharedScaleScope,
                [&meshes, outScale]()
                {
                    std::ostringstream message;
                    message << "Using explicit shared scale=" << outScale
                            << " mesh_count=" << meshes.size()
                            << ".";
                    return message.str();
                });
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
            emitLogLazy(
                LogLevel::Info,
                LogCategory::Io,
                kChooseSharedScaleScope,
                [&meshes]()
                {
                    std::ostringstream message;
                    message << "Selected shared scale=1 for zero-range input mesh_count="
                            << meshes.size()
                            << ".";
                    return message.str();
                });
            return true;
        }

        if (maxAbsCoordinate > static_cast<long double>(kInputCoordinateLimit))
        {
            return failIo(
                kChooseSharedScaleScope,
                outError,
                "OBJ coordinates exceed the 26-bit signed input bound even at scale 1.");
        }

        const long double upperBound = static_cast<long double>(kInputCoordinateLimit) / maxAbsCoordinate;
        std::uint64_t scale = 1;
        while (scale <= (std::numeric_limits<std::uint64_t>::max() / 10ULL) &&
               static_cast<long double>(scale * 10ULL) <= upperBound)
        {
            scale *= 10ULL;
        }

        outScale = scale;
        emitLogLazy(
            LogLevel::Info,
            LogCategory::Io,
            kChooseSharedScaleScope,
            [&meshes, outScale]()
            {
                std::ostringstream message;
                message << "Selected shared scale=" << outScale
                        << " mesh_count=" << meshes.size()
                        << ".";
                return message.str();
            });
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
            return failIo(kBuildPolygonSoupScope, outError, "Shared scale must be a positive integer.");
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
                emitLog(LogLevel::Error, LogCategory::Io, kBuildPolygonSoupScope, outError);
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
                    return failIo(
                        kBuildPolygonSoupScope,
                        outError,
                        "OBJ face " + std::to_string(faceIndex) +
                            " references an out-of-range quantized vertex.");
                }

                faceVertices.push_back(quantizedVertices[vertexIndex]);
            }

            if (hasDuplicateFaceVertex(faceVertices))
            {
                return failIo(
                    kBuildPolygonSoupScope,
                    outError,
                    "OBJ face " + std::to_string(faceIndex) + " contains duplicate vertices after quantization.");
            }

            Plane3i supportPlane;
            if (!findSupportPlane(faceVertices, supportPlane))
            {
                return failIo(
                    kBuildPolygonSoupScope,
                    outError,
                    "OBJ face " + std::to_string(faceIndex) +
                        " degenerates to a collinear or point set after quantization.");
            }

            for (const Vec3i &vertex : faceVertices)
            {
                if (!isVertexOnPlane(vertex, supportPlane))
                {
                    return failIo(
                        kBuildPolygonSoupScope,
                        outError,
                        "OBJ face " + std::to_string(faceIndex) + " is not coplanar after quantization.");
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
                    return failIo(
                        kBuildPolygonSoupScope,
                        outError,
                        "OBJ face " + std::to_string(faceIndex) + " contains a zero-length edge after quantization.");
                }

                const Vec3i edgeNormal = cross(edgeVector, supportPlane.normal());
                if (isZero(edgeNormal.x) && isZero(edgeNormal.y) && isZero(edgeNormal.z))
                {
                    return failIo(
                        kBuildPolygonSoupScope,
                        outError,
                        "OBJ face " + std::to_string(faceIndex) + " produced an invalid edge plane.");
                }

                edgePlanes.push_back(Plane3i::fromPointNormal(start, edgeNormal));
            }

            Polygon256 polygon(supportPlane, std::move(edgePlanes));
            if (!polygon.isValid())
            {
                return failIo(
                    kBuildPolygonSoupScope,
                    outError,
                    "OBJ face " + std::to_string(faceIndex) +
                        " is not a valid strictly convex polygon under the current exact geometry contract.");
            }

            outPolygons.push_back(std::move(polygon));
        }

        emitLogLazy(
            LogLevel::Info,
            LogCategory::Io,
            kBuildPolygonSoupScope,
            [&mesh, sharedScale, &outPolygons]()
            {
                std::ostringstream message;
                message << "Built polygon soup shared_scale=" << sharedScale
                        << " input_vertices=" << mesh.vertices.size()
                        << " input_faces=" << mesh.faces.size()
                        << " output_polygons=" << outPolygons.size()
                        << ".";
                return message.str();
            });

        return true;
    }

    bool writePolygonSoupObj(
        const std::vector<Polygon256> &fragments,
        const std::string &path,
        std::size_t &outFaceCount,
        std::string &outError,
        std::uint64_t coordinateScale)
    {
        outFaceCount = 0;
        outError.clear();

        if (coordinateScale == 0)
        {
            return failIo(kWritePolygonSoupObjScope, outError, "Coordinate scale must be a positive integer.");
        }

        const std::filesystem::path outputPath(path);
        if (outputPath.has_parent_path())
        {
            std::error_code ec;
            std::filesystem::create_directories(outputPath.parent_path(), ec);
            if (ec)
            {
                return failIo(
                    kWritePolygonSoupObjScope,
                    outError,
                    "Failed to create OBJ output directory: " + outputPath.parent_path().string());
            }
        }

        RecoveredPolygonSoupData recovered;
        if (!recoverPolygonSoupData(fragments, recovered, outError))
        {
            outError = "Failed to prepare polygon soup OBJ export: " + outError;
            emitLog(LogLevel::Error, LogCategory::Io, kWritePolygonSoupObjScope, outError);
            return false;
        }

        std::ofstream output(path, std::ios::trunc);
        if (!output)
        {
            return failIo(
                kWritePolygonSoupObjScope,
                outError,
                "Failed to open OBJ output file for writing: " + path);
        }

        // 默认保持 polygon soup 形态：写顶点，再逐面写 OBJ n-gon。
        output << "# Ember exact polygon soup export\n";
        for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        {
            writeObjVertexLine(output, vertex, coordinateScale);
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
        emitLogLazy(
            LogLevel::Info,
            LogCategory::Io,
            kWritePolygonSoupObjScope,
            [&path, &fragments, &recovered, outFaceCount, coordinateScale]()
            {
                std::ostringstream message;
                message << "Wrote OBJ path=" << path
                        << " input_fragments=" << fragments.size()
                        << " unique_vertices=" << recovered.uniqueVertices.size()
                        << " exported_faces=" << outFaceCount
                        << " coordinate_scale=" << coordinateScale
                        << ".";
                return message.str();
            });
        return true;
    }

    bool buildObjMeshFromPolygonSoup(
        const std::vector<Polygon256> &fragments,
        ObjMeshData &outMesh,
        std::string &outError,
        std::uint64_t coordinateScale)
    {
        outMesh = ObjMeshData();
        outError.clear();

        if (coordinateScale == 0)
        {
            return failIo(kBuildObjMeshScope, outError, "Coordinate scale must be a positive integer.");
        }

        RecoveredPolygonSoupData recovered;
        if (!recoverPolygonSoupData(fragments, recovered, outError))
        {
            outError = "Failed to prepare OBJ mesh from polygon soup: " + outError;
            emitLog(LogLevel::Error, LogCategory::Io, kBuildObjMeshScope, outError);
            return false;
        }

        outMesh.vertices.reserve(recovered.uniqueVertices.size());
        for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        {
            outMesh.vertices.push_back(homogeneousPointToObjVertex(vertex, coordinateScale));
        }

        outMesh.faces = std::move(recovered.faces);

        emitLogLazy(
            LogLevel::Info,
            LogCategory::Io,
            kBuildObjMeshScope,
            [&fragments, &outMesh, coordinateScale]()
            {
                std::ostringstream message;
                message << "Built OBJ mesh from polygon soup input_fragments=" << fragments.size()
                        << " output_vertices=" << outMesh.vertices.size()
                        << " output_faces=" << outMesh.faces.size()
                        << " coordinate_scale=" << coordinateScale
                        << ".";
                return message.str();
            });

        return true;
    }
}
