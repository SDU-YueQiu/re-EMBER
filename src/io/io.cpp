/**
 * @file io.cpp
 * @brief 实现 OBJ/STL 导入、量化、多边形集合构建和导出。
 */
#include "io.h"

#include "geometry/plane_geometry256.h"
#include "geometry/polygon_ops.h"
#include "math/math256.h"

#include <stl_reader/stl_reader.h>
#include <tiny_obj_loader.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
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
bool failIo(std::string &outError, const std::string &message)
{
    outError = message;
    return false;
}

// 论文中给出的输入整数坐标安全范围：每个笛卡尔坐标使用 26-bit 有符号整数。
long long kInputCoordinateLimit = (1LL << 25) - 1;

enum class MeshFileFormat
{
    Obj,
    Stl
};

std::string toLowerAscii(std::string value)
{
    std::transform(
        value.begin(),
        value.end(),
        value.begin(),
        [](unsigned char c)
    {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool detectMeshFileFormatFromPath(
    const std::string &path,
    MeshFileFormat &outFormat,
    std::string &outError)
{
    const std::string extension =
        toLowerAscii(std::filesystem::path(path).extension().string());
    if (extension == ".obj")
    {
        outFormat = MeshFileFormat::Obj;
        return true;
    }
    if (extension == ".stl")
    {
        outFormat = MeshFileFormat::Stl;
        return true;
    }

    return failIo(
               outError,
               "Unsupported mesh file extension for path: " + path +
               ". Expected .obj or .stl.");
}

bool ensureOutputDirectoryExists(
    const std::string &path,
    const std::string &formatLabel,
    std::string &outError)
{
    const std::filesystem::path outputPath(path);
    if (!outputPath.has_parent_path())
        return true;

    std::error_code ec;
    std::filesystem::create_directories(outputPath.parent_path(), ec);
    if (ec)
    {
        return failIo(
                   outError,
                   "Failed to create " + formatLabel + " output directory: " +
                   outputPath.parent_path().string());
    }

    return true;
}

void writeLittleEndianUInt32(std::ostream &stream, std::uint32_t value)
{
    char bytes[4];
    bytes[0] = static_cast<char>(value & 0xffu);
    bytes[1] = static_cast<char>((value >> 8u) & 0xffu);
    bytes[2] = static_cast<char>((value >> 16u) & 0xffu);
    bytes[3] = static_cast<char>((value >> 24u) & 0xffu);
    stream.write(bytes, sizeof(bytes));
}

void writeLittleEndianUInt16(std::ostream &stream, std::uint16_t value)
{
    char bytes[2];
    bytes[0] = static_cast<char>(value & 0xffu);
    bytes[1] = static_cast<char>((value >> 8u) & 0xffu);
    stream.write(bytes, sizeof(bytes));
}

void writeLittleEndianFloat32(std::ostream &stream, float value)
{
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float bit width mismatch");
    std::memcpy(&bits, &value, sizeof(bits));
    writeLittleEndianUInt32(stream, bits);
}
ObjVertex subtractObjVertex(const ObjVertex &lhs, const ObjVertex &rhs) noexcept
{
    return ObjVertex{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

ObjVertex crossObjVertex(const ObjVertex &lhs, const ObjVertex &rhs) noexcept
{
    return ObjVertex{
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x};
}

ObjVertex normalizeObjVertex(const ObjVertex &vertex) noexcept
{
    const double length =
        std::sqrt(vertex.x * vertex.x + vertex.y * vertex.y + vertex.z * vertex.z);
    if (length <= 0.0)
        return ObjVertex{};

    return ObjVertex{vertex.x / length, vertex.y / length, vertex.z / length};
}

struct StlTriangle
{
    ObjVertex a;
    ObjVertex b;
    ObjVertex c;
};

ObjVertex computeTriangleNormal(const StlTriangle &triangle) noexcept
{
    const ObjVertex ab = subtractObjVertex(triangle.b, triangle.a);
    const ObjVertex ac = subtractObjVertex(triangle.c, triangle.a);
    return normalizeObjVertex(crossObjVertex(ab, ac));
}

// 按调用方给定的共享 scale 执行四舍五入量化，并同时检查 26-bit 输入界限。
bool quantizeCoordinate(double value, std::uint64_t scale, Integer &outValue, std::string &outError)
{
    if (!std::isfinite(value))
    {
        outError = "Mesh vertex coordinate is not finite.";
        return false;
    }

    const long double scaled = static_cast<long double>(value) * static_cast<long double>(scale);
    const long double rounded = std::round(scaled);
    if (rounded < -static_cast<long double>(kInputCoordinateLimit) ||
            rounded > static_cast<long double>(kInputCoordinateLimit))
    {
        outError = "Quantized mesh vertex coordinate exceeds the 26-bit signed input bound.";
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
        return false;

    outVertex = Vec3i(x, y, z);
    return true;
}

bool computeScaledCoordinateInterval(
    double value,
    std::uint64_t scale,
    Integer &outLower,
    Integer &outUpper,
    std::string &outError)
{
    if (!std::isfinite(value))
    {
        outError = "Mesh vertex coordinate is not finite.";
        return false;
    }

    const long double scaled = static_cast<long double>(value) * static_cast<long double>(scale);
    const long double lower = std::floor(scaled);
    const long double upper = std::ceil(scaled);
    if (lower < static_cast<long double>(std::numeric_limits<int>::lowest()) ||
            upper > static_cast<long double>(std::numeric_limits<int>::max()))
    {
        outError = "Scaled mesh vertex coordinate exceeds the integer AABB conversion bound.";
        return false;
    }

    outLower = Integer(static_cast<int>(lower));
    outUpper = Integer(static_cast<int>(upper));
    return true;
}

void appendScaledVertexIntervalToAABB(
    AABB3i &box,
    const Integer &xMin,
    const Integer &xMax,
    const Integer &yMin,
    const Integer &yMax,
    const Integer &zMin,
    const Integer &zMax) noexcept
{
    AABB3i vertexBox;
    vertexBox.xMin = xMin;
    vertexBox.xMax = xMax;
    vertexBox.yMin = yMin;
    vertexBox.yMax = yMax;
    vertexBox.zMin = zMin;
    vertexBox.zMax = zMax;
    vertexBox.valid = true;
    mergeAABB(box, vertexBox);
}

// 量化后重复顶点会让某些边退化成零长度，先在面级别提前拦截。
bool hasDuplicateFaceVertex(const std::vector<Vec3i> &faceVertices) noexcept
{
    for (std::size_t i = 0; i < faceVertices.size(); ++i)
    {
        for (std::size_t j = i + 1; j < faceVertices.size(); ++j)
        {
            if (faceVertices[i] == faceVertices[j])
                return true;
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

// 量化后仍需保证所有顶点严格共面，否则不能安全构造基于平面的多边形。
bool isVertexOnPlane(const Vec3i &vertex, const Plane3i &plane) noexcept
{
    return isZero(dot(vertex, plane.normal()) + plane.d);
}

enum class PolygonFaceBuildFailure
{
    None,
    DuplicateVertices,
    DegenerateSupport,
    NonCoplanarAfterQuantization,
    ZeroLengthEdge,
    InvalidEdgePlane,
    InvalidPolygon
};

bool buildPolygonFromQuantizedFace(
    const std::vector<Vec3i> &faceVertices,
    std::size_t faceIndex,
    Polygon256 &outPolygon,
    PolygonFaceBuildFailure &outFailure,
    std::string &outError)
{
    outFailure = PolygonFaceBuildFailure::None;
    outError.clear();

    if (hasDuplicateFaceVertex(faceVertices))
    {
        outFailure = PolygonFaceBuildFailure::DuplicateVertices;
        outError = "Mesh face " + std::to_string(faceIndex) + " contains duplicate vertices after quantization.";
        return false;
    }

    Plane3i supportPlane;
    if (!findSupportPlane(faceVertices, supportPlane))
    {
        outFailure = PolygonFaceBuildFailure::DegenerateSupport;
        outError = "Mesh face " + std::to_string(faceIndex) +
                   " degenerates to a collinear or point set after quantization.";
        return false;
    }

    for (const Vec3i &vertex : faceVertices)
    {
        if (!isVertexOnPlane(vertex, supportPlane))
        {
            outFailure = PolygonFaceBuildFailure::NonCoplanarAfterQuantization;
            outError = "Mesh face " + std::to_string(faceIndex) + " is not coplanar after quantization.";
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
            outFailure = PolygonFaceBuildFailure::ZeroLengthEdge;
            outError = "Mesh face " + std::to_string(faceIndex) + " contains a zero-length edge after quantization.";
            return false;
        }

        const Vec3i edgeNormal = cross(edgeVector, supportPlane.normal());
        if (isZero(edgeNormal.x) && isZero(edgeNormal.y) && isZero(edgeNormal.z))
        {
            outFailure = PolygonFaceBuildFailure::InvalidEdgePlane;
            outError = "Mesh face " + std::to_string(faceIndex) + " produced an invalid edge plane.";
            return false;
        }

        edgePlanes.push_back(Plane3i::fromPointNormal(start, edgeNormal));
    }

    Polygon256 polygon(supportPlane, std::move(edgePlanes));
    if (!polygon.isValid())
    {
        outFailure = PolygonFaceBuildFailure::InvalidPolygon;
        outError = "Mesh face " + std::to_string(faceIndex) +
                   " is not a valid strictly convex polygon under the current exact geometry contract.";
        return false;
    }

    outPolygon = std::move(polygon);
    return true;
}

bool appendTriangulatedNonCoplanarFace(
    const std::vector<Vec3i> &faceVertices,
    std::size_t faceIndex,
    std::vector<Polygon256> &outPolygons,
    std::string &outError)
{
    for (std::size_t i = 1; i + 1 < faceVertices.size(); ++i)
    {
        std::vector<Vec3i> triangle;
        triangle.reserve(3);
        triangle.push_back(faceVertices[0]);
        triangle.push_back(faceVertices[i]);
        triangle.push_back(faceVertices[i + 1]);

        Polygon256 polygon;
        PolygonFaceBuildFailure failure = PolygonFaceBuildFailure::None;
        std::string triangleError;
        if (!buildPolygonFromQuantizedFace(triangle, faceIndex, polygon, failure, triangleError))
        {
            outError = "Mesh face " + std::to_string(faceIndex) +
                       " is not coplanar after quantization, and its triangulated fallback failed: " +
                       triangleError;
            return false;
        }

        outPolygons.push_back(std::move(polygon));
    }

    return true;
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

    const std::vector<PlanePoint3i> &cachedVertices = polygon.vertices();
    outVertices.reserve(cachedVertices.size());
    for (const PlanePoint3i &vertex : cachedVertices)
    {
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
                outData.uniqueVertices.push_back(vertex);

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

bool buildStlTrianglesFromPolygonSoup(
    const std::vector<Polygon256> &fragments,
    std::uint64_t coordinateScale,
    std::vector<StlTriangle> &outTriangles,
    std::string &outError)
{
    outTriangles.clear();
    outError.clear();

    for (std::size_t polygonIndex = 0; polygonIndex < fragments.size(); ++polygonIndex)
    {
        std::vector<PlanePoint3i> orderedVertices;
        if (!recoverOrderedPolygonVertices(fragments[polygonIndex], orderedVertices, outError))
        {
            outError = "Failed to recover polygon " + std::to_string(polygonIndex) + ": " + outError;
            return false;
        }
        if (orderedVertices.size() < 3u)
        {
            outError =
                "Failed to triangulate polygon " + std::to_string(polygonIndex) +
                ": fewer than three ordered vertices were recovered.";
            return false;
        }

        const ObjVertex anchor =
            homogeneousPointToObjVertex(orderedVertices[0], coordinateScale);
        for (std::size_t i = 1; i + 1 < orderedVertices.size(); ++i)
        {
            outTriangles.push_back(StlTriangle{
                anchor,
                homogeneousPointToObjVertex(orderedVertices[i], coordinateScale),
                homogeneousPointToObjVertex(orderedVertices[i + 1], coordinateScale)});
        }
    }

    return true;
}
}

bool readObjMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError)
{
    outMesh = ObjMeshData();
    outError.clear();

    tinyobj::ObjReaderConfig config;
    config.triangulate = false;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(path, config))
    {
        std::string message = "Failed to parse OBJ file: " + path;
        if (!reader.Error().empty())
            message += ": " + reader.Error();
        else
            message += ".";
        return failIo(outError, message);
    }

    const tinyobj::attrib_t &attrib = reader.GetAttrib();
    if ((attrib.vertices.size() % 3u) != 0u)
        return failIo(outError, "OBJ vertex attribute array is not divisible by three.");

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
                           outError,
                           "OBJ face index data is incomplete in shape " + std::to_string(shapeIndex) + ".");
            }

            if (faceVertexCount < 3u)
            {
                return failIo(
                           outError,
                           "OBJ face has fewer than three vertices in shape " + std::to_string(shapeIndex) + ".");
            }

            // tinyobjloader 已处理标准 OBJ 索引形式，包括相对索引；
            // 本层只保留项目边界需要的几何位置索引。
            std::vector<std::size_t> face;
            face.reserve(faceVertexCount);
            for (std::size_t i = 0; i < faceVertexCount; ++i)
            {
                const tinyobj::index_t &index = mesh.indices[indexOffset + i];
                if (index.vertex_index < 0)
                {
                    return failIo(
                               outError,
                               "OBJ face contains a missing position index in shape " + std::to_string(shapeIndex) + ".");
                }

                const std::size_t vertexIndex = static_cast<std::size_t>(index.vertex_index);
                if (vertexIndex >= outMesh.vertices.size())
                {
                    return failIo(
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

    return true;
}

bool readStlMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError)
{
    outMesh = ObjMeshData();
    outError.clear();

    std::vector<double> coords;
    std::vector<double> normals;
    std::vector<std::size_t> tris;
    std::vector<std::size_t> solids;
    try
    {
        stl_reader::ReadStlFile(path.c_str(), coords, normals, tris, solids);
    }
    catch (const std::exception &ex)
    {
        return failIo(outError, "Failed to parse STL file: " + path + ": " + ex.what());
    }

    if ((coords.size() % 3u) != 0u)
        return failIo(outError, "STL coordinate array is not divisible by three.");
    if ((tris.size() % 3u) != 0u)
        return failIo(outError, "STL triangle index array is not divisible by three.");
    if (tris.empty())
        return failIo(outError, "STL file does not contain any triangles.");

    outMesh.vertices.reserve(coords.size() / 3u);
    for (std::size_t i = 0; i < coords.size(); i += 3u)
        outMesh.vertices.push_back(ObjVertex{coords[i], coords[i + 1u], coords[i + 2u]});

    outMesh.faces.reserve(tris.size() / 3u);
    for (std::size_t i = 0; i < tris.size(); i += 3u)
    {
        const std::size_t a = tris[i];
        const std::size_t b = tris[i + 1u];
        const std::size_t c = tris[i + 2u];
        if (a >= outMesh.vertices.size() ||
                b >= outMesh.vertices.size() ||
                c >= outMesh.vertices.size())
        {
            return failIo(outError, "STL triangle references a vertex that does not exist.");
        }

        outMesh.faces.push_back({a, b, c});
    }

    return true;
}

bool readMesh(const std::string &path, ObjMeshData &outMesh, std::string &outError)
{
    MeshFileFormat format = MeshFileFormat::Obj;
    if (!detectMeshFileFormatFromPath(path, format, outError))
        return false;

    switch (format)
    {
    case MeshFileFormat::Obj:
        return readObjMesh(path, outMesh, outError);
    case MeshFileFormat::Stl:
        return readStlMesh(path, outMesh, outError);
    }

    return failIo(outError, "Unsupported mesh file format.");
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
            return failIo(outError, "Explicit scale must be a positive integer.");

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
        return failIo(
                   outError,
                   "Mesh coordinates exceed the 26-bit signed input bound even at scale 1.");
    }

    const long double upperBound = static_cast<long double>(kInputCoordinateLimit) / maxAbsCoordinate;
    std::uint64_t scale = 1;
    while (scale <= (std::numeric_limits<std::uint64_t>::max() / 10ULL) &&
            static_cast<long double>(scale * 10ULL) <= upperBound)
        scale *= 10ULL;

    outScale = scale;
    return true;
}

bool computeScaledMeshAABB(
    const ObjMeshData &mesh,
    std::uint64_t sharedScale,
    AABB3i &outAABB,
    std::string &outError)
{
    outAABB = AABB3i();
    outError.clear();

    if (sharedScale == 0)
        return failIo(outError, "Shared scale must be a positive integer.");
    if (mesh.vertices.empty())
        return failIo(outError, "Cannot compute an AABB for a mesh without vertices.");

    for (std::size_t vertexIndex = 0; vertexIndex < mesh.vertices.size(); ++vertexIndex)
    {
        const ObjVertex &vertex = mesh.vertices[vertexIndex];
        Integer xMin;
        Integer xMax;
        Integer yMin;
        Integer yMax;
        Integer zMin;
        Integer zMax;
        if (!computeScaledCoordinateInterval(vertex.x, sharedScale, xMin, xMax, outError) ||
                !computeScaledCoordinateInterval(vertex.y, sharedScale, yMin, yMax, outError) ||
                !computeScaledCoordinateInterval(vertex.z, sharedScale, zMin, zMax, outError))
        {
            outError = "Failed to compute scaled AABB for mesh vertex " +
                       std::to_string(vertexIndex) + ": " + outError;
            outAABB = AABB3i();
            return false;
        }

        appendScaledVertexIntervalToAABB(outAABB, xMin, xMax, yMin, yMax, zMin, zMax);
    }

    return isValidAABB(outAABB);
}

bool buildPolygonSoup(
    const ObjMeshData &mesh,
    std::uint64_t sharedScale,
    std::vector<Polygon256> &outPolygons,
    std::string &outError)
{
    return buildPolygonSoup(mesh, sharedScale, PolygonSoupBuildOptions(), outPolygons, outError);
}

bool buildPolygonSoup(
    const ObjMeshData &mesh,
    std::uint64_t sharedScale,
    const PolygonSoupBuildOptions &options,
    std::vector<Polygon256> &outPolygons,
    std::string &outError)
{
    outPolygons.clear();
    outError.clear();

    if (sharedScale == 0)
        return failIo(outError, "Shared scale must be a positive integer.");

    std::vector<Vec3i> quantizedVertices;
    quantizedVertices.reserve(mesh.vertices.size());
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
    {
        // 顶点先整体量化，后续所有面都复用同一批整数顶点。
        Vec3i quantizedVertex;
        if (!quantizeVertex(mesh.vertices[i], sharedScale, quantizedVertex, outError))
        {
            outError = "Failed to quantize mesh vertex " + std::to_string(i) + ": " + outError;
            return false;
        }

        quantizedVertices.push_back(quantizedVertex);
    }

    outPolygons.reserve(mesh.faces.size());
    std::size_t triangulatedNonCoplanarFaceCount = 0;
    for (std::size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex)
    {
        // 每个输入面都必须在量化后独立满足：无重复点、可定支撑平面、全体共面、严格凸。
        const std::vector<std::size_t> &face = mesh.faces[faceIndex];

        std::vector<Vec3i> faceVertices;
        faceVertices.reserve(face.size());
        for (const std::size_t vertexIndex : face)
        {
            if (vertexIndex >= quantizedVertices.size())
            {
                return failIo(
                           outError,
                           "Mesh face " + std::to_string(faceIndex) +
                           " references an out-of-range quantized vertex.");
            }

            faceVertices.push_back(quantizedVertices[vertexIndex]);
        }

        Polygon256 polygon;
        PolygonFaceBuildFailure failure = PolygonFaceBuildFailure::None;
        std::string faceError;
        if (buildPolygonFromQuantizedFace(faceVertices, faceIndex, polygon, failure, faceError))
        {
            outPolygons.push_back(std::move(polygon));
            continue;
        }

        if (options.triangulateNonCoplanarFaces &&
                failure == PolygonFaceBuildFailure::NonCoplanarAfterQuantization &&
                faceVertices.size() > 3u)
        {
            if (!appendTriangulatedNonCoplanarFace(faceVertices, faceIndex, outPolygons, faceError))
                return failIo(outError, faceError);
            ++triangulatedNonCoplanarFaceCount;
            continue;
        }

        return failIo(outError, faceError);
    }

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
        return failIo(outError, "Coordinate scale must be a positive integer.");
    if (!ensureOutputDirectoryExists(path, "OBJ", outError))
        return false;

    RecoveredPolygonSoupData recovered;
    if (!recoverPolygonSoupData(fragments, recovered, outError))
    {
        outError = "Failed to prepare polygon soup OBJ export: " + outError;
        return false;
    }

    std::ofstream output(path, std::ios::trunc);
    if (!output)
    {
        return failIo(
                   outError,
                   "Failed to open OBJ output file for writing: " + path);
    }

    // 默认保持多边形集合形态：写顶点，再逐面写 OBJ n 边面。
    output << "# Ember exact polygon soup export\n";
    for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        writeObjVertexLine(output, vertex, coordinateScale);

    for (const std::vector<std::size_t> &face : recovered.faces)
    {
        output << "f";
        for (const std::size_t vertexIndex : face)
            output << " " << (vertexIndex + 1);
        output << "\n";
    }

    outFaceCount = recovered.faces.size();
    return true;
}

bool writePolygonSoupStl(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    std::uint64_t coordinateScale)
{
    outFaceCount = 0;
    outError.clear();

    if (coordinateScale == 0)
        return failIo(outError, "Coordinate scale must be a positive integer.");
    if (!ensureOutputDirectoryExists(path, "STL", outError))
        return false;

    std::vector<StlTriangle> triangles;
    if (!buildStlTrianglesFromPolygonSoup(fragments, coordinateScale, triangles, outError))
    {
        outError = "Failed to prepare polygon soup STL export: " + outError;
        return false;
    }
    if (triangles.size() > static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max()))
    {
        return failIo(
                   outError,
                   "STL export triangle count exceeds the 32-bit facet count limit.");
    }

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        return failIo(outError, "Failed to open STL output file for writing: " + path);

    std::array<char, 80> header{};
    const std::string headerText = "Ember exact polygon soup triangulated STL export";
    std::memcpy(
        header.data(),
        headerText.data(),
        std::min(header.size(), headerText.size()));
    output.write(header.data(), static_cast<std::streamsize>(header.size()));
    writeLittleEndianUInt32(output, static_cast<std::uint32_t>(triangles.size()));

    for (const StlTriangle &triangle : triangles)
    {
        const ObjVertex normal = computeTriangleNormal(triangle);
        writeLittleEndianFloat32(output, static_cast<float>(normal.x));
        writeLittleEndianFloat32(output, static_cast<float>(normal.y));
        writeLittleEndianFloat32(output, static_cast<float>(normal.z));

        writeLittleEndianFloat32(output, static_cast<float>(triangle.a.x));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.a.y));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.a.z));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.b.x));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.b.y));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.b.z));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.c.x));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.c.y));
        writeLittleEndianFloat32(output, static_cast<float>(triangle.c.z));
        writeLittleEndianUInt16(output, 0u);
    }

    if (!output)
        return failIo(outError, "Failed to finish writing STL output file: " + path);

    outFaceCount = triangles.size();
    return true;
}

bool writePolygonSoupMesh(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    std::uint64_t coordinateScale)
{
    MeshFileFormat format = MeshFileFormat::Obj;
    if (!detectMeshFileFormatFromPath(path, format, outError))
        return false;

    switch (format)
    {
    case MeshFileFormat::Obj:
        return writePolygonSoupObj(fragments, path, outFaceCount, outError, coordinateScale);
    case MeshFileFormat::Stl:
        return writePolygonSoupStl(fragments, path, outFaceCount, outError, coordinateScale);
    }

    return failIo(outError, "Unsupported mesh file format.");
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
        return failIo(outError, "Coordinate scale must be a positive integer.");

    RecoveredPolygonSoupData recovered;
    if (!recoverPolygonSoupData(fragments, recovered, outError))
    {
        outError = "Failed to prepare OBJ mesh from polygon soup: " + outError;
        return false;
    }

    outMesh.vertices.reserve(recovered.uniqueVertices.size());
    for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        outMesh.vertices.push_back(homogeneousPointToObjVertex(vertex, coordinateScale));

    outMesh.faces = std::move(recovered.faces);

    return true;
}
}
