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
#include <oneapi/tbb/blocked_range.h>
#include <oneapi/tbb/parallel_for.h>
#include <oneapi/tbb/partitioner.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <map>
#include <set>
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

template <typename Function>
void parallelForStatic(std::size_t count, Function &&function)
{
    // 输入/输出阶段的单元成本主要跟顶点、面或片段数量线性相关，静态分块能保持顺序语义稳定。
    oneapi::tbb::parallel_for(
        oneapi::tbb::blocked_range<std::size_t>(0, count),
        [&](const oneapi::tbb::blocked_range<std::size_t> &range)
    {
        for (std::size_t index = range.begin(); index != range.end(); ++index)
            function(index);
    },
        oneapi::tbb::static_partitioner());
}

struct VertexQuantizeResult
{
    Vec3i vertex;
    std::string error;
};

struct VertexAabbResult
{
    AABB3i box;
    std::string error;
};

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

struct FacePolygonBuildResult
{
    std::vector<Polygon256> polygons;
    std::string error;
};

void buildPolygonsFromQuantizedFace(
    const std::vector<Vec3i> &quantizedVertices,
    const std::vector<std::size_t> &face,
    std::size_t faceIndex,
    const PolygonSoupBuildOptions &options,
    FacePolygonBuildResult &outResult)
{
    outResult = FacePolygonBuildResult();

    std::vector<Vec3i> faceVertices;
    faceVertices.reserve(face.size());
    for (const std::size_t vertexIndex : face)
    {
        if (vertexIndex >= quantizedVertices.size())
        {
            outResult.error =
                "Mesh face " + std::to_string(faceIndex) +
                " references an out-of-range quantized vertex.";
            return;
        }

        faceVertices.push_back(quantizedVertices[vertexIndex]);
    }

    Polygon256 polygon;
    PolygonFaceBuildFailure failure = PolygonFaceBuildFailure::None;
    std::string faceError;
    if (buildPolygonFromQuantizedFace(faceVertices, faceIndex, polygon, failure, faceError))
    {
        outResult.polygons.push_back(std::move(polygon));
        return;
    }

    if (options.triangulateNonCoplanarFaces &&
            failure == PolygonFaceBuildFailure::NonCoplanarAfterQuantization &&
            faceVertices.size() > 3u)
    {
        if (!appendTriangulatedNonCoplanarFace(faceVertices, faceIndex, outResult.polygons, faceError))
            outResult.error = faceError;
        return;
    }

    outResult.error = faceError;
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
    const HomPoint4i primitive = primitiveHomPoint(point);
    std::ostringstream stream;
    stream << primitive.x << "|" << primitive.y << "|" << primitive.z << "|" << primitive.w;
    return stream.str();
}

std::string makePlaneKey(const Plane3i &plane)
{
    const Plane3i primitive = primitivePlane(plane);
    std::ostringstream stream;
    stream << primitive.a << "|" << primitive.b << "|" << primitive.c << "|" << primitive.d;
    return stream.str();
}

std::string makeUnorientedPlaneKey(const Plane3i &plane)
{
    Plane3i primitive = primitivePlane(plane);
    if (primitive.a < 0 ||
            (isZero(primitive.a) && (primitive.b < 0 ||
                                     (isZero(primitive.b) && (primitive.c < 0 ||
                                                             (isZero(primitive.c) && primitive.d < 0))))))
    {
        primitive.a = -primitive.a;
        primitive.b = -primitive.b;
        primitive.c = -primitive.c;
        primitive.d = -primitive.d;
    }

    std::ostringstream stream;
    stream << primitive.a << "|" << primitive.b << "|" << primitive.c << "|" << primitive.d;
    return stream.str();
}

std::string makeUnorientedPlanePairKey(const Plane3i &lhs, const Plane3i &rhs)
{
    std::string lhsKey = makeUnorientedPlaneKey(lhs);
    std::string rhsKey = makeUnorientedPlaneKey(rhs);
    if (rhsKey < lhsKey)
        std::swap(lhsKey, rhsKey);
    return lhsKey + "||" + rhsKey;
}

boost::multiprecision::cpp_int toWideInteger(const Integer &value)
{
    return boost::multiprecision::cpp_int(integerToString(value));
}

boost::multiprecision::cpp_int homCoordinateNumerator(const HomPoint4i &point, int axis)
{
    if (axis == 0)
        return toWideInteger(point.x);
    if (axis == 1)
        return toWideInteger(point.y);
    return toWideInteger(point.z);
}

int compareHomCoordinate(const HomPoint4i &lhs, const HomPoint4i &rhs, int axis)
{
    const boost::multiprecision::cpp_int left =
        homCoordinateNumerator(lhs, axis) * toWideInteger(rhs.w);
    const boost::multiprecision::cpp_int right =
        homCoordinateNumerator(rhs, axis) * toWideInteger(lhs.w);
    return (left > right) - (left < right);
}

struct ApproxPoint3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

ApproxPoint3 makeApproxPoint(const PlanePoint3i &point)
{
    const double invW = 1.0 / static_cast<double>(integerToLongDouble(point.x.w));
    return ApproxPoint3{
        static_cast<double>(integerToLongDouble(point.x.x)) * invW,
        static_cast<double>(integerToLongDouble(point.x.y)) * invW,
        static_cast<double>(integerToLongDouble(point.x.z)) * invW};
}

double approxCoordinate(const ApproxPoint3 &point, int axis) noexcept
{
    if (axis == 0)
        return point.x;
    if (axis == 1)
        return point.y;
    return point.z;
}

double reliableEpsilon(double lhs, double rhs) noexcept
{
    const double scale = std::max({1.0, std::fabs(lhs), std::fabs(rhs)});
    return scale * 1e-10;
}

int compareApproxCoordinate(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    std::size_t lhs,
    std::size_t rhs,
    int axis)
{
    const double lhsCoord = approxCoordinate(approxVertices[lhs], axis);
    const double rhsCoord = approxCoordinate(approxVertices[rhs], axis);
    const double diff = lhsCoord - rhsCoord;
    if (std::fabs(diff) > reliableEpsilon(lhsCoord, rhsCoord))
        return (diff > 0.0L) - (diff < 0.0L);

    return compareHomCoordinate(vertices[lhs].x, vertices[rhs].x, axis);
}

int chooseSegmentSortAxis(const PlanePoint3i &start, const PlanePoint3i &end)
{
    if (compareHomCoordinate(start.x, end.x, 0) != 0)
        return 0;
    if (compareHomCoordinate(start.x, end.x, 1) != 0)
        return 1;
    return 2;
}

int chooseSegmentSortAxis(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    std::size_t start,
    std::size_t end)
{
    int bestAxis = 0;
    double bestSpan = std::fabs(approxVertices[start].x - approxVertices[end].x);
    const double ySpan = std::fabs(approxVertices[start].y - approxVertices[end].y);
    const double zSpan = std::fabs(approxVertices[start].z - approxVertices[end].z);
    if (ySpan > bestSpan)
    {
        bestAxis = 1;
        bestSpan = ySpan;
    }
    if (zSpan > bestSpan)
    {
        bestAxis = 2;
        bestSpan = zSpan;
    }

    if (bestSpan > reliableEpsilon(approxCoordinate(approxVertices[start], bestAxis),
                                   approxCoordinate(approxVertices[end], bestAxis)))
        return bestAxis;

    return chooseSegmentSortAxis(vertices[start], vertices[end]);
}

bool pointIsBetweenSegmentEndpoints(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    std::size_t start,
    std::size_t point,
    std::size_t end)
{
    const int axis = chooseSegmentSortAxis(vertices, approxVertices, start, end);
    const int pointVsStart = compareApproxCoordinate(vertices, approxVertices, point, start, axis);
    const int pointVsEnd = compareApproxCoordinate(vertices, approxVertices, point, end, axis);
    if (compareApproxCoordinate(vertices, approxVertices, start, end, axis) < 0)
        return pointVsStart >= 0 && pointVsEnd <= 0;

    return pointVsEnd >= 0 && pointVsStart <= 0;
}

bool pointIsStrictlyInsideEdge(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    const std::vector<std::string> &pointKeys,
    std::size_t start,
    std::size_t point,
    std::size_t end)
{
    if (point == start || point == end)
        return false;
    if (pointKeys[point] == pointKeys[start] || pointKeys[point] == pointKeys[end])
        return false;

    return pointIsBetweenSegmentEndpoints(vertices, approxVertices, start, point, end);
}

struct WideVec3
{
    boost::multiprecision::cpp_int x = 0;
    boost::multiprecision::cpp_int y = 0;
    boost::multiprecision::cpp_int z = 0;
};

WideVec3 wideVectorNumerator(const HomPoint4i &from, const HomPoint4i &to)
{
    const boost::multiprecision::cpp_int fromW = toWideInteger(from.w);
    const boost::multiprecision::cpp_int toW = toWideInteger(to.w);
    return WideVec3{
        toWideInteger(to.x) * fromW - toWideInteger(from.x) * toW,
        toWideInteger(to.y) * fromW - toWideInteger(from.y) * toW,
        toWideInteger(to.z) * fromW - toWideInteger(from.z) * toW};
}

boost::multiprecision::cpp_int orientationAgainstPlane(
    const std::vector<PlanePoint3i> &vertices,
    const Plane3i &plane,
    std::size_t previous,
    std::size_t current,
    std::size_t next)
{
    const WideVec3 first = wideVectorNumerator(vertices[current].x, vertices[next].x);
    const WideVec3 second = wideVectorNumerator(vertices[current].x, vertices[previous].x);
    const boost::multiprecision::cpp_int crossX = first.y * second.z - first.z * second.y;
    const boost::multiprecision::cpp_int crossY = first.z * second.x - first.x * second.z;
    const boost::multiprecision::cpp_int crossZ = first.x * second.y - first.y * second.x;
    return crossX * toWideInteger(plane.a) +
           crossY * toWideInteger(plane.b) +
           crossZ * toWideInteger(plane.c);
}

int signumWide(const boost::multiprecision::cpp_int &value)
{
    return (value > 0) - (value < 0);
}

int orientationSignAgainstPlane(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    const Plane3i &plane,
    std::size_t previous,
    std::size_t current,
    std::size_t next)
{
    const ApproxPoint3 &p = approxVertices[previous];
    const ApproxPoint3 &c = approxVertices[current];
    const ApproxPoint3 &n = approxVertices[next];
    const double ax = n.x - c.x;
    const double ay = n.y - c.y;
    const double az = n.z - c.z;
    const double bx = p.x - c.x;
    const double by = p.y - c.y;
    const double bz = p.z - c.z;
    const double cx = ay * bz - az * by;
    const double cy = az * bx - ax * bz;
    const double cz = ax * by - ay * bx;
    const double nx = static_cast<double>(integerToLongDouble(plane.a));
    const double ny = static_cast<double>(integerToLongDouble(plane.b));
    const double nz = static_cast<double>(integerToLongDouble(plane.c));
    const double value = cx * nx + cy * ny + cz * nz;
    const double scale =
        std::max(1.0, (std::fabs(cx * nx) + std::fabs(cy * ny) + std::fabs(cz * nz)));
    if (std::fabs(value) > scale * 1e-9)
        return (value > 0.0) - (value < 0.0);

    return signumWide(orientationAgainstPlane(vertices, plane, previous, current, next));
}

bool isGeneratedClipEdge(PolygonEdgeProvenance provenance) noexcept
{
    return provenance == PolygonEdgeProvenance::SubdivisionClip ||
           provenance == PolygonEdgeProvenance::ArrangementClip;
}

std::string makeUndirectedEdgeKey(std::size_t lhs, std::size_t rhs)
{
    if (rhs < lhs)
        std::swap(lhs, rhs);
    return std::to_string(lhs) + "|" + std::to_string(rhs);
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
    std::vector<Plane3i> facePlanes;
    std::vector<std::vector<Plane3i>> faceEdgePlanes;
    std::vector<std::vector<PolygonEdgeProvenance>> faceEdgeProvenances;
};

struct RecoveredPolygonBuildResult
{
    std::vector<PlanePoint3i> orderedVertices;
    std::string error;
};

bool recoverPolygonSoupData(
    const std::vector<Polygon256> &fragments,
    RecoveredPolygonSoupData &outData,
    std::string &outError)
{
    outData.uniqueVertices.clear();
    outData.faces.clear();
    outData.facePlanes.clear();
    outData.faceEdgePlanes.clear();
    outData.faceEdgeProvenances.clear();
    outError.clear();

    std::vector<RecoveredPolygonBuildResult> recoveredPolygons(fragments.size());
    parallelForStatic(fragments.size(), [&](std::size_t polygonIndex)
    {
        std::string polygonError;
        if (!recoverOrderedPolygonVertices(
                    fragments[polygonIndex],
                    recoveredPolygons[polygonIndex].orderedVertices,
                    polygonError))
        {
            recoveredPolygons[polygonIndex].error =
                "Failed to recover polygon " + std::to_string(polygonIndex) + ": " + polygonError;
        }
    });

    for (const RecoveredPolygonBuildResult &result : recoveredPolygons)
    {
        if (!result.error.empty())
            return failIo(outError, result.error);
    }

    std::unordered_map<std::string, std::size_t> vertexIndexByKey;
    outData.faces.reserve(fragments.size());
    outData.facePlanes.reserve(fragments.size());
    outData.faceEdgePlanes.reserve(fragments.size());
    outData.faceEdgeProvenances.reserve(fragments.size());
    vertexIndexByKey.reserve(fragments.size() * 4);

    for (std::size_t polygonIndex = 0; polygonIndex < recoveredPolygons.size(); ++polygonIndex)
    {
        const RecoveredPolygonBuildResult &result = recoveredPolygons[polygonIndex];
        std::vector<std::size_t> face;
        face.reserve(result.orderedVertices.size());
        for (const PlanePoint3i &vertex : result.orderedVertices)
        {
            const std::string key = makeHomogeneousPointKey(vertex.x);
            const auto [entry, inserted] = vertexIndexByKey.emplace(key, outData.uniqueVertices.size());
            if (inserted)
                outData.uniqueVertices.push_back(vertex);

            face.push_back(entry->second);
        }

        outData.faces.push_back(std::move(face));
        outData.facePlanes.push_back(fragments[polygonIndex].plane);
        outData.faceEdgePlanes.push_back(fragments[polygonIndex].edgePlanes);
        outData.faceEdgeProvenances.push_back(fragments[polygonIndex].edgeProvenances);
    }

    return true;
}

std::vector<AABB3i> buildVertexAABBs(const std::vector<PlanePoint3i> &vertices)
{
    std::vector<AABB3i> boxes(vertices.size());
    for (std::size_t vertexIndex = 0; vertexIndex < vertices.size(); ++vertexIndex)
        appendPointToAABB(boxes[vertexIndex], vertices[vertexIndex]);
    return boxes;
}

std::vector<std::string> buildPointKeys(const std::vector<PlanePoint3i> &vertices)
{
    std::vector<std::string> keys;
    keys.reserve(vertices.size());
    for (const PlanePoint3i &vertex : vertices)
        keys.push_back(makeHomogeneousPointKey(vertex.x));
    return keys;
}

std::vector<ApproxPoint3> buildApproxPoints(const std::vector<PlanePoint3i> &vertices)
{
    std::vector<ApproxPoint3> points;
    points.reserve(vertices.size());
    for (const PlanePoint3i &vertex : vertices)
        points.push_back(makeApproxPoint(vertex));
    return points;
}

std::unordered_map<std::string, std::vector<std::size_t>> buildVerticesByPlanePair(
    const RecoveredPolygonSoupData &data)
{
    std::unordered_map<std::string, std::vector<std::size_t>> verticesByPlanePair;
    for (std::size_t faceIndex = 0; faceIndex < data.faces.size(); ++faceIndex)
    {
        const std::vector<std::size_t> &face = data.faces[faceIndex];
        if (faceIndex >= data.faceEdgePlanes.size() || data.faceEdgePlanes[faceIndex].size() != face.size())
            continue;

        for (std::size_t edgeIndex = 0; edgeIndex < face.size(); ++edgeIndex)
        {
            const std::string key =
                makeUnorientedPlanePairKey(data.facePlanes[faceIndex], data.faceEdgePlanes[faceIndex][edgeIndex]);
            std::vector<std::size_t> &indices = verticesByPlanePair[key];
            indices.push_back(face[edgeIndex]);
            indices.push_back(face[(edgeIndex + 1u) % face.size()]);
        }
    }

    for (auto &entry : verticesByPlanePair)
    {
        std::vector<std::size_t> &indices = entry.second;
        std::sort(indices.begin(), indices.end());
        indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    }

    return verticesByPlanePair;
}

void removeConsecutiveDuplicateIndices(std::vector<std::size_t> &face)
{
    face.erase(std::unique(face.begin(), face.end()), face.end());
    while (face.size() > 1 && face.front() == face.back())
        face.pop_back();
}

bool insertTJunctionVertices(RecoveredPolygonSoupData &data, std::string &outError)
{
    outError.clear();
    const std::vector<AABB3i> vertexBoxes = buildVertexAABBs(data.uniqueVertices);
    const std::vector<std::string> pointKeys = buildPointKeys(data.uniqueVertices);
    const std::vector<ApproxPoint3> approxVertices = buildApproxPoints(data.uniqueVertices);
    const std::unordered_map<std::string, std::vector<std::size_t>> verticesByPlanePair =
        buildVerticesByPlanePair(data);
    std::vector<std::vector<std::size_t>> refinedFaces(data.faces.size());
    std::vector<std::vector<PolygonEdgeProvenance>> refinedFaceProvenances(data.faces.size());
    std::vector<std::string> faceErrors(data.faces.size());

    parallelForStatic(data.faces.size(), [&](std::size_t faceIndex)
    {
        const std::vector<std::size_t> &face = data.faces[faceIndex];
        if (face.size() < 3u)
        {
            faceErrors[faceIndex] = "Topology recovery found a face with fewer than three vertices.";
            return;
        }
        if (faceIndex >= data.faceEdgePlanes.size() || data.faceEdgePlanes[faceIndex].size() != face.size())
        {
            faceErrors[faceIndex] = "Topology recovery missing original edge planes for a face.";
            return;
        }
        if (faceIndex >= data.faceEdgeProvenances.size() || data.faceEdgeProvenances[faceIndex].size() != face.size())
        {
            faceErrors[faceIndex] = "Topology recovery missing original edge provenance for a face.";
            return;
        }

        std::vector<std::size_t> refinedFace;
        std::vector<PolygonEdgeProvenance> refinedProvenances;
        refinedFace.reserve(face.size());
        refinedProvenances.reserve(face.size());
        for (std::size_t edgeIndex = 0; edgeIndex < face.size(); ++edgeIndex)
        {
            const std::size_t startIndex = face[edgeIndex];
            const std::size_t endIndex = face[(edgeIndex + 1u) % face.size()];
            refinedFace.push_back(startIndex);

            AABB3i edgeBox;
            if (!buildPointPairAABB(data.uniqueVertices[startIndex], data.uniqueVertices[endIndex], edgeBox))
            {
                faceErrors[faceIndex] = "Topology recovery failed to build an edge AABB.";
                return;
            }

            std::vector<std::size_t> interiorPoints;
            const std::string candidateKey =
                makeUnorientedPlanePairKey(data.facePlanes[faceIndex], data.faceEdgePlanes[faceIndex][edgeIndex]);
            const auto candidateEntry = verticesByPlanePair.find(candidateKey);
            if (candidateEntry == verticesByPlanePair.end())
                continue;

            for (std::size_t candidateIndex : candidateEntry->second)
            {
                if (candidateIndex == startIndex || candidateIndex == endIndex)
                    continue;
                if (pointKeys[candidateIndex] == pointKeys[startIndex] ||
                        pointKeys[candidateIndex] == pointKeys[endIndex])
                    continue;
                if (!doAABBsOverlap(edgeBox, vertexBoxes[candidateIndex]))
                    continue;

                const PlanePoint3i &candidate = data.uniqueVertices[candidateIndex];
                if (candidate.classify(data.facePlanes[faceIndex]) != 0)
                    continue;
                if (candidate.classify(data.faceEdgePlanes[faceIndex][edgeIndex]) != 0)
                    continue;
                if (!pointIsStrictlyInsideEdge(
                            data.uniqueVertices,
                            approxVertices,
                            pointKeys,
                            startIndex,
                            candidateIndex,
                            endIndex))
                {
                    continue;
                }

                interiorPoints.push_back(candidateIndex);
            }

            const int axis = chooseSegmentSortAxis(data.uniqueVertices, approxVertices, startIndex, endIndex);
            const bool ascending =
                compareApproxCoordinate(data.uniqueVertices, approxVertices, startIndex, endIndex, axis) < 0;
            std::sort(
                interiorPoints.begin(),
                interiorPoints.end(),
                [&](std::size_t lhs, std::size_t rhs)
                {
                    const int comparison =
                        compareApproxCoordinate(data.uniqueVertices, approxVertices, lhs, rhs, axis);
                    return ascending ? comparison < 0 : comparison > 0;
                });
            interiorPoints.erase(std::unique(interiorPoints.begin(), interiorPoints.end()), interiorPoints.end());
            refinedFace.insert(refinedFace.end(), interiorPoints.begin(), interiorPoints.end());
            const std::size_t segmentCount = interiorPoints.size() + 1u;
            for (std::size_t segmentIndex = 0; segmentIndex < segmentCount; ++segmentIndex)
                refinedProvenances.push_back(data.faceEdgeProvenances[faceIndex][edgeIndex]);
        }

        removeConsecutiveDuplicateIndices(refinedFace);
        if (refinedFace.size() < 3u)
        {
            faceErrors[faceIndex] = "Topology recovery collapsed a face while inserting T-junction vertices.";
            return;
        }
        if (refinedProvenances.size() != refinedFace.size())
        {
            faceErrors[faceIndex] = "Topology recovery produced inconsistent refined edge provenance.";
            return;
        }

        refinedFaces[faceIndex] = std::move(refinedFace);
        refinedFaceProvenances[faceIndex] = std::move(refinedProvenances);
    });

    for (std::size_t faceIndex = 0; faceIndex < faceErrors.size(); ++faceIndex)
    {
        if (!faceErrors[faceIndex].empty())
            return failIo(outError, faceErrors[faceIndex]);

        data.faces[faceIndex] = std::move(refinedFaces[faceIndex]);
        data.faceEdgePlanes[faceIndex].clear();
        data.faceEdgeProvenances[faceIndex] = std::move(refinedFaceProvenances[faceIndex]);
    }

    return true;
}

bool removeRedundantCollinearVertices(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    const Plane3i &plane,
    std::vector<std::size_t> &face,
    std::vector<PolygonEdgeProvenance> &provenances)
{
    removeConsecutiveDuplicateIndices(face);
    if (provenances.size() != face.size())
        return false;
    bool changed = true;
    while (changed && face.size() > 3u)
    {
        changed = false;
        for (std::size_t i = 0; i < face.size(); ++i)
        {
            const std::size_t previous = face[(i + face.size() - 1u) % face.size()];
            const std::size_t current = face[i];
            const std::size_t next = face[(i + 1u) % face.size()];
            if (orientationSignAgainstPlane(vertices, approxVertices, plane, previous, current, next) != 0)
                continue;
            if (!pointIsBetweenSegmentEndpoints(vertices, approxVertices, previous, current, next))
                return false;

            const std::size_t previousEdge = (i + face.size() - 1u) % face.size();
            const PolygonEdgeProvenance mergedProvenance =
                isGeneratedClipEdge(provenances[previousEdge]) ? provenances[previousEdge] : provenances[i];
            provenances[previousEdge] = mergedProvenance;
            face.erase(face.begin() + static_cast<std::ptrdiff_t>(i));
            provenances.erase(provenances.begin() + static_cast<std::ptrdiff_t>(i));
            changed = true;
            break;
        }
    }

    return face.size() >= 3u && provenances.size() == face.size();
}

bool isConvexFace(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    const Plane3i &plane,
    const std::vector<std::size_t> &face)
{
    if (face.size() < 3u)
        return false;

    std::set<std::size_t> uniqueIndices(face.begin(), face.end());
    if (uniqueIndices.size() != face.size())
        return false;

    int expectedSign = 0;
    for (std::size_t i = 0; i < face.size(); ++i)
    {
        const std::size_t previous = face[(i + face.size() - 1u) % face.size()];
        const std::size_t current = face[i];
        const std::size_t next = face[(i + 1u) % face.size()];
        const int sign = orientationSignAgainstPlane(vertices, approxVertices, plane, previous, current, next);
        if (sign == 0)
            return false;
        if (expectedSign == 0)
            expectedSign = sign;
        else if (expectedSign != sign)
            return false;
    }

    return expectedSign != 0;
}

std::vector<std::size_t> buildMergedBoundary(
    const std::vector<std::size_t> &lhs,
    std::size_t lhsFirstSharedEdge,
    std::size_t lhsLastSharedEdge,
    const std::vector<std::size_t> &rhs,
    std::size_t rhsFirstSharedEdge,
    std::size_t rhsLastSharedEdge,
    const std::vector<PolygonEdgeProvenance> &lhsProvenances,
    const std::vector<PolygonEdgeProvenance> &rhsProvenances,
    std::vector<PolygonEdgeProvenance> &outProvenances)
{
    std::vector<std::size_t> boundary;
    outProvenances.clear();
    boundary.reserve(lhs.size() + rhs.size() - 2u);
    outProvenances.reserve(lhs.size() + rhs.size() - 2u);

    std::size_t index = (lhsLastSharedEdge + 1u) % lhs.size();
    boundary.push_back(lhs[index]);
    while (index != lhsFirstSharedEdge)
    {
        outProvenances.push_back(lhsProvenances[index]);
        index = (index + 1u) % lhs.size();
        boundary.push_back(lhs[index]);
    }

    index = (rhsFirstSharedEdge + 1u) % rhs.size();
    while (index != rhsLastSharedEdge)
    {
        outProvenances.push_back(rhsProvenances[index]);
        boundary.push_back(rhs[(index + 1u) % rhs.size()]);
        index = (index + 1u) % rhs.size();
    }

    removeConsecutiveDuplicateIndices(boundary);
    return boundary;
}

bool areReverseEdges(
    const std::vector<std::size_t> &lhs,
    std::size_t lhsEdge,
    const std::vector<std::size_t> &rhs,
    std::size_t rhsEdge)
{
    return lhs[lhsEdge] == rhs[(rhsEdge + 1u) % rhs.size()] &&
           lhs[(lhsEdge + 1u) % lhs.size()] == rhs[rhsEdge];
}

struct SharedBoundaryChain
{
    std::size_t lhsFirstEdge = 0;
    std::size_t lhsLastEdge = 0;
    std::size_t rhsFirstEdge = 0;
    std::size_t rhsLastEdge = 0;
    std::size_t edgeCount = 0;
};

bool findSharedBoundaryChain(
    const std::vector<std::size_t> &lhs,
    std::size_t lhsEdge,
    const std::vector<std::size_t> &rhs,
    std::size_t rhsEdge,
    SharedBoundaryChain &outChain)
{
    // T 形连接补齐后，一条 AABB 裁剪边可能变成多段连续反向子边；合并时要整段删除。
    if (!areReverseEdges(lhs, lhsEdge, rhs, rhsEdge))
        return false;

    outChain.lhsFirstEdge = lhsEdge;
    outChain.lhsLastEdge = lhsEdge;
    outChain.rhsFirstEdge = rhsEdge;
    outChain.rhsLastEdge = rhsEdge;
    outChain.edgeCount = 1u;
    const std::size_t maxSharedEdgeCount = std::min(lhs.size(), rhs.size());

    while (true)
    {
        if (outChain.edgeCount >= maxSharedEdgeCount)
            break;

        const std::size_t lhsPrevious =
            (outChain.lhsFirstEdge + lhs.size() - 1u) % lhs.size();
        const std::size_t rhsNext = (outChain.rhsFirstEdge + 1u) % rhs.size();
        if (!areReverseEdges(lhs, lhsPrevious, rhs, rhsNext))
            break;

        outChain.lhsFirstEdge = lhsPrevious;
        outChain.rhsFirstEdge = rhsNext;
        ++outChain.edgeCount;
    }

    while (true)
    {
        if (outChain.edgeCount >= maxSharedEdgeCount)
            break;

        const std::size_t lhsNext = (outChain.lhsLastEdge + 1u) % lhs.size();
        const std::size_t rhsPrevious =
            (outChain.rhsLastEdge + rhs.size() - 1u) % rhs.size();
        if (!areReverseEdges(lhs, lhsNext, rhs, rhsPrevious))
            break;

        outChain.lhsLastEdge = lhsNext;
        outChain.rhsLastEdge = rhsPrevious;
        ++outChain.edgeCount;
    }

    return outChain.edgeCount < lhs.size() && outChain.edgeCount < rhs.size();
}

bool sharedBoundaryChainHasGeneratedEdge(
    const SharedBoundaryChain &chain,
    const std::vector<PolygonEdgeProvenance> &lhsProvenances,
    const std::vector<PolygonEdgeProvenance> &rhsProvenances)
{
    if (lhsProvenances.empty() || rhsProvenances.empty())
        return false;

    std::size_t index = chain.lhsFirstEdge;
    for (std::size_t count = 0; count < chain.edgeCount; ++count)
    {
        if (index < lhsProvenances.size() && isGeneratedClipEdge(lhsProvenances[index]))
            return true;
        index = (index + 1u) % lhsProvenances.size();
    }

    index = chain.rhsLastEdge;
    for (std::size_t count = 0; count < chain.edgeCount; ++count)
    {
        if (index < rhsProvenances.size() && isGeneratedClipEdge(rhsProvenances[index]))
            return true;
        index = (index + 1u) % rhsProvenances.size();
    }

    return false;
}

bool tryMergeFaces(
    const std::vector<PlanePoint3i> &vertices,
    const std::vector<ApproxPoint3> &approxVertices,
    const Plane3i &plane,
    const std::vector<std::size_t> &lhs,
    std::size_t lhsEdge,
    const std::vector<std::size_t> &rhs,
    std::size_t rhsEdge,
    const std::vector<PolygonEdgeProvenance> &lhsProvenances,
    const std::vector<PolygonEdgeProvenance> &rhsProvenances,
    std::vector<std::size_t> &outMerged,
    std::vector<PolygonEdgeProvenance> &outProvenances)
{
    SharedBoundaryChain chain;
    if (!findSharedBoundaryChain(lhs, lhsEdge, rhs, rhsEdge, chain))
        return false;
    if (!sharedBoundaryChainHasGeneratedEdge(chain, lhsProvenances, rhsProvenances))
        return false;

    std::vector<PolygonEdgeProvenance> provenances;
    std::vector<std::size_t> boundary = buildMergedBoundary(
                                            lhs,
                                            chain.lhsFirstEdge,
                                            chain.lhsLastEdge,
                                            rhs,
                                            chain.rhsFirstEdge,
                                            chain.rhsLastEdge,
                                            lhsProvenances,
                                            rhsProvenances,
                                            provenances);
    if (!removeRedundantCollinearVertices(vertices, approxVertices, plane, boundary, provenances))
        return false;
    if (!isConvexFace(vertices, approxVertices, plane, boundary))
        return false;
    if (provenances.size() != boundary.size())
        return false;

    outMerged = std::move(boundary);
    outProvenances = std::move(provenances);
    return true;
}

void mergeConvexCoplanarFaces(RecoveredPolygonSoupData &data)
{
    struct EdgeOwner
    {
        std::size_t face = 0;
        std::size_t edge = 0;
    };

    const std::vector<ApproxPoint3> approxVertices = buildApproxPoints(data.uniqueVertices);
    bool changed = true;
    while (changed)
    {
        changed = false;
        std::map<std::string, std::vector<EdgeOwner>> ownersByEdge;
        for (std::size_t faceIndex = 0; faceIndex < data.faces.size(); ++faceIndex)
        {
            const std::vector<std::size_t> &face = data.faces[faceIndex];
            for (std::size_t edgeIndex = 0; edgeIndex < face.size(); ++edgeIndex)
            {
                const std::size_t start = face[edgeIndex];
                const std::size_t end = face[(edgeIndex + 1u) % face.size()];
                ownersByEdge[makeUndirectedEdgeKey(start, end)].push_back(EdgeOwner{faceIndex, edgeIndex});
            }
        }

        std::vector<bool> consumed(data.faces.size(), false);
        std::vector<bool> removed(data.faces.size(), false);
        std::vector<std::vector<std::size_t>> replacements(data.faces.size());
        std::vector<std::vector<PolygonEdgeProvenance>> replacementProvenances(data.faces.size());
        for (const auto &entry : ownersByEdge)
        {
            const std::vector<EdgeOwner> &owners = entry.second;
            if (owners.size() < 2u)
                continue;

            for (std::size_t lhsOwnerIndex = 0; lhsOwnerIndex < owners.size(); ++lhsOwnerIndex)
            {
                const EdgeOwner lhsOwner = owners[lhsOwnerIndex];
                for (std::size_t rhsOwnerIndex = lhsOwnerIndex + 1u; rhsOwnerIndex < owners.size(); ++rhsOwnerIndex)
                {
                    const EdgeOwner rhsOwner = owners[rhsOwnerIndex];
                    if (lhsOwner.face == rhsOwner.face)
                        continue;
                    if (consumed[lhsOwner.face] || consumed[rhsOwner.face])
                        continue;
                    if (makePlaneKey(data.facePlanes[lhsOwner.face]) != makePlaneKey(data.facePlanes[rhsOwner.face]))
                        continue;

                    const std::size_t keep = std::min(lhsOwner.face, rhsOwner.face);
                    const std::size_t remove = std::max(lhsOwner.face, rhsOwner.face);
                    const std::vector<std::size_t> &keepFace = data.faces[keep];
                    const std::vector<std::size_t> &removeFace = data.faces[remove];
                    const std::size_t keepEdge = keep == lhsOwner.face ? lhsOwner.edge : rhsOwner.edge;
                    const std::size_t removeEdge = remove == lhsOwner.face ? lhsOwner.edge : rhsOwner.edge;

                    // 同一无向边只在方向相反时表示两个同向共面面的公共边。
                    const std::size_t keepStart = keepFace[keepEdge];
                    const std::size_t keepEnd = keepFace[(keepEdge + 1u) % keepFace.size()];
                    const std::size_t removeStart = removeFace[removeEdge];
                    const std::size_t removeEnd = removeFace[(removeEdge + 1u) % removeFace.size()];
                    if (keepStart != removeEnd || keepEnd != removeStart)
                        continue;

                    std::vector<std::size_t> merged;
                    std::vector<PolygonEdgeProvenance> mergedProvenances;
                    if (!tryMergeFaces(
                                data.uniqueVertices,
                                approxVertices,
                                data.facePlanes[keep],
                                keepFace,
                                keepEdge,
                                removeFace,
                                removeEdge,
                                data.faceEdgeProvenances[keep],
                                data.faceEdgeProvenances[remove],
                                merged,
                                mergedProvenances))
                    {
                        continue;
                    }

                    replacements[keep] = std::move(merged);
                    replacementProvenances[keep] = std::move(mergedProvenances);
                    consumed[keep] = true;
                    consumed[remove] = true;
                    removed[remove] = true;
                    changed = true;
                    break;
                }
            }
        }

        if (changed)
        {
            std::vector<std::vector<std::size_t>> nextFaces;
            std::vector<Plane3i> nextPlanes;
            std::vector<std::vector<Plane3i>> nextEdgePlanes;
            std::vector<std::vector<PolygonEdgeProvenance>> nextEdgeProvenances;
            nextFaces.reserve(data.faces.size());
            nextPlanes.reserve(data.facePlanes.size());
            nextEdgePlanes.reserve(data.faceEdgePlanes.size());
            nextEdgeProvenances.reserve(data.faceEdgeProvenances.size());

            for (std::size_t faceIndex = 0; faceIndex < data.faces.size(); ++faceIndex)
            {
                if (removed[faceIndex])
                    continue;

                if (!replacements[faceIndex].empty())
                {
                    nextFaces.push_back(std::move(replacements[faceIndex]));
                    nextEdgeProvenances.push_back(std::move(replacementProvenances[faceIndex]));
                }
                else
                {
                    nextFaces.push_back(std::move(data.faces[faceIndex]));
                    nextEdgeProvenances.push_back(std::move(data.faceEdgeProvenances[faceIndex]));
                }
                nextPlanes.push_back(data.facePlanes[faceIndex]);
                nextEdgePlanes.emplace_back();
            }

            data.faces = std::move(nextFaces);
            data.facePlanes = std::move(nextPlanes);
            data.faceEdgePlanes = std::move(nextEdgePlanes);
            data.faceEdgeProvenances = std::move(nextEdgeProvenances);
        }
    }
}

bool applyTopologyMode(
    RecoveredPolygonSoupData &data,
    PolygonSoupTopologyMode mode,
    std::string &outError)
{
    outError.clear();
    if (mode == PolygonSoupTopologyMode::Raw)
        return true;

    if (!insertTJunctionVertices(data, outError))
        return false;

    if (mode == PolygonSoupTopologyMode::ConformingMergeConvex)
        mergeConvexCoplanarFaces(data);

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

bool buildStlTrianglesFromRecoveredData(
    const RecoveredPolygonSoupData &recovered,
    std::uint64_t coordinateScale,
    std::vector<StlTriangle> &outTriangles,
    std::string &outError)
{
    outTriangles.clear();
    outError.clear();

    std::size_t triangleCount = 0;
    for (std::size_t polygonIndex = 0; polygonIndex < recovered.faces.size(); ++polygonIndex)
    {
        if (recovered.faces[polygonIndex].size() < 3u)
        {
            outError =
                "Failed to triangulate polygon " + std::to_string(polygonIndex) +
                ": fewer than three ordered vertices were recovered.";
            return false;
        }

        triangleCount += recovered.faces[polygonIndex].size() - 2u;
    }

    outTriangles.reserve(triangleCount);
    for (const std::vector<std::size_t> &face : recovered.faces)
    {
        const ObjVertex anchor =
            homogeneousPointToObjVertex(recovered.uniqueVertices[face[0]], coordinateScale);
        for (std::size_t i = 1; i + 1 < face.size(); ++i)
        {
            outTriangles.push_back(StlTriangle{
                anchor,
                homogeneousPointToObjVertex(recovered.uniqueVertices[face[i]], coordinateScale),
                homogeneousPointToObjVertex(recovered.uniqueVertices[face[i + 1u]], coordinateScale)});
        }
    }

    return true;
}
}

const char *toString(PolygonSoupTopologyMode mode) noexcept
{
    switch (mode)
    {
    case PolygonSoupTopologyMode::Raw:
        return "raw";
    case PolygonSoupTopologyMode::Conforming:
        return "conforming";
    case PolygonSoupTopologyMode::ConformingMergeConvex:
        return "conforming-merge-convex";
    }

    return "raw";
}

bool parsePolygonSoupTopologyMode(const std::string &token, PolygonSoupTopologyMode &outMode) noexcept
{
    if (token == "raw")
    {
        outMode = PolygonSoupTopologyMode::Raw;
        return true;
    }
    if (token == "conforming")
    {
        outMode = PolygonSoupTopologyMode::Conforming;
        return true;
    }
    if (token == "conforming-merge-convex")
    {
        outMode = PolygonSoupTopologyMode::ConformingMergeConvex;
        return true;
    }

    return false;
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

    std::vector<VertexAabbResult> vertexResults(mesh.vertices.size());
    parallelForStatic(mesh.vertices.size(), [&](std::size_t vertexIndex)
    {
        const ObjVertex &vertex = mesh.vertices[vertexIndex];
        Integer xMin;
        Integer xMax;
        Integer yMin;
        Integer yMax;
        Integer zMin;
        Integer zMax;
        std::string vertexError;
        if (!computeScaledCoordinateInterval(vertex.x, sharedScale, xMin, xMax, vertexError) ||
                !computeScaledCoordinateInterval(vertex.y, sharedScale, yMin, yMax, vertexError) ||
                !computeScaledCoordinateInterval(vertex.z, sharedScale, zMin, zMax, vertexError))
        {
            vertexResults[vertexIndex].error =
                "Failed to compute scaled AABB for mesh vertex " +
                std::to_string(vertexIndex) + ": " + vertexError;
            return;
        }

        appendScaledVertexIntervalToAABB(
            vertexResults[vertexIndex].box,
            xMin,
            xMax,
            yMin,
            yMax,
            zMin,
            zMax);
    });

    for (const VertexAabbResult &result : vertexResults)
    {
        if (!result.error.empty())
        {
            outAABB = AABB3i();
            return failIo(outError, result.error);
        }

        mergeAABB(outAABB, result.box);
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

    std::vector<VertexQuantizeResult> quantizedResults(mesh.vertices.size());
    parallelForStatic(mesh.vertices.size(), [&](std::size_t i)
    {
        // 顶点先整体量化，后续所有面都复用同一批整数顶点。
        std::string vertexError;
        if (!quantizeVertex(mesh.vertices[i], sharedScale, quantizedResults[i].vertex, vertexError))
        {
            quantizedResults[i].error =
                "Failed to quantize mesh vertex " + std::to_string(i) + ": " + vertexError;
        }
    });

    std::vector<Vec3i> quantizedVertices(mesh.vertices.size());
    for (std::size_t i = 0; i < quantizedResults.size(); ++i)
    {
        if (!quantizedResults[i].error.empty())
            return failIo(outError, quantizedResults[i].error);

        quantizedVertices[i] = quantizedResults[i].vertex;
    }

    std::vector<FacePolygonBuildResult> faceResults(mesh.faces.size());
    parallelForStatic(mesh.faces.size(), [&](std::size_t faceIndex)
    {
        // 每个输入面都必须在量化后独立满足：无重复点、可定支撑平面、全体共面、严格凸。
        buildPolygonsFromQuantizedFace(
            quantizedVertices,
            mesh.faces[faceIndex],
            faceIndex,
            options,
            faceResults[faceIndex]);
    });

    std::size_t polygonCount = 0;
    for (const FacePolygonBuildResult &result : faceResults)
    {
        if (!result.error.empty())
            return failIo(outError, result.error);

        polygonCount += result.polygons.size();
    }

    outPolygons.reserve(polygonCount);
    for (FacePolygonBuildResult &result : faceResults)
    {
        outPolygons.insert(
            outPolygons.end(),
            std::make_move_iterator(result.polygons.begin()),
            std::make_move_iterator(result.polygons.end()));
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
    PolygonSoupExportOptions options;
    options.coordinateScale = coordinateScale;
    return writePolygonSoupObj(fragments, path, outFaceCount, outError, options);
}

bool writePolygonSoupObj(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    const PolygonSoupExportOptions &options)
{
    outFaceCount = 0;
    outError.clear();

    if (options.coordinateScale == 0)
        return failIo(outError, "Coordinate scale must be a positive integer.");
    if (!ensureOutputDirectoryExists(path, "OBJ", outError))
        return false;

    RecoveredPolygonSoupData recovered;
    if (!recoverPolygonSoupData(fragments, recovered, outError))
    {
        outError = "Failed to prepare polygon soup OBJ export: " + outError;
        return false;

    }
    if (!applyTopologyMode(recovered, options.topologyMode, outError))
    {
        outError = "Failed to apply polygon soup topology recovery: " + outError;
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
        writeObjVertexLine(output, vertex, options.coordinateScale);

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
    PolygonSoupExportOptions options;
    options.coordinateScale = coordinateScale;
    return writePolygonSoupStl(fragments, path, outFaceCount, outError, options);
}

bool writePolygonSoupStl(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    const PolygonSoupExportOptions &options)
{
    outFaceCount = 0;
    outError.clear();

    if (options.coordinateScale == 0)
        return failIo(outError, "Coordinate scale must be a positive integer.");
    if (!ensureOutputDirectoryExists(path, "STL", outError))
        return false;

    RecoveredPolygonSoupData recovered;
    if (!recoverPolygonSoupData(fragments, recovered, outError))
    {
        outError = "Failed to prepare polygon soup STL export: " + outError;
        return false;
    }
    if (!applyTopologyMode(recovered, options.topologyMode, outError))
    {
        outError = "Failed to apply polygon soup topology recovery: " + outError;
        return false;
    }

    std::vector<StlTriangle> triangles;
    if (!buildStlTrianglesFromRecoveredData(recovered, options.coordinateScale, triangles, outError))
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
    PolygonSoupExportOptions options;
    options.coordinateScale = coordinateScale;
    return writePolygonSoupMesh(fragments, path, outFaceCount, outError, options);
}

bool writePolygonSoupMesh(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::size_t &outFaceCount,
    std::string &outError,
    const PolygonSoupExportOptions &options)
{
    MeshFileFormat format = MeshFileFormat::Obj;
    if (!detectMeshFileFormatFromPath(path, format, outError))
        return false;

    switch (format)
    {
    case MeshFileFormat::Obj:
        return writePolygonSoupObj(fragments, path, outFaceCount, outError, options);
    case MeshFileFormat::Stl:
        return writePolygonSoupStl(fragments, path, outFaceCount, outError, options);
    }

    return failIo(outError, "Unsupported mesh file format.");
}

bool buildObjMeshFromPolygonSoup(
    const std::vector<Polygon256> &fragments,
    ObjMeshData &outMesh,
    std::string &outError,
    std::uint64_t coordinateScale)
{
    PolygonSoupExportOptions options;
    options.coordinateScale = coordinateScale;
    return buildObjMeshFromPolygonSoup(fragments, outMesh, outError, options);
}

bool buildObjMeshFromPolygonSoup(
    const std::vector<Polygon256> &fragments,
    ObjMeshData &outMesh,
    std::string &outError,
    const PolygonSoupExportOptions &options)
{
    outMesh = ObjMeshData();
    outError.clear();

    if (options.coordinateScale == 0)
        return failIo(outError, "Coordinate scale must be a positive integer.");

    RecoveredPolygonSoupData recovered;
    if (!recoverPolygonSoupData(fragments, recovered, outError))
    {
        outError = "Failed to prepare OBJ mesh from polygon soup: " + outError;
        return false;

    }
    if (!applyTopologyMode(recovered, options.topologyMode, outError))
    {
        outError = "Failed to apply polygon soup topology recovery: " + outError;
        return false;
    }

    outMesh.vertices.reserve(recovered.uniqueVertices.size());
    for (const PlanePoint3i &vertex : recovered.uniqueVertices)
        outMesh.vertices.push_back(homogeneousPointToObjVertex(vertex, options.coordinateScale));

    outMesh.faces = std::move(recovered.faces);

    return true;
}
}
