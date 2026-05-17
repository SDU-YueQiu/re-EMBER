/**
 * @file nef_postprocess.cpp
 * @brief 实现应用层 CGAL Nef 导出拓扑和 oracle 转换工具。
 */
#include "nef_postprocess.h"

#include "math/math256.h"

#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>

#include <algorithm>
#include <cstddef>
#include <map>
#include <stdexcept>
#include <utility>

namespace ember::app
{
namespace
{
ExactKernel::FT parseIntegerKernelFT(const std::string &text)
{
    bool negative = false;
    std::size_t offset = 0;
    if (!text.empty() && text[0] == '-')
    {
        negative = true;
        offset = 1;
    }

    ExactKernel::FT value(0);
    for (std::size_t i = offset; i < text.size(); ++i)
    {
        if (text[i] < '0' || text[i] > '9')
            throw std::runtime_error("Invalid integer text while converting to CGAL exact number.");
        value = value * 10 + static_cast<int>(text[i] - '0');
    }

    return negative ? -value : value;
}

ExactKernel::FT toKernelFT(const Integer &numerator, const Integer &denominator)
{
    return parseIntegerKernelFT(integerToString(numerator)) /
           parseIntegerKernelFT(integerToString(denominator));
}

std::string homPointKey(const HomPoint4i &point)
{
    const HomPoint4i primitive = primitiveHomPoint(point);
    return integerToString(primitive.x) + "/" +
           integerToString(primitive.y) + "/" +
           integerToString(primitive.z) + "/" +
           integerToString(primitive.w);
}

ExactKernel::Point_3 toKernelPoint(const PlanePoint3i &point)
{
    if (!point.hasUniqueIntersection() || isZero(point.x.w))
        throw std::runtime_error("Polygon contains a vertex without a unique finite homogeneous point.");

    return ExactKernel::Point_3(
               toKernelFT(point.x.x, point.x.w),
               toKernelFT(point.x.y, point.x.w),
               toKernelFT(point.x.z, point.x.w));
}

ExactKernel::FT coordinateByAxis(const ExactKernel::Point_3 &point, int axis)
{
    if (axis == 0)
        return point.x();
    if (axis == 1)
        return point.y();
    return point.z();
}

int chooseSegmentSortAxis(const ExactKernel::Point_3 &start, const ExactKernel::Point_3 &end)
{
    if (start.x() != end.x())
        return 0;
    if (start.y() != end.y())
        return 1;
    return 2;
}

bool pointIsOnSegment(
    const ExactKernel::Point_3 &start,
    const ExactKernel::Point_3 &point,
    const ExactKernel::Point_3 &end)
{
    return CGAL::collinear(start, point, end) &&
           CGAL::collinear_are_ordered_along_line(start, point, end);
}

bool isDegenerateTriangle(
    const std::vector<ExactKernel::Point_3> &points,
    std::size_t a,
    std::size_t b,
    std::size_t c)
{
    return a == b || b == c || a == c ||
           CGAL::collinear(points[a], points[b], points[c]);
}

bool hasNondegenerateTriple(
    const std::vector<ExactKernel::Point_3> &points,
    const std::vector<std::size_t> &face)
{
    if (face.size() < 3u)
        return false;
    for (std::size_t i = 0; i < face.size(); ++i)
    {
        const std::size_t a = face[(i + face.size() - 1u) % face.size()];
        const std::size_t b = face[i];
        const std::size_t c = face[(i + 1u) % face.size()];
        if (!isDegenerateTriangle(points, a, b, c))
            return true;
    }
    return false;
}

void appendConvexEarTriangles(
    const std::vector<ExactKernel::Point_3> &points,
    const std::vector<std::size_t> &face,
    std::vector<std::vector<std::size_t>> &triangles,
    const char *label,
    std::size_t polygonIndex)
{
    std::vector<std::size_t> remaining = face;
    while (remaining.size() > 3u)
    {
        bool clipped = false;
        for (std::size_t i = 0; i < remaining.size(); ++i)
        {
            const std::size_t a = remaining[(i + remaining.size() - 1u) % remaining.size()];
            const std::size_t b = remaining[i];
            const std::size_t c = remaining[(i + 1u) % remaining.size()];
            if (isDegenerateTriangle(points, a, b, c))
                continue;

            std::vector<std::size_t> candidateRemaining = remaining;
            candidateRemaining.erase(candidateRemaining.begin() + static_cast<std::ptrdiff_t>(i));
            if (candidateRemaining.size() >= 3u && !hasNondegenerateTriple(points, candidateRemaining))
                continue;

            triangles.push_back({a, b, c});
            remaining = std::move(candidateRemaining);
            clipped = true;
            break;
        }
        if (!clipped)
            throw std::runtime_error(
                std::string("Failed to triangulate ") + label + " polygon " + std::to_string(polygonIndex) +
                " after exact edge refinement; remaining_vertices=" + std::to_string(remaining.size()) + ".");
    }

    if (!isDegenerateTriangle(points, remaining[0], remaining[1], remaining[2]))
        triangles.push_back({remaining[0], remaining[1], remaining[2]});
}
}

NefPolyhedron makeNefFromExactMesh(
    const ExactMeshData &mesh,
    const char *label,
    const NefBuildOptions &options)
{
    if (mesh.faces.empty())
        return NefPolyhedron(NefPolyhedron::EMPTY);

    std::vector<ExactKernel::Point_3> points;
    std::vector<std::size_t> pointRemap(mesh.vertices.size(), 0u);
    std::map<std::string, std::size_t> pointIndexByKey;
    points.reserve(mesh.vertices.size());

    for (std::size_t vertexIndex = 0; vertexIndex < mesh.vertices.size(); ++vertexIndex)
    {
        const PlanePoint3i &vertex = mesh.vertices[vertexIndex];
        const std::string key = homPointKey(vertex.x);
        const auto found = pointIndexByKey.find(key);
        if (found != pointIndexByKey.end())
        {
            pointRemap[vertexIndex] = found->second;
            continue;
        }

        const std::size_t index = points.size();
        pointIndexByKey.emplace(key, index);
        pointRemap[vertexIndex] = index;
        points.push_back(toKernelPoint(vertex));
    }

    std::vector<std::vector<std::size_t>> polygonFaces;
    polygonFaces.reserve(mesh.faces.size());
    for (const std::vector<std::size_t> &sourceFace : mesh.faces)
    {
        if (sourceFace.size() < 3u)
            throw std::runtime_error("Exact mesh contains a face with fewer than three vertices.");

        std::vector<std::size_t> face;
        face.reserve(sourceFace.size());
        for (const std::size_t vertexIndex : sourceFace)
        {
            if (vertexIndex >= pointRemap.size())
                throw std::runtime_error("Exact mesh contains an out-of-range face vertex index.");
            face.push_back(pointRemap[vertexIndex]);
        }

        polygonFaces.push_back(std::move(face));
    }

    std::vector<std::vector<std::size_t>> triangles;
    for (std::size_t polygonIndex = 0; polygonIndex < polygonFaces.size(); ++polygonIndex)
    {
        const std::vector<std::size_t> &face = polygonFaces[polygonIndex];
        std::vector<std::size_t> refinedFace;
        refinedFace.reserve(face.size());
        if (!options.refineEdgeInteriorPoints)
        {
            refinedFace = face;
        }
        else
        {
            for (std::size_t edgeIndex = 0; edgeIndex < face.size(); ++edgeIndex)
            {
                const std::size_t startIndex = face[edgeIndex];
                const std::size_t endIndex = face[(edgeIndex + 1u) % face.size()];
                refinedFace.push_back(startIndex);

                std::vector<std::size_t> edgeInteriorPoints;
                const ExactKernel::Point_3 &start = points[startIndex];
                const ExactKernel::Point_3 &end = points[endIndex];
                for (std::size_t candidateIndex = 0; candidateIndex < points.size(); ++candidateIndex)
                {
                    if (candidateIndex == startIndex || candidateIndex == endIndex)
                        continue;
                    if (pointIsOnSegment(start, points[candidateIndex], end))
                        edgeInteriorPoints.push_back(candidateIndex);
                }

                const int sortAxis = chooseSegmentSortAxis(start, end);
                const bool ascending = coordinateByAxis(start, sortAxis) < coordinateByAxis(end, sortAxis);
                std::sort(
                    edgeInteriorPoints.begin(),
                    edgeInteriorPoints.end(),
                    [&](std::size_t lhs, std::size_t rhs)
                    {
                        const ExactKernel::FT lhsCoord = coordinateByAxis(points[lhs], sortAxis);
                        const ExactKernel::FT rhsCoord = coordinateByAxis(points[rhs], sortAxis);
                        return ascending ? lhsCoord < rhsCoord : rhsCoord < lhsCoord;
                    });

                refinedFace.insert(refinedFace.end(), edgeInteriorPoints.begin(), edgeInteriorPoints.end());
            }
        }

        refinedFace.erase(
            std::unique(refinedFace.begin(), refinedFace.end()),
            refinedFace.end());
        if (refinedFace.size() > 1u && refinedFace.front() == refinedFace.back())
            refinedFace.pop_back();
        if (refinedFace.size() < 3u)
            continue;

        appendConvexEarTriangles(points, refinedFace, triangles, label, polygonIndex);
    }

    if (triangles.empty())
    {
        if (options.rejectEmptyRegularizedResult)
        {
            throw std::runtime_error(
                std::string("CGAL Nef ") + label +
                " output topology produced no triangles from a non-empty exact mesh.");
        }
        return NefPolyhedron(NefPolyhedron::EMPTY);
    }

    SurfaceMesh surfaceMesh;
    CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, triangles, surfaceMesh);
    if (surfaceMesh.number_of_faces() == 0)
    {
        if (options.rejectEmptyRegularizedResult)
        {
            throw std::runtime_error(
                std::string("CGAL Nef ") + label +
                " output topology produced an empty surface mesh from a non-empty exact mesh.");
        }
        return NefPolyhedron(NefPolyhedron::EMPTY);
    }
    if (CGAL::is_closed(surfaceMesh) && CGAL::is_triangle_mesh(surfaceMesh) &&
            CGAL::Polygon_mesh_processing::does_bound_a_volume(surfaceMesh))
    {
        CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(surfaceMesh);
    }

    NefPolyhedron nef = NefPolyhedron(surfaceMesh).regularization();
    if (options.rejectEmptyRegularizedResult && nef.is_empty())
    {
        throw std::runtime_error(
            std::string("CGAL Nef ") + label +
            " output topology regularized a non-empty exact mesh to an empty set; likely non-closed, misoriented, or invalid candidate soup.");
    }
    return nef;
}

NefPolyhedron makeNefFromPolygons(const std::vector<Polygon256> &polygons, const char *label)
{
    ExactMeshData mesh;
    mesh.faces.reserve(polygons.size());

    std::map<std::string, std::size_t> pointIndexByKey;
    auto getPointIndex = [&](const PlanePoint3i &point) -> std::size_t
    {
        const std::string key = homPointKey(point.x);
        const auto found = pointIndexByKey.find(key);
        if (found != pointIndexByKey.end())
            return found->second;

        const std::size_t index = mesh.vertices.size();
        pointIndexByKey.emplace(key, index);
        mesh.vertices.push_back(point);
        return index;
    };

    for (const Polygon256 &polygon : polygons)
    {
        const std::vector<PlanePoint3i> &vertices = polygon.vertices();
        if (vertices.size() < 3u)
            throw std::runtime_error("Polygon has fewer than three vertices.");

        std::vector<std::size_t> face;
        face.reserve(vertices.size());
        for (const PlanePoint3i &vertex : vertices)
            face.push_back(getPointIndex(vertex));

        mesh.faces.push_back(std::move(face));
    }

    return makeNefFromExactMesh(mesh, label);
}

NefPolyhedron applyBoolean(const NefPolyhedron &lhs, const NefPolyhedron &rhs, BoolOp op)
{
    switch (op)
    {
    case BoolOp::Union:
        return (lhs + rhs).regularization();
    case BoolOp::Intersection:
        return (lhs * rhs).regularization();
    case BoolOp::Difference:
        return (lhs - rhs).regularization();
    }

    return NefPolyhedron(NefPolyhedron::EMPTY);
}

ObjMeshData makeObjMeshData(const NefPolyhedron &nef, std::uint64_t coordinateScale)
{
    if (coordinateScale == 0u)
        throw std::runtime_error("Coordinate scale must be a positive integer.");
    if (nef.is_empty())
        return ObjMeshData();
    if (!nef.is_simple())
        throw std::runtime_error("CGAL Nef result is not simple and cannot be converted to a mesh.");

    SurfaceMesh surfaceMesh;
    NefPolyhedron copy = nef;
    CGAL::convert_nef_polyhedron_to_polygon_mesh(copy, surfaceMesh, false);

    ObjMeshData mesh;
    mesh.vertices.reserve(surfaceMesh.number_of_vertices());
    std::map<std::size_t, std::size_t> vertexMap;
    for (const auto vertex : surfaceMesh.vertices())
    {
        const ExactKernel::Point_3 point = surfaceMesh.point(vertex);
        vertexMap.emplace(static_cast<std::size_t>(vertex.idx()), mesh.vertices.size());
        mesh.vertices.push_back(ObjVertex{
            CGAL::to_double(point.x()) / static_cast<double>(coordinateScale),
            CGAL::to_double(point.y()) / static_cast<double>(coordinateScale),
            CGAL::to_double(point.z()) / static_cast<double>(coordinateScale)});
    }

    mesh.faces.reserve(surfaceMesh.number_of_faces());
    for (const auto face : surfaceMesh.faces())
    {
        std::vector<std::size_t> meshFace;
        for (const auto vertex : CGAL::vertices_around_face(surfaceMesh.halfedge(face), surfaceMesh))
            meshFace.push_back(vertexMap.at(static_cast<std::size_t>(vertex.idx())));

        if (meshFace.size() < 3u)
            throw std::runtime_error("CGAL surface mesh face is degenerate after Nef conversion.");

        mesh.faces.push_back(std::move(meshFace));
    }

    return mesh;
}

bool buildNefPostprocessedMeshFromPolygons(
    const std::vector<Polygon256> &fragments,
    std::uint64_t coordinateScale,
    ObjMeshData &outMesh,
    std::string &outError)
{
    outMesh = ObjMeshData();
    outError.clear();

    try
    {
        ExactMeshData exactMesh;
        if (!buildExactMeshFromPolygonSoup(
                    fragments,
                    exactMesh,
                    outError,
                    PolygonSoupTopologyMode::Conforming))
        {
            return false;
        }
        if (!fragments.empty() && exactMesh.faces.empty())
        {
            outError = "Nef output topology received non-empty fragments but topology recovery produced no faces.";
            return false;
        }

        NefBuildOptions nefOptions;
        nefOptions.refineEdgeInteriorPoints = false;
        nefOptions.rejectEmptyRegularizedResult = !fragments.empty();
        const NefPolyhedron nef = makeNefFromExactMesh(exactMesh, "candidate", nefOptions);
        outMesh = makeObjMeshData(nef, coordinateScale);
        if (!fragments.empty() && outMesh.faces.empty())
        {
            outError = "Nef output topology converted a non-empty regularized result to an empty mesh.";
            return false;
        }
        return true;
    }
    catch (const std::exception &ex)
    {
        outError = ex.what();
        return false;
    }
}

bool writeNefPostprocessedMesh(
    const std::vector<Polygon256> &fragments,
    const std::string &path,
    std::uint64_t coordinateScale,
    std::size_t &outFaceCount,
    std::string &outError)
{
    outFaceCount = 0;
    ObjMeshData mesh;
    if (!buildNefPostprocessedMeshFromPolygons(
                fragments,
                coordinateScale,
                mesh,
                outError))
    {
        outError = "Failed to apply Nef output topology: " + outError;
        return false;
    }

    return writeMesh(mesh, path, outFaceCount, outError);
}
}
