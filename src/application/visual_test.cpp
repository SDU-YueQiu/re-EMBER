/**
 * @file visual_test.cpp
 * @brief 实现可选的交互式可视化测试程序。
 */
#include "core/bool_problem.h"
#include "io/io.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>

#include <imgui.h>
#include <imgui_impl_glfw.h>

#include <GLFW/glfw3.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ember::visual_test
{
using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;
using SurfaceMesh = CGAL::Surface_mesh<Kernel::Point_3>;
using NefPolyhedron = CGAL::Nef_polyhedron_3<Kernel>;

namespace
{
/**
 * @brief 解析 visual-test 默认 OBJ 资源的仓库内路径。
 * @param relativePath 相对仓库根目录的资源路径。
 * @return 可直接传给 OBJ 读取器的路径字符串；若未命中任何候选，则退回原始相对路径。
 */
std::string resolveDefaultAssetPath(const std::string &relativePath)
{
#ifdef REEMBER_SOURCE_DIR
    const std::filesystem::path sourceCandidate =
        std::filesystem::path(REEMBER_SOURCE_DIR) / relativePath;
    std::error_code error;
    if (std::filesystem::exists(sourceCandidate, error))
        return sourceCandidate.string();
#endif

    std::filesystem::path current = std::filesystem::current_path();
    for (int depth = 0; depth < 8; ++depth)
    {
        const std::filesystem::path candidate = current / relativePath;
        std::error_code error;
        if (std::filesystem::exists(candidate, error))
            return candidate.string();

        const std::filesystem::path parent = current.parent_path();
        if (parent == current || parent.empty())
            break;
        current = parent;
    }

    return relativePath;
}
}
using Clock = std::chrono::steady_clock;

inline std::size_t kLeafThreshold = 25;
inline double kRotationMinDegrees = -180.0;
inline double kRotationMaxDegrees = 180.0;
inline double kToolScaleMin = 0.01;
inline double kToolScaleMax = 100.0;
inline double kDefaultTx = 0.0;
inline double kDefaultTy = 0.0;
inline double kDefaultTz = 0.0;
inline std::uint64_t kVisualTestManualScale = 1000;

enum class EngineKind
{
    Ember,
    Cgal
};

struct ResultStats
{
    double solveMs = 0.0;
    double convertMs = 0.0;
    std::size_t resultVertexCount = 0;
    std::size_t resultFaceCount = 0;
    std::uint64_t sharedScale = 0;
};

struct Bounds3d
{
    Eigen::Vector3d min = Eigen::Vector3d::Zero();
    Eigen::Vector3d max = Eigen::Vector3d::Zero();
    bool valid = false;
};

struct ToolPose
{
    double tx = kDefaultTx;
    double ty = kDefaultTy;
    double tz = kDefaultTz;
    double scale = 1.0;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
};

struct UiState
{
    EngineKind engine = EngineKind::Ember;
    BoolOp operation = BoolOp::Difference;
    ToolPose pose;
    bool emberAutoScale = true;
    std::uint64_t emberManualScale = kVisualTestManualScale;
    std::size_t leafThreshold = kLeafThreshold;
    bool previewDirty = false;
    bool solveRequested = false;
    bool continuousSolve = false;
    bool resultStale = false;
    std::string lastError;
    ResultStats stats;
};

struct SceneData
{
    std::string workpiecePath;
    std::string toolPath;
    ObjMeshData workpieceMesh;
    ObjMeshData toolOriginalMesh;
    ObjMeshData toolCurrentMesh;
    ObjMeshData workpieceDisplayMesh;
    ObjMeshData toolDisplayMesh;
    ObjMeshData resultDisplayMesh;
    std::vector<Polygon256> workpiecePolygons;
    AABB3i workpieceAABB;
    std::uint64_t emberSharedScale = 0;
    std::unique_ptr<NefPolyhedron> workpieceNef;
    Bounds3d workpieceBounds;
    Bounds3d toolOriginalBounds;
    Eigen::Vector3d toolBaseTranslation = Eigen::Vector3d::Zero();
};

struct MeshLayerStyle
{
    Eigen::RowVector3d color;
    Eigen::RowVector3d lineColor;
    bool showFaces = true;
    bool showLines = false;
    bool doubleSided = true;
};

class FixedImGuiPlugin final : public igl::opengl::glfw::imgui::ImGuiPlugin
{
public:
    bool mouse_up(int button, int modifier) override
    {
        ImGui_ImplGlfw_MouseButtonCallback(viewer->window, button, GLFW_RELEASE, modifier);
        if (ImGui::GetIO().WantCaptureMouse)
            return true;

        for (auto *widget : widgets)
        {
            if (widget->mouse_up(button, modifier))
                return true;
        }

        return false;
    }
};

const char *toString(EngineKind engine) noexcept
{
    switch (engine)
    {
    case EngineKind::Ember:
        return "ember";
    case EngineKind::Cgal:
        return "cgal";
    }

    return "unknown";
}

const char *toString(BoolOp op) noexcept
{
    switch (op)
    {
    case BoolOp::Union:
        return "union";
    case BoolOp::Intersection:
        return "intersection";
    case BoolOp::Difference:
        return "difference";
    }

    return "unknown";
}

double elapsedMilliseconds(const Clock::time_point &start, const Clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

Bounds3d computeBounds(const ObjMeshData &mesh)
{
    Bounds3d bounds;
    if (mesh.vertices.empty())
        return bounds;

    bounds.valid = true;
    bounds.min = Eigen::Vector3d(
        mesh.vertices.front().x,
        mesh.vertices.front().y,
        mesh.vertices.front().z);
    bounds.max = bounds.min;
    for (const ObjVertex &vertex : mesh.vertices)
    {
        const Eigen::Vector3d point(vertex.x, vertex.y, vertex.z);
        bounds.min = bounds.min.cwiseMin(point);
        bounds.max = bounds.max.cwiseMax(point);
    }
    return bounds;
}

Eigen::Vector3d boundsCenter(const Bounds3d &bounds)
{
    if (!bounds.valid)
        return Eigen::Vector3d::Zero();
    return 0.5 * (bounds.min + bounds.max);
}

Eigen::Vector3d boundsExtent(const Bounds3d &bounds)
{
    if (!bounds.valid)
        return Eigen::Vector3d::Zero();
    return bounds.max - bounds.min;
}

void initializeScenePlacement(SceneData &scene)
{
    scene.workpieceBounds = computeBounds(scene.workpieceMesh);
    scene.toolOriginalBounds = computeBounds(scene.toolOriginalMesh);
    scene.toolBaseTranslation = boundsCenter(scene.workpieceBounds) - boundsCenter(scene.toolOriginalBounds);
}

Eigen::Vector3d toolWorldTranslation(const SceneData &scene, const UiState &ui)
{
    return scene.toolBaseTranslation + Eigen::Vector3d(ui.pose.tx, ui.pose.ty, ui.pose.tz);
}

Eigen::Vector3d scaledToolExtent(const SceneData &scene, const UiState &ui)
{
    return boundsExtent(scene.toolOriginalBounds) * std::clamp(ui.pose.scale, kToolScaleMin, kToolScaleMax);
}

Eigen::Vector3d currentTouchOffsets(const SceneData &scene, const UiState &ui)
{
    return 0.5 * (boundsExtent(scene.workpieceBounds) + scaledToolExtent(scene, ui));
}

double currentTranslationRange(const SceneData &scene, const UiState &ui)
{
    const Eigen::Vector3d workpieceExtent = boundsExtent(scene.workpieceBounds);
    const Eigen::Vector3d toolExtent = scaledToolExtent(scene, ui);
    const double dominantExtent = std::max(
        {workpieceExtent.x(), workpieceExtent.y(), workpieceExtent.z(),
         toolExtent.x(), toolExtent.y(), toolExtent.z(), 1.0});
    return 2.0 * dominantExtent;
}

void clampPose(UiState &ui, const SceneData &scene)
{
    const double translationRange = currentTranslationRange(scene, ui);
    ui.pose.tx = std::clamp(ui.pose.tx, -translationRange, translationRange);
    ui.pose.ty = std::clamp(ui.pose.ty, -translationRange, translationRange);
    ui.pose.tz = std::clamp(ui.pose.tz, -translationRange, translationRange);
    ui.pose.scale = std::clamp(ui.pose.scale, kToolScaleMin, kToolScaleMax);
    ui.pose.rx = std::clamp(ui.pose.rx, kRotationMinDegrees, kRotationMaxDegrees);
    ui.pose.ry = std::clamp(ui.pose.ry, kRotationMinDegrees, kRotationMaxDegrees);
    ui.pose.rz = std::clamp(ui.pose.rz, kRotationMinDegrees, kRotationMaxDegrees);
}

ObjMeshData transformToolMesh(const ObjMeshData &mesh, const SceneData &scene, const UiState &ui)
{
    ObjMeshData transformed = mesh;

    const double rx = ui.pose.rx * 3.14159265358979323846 / 180.0;
    const double ry = ui.pose.ry * 3.14159265358979323846 / 180.0;
    const double rz = ui.pose.rz * 3.14159265358979323846 / 180.0;
    const double uniformScale = std::clamp(ui.pose.scale, kToolScaleMin, kToolScaleMax);
    const Eigen::Matrix3d rotation =
        Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d translation = toolWorldTranslation(scene, ui);
    const Eigen::Vector3d pivot = boundsCenter(scene.toolOriginalBounds);

    for (ObjVertex &vertex : transformed.vertices)
    {
        const Eigen::Vector3d point(vertex.x, vertex.y, vertex.z);
        const Eigen::Vector3d transformedPoint =
            rotation * (uniformScale * (point - pivot)) + pivot + translation;
        vertex.x = transformedPoint.x();
        vertex.y = transformedPoint.y();
        vertex.z = transformedPoint.z();
    }

    return transformed;
}

void updateToolPreview(SceneData &scene, const UiState &ui)
{
    scene.toolCurrentMesh = transformToolMesh(scene.toolOriginalMesh, scene, ui);
    scene.toolDisplayMesh = scene.toolCurrentMesh;
}

SurfaceMesh makeSurfaceMesh(const ObjMeshData &mesh)
{
    SurfaceMesh surfaceMesh;
    std::vector<SurfaceMesh::Vertex_index> vertexHandles;
    vertexHandles.reserve(mesh.vertices.size());
    for (const ObjVertex &vertex : mesh.vertices)
        vertexHandles.push_back(surfaceMesh.add_vertex(Kernel::Point_3(vertex.x, vertex.y, vertex.z)));

    for (std::size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex)
    {
        const std::vector<std::size_t> &face = mesh.faces[faceIndex];
        if (face.size() < 3)
            throw std::runtime_error("OBJ face " + std::to_string(faceIndex) + " has fewer than three vertices.");

        for (const std::size_t vertexIndex : face)
        {
            if (vertexIndex >= vertexHandles.size())
                throw std::runtime_error("OBJ face " + std::to_string(faceIndex) + " references an out-of-range vertex.");
        }

        if (face.size() == 3)
        {
            const auto newFace = surfaceMesh.add_face(
                                     vertexHandles[face[0]],
                                     vertexHandles[face[1]],
                                     vertexHandles[face[2]]);
            if (newFace == SurfaceMesh::null_face())
                throw std::runtime_error("Failed to add OBJ face " + std::to_string(faceIndex) + " as a valid CGAL face.");
            continue;
        }

        for (std::size_t i = 1; i + 1 < face.size(); ++i)
        {
            const auto newFace = surfaceMesh.add_face(
                                     vertexHandles[face[0]],
                                     vertexHandles[face[i]],
                                     vertexHandles[face[i + 1]]);
            if (newFace == SurfaceMesh::null_face())
                throw std::runtime_error("Failed to triangulate OBJ face " + std::to_string(faceIndex) + " for CGAL.");
        }
    }

    return surfaceMesh;
}

ObjMeshData makeObjMeshData(const SurfaceMesh &surfaceMesh)
{
    ObjMeshData mesh;

    if (surfaceMesh.number_of_vertices() == 0 || surfaceMesh.number_of_faces() == 0)
        return mesh;

    std::unordered_map<std::size_t, std::size_t> vertexMap;
    mesh.vertices.reserve(surfaceMesh.number_of_vertices());
    for (const auto vertex : surfaceMesh.vertices())
    {
        const auto point = surfaceMesh.point(vertex);
        vertexMap.emplace(static_cast<std::size_t>(vertex.idx()), mesh.vertices.size());
        mesh.vertices.push_back(ObjVertex{
            CGAL::to_double(point.x()),
            CGAL::to_double(point.y()),
            CGAL::to_double(point.z())});
    }

    mesh.faces.reserve(surfaceMesh.number_of_faces());
    for (const auto face : surfaceMesh.faces())
    {
        std::vector<std::size_t> meshFace;
        for (const auto vertex : CGAL::vertices_around_face(surfaceMesh.halfedge(face), surfaceMesh))
            meshFace.push_back(vertexMap.at(static_cast<std::size_t>(vertex.idx())));

        if (meshFace.size() < 3)
            throw std::runtime_error("CGAL surface mesh face is degenerate after conversion.");

        mesh.faces.push_back(std::move(meshFace));
    }

    return mesh;
}

NefPolyhedron makeNef(const ObjMeshData &mesh)
{
    return NefPolyhedron(makeSurfaceMesh(mesh));
}

ObjMeshData makeObjMeshData(const NefPolyhedron &nef)
{
    if (nef.is_empty())
        return ObjMeshData();
    if (!nef.is_simple())
        throw std::runtime_error("CGAL Nef result is not simple and cannot be converted to a mesh.");

    SurfaceMesh surfaceMesh;
    CGAL::convert_nef_polyhedron_to_polygon_mesh(nef, surfaceMesh, false);
    CGAL::Polygon_mesh_processing::triangulate_faces(surfaceMesh);
    return makeObjMeshData(surfaceMesh);
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

void storeResultStats(
    ResultStats &outStats,
    const ObjMeshData &mesh,
    const Clock::time_point &solveStart,
    const Clock::time_point &solveEnd,
    const Clock::time_point &convertStart,
    const Clock::time_point &convertEnd,
    std::uint64_t sharedScale = 0)
{
    outStats.solveMs = elapsedMilliseconds(solveStart, solveEnd);
    outStats.convertMs = elapsedMilliseconds(convertStart, convertEnd);
    outStats.resultVertexCount = mesh.vertices.size();
    outStats.resultFaceCount = mesh.faces.size();
    outStats.sharedScale = sharedScale;
}

bool prepareEmberInput(SceneData &scene, const UiState &ui, std::string &outError)
{
    outError.clear();
    if (!ui.emberAutoScale && ui.emberManualScale == 0)
    {
        outError = "EMBER shared scale must be a positive integer.";
        return false;
    }

    QuantizeOptions options;
    if (!ui.emberAutoScale)
        options.explicitScale = ui.emberManualScale;

    std::uint64_t sharedScale = 0;
    if (!ember::chooseSharedScale({scene.workpieceMesh, scene.toolCurrentMesh}, options, sharedScale, outError))
    {
        return false;
    }
    if (scene.emberSharedScale == sharedScale && !scene.workpiecePolygons.empty())
        return true;

    PolygonSoupBuildOptions polygonBuildOptions;
    polygonBuildOptions.triangulateNonCoplanarFaces = true;

    std::vector<Polygon256> workpiecePolygons;
    AABB3i workpieceAABB;
    if (!ember::computeScaledMeshAABB(scene.workpieceMesh, sharedScale, workpieceAABB, outError))
    {
        outError = "Failed to compute the EMBER workpiece AABB: " + outError;
        return false;
    }
    if (!ember::buildPolygonSoup(
                scene.workpieceMesh,
                sharedScale,
                polygonBuildOptions,
                workpiecePolygons,
                outError))
    {
        outError = "Failed to build the EMBER workpiece polygon soup: " + outError;
        return false;
    }

    scene.emberSharedScale = sharedScale;
    scene.workpiecePolygons = std::move(workpiecePolygons);
    scene.workpieceAABB = workpieceAABB;
    return true;
}

bool computeEmberResult(SceneData &scene, const UiState &ui, ResultStats &outStats, std::string &outError)
{
    outStats = ResultStats();
    outError.clear();

    try
    {
        if (!prepareEmberInput(scene, ui, outError))
        {
            outError = "Failed to prepare the EMBER workpiece input: " + outError;
            return false;
        }
        outStats.sharedScale = scene.emberSharedScale;

        PolygonSoupBuildOptions polygonBuildOptions;
        polygonBuildOptions.triangulateNonCoplanarFaces = true;

        std::vector<Polygon256> toolPolygons;
        AABB3i toolAABB;
        if (!ember::computeScaledMeshAABB(scene.toolCurrentMesh, scene.emberSharedScale, toolAABB, outError))
        {
            outError = "Failed to compute the EMBER tool AABB: " + outError;
            return false;
        }
        if (!ember::buildPolygonSoup(
                    scene.toolCurrentMesh,
                    scene.emberSharedScale,
                    polygonBuildOptions,
                    toolPolygons,
                    outError))
        {
            outError = "Failed to build the EMBER tool polygon soup: " + outError;
            return false;
        }

        AABB3i sceneAABB;
        ember::mergeAABB(sceneAABB, scene.workpieceAABB);
        ember::mergeAABB(sceneAABB, toolAABB);
        ember::expandAABB(sceneAABB, 1);

        ember::BoolProblem problem(ui.leafThreshold);
        problem.setOperation(ui.operation);
        problem.setOperandAssumptions(
            ember::BoolOperandAssumptions{true, true},
            ember::BoolOperandAssumptions{true, true});
        problem.setOperands(scene.workpiecePolygons, toolPolygons);

        const Clock::time_point solveStart = Clock::now();
        problem.solve(sceneAABB);
        const Clock::time_point solveEnd = Clock::now();

        const Clock::time_point convertStart = Clock::now();
        if (!ember::buildObjMeshFromPolygonSoup(
                    problem.resultFragments(),
                    scene.resultDisplayMesh,
                    outError,
                    scene.emberSharedScale))
        {
            outError = "Failed to convert the EMBER result polygon soup to an OBJ mesh: " + outError;
            return false;
        }
        const Clock::time_point convertEnd = Clock::now();

        storeResultStats(
            outStats,
            scene.resultDisplayMesh,
            solveStart,
            solveEnd,
            convertStart,
            convertEnd,
            scene.emberSharedScale);
        return true;
    }
    catch (const std::exception &ex)
    {
        outError = std::string("EMBER solve failed: ") + ex.what();
        return false;
    }
}

bool computeCgalResult(SceneData &scene, const UiState &ui, ResultStats &outStats, std::string &outError)
{
    outStats = ResultStats();
    outError.clear();

    try
    {
        if (!scene.workpieceNef)
            scene.workpieceNef = std::make_unique<NefPolyhedron>(makeNef(scene.workpieceMesh));

        const NefPolyhedron toolNef = makeNef(scene.toolCurrentMesh);

        const Clock::time_point solveStart = Clock::now();
        const NefPolyhedron result = applyBoolean(*scene.workpieceNef, toolNef, ui.operation);
        const Clock::time_point solveEnd = Clock::now();

        const Clock::time_point convertStart = Clock::now();
        scene.resultDisplayMesh = makeObjMeshData(result);
        const Clock::time_point convertEnd = Clock::now();

        storeResultStats(outStats, scene.resultDisplayMesh, solveStart, solveEnd, convertStart, convertEnd);
        return true;
    }
    catch (const std::exception &ex)
    {
        outError = std::string("CGAL solve failed: ") + ex.what();
        return false;
    }
}

bool recomputeScene(SceneData &scene, UiState &ui)
{
    ui.lastError.clear();

    updateToolPreview(scene, ui);

    switch (ui.engine)
    {
    case EngineKind::Ember:
        return computeEmberResult(scene, ui, ui.stats, ui.lastError);
    case EngineKind::Cgal:
        return computeCgalResult(scene, ui, ui.stats, ui.lastError);
    }

    ui.lastError = "Unknown engine.";
    return false;
}

ToolPose makeToolPose(
    double tx,
    double ty,
    double tz,
    double scale = 1.0,
    double rx = 0.0,
    double ry = 0.0,
    double rz = 0.0) noexcept
{
    ToolPose pose;
    pose.tx = tx;
    pose.ty = ty;
    pose.tz = tz;
    pose.scale = scale;
    pose.rx = rx;
    pose.ry = ry;
    pose.rz = rz;
    return pose;
}

bool drawPosePresetButton(const char *label, ToolPose &pose, const ToolPose &preset)
{
    if (!ImGui::Button(label))
        return false;

    pose = preset;
    return true;
}

bool drawOffsetPresetButton(const char *label, ToolPose &pose, const Eigen::Vector3d &offset)
{
    if (!ImGui::Button(label))
        return false;

    pose.tx = offset.x();
    pose.ty = offset.y();
    pose.tz = offset.z();
    return true;
}

bool drawDoubleSliderInput(
    const char *label,
    double &value,
    double minValue,
    double maxValue,
    double inputStep,
    const char *format = "%.6f",
    ImGuiSliderFlags sliderFlags = ImGuiSliderFlags_AlwaysClamp)
{
    bool changed = false;

    ImGui::PushID(label);
    ImGui::TextUnformatted(label);
    ImGui::SameLine(32.0f);

    ImGui::SetNextItemWidth(150.0f);
    double sliderValue = value;
    if (ImGui::SliderScalar(
                "##slider",
                ImGuiDataType_Double,
                &sliderValue,
                &minValue,
                &maxValue,
                format,
                sliderFlags))
    {
        value = sliderValue;
        changed = true;
    }

    ImGui::SameLine();
    ImGui::SetNextItemWidth(110.0f);
    double inputValue = value;
    if (ImGui::InputDouble("##input", &inputValue, inputStep, inputStep * 10.0, format))
    {
        value = inputValue;
        changed = true;
    }

    ImGui::PopID();
    return changed;
}

MeshLayerStyle workpieceLayerStyle()
{
    const Eigen::RowVector3d blue(0.25, 0.45, 0.95);
    return MeshLayerStyle{blue, blue, false, true, true};
}

MeshLayerStyle resultLayerStyle()
{
    return MeshLayerStyle{
        Eigen::RowVector3d(0.95, 0.55, 0.20),
        Eigen::RowVector3d(0.20, 0.08, 0.02),
        true,
        true,
        true};
}

MeshLayerStyle toolLayerStyle()
{
    return MeshLayerStyle{
        Eigen::RowVector3d(0.15, 0.80, 0.25),
        Eigen::RowVector3d(0.05, 0.55, 0.12),
        false,
        true,
        true};
}

void setViewerMesh(
    igl::opengl::glfw::Viewer &viewer,
    int meshId,
    const ObjMeshData &mesh,
    const MeshLayerStyle &style)
{
    Eigen::MatrixXd vertices;
    vertices.resize(static_cast<Eigen::Index>(mesh.vertices.size()), 3);
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i)
    {
        vertices(static_cast<Eigen::Index>(i), 0) = mesh.vertices[i].x;
        vertices(static_cast<Eigen::Index>(i), 1) = mesh.vertices[i].y;
        vertices(static_cast<Eigen::Index>(i), 2) = mesh.vertices[i].z;
    }

    std::size_t viewerFaceCount = 0;
    for (const std::vector<std::size_t> &face : mesh.faces)
    {
        if (face.size() >= 3)
            viewerFaceCount += face.size() - 2;
    }

    Eigen::MatrixXi viewerFaces(static_cast<Eigen::Index>(viewerFaceCount), 3);
    std::size_t viewerFaceIndex = 0;
    for (const std::vector<std::size_t> &face : mesh.faces)
    {
        for (std::size_t i = 1; i + 1 < face.size(); ++i)
        {
            if (face[0] >= mesh.vertices.size() ||
                    face[i] >= mesh.vertices.size() ||
                    face[i + 1] >= mesh.vertices.size())
                continue;

            viewerFaces(static_cast<Eigen::Index>(viewerFaceIndex), 0) = static_cast<int>(face[0]);
            viewerFaces(static_cast<Eigen::Index>(viewerFaceIndex), 1) = static_cast<int>(face[i]);
            viewerFaces(static_cast<Eigen::Index>(viewerFaceIndex), 2) = static_cast<int>(face[i + 1]);
            ++viewerFaceIndex;
        }
    }
    viewerFaces.conservativeResize(static_cast<Eigen::Index>(viewerFaceIndex), 3);

    viewer.data(meshId).clear();
    if (vertices.rows() > 0 && viewerFaces.rows() > 0)
        viewer.data(meshId).set_mesh(vertices, viewerFaces);
    viewer.data(meshId).set_face_based(true);
    viewer.data(meshId).set_colors(style.color);
    viewer.data(meshId).double_sided = style.doubleSided;
    viewer.data(meshId).show_faces = style.showFaces;
    viewer.data(meshId).show_lines = false;

    if (style.showLines)
    {
        std::size_t edgeCount = 0;
        for (const std::vector<std::size_t> &face : mesh.faces)
            edgeCount += face.size();

        Eigen::MatrixXd edgeStarts(static_cast<Eigen::Index>(edgeCount), 3);
        Eigen::MatrixXd edgeEnds(static_cast<Eigen::Index>(edgeCount), 3);
        std::size_t edgeIndex = 0;
        for (const std::vector<std::size_t> &face : mesh.faces)
        {
            for (std::size_t i = 0; i < face.size(); ++i)
            {
                const std::size_t start = face[i];
                const std::size_t end = face[(i + 1) % face.size()];
                if (start >= mesh.vertices.size() || end >= mesh.vertices.size())
                    continue;

                const ObjVertex &startVertex = mesh.vertices[start];
                const ObjVertex &endVertex = mesh.vertices[end];
                edgeStarts(static_cast<Eigen::Index>(edgeIndex), 0) = startVertex.x;
                edgeStarts(static_cast<Eigen::Index>(edgeIndex), 1) = startVertex.y;
                edgeStarts(static_cast<Eigen::Index>(edgeIndex), 2) = startVertex.z;
                edgeEnds(static_cast<Eigen::Index>(edgeIndex), 0) = endVertex.x;
                edgeEnds(static_cast<Eigen::Index>(edgeIndex), 1) = endVertex.y;
                edgeEnds(static_cast<Eigen::Index>(edgeIndex), 2) = endVertex.z;
                ++edgeIndex;
            }
        }

        edgeStarts.conservativeResize(static_cast<Eigen::Index>(edgeIndex), 3);
        edgeEnds.conservativeResize(static_cast<Eigen::Index>(edgeIndex), 3);
        if (edgeStarts.rows() > 0)
            viewer.data(meshId).add_edges(edgeStarts, edgeEnds, style.lineColor);
    }
}
}

int main()
{
    using namespace ember::visual_test;

    SceneData scene;
    scene.workpiecePath = resolveDefaultAssetPath("assets/visual_test/workpiece_block.obj");
    scene.toolPath = resolveDefaultAssetPath("assets/visual_test/tool_box.obj");

    std::string error;
    if (!ember::readObjMesh(scene.workpiecePath, scene.workpieceMesh, error))
    {
        std::cerr << "Failed to read the workpiece OBJ: " << error << std::endl;
        return 1;
    }
    if (!ember::readObjMesh(scene.toolPath, scene.toolOriginalMesh, error))
    {
        std::cerr << "Failed to read the tool OBJ: " << error << std::endl;
        return 1;
    }
    scene.workpieceDisplayMesh = scene.workpieceMesh;
    initializeScenePlacement(scene);

    UiState ui;
    updateToolPreview(scene, ui);
    ui.resultStale = true;

    igl::opengl::glfw::Viewer viewer;
    FixedImGuiPlugin plugin;
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&plugin);
    plugin.widgets.push_back(&menu);

    const int workpieceMeshId = viewer.data().id;
    setViewerMesh(viewer, workpieceMeshId, scene.workpieceDisplayMesh, workpieceLayerStyle());

    viewer.append_mesh();
    const int resultMeshId = viewer.data().id;
    setViewerMesh(viewer, resultMeshId, scene.resultDisplayMesh, resultLayerStyle());

    viewer.append_mesh();
    const int toolMeshId = viewer.data().id;
    setViewerMesh(viewer, toolMeshId, scene.toolDisplayMesh, toolLayerStyle());

    auto updateViewerMeshes = [&]()
    {
        setViewerMesh(viewer, resultMeshId, scene.resultDisplayMesh, resultLayerStyle());
        setViewerMesh(viewer, toolMeshId, scene.toolDisplayMesh, toolLayerStyle());
    };

    menu.callback_draw_viewer_menu = [&]()
    {
        UiState proposed = ui;
        bool changed = false;
        bool solveModeChanged = false;
        const double translationRange = currentTranslationRange(scene, proposed);
        const Eigen::Vector3d touchOffsets = currentTouchOffsets(scene, proposed);
        const Eigen::Vector3d worldTranslation = toolWorldTranslation(scene, proposed);

        ImGui::Text("visual-test");
        ImGui::Separator();

        ImGui::Text("Engine: %s", toString(ui.engine));
        if (ImGui::Button("Use ember"))
        {
            proposed.engine = EngineKind::Ember;
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Use cgal"))
        {
            proposed.engine = EngineKind::Cgal;
            changed = true;
        }

        ImGui::Text("Operation: %s", toString(ui.operation));
        if (ImGui::Button("Union"))
        {
            proposed.operation = ember::BoolOp::Union;
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Intersection"))
        {
            proposed.operation = ember::BoolOp::Intersection;
            changed = true;
        }
        ImGui::SameLine();
        if (ImGui::Button("Difference"))
        {
            proposed.operation = ember::BoolOp::Difference;
            changed = true;
        }

        ImGui::Separator();
        ImGui::Text("offset range = +/- %.6f", translationRange);
        ImGui::Text(
            "auto-align world t = (%.6f, %.6f, %.6f)",
            scene.toolBaseTranslation.x(),
            scene.toolBaseTranslation.y(),
            scene.toolBaseTranslation.z());
        ImGui::Text(
            "current world t = (%.6f, %.6f, %.6f)",
            worldTranslation.x(),
            worldTranslation.y(),
            worldTranslation.z());

        if (ImGui::Checkbox("auto scale", &proposed.emberAutoScale))
            changed = true;
        if (!proposed.emberAutoScale)
        {
            std::uint64_t proposedScale = proposed.emberManualScale;
            if (ImGui::InputScalar("scale", ImGuiDataType_U64, &proposedScale))
            {
                proposed.emberManualScale = proposedScale == 0 ? 1 : proposedScale;
                changed = true;
            }
        }
        if (ImGui::Checkbox("continuous solve", &proposed.continuousSolve))
            solveModeChanged = true;

        int proposedLeafThreshold = static_cast<int>(proposed.leafThreshold);
        if (ImGui::InputInt("leaf threshold", &proposedLeafThreshold))
        {
            proposed.leafThreshold = static_cast<std::size_t>(std::max(1, proposedLeafThreshold));
            changed = true;
        }

        changed = drawDoubleSliderInput(
                      "tool scale",
                      proposed.pose.scale,
                      kToolScaleMin,
                      kToolScaleMax,
                      0.01,
                      "%.4f",
                      ImGuiSliderFlags_AlwaysClamp | ImGuiSliderFlags_Logarithmic) || changed;
        changed = drawDoubleSliderInput("dx", proposed.pose.tx, -translationRange, translationRange, 0.001) || changed;
        changed = drawDoubleSliderInput("dy", proposed.pose.ty, -translationRange, translationRange, 0.001) || changed;
        changed = drawDoubleSliderInput("dz", proposed.pose.tz, -translationRange, translationRange, 0.001) || changed;
        changed = drawDoubleSliderInput("rx", proposed.pose.rx, kRotationMinDegrees, kRotationMaxDegrees, 0.01) || changed;
        changed = drawDoubleSliderInput("ry", proposed.pose.ry, kRotationMinDegrees, kRotationMaxDegrees, 0.01) || changed;
        changed = drawDoubleSliderInput("rz", proposed.pose.rz, kRotationMinDegrees, kRotationMaxDegrees, 0.01) || changed;

        if (ImGui::Button("Reset Pose"))
        {
            proposed.pose = ToolPose();
            changed = true;
        }
        ImGui::SameLine();
        changed = drawOffsetPresetButton("Center", proposed.pose, Eigen::Vector3d::Zero()) || changed;

        ImGui::Text("AABB touch presets");
        changed = drawOffsetPresetButton("X min", proposed.pose, Eigen::Vector3d(-touchOffsets.x(), 0.0, 0.0)) || changed;
        ImGui::SameLine();
        changed = drawOffsetPresetButton("X max", proposed.pose, Eigen::Vector3d(touchOffsets.x(), 0.0, 0.0)) || changed;
        ImGui::SameLine();
        changed = drawOffsetPresetButton("Y min", proposed.pose, Eigen::Vector3d(0.0, -touchOffsets.y(), 0.0)) || changed;
        ImGui::SameLine();
        changed = drawOffsetPresetButton("Y max", proposed.pose, Eigen::Vector3d(0.0, touchOffsets.y(), 0.0)) || changed;
        changed = drawOffsetPresetButton("Z min", proposed.pose, Eigen::Vector3d(0.0, 0.0, -touchOffsets.z())) || changed;
        ImGui::SameLine();
        changed = drawOffsetPresetButton("Z max", proposed.pose, Eigen::Vector3d(0.0, 0.0, touchOffsets.z())) || changed;

        if (changed)
        {
            ui = proposed;
            ui.previewDirty = true;
            ui.resultStale = true;
            ui.solveRequested = ui.continuousSolve;
            ui.lastError.clear();
            ui.stats = ResultStats();
        }
        else if (solveModeChanged)
        {
            ui.continuousSolve = proposed.continuousSolve;
            if (ui.continuousSolve && ui.resultStale)
                ui.solveRequested = true;
        }

        if (ImGui::Button("Run Boolean"))
        {
            ui.solveRequested = true;
        }
        ImGui::SameLine();
        ImGui::Text(
            "%s | %s",
            ui.resultStale ? "result stale" : "result current",
            ui.continuousSolve ? "continuous" : "manual");

        if (ui.resultStale)
        {
            ImGui::TextWrapped(
                ui.continuousSolve
                    ? "移动和参数修改会自动重新求解。"
                    : "移动和参数修改只更新刀具预览；点击 Run Boolean 后再执行求解。");
        }

        ImGui::Separator();
        ImGui::Text("solve_ms=%.3f convert_ms=%.3f", ui.stats.solveMs, ui.stats.convertMs);
        ImGui::Text(
            "result_vertices=%zu result_faces=%zu",
            ui.stats.resultVertexCount,
            ui.stats.resultFaceCount);
        if (ui.engine == EngineKind::Ember)
        {
            ImGui::Text("shared_scale=%llu", static_cast<unsigned long long>(ui.stats.sharedScale));
            ImGui::Text("leaf_threshold=%zu", ui.leafThreshold);
        }
        ImGui::Text("workpiece=%s", scene.workpiecePath.c_str());
        ImGui::Text("tool=%s", scene.toolPath.c_str());

        if (!ui.lastError.empty())
        {
            ImGui::Separator();
            ImGui::TextWrapped("Last error: %s", ui.lastError.c_str());
        }
    };

    viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer &) -> bool
    {
        if (ui.previewDirty)
        {
            clampPose(ui, scene);
            updateToolPreview(scene, ui);
            setViewerMesh(viewer, toolMeshId, scene.toolDisplayMesh, toolLayerStyle());

            if (ui.resultStale)
            {
                scene.resultDisplayMesh = {};
                setViewerMesh(viewer, resultMeshId, scene.resultDisplayMesh, resultLayerStyle());
            }

            ui.previewDirty = false;
        }

        if (!ui.solveRequested)
            return false;

        clampPose(ui, scene);
        ui.solveRequested = false;
        if (!recomputeScene(scene, ui))
        {
            ui.lastError = ui.lastError.empty() ? "visual-test recompute failed." : ui.lastError;
            scene.resultDisplayMesh = {};
            ui.stats = ResultStats();
            ui.resultStale = true;
            updateViewerMeshes();
            return false;
        }

        ui.resultStale = false;
        updateViewerMeshes();
        return false;
    };

    viewer.core().background_color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 60.0f;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.launch();
    return 0;
}
