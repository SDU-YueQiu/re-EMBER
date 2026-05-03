#include "core/bool_problem.h"
#include "io/io.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Nef_polyhedron_3.h>
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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
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
    using Clock = std::chrono::steady_clock;

    inline constexpr std::size_t kLeafThreshold = 25;
    inline constexpr double kTranslationMin = -0.25;
    inline constexpr double kTranslationMax = 1.25;
    inline constexpr double kRotationMinDegrees = -180.0;
    inline constexpr double kRotationMaxDegrees = 180.0;
    inline constexpr double kDefaultTx = 0.5;
    inline constexpr double kDefaultTy = 0.5;
    inline constexpr double kDefaultTz = 0.35;

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

    struct ToolPose
    {
        double tx = kDefaultTx;
        double ty = kDefaultTy;
        double tz = kDefaultTz;
        double rx = 0.0;
        double ry = 0.0;
        double rz = 0.0;
    };

    struct UiState
    {
        EngineKind engine = EngineKind::Ember;
        BoolOp operation = BoolOp::Difference;
        ToolPose pose;
        bool dirty = false;
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
        std::uint64_t emberSharedScale = 0;
        NefPolyhedron workpieceNef;
    };

    struct MeshLayerStyle
    {
        Eigen::RowVector3d color;
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
            {
                return true;
            }

            for (auto *widget : widgets)
            {
                if (widget->mouse_up(button, modifier))
                {
                    return true;
                }
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

    ObjMeshData transformToolMesh(const ObjMeshData &mesh, const UiState &ui)
    {
        ObjMeshData transformed = mesh;

        const double rx = ui.pose.rx * 3.14159265358979323846 / 180.0;
        const double ry = ui.pose.ry * 3.14159265358979323846 / 180.0;
        const double rz = ui.pose.rz * 3.14159265358979323846 / 180.0;
        const Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()).toRotationMatrix();
        const Eigen::Vector3d translation(ui.pose.tx, ui.pose.ty, ui.pose.tz);

        for (ObjVertex &vertex : transformed.vertices)
        {
            const Eigen::Vector3d point(vertex.x, vertex.y, vertex.z);
            const Eigen::Vector3d transformedPoint = rotation * point + translation;
            vertex.x = transformedPoint.x();
            vertex.y = transformedPoint.y();
            vertex.z = transformedPoint.z();
        }

        return transformed;
    }

    SurfaceMesh makeSurfaceMesh(const ObjMeshData &mesh)
    {
        SurfaceMesh surfaceMesh;
        std::vector<SurfaceMesh::Vertex_index> vertexHandles;
        vertexHandles.reserve(mesh.vertices.size());
        for (const ObjVertex &vertex : mesh.vertices)
        {
            vertexHandles.push_back(surfaceMesh.add_vertex(Kernel::Point_3(vertex.x, vertex.y, vertex.z)));
        }

        for (std::size_t faceIndex = 0; faceIndex < mesh.faces.size(); ++faceIndex)
        {
            const std::vector<std::size_t> &face = mesh.faces[faceIndex];
            if (face.size() < 3)
            {
                throw std::runtime_error("OBJ face " + std::to_string(faceIndex) + " has fewer than three vertices.");
            }

            std::vector<SurfaceMesh::Vertex_index> faceVertices;
            faceVertices.reserve(face.size());
            for (const std::size_t vertexIndex : face)
            {
                if (vertexIndex >= vertexHandles.size())
                {
                    throw std::runtime_error("OBJ face " + std::to_string(faceIndex) + " references an out-of-range vertex.");
                }

                faceVertices.push_back(vertexHandles[vertexIndex]);
            }

            const auto newFace = surfaceMesh.add_face(faceVertices);
            if (newFace == SurfaceMesh::null_face())
            {
                throw std::runtime_error("Failed to add OBJ face " + std::to_string(faceIndex) + " as a valid CGAL face.");
            }
        }

        return surfaceMesh;
    }

    ObjMeshData makeObjMeshData(const SurfaceMesh &surfaceMesh)
    {
        ObjMeshData mesh;

        if (surfaceMesh.number_of_vertices() == 0 || surfaceMesh.number_of_faces() == 0)
        {
            return mesh;
        }

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
            {
                meshFace.push_back(vertexMap.at(static_cast<std::size_t>(vertex.idx())));
            }

            if (meshFace.size() < 3)
            {
                throw std::runtime_error("CGAL surface mesh face is degenerate after conversion.");
            }

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
        {
            return ObjMeshData();
        }
        if (!nef.is_simple())
        {
            throw std::runtime_error("CGAL Nef result is not simple and cannot be converted to a mesh.");
        }

        SurfaceMesh surfaceMesh;
        CGAL::convert_nef_polyhedron_to_polygon_mesh(nef, surfaceMesh, false);
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

    bool initializeEmberSharedScale(SceneData &scene, std::string &outError)
    {
        outError.clear();

        double maxToolRadius = 0.0;
        for (const ObjVertex &vertex : scene.toolOriginalMesh.vertices)
        {
            const double radius = std::sqrt(vertex.x * vertex.x + vertex.y * vertex.y + vertex.z * vertex.z);
            maxToolRadius = std::max(maxToolRadius, radius);
        }

        const double maxTranslateAbs = std::max(std::abs(kTranslationMin), std::abs(kTranslationMax));
        const double bound = maxTranslateAbs + maxToolRadius;

        ObjMeshData boundMesh;
        boundMesh.vertices = {
            ObjVertex{ bound, 0.0, 0.0},
            ObjVertex{0.0,  bound, 0.0},
            ObjVertex{0.0, 0.0,  bound},
            ObjVertex{-bound, 0.0, 0.0},
            ObjVertex{0.0, -bound, 0.0},
            ObjVertex{0.0, 0.0, -bound}};
        boundMesh.faces = {{0, 1, 2}, {3, 4, 5}};

        std::uint64_t sharedScale = 0;
        if (!ember::chooseSharedScale({scene.workpieceMesh, boundMesh}, QuantizeOptions(), sharedScale, outError))
        {
            return false;
        }

        scene.emberSharedScale = sharedScale;
        return true;
    }

    bool computeEmberResult(SceneData &scene, const UiState &ui, ResultStats &outStats, std::string &outError)
    {
        outStats = ResultStats();
        outError.clear();

        std::vector<Polygon256> toolPolygons;
        if (!ember::buildPolygonSoup(scene.toolCurrentMesh, scene.emberSharedScale, toolPolygons, outError))
        {
            outError = "Failed to build the EMBER tool polygon soup: " + outError;
            return false;
        }

        ember::BoolProblem problem(kLeafThreshold);
        problem.setOperation(ui.operation);
        problem.setOperands(scene.workpiecePolygons, toolPolygons);

        const Clock::time_point solveStart = Clock::now();
        problem.solve();
        const Clock::time_point solveEnd = Clock::now();

        const Clock::time_point convertStart = Clock::now();
        if (!ember::buildObjMeshFromPolygonSoup(problem.resultFragments(), scene.resultDisplayMesh, outError))
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

    bool computeCgalResult(SceneData &scene, const UiState &ui, ResultStats &outStats, std::string &outError)
    {
        outStats = ResultStats();
        outError.clear();

        try
        {
            const NefPolyhedron toolNef = makeNef(scene.toolCurrentMesh);

            const Clock::time_point solveStart = Clock::now();
            const NefPolyhedron result = applyBoolean(scene.workpieceNef, toolNef, ui.operation);
            const Clock::time_point solveEnd = Clock::now();

            const Clock::time_point convertStart = Clock::now();
            scene.resultDisplayMesh = makeObjMeshData(result);
            const Clock::time_point convertEnd = Clock::now();

            storeResultStats(outStats, scene.resultDisplayMesh, solveStart, solveEnd, convertStart, convertEnd);
            return true;
        }
        catch (const std::exception &ex)
        {
            outError = ex.what();
            return false;
        }
    }

    bool recomputeScene(SceneData &scene, UiState &ui)
    {
        ui.lastError.clear();

        scene.toolCurrentMesh = transformToolMesh(scene.toolOriginalMesh, ui);
        scene.toolDisplayMesh = scene.toolCurrentMesh;

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

    bool drawDoubleSlider(const char *label, double &value, double minValue, double maxValue)
    {
        float sliderValue = static_cast<float>(value);
        if (!ImGui::SliderFloat(label, &sliderValue, static_cast<float>(minValue), static_cast<float>(maxValue)))
        {
            return false;
        }

        value = static_cast<double>(sliderValue);
        return true;
    }

    MeshLayerStyle workpieceLayerStyle()
    {
        return MeshLayerStyle{Eigen::RowVector3d(0.25, 0.45, 0.95), false, true, true};
    }

    MeshLayerStyle resultLayerStyle()
    {
        return MeshLayerStyle{Eigen::RowVector3d(0.95, 0.55, 0.20), true, false, true};
    }

    MeshLayerStyle toolLayerStyle(BoolOp operation)
    {
        return MeshLayerStyle{
            Eigen::RowVector3d(0.15, 0.80, 0.25),
            operation != BoolOp::Difference,
            false,
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
            {
                viewerFaceCount += face.size() - 2;
            }
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
                {
                    continue;
                }

                viewerFaces(static_cast<Eigen::Index>(viewerFaceIndex), 0) = static_cast<int>(face[0]);
                viewerFaces(static_cast<Eigen::Index>(viewerFaceIndex), 1) = static_cast<int>(face[i]);
                viewerFaces(static_cast<Eigen::Index>(viewerFaceIndex), 2) = static_cast<int>(face[i + 1]);
                ++viewerFaceIndex;
            }
        }
        viewerFaces.conservativeResize(static_cast<Eigen::Index>(viewerFaceIndex), 3);

        viewer.data(meshId).clear();
        if (vertices.rows() > 0 && viewerFaces.rows() > 0)
        {
            viewer.data(meshId).set_mesh(vertices, viewerFaces);
        }
        viewer.data(meshId).set_face_based(true);
        viewer.data(meshId).set_colors(style.color);
        viewer.data(meshId).double_sided = style.doubleSided;
        viewer.data(meshId).show_faces = style.showFaces;
        viewer.data(meshId).show_lines = false;

        if (style.showLines)
        {
            std::size_t edgeCount = 0;
            for (const std::vector<std::size_t> &face : mesh.faces)
            {
                edgeCount += face.size();
            }

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
                    {
                        continue;
                    }

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
            {
                viewer.data(meshId).add_edges(edgeStarts, edgeEnds, style.color);
            }
        }
    }
}

int main()
{
    using namespace ember::visual_test;

    SceneData scene;
#ifdef KEMBER_SOURCE_DIR
    const std::string sourceDir = KEMBER_SOURCE_DIR;
    scene.workpiecePath = sourceDir + "/assets/visual_test/workpiece_block.obj";
    scene.toolPath = sourceDir + "/assets/visual_test/tool_box.obj";
#else
    scene.workpiecePath = "assets/visual_test/workpiece_block.obj";
    scene.toolPath = "assets/visual_test/tool_box.obj";
#endif

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
    if (!initializeEmberSharedScale(scene, error))
    {
        std::cerr << "Failed to choose a fixed EMBER shared scale: " << error << std::endl;
        return 1;
    }
    if (!ember::buildPolygonSoup(scene.workpieceMesh, scene.emberSharedScale, scene.workpiecePolygons, error))
    {
        std::cerr << "Failed to build the EMBER workpiece polygon soup: " << error << std::endl;
        return 1;
    }
    try
    {
        scene.workpieceNef = makeNef(scene.workpieceMesh);
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Failed to build the CGAL workpiece Nef polyhedron: " << ex.what() << std::endl;
        return 1;
    }

    UiState ui;
    if (!recomputeScene(scene, ui))
    {
        std::cerr << ui.lastError << std::endl;
        return 1;
    }

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
    setViewerMesh(viewer, toolMeshId, scene.toolDisplayMesh, toolLayerStyle(ui.operation));

    auto updateViewerMeshes = [&]()
    {
        setViewerMesh(viewer, resultMeshId, scene.resultDisplayMesh, resultLayerStyle());
        setViewerMesh(viewer, toolMeshId, scene.toolDisplayMesh, toolLayerStyle(ui.operation));
    };

    menu.callback_draw_viewer_menu = [&]()
    {
        UiState proposed = ui;
        bool changed = false;

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
        changed = drawDoubleSlider("tx", proposed.pose.tx, kTranslationMin, kTranslationMax) || changed;
        changed = drawDoubleSlider("ty", proposed.pose.ty, kTranslationMin, kTranslationMax) || changed;
        changed = drawDoubleSlider("tz", proposed.pose.tz, kTranslationMin, kTranslationMax) || changed;
        changed = drawDoubleSlider("rx", proposed.pose.rx, kRotationMinDegrees, kRotationMaxDegrees) || changed;
        changed = drawDoubleSlider("ry", proposed.pose.ry, kRotationMinDegrees, kRotationMaxDegrees) || changed;
        changed = drawDoubleSlider("rz", proposed.pose.rz, kRotationMinDegrees, kRotationMaxDegrees) || changed;

        if (ImGui::Button("Reset Pose"))
        {
            proposed.pose = ToolPose();
            changed = true;
        }

        if (changed)
        {
            ui = proposed;
            ui.dirty = true;
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
        if (!ui.dirty)
        {
            return false;
        }

        const UiState previous = ui;
        ui.pose.tx = std::clamp(ui.pose.tx, kTranslationMin, kTranslationMax);
        ui.pose.ty = std::clamp(ui.pose.ty, kTranslationMin, kTranslationMax);
        ui.pose.tz = std::clamp(ui.pose.tz, kTranslationMin, kTranslationMax);
        ui.pose.rx = std::clamp(ui.pose.rx, kRotationMinDegrees, kRotationMaxDegrees);
        ui.pose.ry = std::clamp(ui.pose.ry, kRotationMinDegrees, kRotationMaxDegrees);
        ui.pose.rz = std::clamp(ui.pose.rz, kRotationMinDegrees, kRotationMaxDegrees);

        if (!recomputeScene(scene, ui))
        {
            const std::string recomputeError = ui.lastError;
            ui = previous;
            ui.lastError = recomputeError.empty() ? "visual-test recompute failed." : recomputeError;
            ui.dirty = false;
            return false;
        }

        updateViewerMeshes();
        ui.dirty = false;
        return false;
    };

    viewer.core().background_color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
    viewer.core().is_animating = true;
    viewer.core().animation_max_fps = 60.0f;
    viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);
    viewer.launch();
    return 0;
}
