#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/convex_hull_3.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel K;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::Surface_mesh<K::Point_3> Surface_mesh;
typedef CGAL::Nef_polyhedron_3<K> Nef_polyhedron;
typedef K::Point_3 Point_3;

// 全局变量
Eigen::MatrixXd V_box, V_tool, V_result;
Eigen::MatrixXi F_box, F_tool, F_result;
Eigen::MatrixXd V_tool_original;
double time_param = 0.0;
bool animation_running = true;
const double PI = 3.14159265358979323846;

// Nef 多面体 - 保持累积结果
Nef_polyhedron nef_result;
int mesh_id_result = 0;
int mesh_id_tool = 1;
Eigen::MatrixXd V_tool_prev;  // 注意：稍后会在 create_tool_mesh 后初始化

// 生成两帧之间刀具的扫掠体（3D凸包）
Polyhedron generate_swept_volume(const Eigen::MatrixXd& V_prev, const Eigen::MatrixXd& V_curr) 
{
	//TODO：现在是直接求凸包，在旋转、曲线等情况下步进越高精度越低，需要插值等优化
	std::vector<Point_3> points;
	points.reserve(V_prev.rows() + V_curr.rows());

	for (int i = 0; i < V_prev.rows(); ++i) {
		points.push_back(Point_3(V_prev(i, 0), V_prev(i, 1), V_prev(i, 2)));
	}
	for (int i = 0; i < V_curr.rows(); ++i) {
		points.push_back(Point_3(V_curr(i, 0), V_curr(i, 1), V_curr(i, 2)));
	}

	Polyhedron swept_poly;
	CGAL::convex_hull_3(points.begin(), points.end(), swept_poly);
	return swept_poly;
}

// Eigen 矩阵转 Polyhedron
Polyhedron eigen_to_polyhedron(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F) {
	Surface_mesh sm;
	std::vector<Surface_mesh::Vertex_index> vertices;

	for (int i = 0; i < V.rows(); ++i) {
		vertices.push_back(sm.add_vertex(Point_3(V(i, 0), V(i, 1), V(i, 2))));
	}
	for (int i = 0; i < F.rows(); ++i) {
		sm.add_face(vertices[F(i, 0)], vertices[F(i, 1)], vertices[F(i, 2)]);
	}

	Polyhedron poly;
	CGAL::copy_face_graph(sm, poly);
	return poly;
}

// Nef_polyhedron 转 Eigen 矩阵
void nef_to_eigen(const Nef_polyhedron& nef, Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
	if (nef.is_empty() || !nef.is_simple()) {
		return;
	}

	Surface_mesh sm;
	CGAL::convert_nef_polyhedron_to_polygon_mesh(nef, sm);
	CGAL::Polygon_mesh_processing::triangulate_faces(sm);

	V.resize(sm.number_of_vertices(), 3);
	F.resize(sm.number_of_faces(), 3);

	std::map<Surface_mesh::Vertex_index, int> v_map;
	int idx = 0;
	for (auto v : sm.vertices()) {
		auto p = sm.point(v);
		V(idx, 0) = CGAL::to_double(p.x());
		V(idx, 1) = CGAL::to_double(p.y());
		V(idx, 2) = CGAL::to_double(p.z());
		v_map[v] = idx++;
	}

	idx = 0;
	for (auto f : sm.faces()) {
		auto h = sm.halfedge(f);
		int i = 0;
		for (auto v : sm.vertices_around_face(h)) {
			if (i < 3) F(idx, i++) = v_map[v];
		}
		idx++;
	}
}

// 创建简单的刀具（圆柱体近似）
void create_tool_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F,
	double radius = 0.02, double height = 1.0, int segments = 8)
{
	int n = segments;
	V.resize(2 * n + 2, 3);

	// 顶部和底部中心点
	V.row(0) = Eigen::RowVector3d(0, 0, height / 2);
	V.row(1) = Eigen::RowVector3d(0, 0, -height / 2);

	// 圆周上的点
	for (int i = 0; i < n; ++i) {
		double angle = 2.0 * PI * i / n;
		double x = radius * cos(angle);
		double y = radius * sin(angle);
		V.row(2 + i) = Eigen::RowVector3d(x, y, height / 2);
		V.row(2 + n + i) = Eigen::RowVector3d(x, y, -height / 2);
	}

	// 创建面
	F.resize(4 * n, 3);
	for (int i = 0; i < n; ++i) {
		int next = (i + n - 1) % n;
		// 顶面
		F.row(i) = Eigen::RowVector3i(0, 2 + next, 2 + i);
		// 底面
		F.row(n + i) = Eigen::RowVector3i(1, 2 + n + i, 2 + n + next);
		// 侧面
		F.row(2 * n + i) = Eigen::RowVector3i(2 + i, 2 + next, 2 + n + i);
		F.row(3 * n + i) = Eigen::RowVector3i(2 + next, 2 + n + next, 2 + n + i);
	}
}

// 刀具变换：实现连续平滑的分段路径
Eigen::MatrixXd transform_tool(const Eigen::MatrixXd& V, double t) {
	Eigen::MatrixXd V_transformed = V;
	double current_t = std::min(t, 20.0); // 限制最大时间

	// 基础中心点和全局统一的高度
	double center_x = 0.5;
	double center_y = 0.5;
	double base_z = 1.4; // 【修复】全局保持这个基础高度

	// 阶段1：阿基米德螺旋的持续时间
	double t_phase1 = 2;

	// 预先计算第一阶段结束时的最终状态，确保第二阶段能无缝接上
	double max_theta = t_phase1 * 6.0;
	double max_radius = 0.015 * max_theta;

	if (current_t <= t_phase1) {
		// ================= 第一阶段：阿基米德螺旋 =================
		double theta = current_t * 6.0; // 当前旋转角度
		double r = 0.015 * theta;       // 当前半径

		double offset_x = center_x + r * cos(theta);
		double offset_y = center_y + r * sin(theta);

		for (int i = 0; i < V.rows(); ++i) {
			V_transformed(i, 0) += offset_x;
			V_transformed(i, 1) += offset_y;
			V_transformed(i, 2) += base_z;
		}
	}
	else {
		double t_phase2 = current_t - t_phase1;

		double revolve_angle = max_theta + t_phase2 * 3.0;

		double target_tilt_angle = 50.0 * PI / 180.0;
		double tilt_transition_time = 0.5;
		double current_tilt_angle = 0.0;

		if (t_phase2 < tilt_transition_time) {
			current_tilt_angle = target_tilt_angle * (t_phase2 / tilt_transition_time);
		}
		else {
			current_tilt_angle = target_tilt_angle;
		}

		Eigen::AngleAxisd tilt_rot(current_tilt_angle, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd revolve_rot(revolve_angle, Eigen::Vector3d::UnitZ());
		Eigen::Matrix3d R = revolve_rot.toRotationMatrix() * tilt_rot.toRotationMatrix();

		double offset_x = center_x + max_radius * cos(revolve_angle);
		double offset_y = center_y + max_radius * sin(revolve_angle);

		for (int i = 0; i < V.rows(); ++i) {
			Eigen::Vector3d p = V.row(i);

			// 把旋转中心下移到刀具底部（刀尖），这样倾斜时刀尖位置不会乱跑
			p(2) += 0.5; // 原点下移到 Z=-0.5 处
			p = R * p;   // 进行旋转
			p(2) -= 0.5; // 把原点移回去

			// 加上 XY 的平移 和 统一的 Z 基础高度
			V_transformed(i, 0) = p(0) + offset_x;
			V_transformed(i, 1) = p(1) + offset_y;
			V_transformed(i, 2) = p(2) + base_z; // 【关键修复2：高度连续】
		}
	}
	return V_transformed;
}

int main(int argc, char* argv[])
{
	bool sweep = false;
	std::cout << "输入1启用扫掠体，0单模型";
	std::cin >> sweep;

	V_box = (Eigen::MatrixXd(8, 3) <<
		0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 1.0,
		1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0).finished();

	F_box = (Eigen::MatrixXi(12, 3) <<
		0, 6, 4, 0, 2, 6, 0, 3, 2, 0, 1, 3,
		2, 7, 6, 2, 3, 7, 4, 6, 7, 4, 7, 5,
		0, 4, 5, 0, 5, 1, 1, 5, 7, 1, 7, 3).finished();

	create_tool_mesh(V_tool_original, F_tool);
	V_tool_prev = transform_tool(V_tool_original, 0.0);

	// 初始化 Nef 多面体
	Polyhedron poly_box = eigen_to_polyhedron(V_box, F_box);
	nef_result = Nef_polyhedron(poly_box);

	igl::opengl::glfw::Viewer viewer;

	V_result = V_box;
	F_result = F_box;
	viewer.data().set_mesh(V_result, F_result);
	mesh_id_result = viewer.data().id;
	viewer.data().set_face_based(true);
	viewer.data().set_colors(Eigen::RowVector3d(0.9, 0.9, 0.9));

	viewer.append_mesh();
	mesh_id_tool = viewer.data().id;
	V_tool = transform_tool(V_tool_original, time_param);
	viewer.data().set_mesh(V_tool, F_tool);
	viewer.data().set_face_based(true);
	viewer.data().set_colors(Eigen::RowVector3d(0.1, 0.9, 0.2));
	viewer.data().show_lines = false;

	viewer.callback_pre_draw = [&](igl::opengl::glfw::Viewer& v) -> bool {
		if (animation_running) {
			time_param += 0.01;
			Eigen::MatrixXd V_tool_curr = transform_tool(V_tool_original, time_param);

			if (sweep)
			{
				Polyhedron poly_sweep = generate_swept_volume(V_tool_prev, V_tool_curr);
				Nef_polyhedron nef_sweep(poly_sweep);

				auto t1 = std::chrono::high_resolution_clock::now();
				nef_result = nef_result - nef_sweep;
				auto t2 = std::chrono::high_resolution_clock::now();

				auto t3 = std::chrono::high_resolution_clock::now();
				nef_to_eigen(nef_result, V_result, F_result);
				auto t4 = std::chrono::high_resolution_clock::now();

				auto diff_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
				auto conv_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
				std::cout << "Frame " << static_cast<int>(time_param / 0.005)
					<< " | Sweep+Boolean: " << diff_ms << " ms"
					<< " | Convert: " << conv_ms << " ms" << std::endl;
			}
			else
			{
				//无扫掠体运算
				Polyhedron poly_tool = eigen_to_polyhedron(V_tool_curr, F_tool);
				Nef_polyhedron nef_tool(poly_tool);
				auto t1 = std::chrono::high_resolution_clock::now();
				nef_result = nef_result - nef_tool;
				auto t2 = std::chrono::high_resolution_clock::now();

				auto t3 = std::chrono::high_resolution_clock::now();
				nef_to_eigen(nef_result, V_result, F_result);
				auto t4 = std::chrono::high_resolution_clock::now();

				auto diff_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
				auto conv_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();
				std::cout << "Frame " << static_cast<int>(time_param / 0.005)
					<< " | Boolean: " << diff_ms << " ms"
					<< " | Convert: " << conv_ms << " ms" << std::endl;
			}

			v.data(mesh_id_result).clear();
			v.data(mesh_id_result).set_mesh(V_result, F_result);
			v.data(mesh_id_result).set_face_based(true);
			v.data(mesh_id_result).set_colors(Eigen::RowVector3d(0.9, 0.9, 0.9));
			v.data(mesh_id_result).show_lines = false;

			v.data(mesh_id_tool).clear();
			v.data(mesh_id_tool).set_mesh(V_tool_curr, F_tool);
			v.data(mesh_id_tool).set_face_based(true);
			v.data(mesh_id_tool).set_colors(Eigen::RowVector3d(0.1, 0.9, 0.2));
			v.data(mesh_id_tool).show_lines = false;

			V_tool_prev = V_tool_curr;
		}
		return false;
		};

	viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& v, unsigned char key, int) -> bool {
		if (key == ' ') {
			animation_running = !animation_running;
			return true;
		}
		if (key == 'R' || key == 'r') {
			Polyhedron poly_box = eigen_to_polyhedron(V_box, F_box);
			nef_result = Nef_polyhedron(poly_box);
			V_result = V_box;
			F_result = F_box;
			time_param = 0.0;
			return true;
			}
		return false;
		};

	viewer.core().is_animating = true;
	viewer.core().animation_max_fps = 30;

	viewer.core().background_color = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);

	viewer.core().set_rotation_type(igl::opengl::ViewerCore::ROTATION_TYPE_TRACKBALL);

	viewer.launch();
	return 0;
}