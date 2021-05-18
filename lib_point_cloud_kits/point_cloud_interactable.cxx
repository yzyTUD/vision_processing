#include <algorithm>
#include "point_cloud_interactable.h"
#include <cgv/gui/trigger.h>
#include <cgv/gui/key_event.h>
#include "ann_tree.h"
#include <cgv/base/find_action.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/import.h>
#include <cgv/utils/file.h>
#include <cgv/render/view.h>
#include <cgv/gui/dialog.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/mouse_event.h>
#include <cgv/gui/animate.h>
#include <cgv/gui/file_dialog.h>
#include <cgv/reflect/reflect_extern.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/utils/tokenizer.h>
#include <libs\cgv_gl\sphere_renderer.h>

#include "pca.h"

#define FILE_OPEN_TITLE "Open Point Cloud"
#define FILE_APPEND_TITLE "Append Point Cloud"
#define FILE_OPEN_FILTER "Point Clouds (apc,bpc):*.apc;*.bpc|Mesh Files (obj,ply,pct):*.obj;*.ply;*.pct|All Files:*.*"

#define FILE_SAVE_TITLE "Save Point Cloud"
#define FILE_SAVE_FILTER "Point Clouds (apc,bpc):*.apc;*.bpc|Mesh Files (obj,ply):*.obj;*.ply|All Files:*.*"
///
void point_cloud_interactable::print_pc_information() {
	std::cout << "### begin: point cloud info: ###" << std::endl;
	std::cout << "pc.P.size: " << pc.P.size() << std::endl;
	std::cout << "pc.N.size: " << pc.N.size() << std::endl;
	std::cout << "pc.C.size: " << pc.C.size() << std::endl;
	std::cout << "pc.T.size: " << pc.T.size() << std::endl;
	std::cout << "pc.I.size: " << pc.I.size() << std::endl;
	std::cout << "pc.F.size: " << pc.F.size() << std::endl;
	std::cout << "pc.face_id.size: " << pc.face_id.size() << std::endl;
	std::cout << "pc.topo_id.size: " << pc.topo_id.size() << std::endl;
	std::cout << "pc.point_scan_index.size: " << pc.point_scan_index.size() << std::endl;
	std::cout << "pc.lods.size: " << pc.lods.size() << std::endl;
	std::cout << "pc.F_conn.size(): " << pc.F_conn.size() << std::endl;

	std::cout << "pc.curvinfo.curvinfo.minimum_curvature_difference = " << pc.curvinfo.minimum_curvature_difference << std::endl;
	std::cout << "pc.curvinfo.max_mean_curvature = " << pc.curvinfo.max_mean_curvature << std::endl;
	std::cout << "pc.curvinfo.min_mean_curvature = " << pc.curvinfo.min_mean_curvature << std::endl;
	std::cout << "pc.curvinfo.max_gaussian_curvature = " << pc.curvinfo.max_gaussian_curvature << std::endl;
	std::cout << "pc.curvinfo.min_gaussian_curvature = " << pc.curvinfo.min_gaussian_curvature << std::endl;
	//for (auto f : pc.F_conn) {
	//	std::cout << "current f.size(): " << f.size() << std::endl;
	//}
	std::cout << "### end: point cloud info: ###" << std::endl;
}

void point_cloud_interactable::update_file_name(const std::string& ffn, bool append)
{
	/*std::string new_path = cgv::utils::file::get_path(ffn);
	if (!new_path.empty()) {
		data_path = new_path;
		update_member(&data_path);
	}
	std::string new_name = cgv::utils::file::get_file_name(cgv::utils::file::drop_extension(ffn));
	if (append && !file_name.empty()) {
		file_name += std::string("#")+ new_name;
	}
	else
		file_name = new_name;
	new_file_name = file_name;
	update_member(&new_file_name);*/
}
/// file io
bool point_cloud_interactable::save(const std::string& fn)
{
	if (!write(fn)) {
		cgv::gui::message(std::string("could not write ") + fn);
		return false;
	}
	update_file_name(fn);
	return true;
}
/// file io
bool point_cloud_interactable::open(const std::string& fn)
{
	if (!read(fn)) {
		cgv::gui::message(std::string("could not read ") + fn);
		return false;
	}
	// manage vars after change of the point cloud 
	if (pc.get_nr_points() > 0) // sometimes, we are not realy reading point clouds 
		on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	using_directly_buffer_loading = false;
	//update_file_name(fn);
	//store_original_pc();
	//auto_downsampling();
	return true;
}

/// 
void point_cloud_interactable::generate_pc_random_sphere() {
	std::default_random_engine g;
	std::uniform_real_distribution<float> d(-1, 1);
	int nr_vertices = 5000;

	//
	pc.clear_all();

	// collect 
	for (int i = 0; i < nr_vertices; i++) {
		vec3 sampled_vector = vec3(d(g), d(g), d(g));
		sampled_vector.normalize();
		vec3 p = sampled_vector;
		vec3 nml = sampled_vector;
		rgb c = rgb(0.4);
		int scan_index = 0;
		int selection_index = 1;
		pc.add_point(p, c, nml, scan_index, selection_index);
	}

	//
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_face_selection = true;

	//
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	finished_loading_points = true;

}

void point_cloud_interactable::generate_pc_unit_torus() {
	//
	pc.clear_all();
	//
	int resolution = 100; int nr_vertices = 2 * (resolution + 1); 
	float minor_radius = 0.2;

	std::vector<float> V; V.resize(6 * (resolution + 1));
	std::vector<float> N; N.resize(6 * (resolution + 1));
	std::vector<float> T; T.resize(4 * (resolution + 1));
	std::vector<int> F; F.resize(2 * (resolution + 1));
	int i;
	for (int i = 0; i <= resolution; ++i) {
		F[2 * i] = 2 * i;
		F[2 * i + 1] = 2 * i + 1;
	}
	float step = float(2 * M_PI / resolution);
	float phi = 0;
	float cp1 = 1, sp1 = 0;
	float u = 0;
	float duv = float(1.0 / resolution);
	for (i = 0; i < resolution; ++i, u += duv) {
		float cp0 = cp1, sp0 = sp1;
		phi += step;
		cp1 = cos(phi);
		sp1 = sin(phi);
		float theta = 0;
		float v = 0;
		int kv = 0, kn = 0, kt = 0;
		for (int j = 0; j <= resolution; ++j, theta += step, v += duv) {
			float ct = cos(theta), st = sin(theta);
			N[kn++] = ct * cp0;
			N[kn++] = ct * sp0;
			N[kn++] = st;
			T[kt++] = u;
			T[kt++] = v;
			V[kv++] = cp0 + minor_radius * cp0 * ct;
			V[kv++] = sp0 + minor_radius * sp0 * ct;
			V[kv++] = minor_radius * st;
			N[kn++] = ct * cp1;
			N[kn++] = ct * sp1;
			N[kn++] = st;
			T[kt++] = u + duv;
			T[kt++] = v;
			V[kv++] = cp1 + minor_radius * cp1 * ct;
			V[kv++] = sp1 + minor_radius * sp1 * ct;
			V[kv++] = minor_radius * st;
		}
		//
		std::vector<vec3> Parray;
		std::vector<vec3> Narray;
		Parray.resize(nr_vertices);
		Narray.resize(nr_vertices);
		for (int i = 0; i < nr_vertices; ++i)
			Parray[i] = *reinterpret_cast<const vec3*>(&V[0] + 3 * F[i]);
		for (int i = 0; i < nr_vertices; ++i)
			Narray[i] = *reinterpret_cast<const vec3*>(&N[0] + 3 * F[i]);

		// collect 
		for (int i = 0; i < nr_vertices; i++) {
			vec3 p = Parray.at(i);
			vec3 nml = Narray.at(i);
			rgb c = rgb(0.4);
			int scan_index = 0;
			int selection_index = 1;
			pc.add_point(p, c, nml, scan_index, selection_index);
		}
	}

	//
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_face_selection = true;

	//
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	finished_loading_points = true;
}

void point_cloud_interactable::generate_pc_unit_cylinder() {
	int resolution = 50;
	std::vector<float> V; V.reserve(6 * (resolution + 1));
	std::vector<float> N; N.reserve(6 * (resolution + 1));
	std::vector<float> T; T.reserve(4 * (resolution + 1));

	std::vector<int> F; F.resize(2 * (resolution + 1));
	int i;
	for (i = 0; i <= 2 * resolution + 1; ++i)
		F[i] = i;

	float step = float(2 * M_PI / resolution);
	float phi = 0;
	float u = 0;
	float duv = float(1.0 / resolution);
	for (int i = 0; i <= resolution; ++i, u += duv, phi += step) {
		float cp = cos(phi);
		float sp = sin(phi);
		N.push_back(cp);
		N.push_back(sp);
		N.push_back(0);
		T.push_back(u);
		T.push_back(1);
		V.push_back(cp);
		V.push_back(sp);
		V.push_back(1);
		N.push_back(cp);
		N.push_back(sp);
		N.push_back(0);
		T.push_back(u);
		T.push_back(0);
		V.push_back(cp);
		V.push_back(sp);
		V.push_back(-1);
	}
	unsigned nr_vertices = 2 * (resolution + 1);
	std::vector<vec3> Parray;
	std::vector<vec3> Narray;
	Parray.resize(nr_vertices);
	Narray.resize(nr_vertices);
	for (int i = 0; i < nr_vertices; ++i)
		Parray[i] = *reinterpret_cast<const vec3*>(&V[0] + 3 * i);
	for (int i = 0; i < nr_vertices; ++i)
		Narray[i] = *reinterpret_cast<const vec3*>(&N[0] + 3 * i);

	//
	pc.clear_all();

	// collect 
	for (int i = 0; i < nr_vertices; i++) {
		vec3 p = Parray.at(i);
		vec3 nml = Narray.at(i);
		rgb c = rgb(0.4);
		int scan_index = 0;
		int selection_index = 1;
		pc.add_point(p, c, nml, scan_index, selection_index);
	}

	//
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_face_selection = true;

	//
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	finished_loading_points = true;
}

/// 
void point_cloud_interactable::generate_pc_init_sphere() {
	//
	pc.clear_all();
	//
	int resolution = 50; int nr_vertices = 2 * (resolution + 1);
	std::vector<float> V; V.resize(6 * (resolution + 1));
	std::vector<float> N; N.resize(6 * (resolution + 1));
	std::vector<float> T; T.resize(4 * (resolution + 1));
	std::vector<int> F; F.resize(2 * (resolution + 1));
	int i;
	for (int i = 0; i <= resolution; ++i) {
		F[2 * i] = 2 * i;
		F[2 * i + 1] = 2 * i + 1;
	}
	float step = float(M_PI / resolution);
	float phi = 0;
	float cp1 = 1, sp1 = 0;
	float u = 0;
	float duv = float(1.0 / resolution);
	for (i = 0; i < resolution; ++i, u += duv) {
		float cp0 = cp1, sp0 = sp1;
		phi += 2 * step;
		cp1 = cos(phi);
		sp1 = sin(phi);
		float theta = float(-0.5 * M_PI);
		float v = 0;
		int kv = 0, kn = 0, kt = 0;
		for (int j = 0; j <= resolution; ++j, theta += step, v += duv) {
			float ct = cos(theta), st = sin(theta);
			N[kn++] = ct * cp0;
			N[kn++] = ct * sp0;
			N[kn++] = st;
			T[kt++] = u;
			T[kt++] = v;
			V[kv++] = ct * cp0;
			V[kv++] = ct * sp0;
			V[kv++] = st;
			N[kn++] = ct * cp1;
			N[kn++] = ct * sp1;
			N[kn++] = st;
			T[kt++] = u + duv;
			T[kt++] = v;
			V[kv++] = ct * cp1;
			V[kv++] = ct * sp1;
			V[kv++] = st;
		}
		//
		std::vector<vec3> Parray;
		std::vector<vec3> Narray;
		Parray.resize(nr_vertices);
		Narray.resize(nr_vertices);
		for (int i = 0; i < nr_vertices; ++i)
			Parray[i] = *reinterpret_cast<const vec3*>(&V[0] + 3 * F[i]);
		for (int i = 0; i < nr_vertices; ++i)
			Narray[i] = *reinterpret_cast<const vec3*>(&N[0] + 3 * F[i]);

		// collect 
		for (int i = 0; i < nr_vertices; i++) {
			vec3 p = Parray.at(i);
			vec3 nml = Narray.at(i);
			rgb c = rgb(0.4);
			int scan_index = 0;
			int selection_index = 1;
			pc.add_point(p, c, nml, scan_index, selection_index);
		}
	}

	//
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_face_selection = true;

	//
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	finished_loading_points = true;
}

/// procedure point cloud generation, a hemisphere
bool point_cloud_interactable::generate_pc_hemisphere() {
	pc.clear_all();
	int samples_per_row = 200;
	int nr_rows = 400;
	Pnt extent = Pnt(1, 1, 1);
	rgba c = rgba(0.5, 0.5, 0.5, 1);
	float R = 1.0f;
	Pnt offset = Pnt(0, 1, 0);
	for (Idx li = 0; li < nr_rows; ++li) {
		float y = (float)li / (nr_rows - 1) - 0.5f;
		for (Idx ci = 0; ci < samples_per_row; ++ci) {
			float x = (float)ci / (samples_per_row - 1) - 0.5f;
			Pnt p, n;
			float r2 = x * x + y * y;
			if (r2 > 0.25f) {
				p = Pnt(x, 0, y) + offset;
				//n = vec3(0, 0, 1);
			}
			else {
				float g = sqrt(R * R - r2);
				float f = g - sqrt(R * R - 0.25f);
				p = Pnt(x, f, y) + offset;
				//n = vec3(x / g, y / g, 1);
			}
			pc.add_point(extent * p, c, 0, 1u);
			//pc.nml(pi) = normalize(n/extent);
		}
	}
	pc.create_colors();
	pc.has_face_selection = true;
	pc.has_scan_index = true;
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	return true;
}

/// uniform sampling in a plane 
void point_cloud_interactable::generate_testing_plane() {
	pc.clear_all();
	float nr_rows = 400;
	float nr_cols = 400;
	float extend = 1;
	for (int li = 0; li < nr_rows; ++li) {
		for (int lj = 0; lj < nr_cols; ++lj) {
			vec3 p = vec3(li / nr_rows, 0, lj / nr_cols);
			rgb c = rgb(0.4);
			vec3 nml = vec3(0, 1, 0);
			int scan_index = 0;
			int selection_index = 1;
			pc.add_point(p,c,nml,scan_index,selection_index);
		}
	}

	//
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_face_selection = true;

	//
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	finished_loading_points = true;
}
/// procedure point cloud generation, a cube 
bool point_cloud_interactable::generate_pc_cube() {
	pc.clear_all();
	int samples_per_row = 200;
	int nr_rows = 400;
	vec3 extent = vec3(1, 1, 1);
	rgba c = rgba(0.5, 0.5, 0.5, 1);
	for (int li = 0; li < nr_rows; ++li) {
		float y = (float)li / (nr_rows - 1) - 0.5f;
		for (int ci = 0; ci < samples_per_row; ++ci) {
			float x = (float)ci / (samples_per_row - 1) - 0.5f;
			vec3 p, n;
			if (x < 0.25 && x > -0.25 && y < 0.25 && y > -0.25) {
				if (abs(x - 0.25f) < 0.005f || abs(x + 0.25f) < 0.005f ||
					abs(y - 0.25f) < 0.0025f || abs(y + 0.25f) < 0.0025f) {
					for (int zi = 0; zi < 50; ++zi) {
						float z = ((float)zi / (50 - 1)) * 0.25f;
						p = vec3(x, z, y);
						pc.add_point(extent * p, c, 0, 1u);
					}
				}
				else {
					p = vec3(x, 0.25f, y);
					pc.add_point(extent * p, c, 0, 1u);
				}
			}
			else {
				p = vec3(x, 0, y);
				pc.add_point(extent * p, c, 0, 1u);
			}
		}
	}
	pc.create_colors();
	pc.has_face_selection = true;
	pc.has_scan_index = true;
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	return true;
}
///
void point_cloud_interactable::downsampling(int step, int num_of_points_wanted, int which_strategy) {
	if(which_strategy == 0)
		pc.downsampling(step);
	if(which_strategy == 1)
		pc.downsampling_expected_num_of_points(num_of_points_wanted);
	std::cout << "points remind:" << pc.get_nr_points() << std::endl;
	on_point_cloud_change_callback(PCC_POINTS_RESIZE);
	post_redraw();
}
// inner function, backup the current point cloud 
void point_cloud_interactable::store_original_pc() {
	oripc = pc;
}
// inner function, perform an auto downsampling 
void point_cloud_interactable::auto_downsampling() {
	int best_point_num = 1000 * 1000;
	if(pc.get_nr_points()> best_point_num)
		downsampling(-1, best_point_num, 1);
}
// external invoke expected: called in data_store header, from vr_kit_selection 
void point_cloud_interactable::supersampling_within_clips(std::vector<Pnt> positions, std::vector<Dir> dirs) {
	pc_to_be_append = oripc;
	pc_to_be_append.preserve_bounded_points_with_drawn_data(positions, dirs);
	// should be smart_not_overwrite_append 
	pc.append(pc_to_be_append);
	std::cout << "points now:" << pc.get_nr_points() << std::endl;
	on_point_cloud_change_callback(PCC_POINTS_RESIZE);
	post_redraw();
}
// external invoke expected: not called, designed in data_storage, modify bbox in vr_kit_selection
void point_cloud_interactable::supersampling_with_bbox(box3 range_as_box) {
	// store original pc 
	pc_to_be_append = oripc;
	pc_to_be_append.subsampling_with_bbox(range_as_box);
	pc.append(pc_to_be_append); // should be pc.smart_append_skip_equal(pc_to_be_append);
	std::cout << "points now:" << pc.get_nr_points() << std::endl;
	on_point_cloud_change_callback(PCC_POINTS_RESIZE);
	post_redraw();
}
// external invoke expected: called in main class, gui 
void point_cloud_interactable::restore_supersampling() {
	pc = oripc;
	auto_downsampling();
	std::cout << "points now:" << pc.get_nr_points() << std::endl;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	post_redraw();
}
// external invoke expected
void point_cloud_interactable::render_with_fullpc() {
	// store original pc
	// keep the changes in current pc!, preform a position matching for the points? 
	point_cloud modipc;
	//modipc = pc;
	//oripc.smart_append_overwrite_equal(modipc); // smart_overwrite_append
	pc = oripc;
	std::cout << "points now:" << pc.get_nr_points() << std::endl;
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	post_redraw();
}
///
bool point_cloud_interactable::open_directory(const std::string& dn)
{
	std::vector<std::string> file_names;
	void* handle = cgv::utils::file::find_first(directory_name + "/*.*");
	while (handle) {
		if (!cgv::utils::file::find_directory(handle))
			file_names.push_back(cgv::utils::file::find_name(handle));
		handle = cgv::utils::file::find_next(handle);
	}
	if (file_names.empty()) {
		std::cerr << "did not find files in directory <" << dn << ">" << std::endl;
		return false;
	}
	unsigned i;
	for (i = 0; i < file_names.size(); ++i) {
		std::cout << i << "(" << file_names.size() << "): " << file_names[i];
		std::cout.flush();
		if (!do_append && i == 0)
			open(directory_name + "/" + file_names[i]);
		else
			open_and_append(directory_name + "/" + file_names[i]);
		std::cout << std::endl;
	}
	for (i = 0; i < pc.get_nr_components(); ++i) {
		pc.component_color(i) = cgv::media::color<float, cgv::media::HLS, cgv::media::OPACITY>(float(i) / float(pc.get_nr_components() - 1), 0.5f, 1.0f, 1.0f);
	}
	use_component_colors = true;
	update_member(&use_component_colors);
	if (!do_append)
		on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	update_file_name(dn);
	return true;
}
///
bool point_cloud_interactable::open_and_append(const std::string& fn)
{
	if (!append(fn)) {
		cgv::gui::message(std::string("could not append ") + fn);
		return false;
	}
	on_point_cloud_change_callback(PointCloudChangeEvent(PCC_POINTS_RESIZE + PCC_COMPONENTS_RESIZE));
	update_file_name(fn, true);
	return true;
}
///
bool point_cloud_interactable::open_or_append(cgv::gui::event& e, const std::string& file_name)
{
	cgv::utils::tokenizer T(file_name);
	T.set_ws("\n").set_sep("");
	std::vector<cgv::utils::token> toks;
	T.bite_all(toks);
	bool res = false;
	for (unsigned i = 0; i<toks.size(); ++i) {
		std::string file_path = cgv::base::find_data_file(to_string(toks[i]), "CM", "", data_path);
		if (e.get_modifiers() == cgv::gui::EM_ALT || i > 0)
			res |= open_and_append(file_path);
		else
			res |= open(file_path);
	}
	return res;
}
/// read point cloud with a dialog 
bool point_cloud_interactable::read_pc_with_dialog(bool append) {
	file_dir = cgv::gui::file_open_dialog("Open", "Point Cloud:*.*");
	auto start = std::chrono::high_resolution_clock::now();
	data_path = cgv::utils::file::get_path(file_dir);
	file_name = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(file_dir));
	reading_from_raw_scan = append;
	if (!append) 
		clear_all();
	if (!open(file_dir))
		return false;
	finished_loading_points = true;
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> d = finish - start;
	std::cout << "elapsed time: " << d.count() << std::endl;
	//std::cout << "read file_name: " << file_name << std::endl;
	//std::cout << "read data_path: " << data_path << std::endl;
	return true;
}
bool point_cloud_interactable::read_cgvcad_with_dialog() {
	std::string f = cgv::gui::file_open_dialog("Open", "cgvcad:*");
	if (!open(f))
		return false;
	return true;
}
///
bool point_cloud_interactable::read_pc_with_dialog_queue(bool append) {
	if (!append)
		clear_all();
	std::vector<std::string> f_names;
	cgv::gui::files_open_dialog(f_names, "Open", "Point Cloud:*");
	for(auto& f:f_names)
		open(f);
	return true;
}
/// read point cloud and perform a automatic downsampling 
bool point_cloud_interactable::read_pc_subsampled_with_dialog() {
	std::string f = cgv::gui::file_open_dialog("Open", "Point Cloud:*");
	clear_all();
	pc.read_pts_subsampled(f,0.05);
	show_point_begin = 0;
	show_point_end = pc.get_nr_points();
	return true;
}
///
void point_cloud_interactable::write_pc_to_file() {
	/*auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
	std::chrono::system_clock::now().time_since_epoch()).count();
	std::string filename_base = cgv::base::ref_data_path_list()[0] + "\\object_scanns\\pointcloud_all_" + std::to_string(microsecondsUTC);
	std::string filename = filename_base + ".obj";*/
	std::string f = cgv::gui::file_save_dialog("Open", "Save Point Cloud:*");
	pc.suggested_point_size = surfel_style.point_size; // write point size by default 
	pc.write(f);
	std::cout << "saved!" << std::endl;
}
///
void point_cloud_interactable::write_pc_to_file_with_given_dir(const std::string f) {
	/*auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
	std::to_string(microsecondsUTC)*/
	pc.write(f);
	std::cout << "saved!" << std::endl;
}
/// read camera positions with a dialog, .campose files are accepted
bool point_cloud_interactable::read_pc_campose(cgv::render::context& ctx, quat initialcamq) {
	std::string f = cgv::gui::file_open_dialog("Open", "Point Cloud:*");
	pc.read_campose(f);

	if (render_camposes) {
		//alig with cgv fram
		align_leica_scans_with_cgv();
		// prepare for rendering 
		for (int i = 0; i < pc.num_of_shots; i++) {
			vr_kit_image_renderer tmp_renderer;
			std::string folder_name = cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(f));
			std::string panorama_fn = cgv::utils::file::get_path(f) + "/pano/apb_" + std::to_string(i + 1) + ".jpg";
			std::cout << panorama_fn << std::endl; //pc.list_cam_rotation.at(i) //initialcamq * pc.list_cam_rotation.at(i)
			tmp_renderer.bind_image_to_camera_position(ctx, panorama_fn, quat(), pc.list_cam_translation.at(i));
			image_renderer_list.push_back(tmp_renderer);
		}
	}

	return true;
}
///
bool point_cloud_interactable::check_valid_pc_and_campose()
{
	bool succ = pc.cam_posi_list.front().x() != -1000
				&& pc.cam_posi_list.front().y() != -1000
				&& pc.cam_posi_list.front().z() != -1000
				&& pc.get_nr_points() == pc.num_of_points_in_campose
				&& pc.has_cam_posi;
	if (!succ) {
		std::cout << "invalid! check following vars:" << std::endl;
		std::cout << "pc.cam_posi_list front: " << pc.cam_posi_list.front() << std::endl;
		std::cout << "pc.get_nr_points(): " << pc.get_nr_points() << std::endl;
		std::cout << "pc.num_of_points_in_campose: " << pc.num_of_points_in_campose << std::endl;
		std::cout << "pc.has_cam_posi: " << pc.has_cam_posi << std::endl;
	}
	return succ;
}
///
void point_cloud_interactable::apply_further_transformation(int which, quat q, vec3 t) {
	image_renderer_list.at(which).apply_further_transformation(q, t);
}
///
void point_cloud_interactable::align_leica_scans_with_cgv() {
	quat rz = quat(vec3(0, 0, 1), 25 * M_PI / 180);
	quat rx = quat(vec3(1, 0, 0), -90 * M_PI / 180);
	quat r_align = rx * rz;
	for (auto& t : pc.list_cam_translation) {
		r_align.rotate(t);
	}
	for (auto& r : pc.list_cam_rotation) {
		r = r_align * r;
	}
	pc.rotate(r_align);
	// approximate 1m from ground 
	//pc.translate(vec3(0,1,0));
}

/*marking on point cloud, cpu side, ineffecient when upload to gpu*/
/// clear seeds, setup color indices ...
void point_cloud_interactable::prepare_marking() {
	pc.face_id.resize(pc.get_nr_points());
	for (auto& v : pc.face_id) v = 0;
	pc.point_visited.resize(pc.get_nr_points());
	for (auto& w : pc.point_visited) w = false;

	// modified rendering process, no need anymore 
	//use_these_point_palette = psc;
	//use_these_point_color_indices = &pc.face_id;
}
/// current operation is a list of entries 
void point_cloud_interactable::new_history_recording() {
	std::vector<pointHistoryEntry> current_selection;
	point_marking_history.push(current_selection);
}
///
void point_cloud_interactable::reset_last_marking_non_processed_part(int which_is_marked_and_not_used) {
	// iterate all point indices recorded last time 
	//for (int i = 0; i < point_marking_history.back().size(); i++) {
	//	// reset if not changed marking, that is, which_is_marked_and_not_used
	//	// current point index: point_marking_history.back().at(i).point_index
	//	// current selection: pc.face_id.at(point_marking_history.back().at(i))
	//	if (pc.face_id.at(point_marking_history.back().at(i).point_index) == (int)which_is_marked_and_not_used) {
	//		pc.face_id.at(point_marking_history.back().at(i).point_index) 
	//			= point_cloud::TOPOAttribute::ORI;
	//	}
	//}
}
/// not finished -> to be tested
void point_cloud_interactable::reset_last_marked_points() {
	// recover the top entry list, undo the point face marking 
	for (int i = 0; i < point_marking_history.top().size(); i++) {
		pc.face_id.at(point_marking_history.top().at(i).point_index) = 
			point_marking_history.top().at(i).from_face_id;
	}
	// and pop 
	point_marking_history.pop();
}
// errors may occour when step too much 
void point_cloud_interactable::step_back_last_selection() {
	reset_last_marked_points();
	// reset rendering 
	render_with_topo_selctions_only = false;
}
// not finished -> to be tested
// errors may occour when step too much 
void point_cloud_interactable::step_forward_selection() {

}
///
void point_cloud_interactable::mark_all_points_and_push_to_tmp_pointcloud_test(Pnt p, float r, int ignore_id = -1) {
	bool some_points_copied = false;
	float scan_index = 0;
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		//if ((pc.pnt(i) - p).length() < r) {
			if (pc.face_id.at(i) != ignore_id) {
				if (pc.point_scan_index.size() > 0)
					scan_index = pc.point_scan_index.at(i);
				else
					scan_index = 0;
				to_be_copied_pointcloud.add_point(pc.pnt(i), pc.clr(i), pc.nml(i), scan_index, pc.face_id.at(i));
				some_points_copied = true;
			}
		//}
	}
	if (some_points_copied) {
		// require properties 
		to_be_copied_pointcloud.create_colors();
		to_be_copied_pointcloud.create_normals();
		to_be_copied_pointcloud.has_scan_index = true;
		to_be_copied_pointcloud.has_face_selection = true;
		to_be_copied_pointcloud.box_out_of_date = true;
	}
}
///
void point_cloud_interactable::mark_points_and_push_to_tmp_pointcloud(Pnt p, float r, int ignore_id = -1) {
	bool some_points_copied = false;
	float scan_index = 0;
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		if ((pc.pnt(i) - p).length() < r) {
			if (pc.face_id.at(i) != ignore_id) {
				if (pc.point_scan_index.size() > 0)
					scan_index = pc.point_scan_index.at(i);
				else
					scan_index = 0;
				to_be_copied_pointcloud.add_point(pc.pnt(i), pc.clr(i), pc.nml(i), scan_index, pc.face_id.at(i));
				some_points_copied = true;
			}
		}
	}
	if (some_points_copied) {
		// require properties 
		to_be_copied_pointcloud.create_colors();
		to_be_copied_pointcloud.create_normals();
		to_be_copied_pointcloud.has_scan_index = true;
		to_be_copied_pointcloud.has_face_selection = true;
		to_be_copied_pointcloud.box_out_of_date = true;
	}
}

void point_cloud_interactable::mark_topo_id_with_controller(Pnt p, float r, int objctive) {
	if (pc.get_nr_points() == 0)
		return;
	// todo: improve this ref. below 
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		if ((pc.pnt(i) - p).length() < r) {
			// ignore deleted points 
			if (pc.topo_id.at(i) == point_cloud::TOPOAttribute::DEL) {
				continue;
			}
			// ignore marked points if is highlighting unmarked points 
			if (highlight_unmarked_points) {
				if (pc.face_id.at(i) > 0) { // do not draw on already marked, 0 means unmarked  
					continue;
				}
			}
			//// ignore points that are not having correct scaning index, ineffecient 
			//if (objctive == point_cloud::TOPOAttribute::ICP_SOURCE_A) {
			//	if (pc.point_scan_index.at(i) != src_scan_idx) {
			//		continue;
			//	}
			//}
			//// ignore points that are not having correct scaning index 
			//if (objctive == point_cloud::TOPOAttribute::ICP_TARGET_A) {
			//	if (pc.point_scan_index.at(i) != target_scan_idx) {
			//		continue;
			//	}
			//}
			// record topo id change information: todo 
			// perform real operations 
			pc.topo_id.at(i) = objctive;
			pc.has_topo_selection = true; // has real data, do not overwrite 
		}
	}
}
///
void point_cloud_interactable::mark_leaking_points_face_id_and_other_attributes(Pnt p, float r) {
	if (pc.get_nr_points()) {
		ensure_tree_ds();
		float closest_dist = -1;
		int closest_idx = -1;
		tree_ds->find_closest_and_its_dist(p, closest_dist, closest_idx);
		if (closest_dist > r) {
			// do nothing 
		}
		else {
			for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
				if ((pc.pnt(i) - p).length() < r) {
					// ignore deleted points 
					if (pc.topo_id.at(i) == point_cloud::TOPOAttribute::DEL) {
						continue;
					}
					// ignore points already marked as original 
					if (pc.face_id.at(i) != 0) { 
						pc.face_id.at(i) = 0;
						pc.point_visited.at(i) = false;
						pc.point_in_queue.at(i) = false;
						pc.point_in_queue_which_group.at(i) = 0;
						points_grown--;
					}
				}
			}
		}
	}
}
///
void point_cloud_interactable::mark_points_in_queue_to_original(Pnt p, float r) {
	if (pc.get_nr_points()) {
		ensure_tree_ds();
		float closest_dist = -1;
		int closest_idx = -1;
		tree_ds->find_closest_and_its_dist(p, closest_dist, closest_idx);
		if (closest_dist > r) {
			// do nothing 
		}
		else {
			for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
				if ((pc.pnt(i) - p).length() < r) {
					// ignore deleted points 
					if (pc.topo_id.at(i) == point_cloud::TOPOAttribute::DEL) {
						continue;
					}
					// only mark points in queue, ignore other points 
					if (pc.point_in_queue.at(i) == true) {
						pc.face_id.at(i) = 0;
						pc.point_visited.at(i) = false;
						pc.point_in_queue.at(i) = false; // will be ignored when dequeue, this flag has dual usage 
						pc.point_in_queue_which_group.at(i) = 0;
						points_grown--;
					}
				}
			}
		}
	}
}
/// currently, we set confirmed to true. The visual feedback is better to be impl. in shaders.
/// ignore deleted points by default 
void point_cloud_interactable::mark_face_id_with_controller(Pnt p, float r, int objctive) {
	new_history_recording();
	if (pc.get_nr_points()) {
		ensure_tree_ds();
		float closest_dist = -1;
		int closest_idx = -1;
		tree_ds->find_closest_and_its_dist(p, closest_dist, closest_idx);
		//std::cout << "closest_dist = "<< closest_dist << std::endl;
		if (closest_dist > r) {
			// do nothing 
		}
		else {
			for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
				if ((pc.pnt(i) - p).length() < r) {
					// ignore deleted points 
					if (pc.topo_id.at(i) == point_cloud::TOPOAttribute::DEL) {
						continue;
					}
					// ignore marked points if is highlighting unmarked points 
					if (highlight_unmarked_points) {
						if (pc.face_id.at(i) > 0) { // ignore already marked, 0 means unmarked  
							continue;
						}
					}
					//// ignore points that are not having correct scaning index, ineffecient 
					//if (objctive == point_cloud::TOPOAttribute::ICP_SOURCE_A) {
					//	if (pc.point_scan_index.at(i) != src_scan_idx) {
					//		continue;
					//	}
					//}
					//// ignore points that are not having correct scaning index 
					//if (objctive == point_cloud::TOPOAttribute::ICP_TARGET_A) {
					//	if (pc.point_scan_index.at(i) != target_scan_idx) {
					//		continue;
					//	}
					//}
					// record tracing information
					pointHistoryEntry phe;
					phe.point_index = i;
					phe.from_face_id = pc.face_id.at(i);
					phe.to_face_id = objctive;
					point_marking_history.top().push_back(phe);
					// perform real operations 
					pc.face_id.at(i) = objctive;
					pc.has_face_selection = true;
				}
			}
		}
		on_point_cloud_change_callback(PCC_COLORS);
	}
}
/// mark_all_points_as_given_group, still, you have to prepare first, typically done in reading process
void point_cloud_interactable::marking_test_mark_all_points_as_given_group(int objective) {
	for (int i = 0; i < pc.get_nr_points(); i++) {
		// record tracing information
		pointHistoryEntry phe;
		phe.point_index = i;
		phe.from_face_id = pc.face_id.at(i);
		phe.to_face_id = objective;
		point_marking_history.top().push_back(phe);
		// int -> uint8
		// perform real operations 
		pc.face_id[i] = objective;
	}
}
/// TODO history support 
void point_cloud_interactable::mark_points_with_clipping_plane(Pnt p,Nml plane_normal,int objective) {
	new_history_recording();
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		if (dot(p - pc.pnt(i), plane_normal) < 0) {
			pc.face_id[i] = objective;
		}
	}
}

/*operations after marked */
/*
	algorithm:
		for each point collect all face indices of neighbors with knn, classify point into
            a.interior, if all face indices are identical
            b.boundary, if two different face indices arise
            c.corner otherwise, number of different face indices is called valence of point
        a. face_extraction
            loop over all face points, fill the data structure with point information.
        b. corner_extraction
            find one corner point that is not visited, if can not find, terminate.
            Do region growing with knn neighbor graph to collect neighbor points, add them to data structure.
                stopping criteria: find non corner point or corner point whose face indices are not 
                    the same face indices of reference point.
        c. edge_extraction
            find one boundary point that is not visited, if can not find, terminate.
			do region growing with stopping criteria: find non edge point or edge point whose face indices are not 
                    the same face indices of reference point.
*/
/// pre-requirement: rg is done, per point selection ready. after marked.
/// result will be stored and visualized with TOPOAttribute
#include <set>
#include <unordered_set>
void point_cloud_interactable::point_classification() {
	std::vector<int> will_be_marked_as_boundary;
	std::vector<int> will_be_marked_as_corner;
	//new_history_recording();
	std::vector<pointHistoryEntry> current_selection;
	//pc.per_vertex_topology.resize(pc.get_nr_points());
	ensure_tree_ds();
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		// initialize the topology information 
		// pc.per_vertex_topology = 
		// ignore deleted points 
		if (pc.topo_id.at(i) == point_cloud::TOPOAttribute::DEL)
			continue;
		// find knn points 
		std::vector<int> knn;
		tree_ds->find_closest_points(pc.pnt(i), 30, &knn);
		std::set<int> incident_point_ids;
		for (auto k: knn) { // k is the index 
			incident_point_ids.insert(pc.face_id.at(k));
		}
		//
		if (incident_point_ids.size() == 1) { // interior points, keep the current selection (face id )
			F_conn_info f_conn_info;
			f_conn_info.point_id = i;
			pc.F_conn.push_back(f_conn_info);
		}
		if (incident_point_ids.size() == 2)  // boundary 
		{
			// fill the E_conn_info structure 
			E_conn_info e_conn_info;
			e_conn_info.point_id = i;
			e_conn_info.incident_ids = incident_point_ids;
			e_conn_info.valence = incident_point_ids.size();
			e_conn_info.visited = false;
			pc.E_conn.push_back(e_conn_info);
			// fill the hashmap structure for later use 
			pc.pid_to_E_conn_info_map[i] = e_conn_info;
			// used locally 
			will_be_marked_as_boundary.push_back(i);
		}
		if (incident_point_ids.size() >= 3)  // corner 
		{
			// fill the V_conn_info structure 
			V_conn_info v_conn_info;
			v_conn_info.point_id = i;
			v_conn_info.incident_ids = incident_point_ids;
			v_conn_info.valence = incident_point_ids.size();
			v_conn_info.visited = false;
			pc.V_conn.push_back(v_conn_info);
			// fill the hashmap structure for later use 
			pc.pid_to_V_conn_info_map[i] = v_conn_info;
			// used locally 
			will_be_marked_as_corner.push_back(i);
		}
		// after this, all points will be classified to V_conn/E_conn or F_conn
		// need further process to find out which point belones to which individual vertex/edge/face
	}
	for (auto b : will_be_marked_as_boundary) {
		// trace the marking: skipped
		// perform 
		pc.topo_id.at(b) = point_cloud::TOPOAttribute::BOUNDARIES;
	}
	for (auto c : will_be_marked_as_corner) {
		// trace the marking: skipped
		// perform 
		pc.topo_id.at(c) = point_cloud::TOPOAttribute::CORNER;
	}
	on_point_cloud_change_callback(PCC_COLORS);
	render_with_topo_selctions_only = true; // not working for now ...
}

/// topology extraction: faces 
void point_cloud_interactable::face_extraction() {
	std::set<int> regions;
	for (auto& f : pc.F_conn) { // loop all, no matter which face 
		f.face_id = pc.face_id.at(f.point_id); // face id starts from 1 if marked  
		regions.insert(f.face_id);
	}
	pc.num_face_ids = regions.size();
}

///
template <class ADAPTER>
const typename ADAPTER::container_type& get_container(ADAPTER& a)
{
	struct hack : private ADAPTER {
		static typename ADAPTER::container_type& get(ADAPTER& a) {
			return a.*(&hack::c);
		}
	};

	return hack::get(a);
}

///
/// fine grained classification, region grow to find neighbor points, push them to the global list 
void point_cloud_interactable::corner_extraction() {
	//
	std::vector<int> knn;
	ensure_tree_ds();
	// classify points to each vertex, global info about the vertex we are working on
	int curr_corner_id = 0;
	//
	bool not_done = false;
	int seed_pnt_id;
	for (auto& vc : pc.V_conn) { // check if all points classified 
		if (vc.visited == false) {
			not_done = true;
			seed_pnt_id = vc.point_id; // found an unvisited point as seed
		}
	}
	while (not_done) {
		// goal: do rg to find all neighbour points
		std::queue<int> q; q.push(seed_pnt_id);
		while (!q.empty()) { 
			int cur_seed_pnt_id = q.front(); q.pop(); // fetch the front point
			// visit the current point 
			// match in pc.V_conn
			int curr_point_id = -1;
			std::set<int> curr_incident_ids;
			for (auto& vc : pc.V_conn) {
				if (vc.point_id == cur_seed_pnt_id) {
					vc.corner_id = curr_corner_id;
					vc.visited = true;
					curr_incident_ids = vc.incident_ids;
				}
			}
			// find neighbour points 
			tree_ds->find_closest_points(pc.pnt(cur_seed_pnt_id), 30, &knn); // find knn points 
			for (auto k : knn) { // loop over knn points
				bool will_be_pushed = true;
				for (auto& vc : pc.V_conn) {
					if (vc.point_id == k) {
						if (vc.visited)// if visited, ignore 
							will_be_pushed = false;
						if (vc.incident_ids != curr_incident_ids)// if not belones to the same corner 
							will_be_pushed = false;
						if (vc.valence < 3)
							will_be_pushed = false;
						auto& c = get_container(q);
						const auto it = std::find(c.cbegin(), c.cend(), k);
						const auto position = std::distance(c.cbegin(), it);
						if (position < q.size()) // if already exists 
							will_be_pushed = false;
						if(will_be_pushed)
							q.push(vc.point_id);
						break; // assume only one match 
					}
				}
			}
		}
		// state: an other region is done 
		curr_corner_id++;
		// goal: check again if all points are classified 
		not_done = false;
		for (auto& vc : pc.V_conn) { 
			if (vc.visited == false) {
				not_done = true;
				seed_pnt_id = vc.point_id; // found an unvisited point as seed
			}
		}
	}
	// state: all corners should be extracted, they are built from points 
	// goal: quick check, how many regions are here? / corners 
	std::set<int> regions;
	for (auto& vc : pc.V_conn) {
		regions.insert(vc.corner_id);
	}
	std::cout << "number of corners extracted: " << regions.size()
		<< " or, curr_corner_id = " << curr_corner_id << std::endl;
	pc.num_corner_ids = regions.size();
}

///
/// an other grow to find and push to edges E_conn
void point_cloud_interactable::edge_extraction() {
	//
	std::vector<int> knn;
	ensure_tree_ds();
	// classify points to each vertex, global info about the vertex we are working on
	int curr_edge_id = 0;
	//
	bool not_done = false;
	int seed_pnt_id;
	for (auto& ec : pc.E_conn) { // check if all points classified 
		if (ec.visited == false) {
			not_done = true;
			seed_pnt_id = ec.point_id; // found an unvisited point as seed
		}
	}
	while (not_done) {
		// goal: do rg to find all neighbour points
		std::queue<int> q; q.push(seed_pnt_id);
		while (!q.empty()) {
			int cur_seed_pnt_id = q.front(); q.pop(); // fetch the front point
			// visit the current point 
			// match in pc.V_conn
			int curr_point_id = -1;
			std::set<int> curr_incident_ids;
			for (auto& ec : pc.E_conn) {
				if (ec.point_id == cur_seed_pnt_id) {
					ec.edge_id = curr_edge_id;
					ec.visited = true;
					curr_incident_ids = ec.incident_ids;
				}
			}
			// find neighbour points 
			tree_ds->find_closest_points(pc.pnt(cur_seed_pnt_id), 30, &knn); // find knn points 
			for (auto k : knn) { // loop over knn points
				bool will_be_pushed = true;
				for (auto& ec : pc.E_conn) {
					if (ec.point_id == k) {
						if (ec.visited)// if visited, ignore 
							will_be_pushed = false;
						if (ec.incident_ids != curr_incident_ids)// if not belones to the same edge 
							will_be_pushed = false;
						auto& c = get_container(q);
						const auto it = std::find(c.cbegin(), c.cend(), k);
						const auto position = std::distance(c.cbegin(), it);
						if (position < q.size()) // if already exists 
							will_be_pushed = false;
						if (will_be_pushed)
							q.push(k);
						break; // assume only one match 
					}
				}
			}
		}
		// state: an other region is done 
		curr_edge_id++;
		// goal: check again if all points are classified 
		not_done = false;
		for (auto& ec : pc.E_conn) {
			if (ec.visited == false) {
				not_done = true;
				seed_pnt_id = ec.point_id; // found an unvisited point as seed
			}
		}
	}
	// state: all point-based edges should be extracted
	// goal: quick check, how many regions are here? / edges  
	std::set<int> regions;
	for (auto& ec : pc.E_conn) {
		regions.insert(ec.edge_id);
	}
	std::cout << "number of point-besed edges extracted: " << regions.size()  
		<< " or, curr_edge_id = " << curr_edge_id  << std::endl;
	pc.num_edge_ids = regions.size();
}
///
void point_cloud_interactable::extract_all() {
	face_extraction();
	corner_extraction();
	edge_extraction();
}
///
void point_cloud_interactable::fitting_render_control_points_test() {
	/*pc.control_points.push_back(pc.pnt(78673));
	pc.control_point_colors.push_back(rgb(1, 1, 0));
	pc.control_points.push_back(pc.pnt(17268));
	pc.control_point_colors.push_back(rgb(0, 1, 0));*/

	/*pc.face_id.at(78673) = 2u;
	pc.face_id.at(17268) = 2u;*/

	//pc.demo_surface.push_back();

	pc.control_points.clear();
	pc.control_point_colors.clear();
	pc.demo_surface.clear();

	pc.control_points.resize(16);
	pc.control_point_colors.resize(16);
	pc.demo_surface.resize(16);

	pc.control_points.at(0) = vec3(0, 0, 0);
	pc.control_points.at(1) = vec3(0, 1.01, 1);
	pc.control_points.at(2) = vec3(0, 1.08, 2);
	pc.control_points.at(3) = vec3(0, 0, 3);

	pc.control_points.at(4) = vec3(1, 1.2, 0);
	pc.control_points.at(5) = vec3(1, 1.3, 1);
	pc.control_points.at(6) = vec3(1, 1.2, 2);
	pc.control_points.at(7) = vec3(1, 1.1, 3);

	pc.control_points.at(8) = vec3(2, 1.4, 0);
	pc.control_points.at(9) = vec3(2, 1.6, 1);
	pc.control_points.at(10) = vec3(2, 1.1, 2);
	pc.control_points.at(11) = vec3(2, 1.2, 3);

	pc.control_points.at(12) = vec3(3, 0, 0);
	pc.control_points.at(13) = vec3(3, 1.3, 1);
	pc.control_points.at(14) = vec3(3, 1.4, 2);
	pc.control_points.at(15) = vec3(3, 0, 3);

	for (int i = 0; i < 16; i++)
		pc.control_point_colors.at(i) = rgb(0, 1, 0);

	for (int i = 0; i < 16; i++)
		pc.demo_surface.at(i) = i;
}
/// 
void point_cloud_interactable::selective_subsampling_cpu() {
	std::default_random_engine g;
	std::uniform_real_distribution<float> d(0, 1);
	// iterate all points 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		// if marked as TO_BE_SUBSAMPLED 
		if (point_cloud::TOPOAttribute::TO_BE_SUBSAMPLED == pc.topo_id[i]) {
			// if random condition fullfilled, the larger the radio, more points will be deleted
			if (d(g) < selective_subsampling_radio) {
				// mark it as deleted 
				pc.topo_id[i] = point_cloud::TOPOAttribute::DEL;
			}
		}
	}
}
/*point addition, requiures the scan index present */
void point_cloud_interactable::spawn_points_in_the_handhold_quad(quat controllerq, vec3 controllert, vec3 quadextent) {

	if (!pc.has_normals() || !pc.has_scan_index) {
		std::cout << "does not have normal or scan index! quit" << std::endl;
	}

	std::default_random_engine g;
	std::uniform_real_distribution<float> d_x(-quadextent.x() * 0.5, quadextent.x() * 0.5);
	std::uniform_real_distribution<float> d_y(-quadextent.y() * 0.5, quadextent.y() * 0.5);
	std::uniform_real_distribution<float> d_z(-quadextent.z() * 0.5, quadextent.z() * 0.5);
	//point_cloud tmppc;
	// todo: regular/ random switch 
	for (int i = 0; i < num_of_points_to_be_added; i++) {
		/*spawn a new point: randomly inside the given range*/ 
		Pnt newP = Pnt(d_x(g), d_y(g), 0);
		// local to global
		controllerq.rotate(newP);
		newP = newP + controllert;

		/*new color attached*/
		rgba newC = rgba(0.6, 0.6, 0.6, 1);

		/*new normal*/
		Nml newN = Nml(0,0,1);
		controllerq.rotate(newN);

		/*scan index  */
		float newScanIndex = 0;

		/*selection index*/
		int newSelectionIndex = point_cloud::TOPOAttribute::NEWLY_GENERATED;

		// add to pc 
		pc.add_point(newP, newC, newN, newScanIndex, newSelectionIndex);
	}

	// require properties 
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_face_selection = true;

	// rebuild ann tree, managing rendering range..., crutial task when point cloud changes
	pc.box_out_of_date = true;
	on_point_cloud_change_callback(PCC_POINTS_RESIZE);
}
///
void point_cloud_interactable::auto_scale_after_read_points() {
	if (pc.get_nr_points() == 0)
		return;
	model_translation.identity();
	inv_model_translation.identity();
	float ext = (pc.box().get_max_pnt() - pc.box().get_min_pnt()).length();
	vec3 target_center = pc.box().get_center();
	model_scale = 1.0f / ext;
	mat4 m = cgv::math::translate4(-target_center);
	mat4 table_offset = cgv::math::translate4(vec3(0, 2, 0));
	float buttom_gap = target_center.y() - pc.box().get_min_pnt().y() + 0.1f; // 0.1 is table dick 
	mat4 buttom_offset = cgv::math::translate4(vec3(0, buttom_gap, 0));
	
	// adjust point size with bbox and num of points? record point size, save to file! 
	surfel_style.point_size = pc.suggested_point_size > 0 ? pc.suggested_point_size : 0.2f; 
		//pow(pc.box().get_extent().length(),20) / pc.get_nr_points();

	// perform actual transforms 
	pc.transform(m); // move to 0,0,0
	scale_model();
	vec3 new_center_offset = pc.box().get_center();
	mat4 new_center_offmat = cgv::math::translate4(-new_center_offset);
	model_translation = table_offset * buttom_offset * new_center_offmat; // record current model traslation and inverse 
	pc.transform(model_translation); // move to desired position 
	inv_model_translation = inv(model_translation);

}
/*region growing after marked*/
/// prepare/ reset region grow 
void point_cloud_interactable::prepare_grow(bool overwrite_face_selection) {
	// atomic operation 
	can_parallel_edit = false;

	// reconstruct face id if not present for proper growing  
	if (!pc.has_face_selections() || overwrite_face_selection) {
		pc.face_id.resize(pc.get_nr_points());
		for (auto& v : pc.face_id) v = 0; // 0 means unmarked 
		//pc.topo_id.resize(pc.get_nr_points());
		//for (auto& v : pc.topo_id) v = 0; // 0 means unmarked 
	}

	// reset visiting infos 
	pc.point_visited.resize(pc.get_nr_points());
	for (auto& v : pc.point_visited) v = false;
	pc.point_in_queue.resize(pc.get_nr_points());
	for (auto& v : pc.point_in_queue) v = false;
	pc.point_in_queue_which_group.resize(pc.get_nr_points());
	for (auto& v : pc.point_in_queue_which_group) v = 0; // group is face id, 1-25 
	num_of_knn_used_for_each_group.resize(pc.num_of_face_selections_rendered);
	for (auto& n : num_of_knn_used_for_each_group) { n = k;} // init to 30 at the beginning 
	num_of_points_curr_region.resize(pc.num_of_face_selections_rendered);
	for (auto& n : num_of_points_curr_region) { n = 0; }

	// init queue and seed tracking 
	queue_for_regions.clear();
	queue_for_regions.resize(pc.num_of_face_selections_rendered, std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>());
	suspend_queue_for_regions.clear();
	suspend_queue_for_regions.resize(pc.num_of_face_selections_rendered, std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>());
	backward_queue_for_regions.clear();
	backward_queue_for_regions.resize(pc.num_of_face_selections_rendered, std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>());
	seed_for_regions.clear();
	seed_for_regions.resize(pc.num_of_face_selections_rendered);
	for (auto& sid : seed_for_regions) { sid = -1; }

	// init number of grown points to 0 
	points_grown = 0;

	//
	growing_history_for_region.resize(pc.num_of_face_selections_rendered);
	for (auto& e : growing_history_for_region) { e.clear(); }

	// 
	is_residual_grow = false;

	// enable again  
	can_parallel_edit = true;
}
///
void point_cloud_interactable::extract_neighbours() {
	std::cout << "extracting neighbours..." << std::endl;
	ensure_tree_ds();
	pc.nearest_neighbour_indices.resize(pc.get_nr_points());
	for (Idx i= 0; i < pc.get_nr_points(); i++) {
		pc.nearest_neighbour_indices.at(i).resize(k);
		tree_ds->find_closest_points(pc.pnt(i), k+1, &knn); // knn will be resized inside 
		for (Idx j = 0; j < k; ++j)
			pc.nearest_neighbour_indices.at(i).at(j) = knn.at(j);
	}
	std::cout << "done..." << std::endl;
}
///
void point_cloud_interactable::rerender_seeds() {
	//// reset face ids 
	//for (int i = 0; i < pc.get_nr_points(); i++) 
	//	pc.face_id.at(i) = 0;
	//for (int curr_face_selecting_id = 0; curr_face_selecting_id < seed_for_regions.size(); curr_face_selecting_id++) {
	//	int pid = seed_for_regions[curr_face_selecting_id];
	//	if (pid != -1) {
	//		pc.face_id.at(pid) = curr_face_selecting_id;
	//	}
	//}
}
///
void point_cloud_interactable::clear_previous_queue(int prev_pid, int which_group) {
	pc.face_id.at(prev_pid) = 0; // reset face selection 
	pc.point_in_queue[prev_pid] = false; // reset
	pc.point_in_queue_which_group[prev_pid] = 0;
	pc.point_visited[prev_pid] = false; // not visited
	queue_for_regions[which_group].pop();
}
/// do not modify others. will be expend. interactive region growing  
void point_cloud_interactable::add_seed_to_queue(int which_group) {
	int pid = seed_for_regions[which_group]; // Wrote before 
	if (pid != -1) {
		pc.face_id.at(pid) = which_group; // visual feedback 
		pc.point_in_queue[pid] = true; // mark 
		pc.point_in_queue_which_group[pid] = which_group;
		pc.point_visited[pid] = false; // not visited for now
		queue_for_regions[which_group].push(std::make_tuple(pid, 0, 0));
	}
}
/// reset_queue_with_seeds
void point_cloud_interactable::reset_queue_with_seeds() {
	// delete all marked and redo the marking, iterate all points, less effecient for now 
	// reset face id 
	pc.face_id.resize(pc.get_nr_points());
	for (auto& v : pc.face_id) v = 0; // 0 means unmarked s
	//pc.topo_id.resize(pc.get_nr_points()); // not used for growing 
	//for (auto& v : pc.topo_id) v = 0; // 0 means unmarked // not used for growing 

	// reset visiting infos 
	pc.point_visited.resize(pc.get_nr_points());
	for (auto& v : pc.point_visited) v = false;
	pc.point_in_queue.resize(pc.get_nr_points());
	for (auto& v : pc.point_in_queue) v = false;
	pc.point_in_queue_which_group.resize(pc.get_nr_points());
	for (auto& v : pc.point_in_queue_which_group) v = 0; // group is face id, 1-25 
	num_of_knn_used_for_each_group.resize(pc.num_of_face_selections_rendered);
	for (auto& n : num_of_knn_used_for_each_group) { n = k; } // init to 30 at the beginning 
	num_of_points_curr_region.resize(pc.num_of_face_selections_rendered);
	for (auto& n : num_of_points_curr_region) { n = 0; }


	// recover queue from seeds 
	queue_for_regions.clear();
	queue_for_regions.resize(seed_for_regions.size(), std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>());
	for (int curr_face_selecting_id = 0; curr_face_selecting_id < seed_for_regions.size(); curr_face_selecting_id++) {
		int pid = seed_for_regions[curr_face_selecting_id];
		if (pid != -1) {
			pc.face_id.at(pid) = curr_face_selecting_id;
			pc.point_visited[pid] = false; 
			pc.point_in_queue[pid] = true;		
			pc.point_in_queue_which_group[pid] = curr_face_selecting_id;
			queue_for_regions[curr_face_selecting_id].push(std::make_tuple(pid, 0, 0));
		}
	}
	suspend_queue_for_regions.clear();
	suspend_queue_for_regions.resize(pc.num_of_face_selections_rendered, std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>());

	// reset number of grown points to 0 
	points_grown = 0;
}
/// reset the seed lists 
void point_cloud_interactable::reset_region_growing_seeds() {
	/*queue_for_regions.clear();
	queue_for_regions.resize(pc.num_of_face_selections_rendered, *(new non_redundant_priority_queue));
	seed_for_regions.clear();
	seed_for_regions.resize(pc.num_of_face_selections_rendered);*/
}
/// size of the vector(int64_t), content
void point_cloud_interactable::record_seed_for_regions(std::string fn) {
	std::cout << "writting file: " << fn << std::endl;
	FILE* fp = fopen(fn.c_str(), "wb");
	uint64_t n = (uint64_t)seed_for_regions.size(); 
	bool success;

	//
	success = fwrite(&n, sizeof(uint64_t), 1, fp) == 1; // write header, number of points 
	//
	success = fwrite(&seed_for_regions[0], sizeof(int), n, fp) == n;
	
	fclose(fp);
	if(success)
		std::cout << "saved!" << std::endl;
}
/// push to queue operation, as an init to RG 
void point_cloud_interactable::init_region_growing_by_collecting_group_and_seeds_vr(int curr_face_selecting_id) {
	// collect all related idx and push to queue, ready to grow after this 
	// only region selection are accepted
	// unsigned int shoud cast to int! -> only when comparing bet. them 
	for (int pid = 0; pid < pc.get_nr_points(); pid++) {
		if (pc.face_id.at(pid) == curr_face_selecting_id) {
			// these two can be recorded and recovered 
			queue_for_regions[curr_face_selecting_id].push(std::make_tuple(pid, 0, 0)); // seeds will be added to queue directly, so have a priority of 0
			seed_for_regions[curr_face_selecting_id] = pid; // record to variable 
			pc.point_visited[pid] = true; // mark seeds as visited 
			pc.point_in_queue[pid] = true;
			pc.point_in_queue_which_group[pid] = curr_face_selecting_id; 
		}
	}
}
/// init growing group
void point_cloud_interactable::init_region_growing_by_setting_group_and_seeds(int growing_group, std::queue<int> picked_id_list) {
	//region_id_and_seeds[growing_group] = picked_id_list;
	////region_id_and_seeds.insert(std::make_pair(growing_group, picked_id_list));
	//// use the first seed's normal for test 
	//region_id_and_nml[growing_group] = pc.nml(picked_id_list.front());
	//pc.has_face_selection = true;
}
/// perform region grow in timer event 
void point_cloud_interactable::do_region_growing_timer_event(double t, double dt) {
	//if (!can_sleep && do_region_growing_directly) {
	//	int i = 0;
	//	bool can_not_sleep = false;
	//	while (i < steps_per_event_as_speed) {
	//		// grow marked regions, not functional ones 
	//		for (int gi = pc.num_of_topo_selections_rendered; gi < pc.num_of_palette_spheres_rendered; gi++)
	//			can_not_sleep = can_not_sleep || grow_one_step_bfs(true, gi);
	//		//post_redraw();
	//		// check if can sleep: all grow_xxx return false, todo 
	//		//if (!can_not_sleep)
	//		//	can_sleep = true;
	//		i++;
	//	}
	//}
	//on_point_cloud_change_callback(PCC_COLORS);
}
/// parallel version 
void point_cloud_interactable::grow_one_region(int gi) {
	//const int points_one_chunk = 100;
	//int points_grown = 0;
	//// if not all point are grown 
	//while ((points_grown < pc.get_nr_points()) && !pause_growing) { // thread will exit if pausing 
	//	//
	//	grow_one_step_bfs(false, gi);

	//	// sleep for a while 
	//	points_grown++;

	//	// 
	//	if (points_grown > points_one_chunk) {
	//		// sleep for a while 
	//		if (growing_latency != 0)
	//			std::this_thread::sleep_for(std::chrono::milliseconds(growing_latency));
	//		points_grown = 0; // reset 
	//	}
	//}
}
/*interactive region growing */
///
void point_cloud_interactable::clear_seed_for_regions() {
	// reset seed array 
	for (auto& s : seed_for_regions)
		s = -1;

	// reset queue 
	for (int curr_region = 0; curr_region < pc.num_of_face_selections_rendered; curr_region++) {
		if (!queue_for_regions[curr_region].empty()) {
			point_priority_mapping curr_top = queue_for_regions[curr_region].top();
			pc.face_id.at(std::get<ID>(curr_top)) = 0; // reset color 
			pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
			pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = 0;
			pc.point_visited.at(std::get<ID>(curr_top)) = false;
			queue_for_regions[curr_region].pop();
		}
	}
}
/// load only, not modify directly 
void point_cloud_interactable::load_seed_for_regions(std::string fn) {
	FILE* fp = fopen(fn.c_str(), "rb");
	loaded_seeds_for_regions.clear();
	uint64_t n;
	bool success;

	//
	success = fread(&n, sizeof(uint64_t), 1, fp) == 1; // write header, number of points 
	//
	loaded_seeds_for_regions.resize(n); success = fread(&loaded_seeds_for_regions[0], sizeof(int), n, fp) == n;

	fclose(fp);
	if (success)
		std::cout << "seeds read successfully!" << std::endl;

	// recover to queue_for_regions as below 
	if (loaded_seeds_for_regions.size() != pc.num_of_face_selections_rendered) {
		std::cout << "number of regions doesn't match, return!" << std::endl;
		return;
	}
}
/// recover to seed for regions and queue for regions 
void point_cloud_interactable::recover_seed_for_regions(std::string fn) {
	FILE* fp = fopen(fn.c_str(), "rb");
	seed_for_regions.clear();
	uint64_t n;
	bool success;

	//
	success = fread(&n, sizeof(uint64_t), 1, fp) == 1; // write header, number of points 
	//
	seed_for_regions.resize(n); success = fread(&seed_for_regions[0], sizeof(int), n, fp) == n;

	fclose(fp);
	if (success)
		std::cout << "seeds read successfully!" << std::endl;

	// recover to queue_for_regions as below 
	if (seed_for_regions.size() != pc.num_of_face_selections_rendered) {
		std::cout << "number of regions doesn't match, return!" << std::endl;
		return;
	}

	// state: seed_for_regions updated 
	reset_queue_with_seeds();
}
/// 13-05-2021 find next seed if possible 
int point_cloud_interactable::find_next_seed_in_low_curvature_area() {
	for (int i = 0; i < pc.get_nr_points(); i++) {
		bool accept = true;
		if (pc.curvature.at(i).mean_curvature > pc.curvinfo.coloring_threshold)
			accept = false;
		if (pc.point_visited.at(i) == true)
			accept = false;
		if (pc.point_in_queue.at(i) == true)
			accept = false;
		if (pc.face_id.at(i) != 0)
			accept = false;
		if (accept)
			return i;
	}
	return -1;
}
/// current face_id will be fixed. queue for current group will be empty, will not grow any more.
/// do not ahve to do this explicitly if auto. stops 
void point_cloud_interactable::submit_face() {
	point_priority_mapping curr_top;
	for (int curr_group = 1; curr_group < pc.num_of_face_selections_rendered; curr_group++) {
		while (!queue_for_regions[curr_group].empty()) {
			curr_top = queue_for_regions[curr_group].top();
			pc.face_id.at(std::get<ID>(curr_top)) = 17; // set color for high curvature regions 
			suspend_queue_for_regions[curr_group].push(curr_top); // save to an other queue
			queue_for_regions[curr_group].pop();
		}
	}
}
/// not used 
void point_cloud_interactable::clear_queue_and_restore_attributes() {
	point_priority_mapping curr_top;
	for (int curr_group = 1; curr_group < pc.num_of_face_selections_rendered; curr_group++) {
		while (!queue_for_regions[curr_group].empty()) {
			curr_top = queue_for_regions[curr_group].top();
			pc.face_id.at(std::get<ID>(curr_top)) = 0;
			pc.point_visited.at(std::get<ID>(curr_top)) = false;
			pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
			pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = 0;
			queue_for_regions[curr_group].pop();
		}
	}
}
///
void point_cloud_interactable::clear_curr_queue_and_restore_attributes(int curr_group) {
	point_priority_mapping curr_top;
	while (!queue_for_regions[curr_group].empty()) {
		curr_top = queue_for_regions[curr_group].top();
		pc.face_id.at(std::get<ID>(curr_top)) = 0;
		pc.point_visited.at(std::get<ID>(curr_top)) = false;
		pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
		pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = 0;
		queue_for_regions[curr_group].pop();
	}
}
/// all suspend_queue_for_regions empty, everything pushed to queue, can grow an other time 
/// serves for automatic methods: modify the state of the queue
void point_cloud_interactable::resume_queue() {
	std::vector<std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>> tmpq(suspend_queue_for_regions); 
	point_priority_mapping curr_top;
	for (int curr_group = 1; curr_group < pc.num_of_face_selections_rendered; curr_group++) {
		while (!tmpq[curr_group].empty()) {
			curr_top = tmpq[curr_group].top();
			pc.face_id.at(std::get<ID>(curr_top)) = 16; // visual feedback: just an other color 
			queue_for_regions[curr_group].push(curr_top); 
			tmpq[curr_group].pop();
		}
		// state: suspend_queue_for_regions[curr_group] is empty 
	}
}
///backup face_id, point_visited, point_in_queue, point_in_queue_group, 
void point_cloud_interactable::record_current_state_before_sync_grow() {
	bkp_face_id.resize(pc.get_nr_points());
	bkp_point_visited.resize(pc.get_nr_points());
	bkp_point_in_queue.resize(pc.get_nr_points());
	bkp_point_in_queue_which_group.resize(pc.get_nr_points());
	for (int i = 0; i < pc.get_nr_points(); i++) { bkp_face_id[i] = pc.face_id[i]; }
	for (int i = 0; i < pc.get_nr_points(); i++) { bkp_point_visited[i] = pc.point_visited[i]; }
	for (int i = 0; i < pc.get_nr_points(); i++) { bkp_point_in_queue[i] = pc.point_in_queue[i]; }
	for (int i = 0; i < pc.get_nr_points(); i++) { bkp_point_in_queue_which_group[i] = pc.point_in_queue_which_group[i];}

	bkp_points_grown = points_grown;
}
///
void point_cloud_interactable::undo_sync_grow() {
	/*if (!can_parallel_edit) {
		std::cout << "undo_sync_grow: growing in progress! try later!" << std::endl;
		return;
	}*/
	for (int i = 0; i < pc.get_nr_points(); i++) { pc.face_id[i]= bkp_face_id[i]; }
	for (int i = 0; i < pc.get_nr_points(); i++) { pc.point_visited[i]= bkp_point_visited[i]; }
	for (int i = 0; i < pc.get_nr_points(); i++) { pc.point_in_queue[i]= bkp_point_in_queue[i]; }
	for (int i = 0; i < pc.get_nr_points(); i++) { pc.point_in_queue_which_group[i]= bkp_point_in_queue_which_group[i]; }
	
	points_grown = bkp_points_grown;

	resume_queue(); // restore queue from suspend_queue_for_regions as a trick, suspend_queue_for_regions stored implicitly 
}
///
void point_cloud_interactable::undo_curr_region(int curr_region) {
	// face_id, in_queue,point_in_queue_which_group , point_visited
	// clear current queue and final queue for the selected region  

	// safty check 
	if (curr_region < 1 || curr_region>queue_for_regions.size()) {
		std::cout << "error curr_region!" << std::endl;
		return;
	}

	// recover points in queue if present
	while (!queue_for_regions[curr_region].empty()) {
		point_priority_mapping curr_top = queue_for_regions[curr_region].top();
		queue_for_regions[curr_region].pop();
		pc.face_id.at(std::get<ID>(curr_top)) = 0; // reset color 
		pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
		pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = 0;
		pc.point_visited.at(std::get<ID>(curr_top)) = false;
	}

	// recover point in final queue if present
	while (!suspend_queue_for_regions[curr_region].empty()) { 
		point_priority_mapping curr_top = suspend_queue_for_regions[curr_region].top();
		pc.face_id.at(std::get<ID>(curr_top)) = 0; // reset color 
		pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
		pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = 0;
		pc.point_visited.at(std::get<ID>(curr_top)) = false;
		suspend_queue_for_regions[curr_region].pop(); 
	}

	// loop over to reset point face id, just one time  
	for (int i = 0; i < pc.get_nr_points(); i++) {
		if (pc.face_id.at(i) == curr_region) {
			pc.face_id.at(i) = 0; // reset color 
			pc.point_in_queue.at(i) = false; // should already been false 
			pc.point_in_queue_which_group.at(i) = 0; // just make it safe 
			pc.point_visited.at(i) = false; 
		}
	}

	// re-add seed to queue 
	add_seed_to_queue(curr_region);
}
/// apply model scalling 
void point_cloud_interactable::scale_model() {
	pc.transform(inv_model_translation);
	std::cout << "model has been scaled with factor of: " << model_scale << std::endl;
	float tmp = model_scale;
	model_scale *= 1.0f / last_model_scale; // recover last scale first 
	mat3 scale_matrix = cgv::math::scale3(vec3(model_scale, model_scale, model_scale));
	pc.transform(scale_matrix);
	last_model_scale = tmp;
	pc.transform(model_translation);
}
/*
	algorithm: 
		while not all faces are grown:
			a. select one seed per-region with controller
			b. grow current region adaptive to curvature and accumulated distance  
			c. adjust growing speed with controller (adjust the sleeping time)
				a direct visual feedback is provided
				users can also pause/ continue the growing
			d. slower if the current region is almost extracted
			e. if the boundary is acceptable
				stop and goto a. for the next region 
			f. else, delete and recover all growing parameters, goto b. 
		g. perform a final grow to fill the gaps between different regions.

	v2: 
		interactive point cloud segmentation:
			a. curvature computation and clustering
			b. observe the curvature of the points, adjust the threshold.  
			c. coarse level segmentation
				dequeue to prevent from leaks
					while not all faces are grown:
						d. select one seed per region
						e. start to grow 
						f. control the growing
							iterate all points found within the range
								if a point is marked as "in queue":
									save points to a suspend_queue 
									dequeue current point 
			j. restore queue (push points from suspend_queue back to queue)
			k. final grow to fill the gap.

*/
void point_cloud_interactable::step_back_one_point() {

}
/// modify: 5 attributes + 1 queue
void point_cloud_interactable::backward_grow_one_step(int curr_region) {
	if (!queue_for_regions[curr_region].empty()) {
		clear_curr_queue_and_restore_attributes(curr_region);
	}

	growing_history_element e = growing_history_for_region[curr_region][num_of_points_curr_region[curr_region] -1]; // size of growing_history should be the same as points_grown
	
	// recover last point 
	pc.point_in_queue[e.curr_point_id] = false;
	pc.point_visited[e.curr_point_id] = false;
	pc.face_id[e.curr_point_id] = 0;
	pc.point_in_queue_which_group[e.curr_point_id] = 0;

	points_grown--;
	num_of_points_curr_region[curr_region]--;
}
/// modify: 5 attributes + 1 queue 
/// what about grow again? back and grow will cause complexity 
void point_cloud_interactable::backward_grow_current_region(int curr_region) {
	// step back one point: 
		// curr queue -> origin (queue modified )
		// curr point -> origin 
		// curr point -> queue 
	// but have to save enough information 
		// std::vector<growing_history_element> growing_history;
		// 
	const int points_one_chunk = 100;
	int num_points = 0;
	while (!pause_growing) {
		//
		if (num_of_points_curr_region[curr_region] == 0)
			return;

		//
		backward_grow_one_step(curr_region);
		
		// for sleeping 
		num_points++;

		//
		if (num_points > points_one_chunk) {
			// sleep for a while 
			if (growing_latency != 0)
				std::this_thread::sleep_for(std::chrono::milliseconds(growing_latency));
			num_points = 0; // reset 
		}
	}
}
/// curr_region is ignored when growing in sync mode 
void point_cloud_interactable::grow_curr_region(int curr_region) {
	// update distances 
	max_accu_dist = 0;
	max_dist = (pc.box().get_max_pnt() - pc.box().get_min_pnt()).length();
	std::cout << "parallel region growing: starts" << std::endl;
	can_parallel_edit = false; // atomic: stop seleciton when parallel growing 

	// if residual grow, be careful, otherwise, restore default number 
	if (is_residual_grow|| is_synchronous_growth) {
		num_of_knn_used_for_each_group.resize(pc.num_of_face_selections_rendered);
		for (auto& n : num_of_knn_used_for_each_group) { n = minimum_searching_neighbor_points; } // reset knn searching radius in final step 
	}
	else {
		// larger minimum_searching_neighbor_points incase some points in one region can not be searched for sparse pc 
		num_of_knn_used_for_each_group.resize(pc.num_of_face_selections_rendered);
		for (auto& n : num_of_knn_used_for_each_group) { n = k; }
	}

	const int points_one_chunk = 1000;
	int num_points = 0;
	int last_points_grown = 0;
	int no_progress_iters_for_sleeping = 0;
	// if not all point are grown 
	while (!pause_growing) { // thread will exit if pausing, but queue stays the unchanged 
		
		//
		if(!is_synchronous_growth)
			grow_one_step_bfs(false, curr_region); // not final growing 
		else {
			for (int gi = 1; gi < pc.num_of_face_selections_rendered; gi++)
				grow_one_step_bfs(false, gi); // boolean not used 
		}

		// for sleeping 
		num_points++;

		// slower growing speed and undo support for residual growing 
		if (!is_residual_grow && !is_synchronous_growth) {
			if (num_points > points_one_chunk) {
				// sleep for a while 
				if (growing_latency != 0)
					std::this_thread::sleep_for(std::chrono::milliseconds(growing_latency));
				num_points = 0; // reset 
			}
		}
		else {
			// pointwise
			std::this_thread::sleep_for(std::chrono::milliseconds(growing_latency));
		}

		if (!is_synchronous_growth) {
			//
			if (queue_for_regions[curr_region].empty()) {
				std::cout << "current queue empty, quit" << std::endl;
				return;
			}
		}
		else {
			// stopping criteria for sync grow
			if (points_grown == pc.get_nr_points()) {
				std::cout << "all points are grown, quit" << std::endl;
				return;
			}
		}

		if (points_grown == last_points_grown) {
			no_progress_iters_for_sleeping++;
		}

		// terminate the thread if havent progress for too many times
		if (no_progress_iters_for_sleeping > 10) {
			std::wcout << "havent progress for too many times, quit" << std::endl;
			break;
		}

		last_points_grown = points_grown;
	}

	if (is_synchronous_growth)
		is_synchronous_growth = false;
	if (is_residual_grow)
		is_residual_grow = false;

	std::cout << "parallel region growing: done." << std::endl;
	can_parallel_edit = true;
}
/// just grow every region at the same time 
void point_cloud_interactable::sync_grow() {
}
/// entry function, split to two functions above, not used 
void point_cloud_interactable::region_growing() {
	std::cout << "deprecated!..." << std::endl;
}
///
void point_cloud_interactable::show_num_of_points_per_region() {
	for (int i = 0; i < pc.num_of_face_selections_rendered; i++) {
		std::cout << "num_of_points_curr_region: " << i << " = " << num_of_points_curr_region[i] << std::endl;
	}
}
/*
	algorithm:
		if the priority queue is not empty:  
			a. pop the priority queue and visit current point 
			b. restore point attribute after dequeue 
				point_in_queue is a flag indicates if current point in queue used for redundancy checking
			check all neighbour points:
				c. decrease searching radius when high curvature points found 
				d. compute a property eg. accumulated distance 
				e. push to queue and set flags 

	in this work, the computed property is a "scaled accumulated distance", here is the impl:
		property = distance_scale * distance_between_current_point_and_neighbor_point + accumulated_distance_of_current_point;
		where dist_scale is 1.0f + a_very_large_factor * pc.curvature.at(kid).mean_curvature; 
		a_very_large_factor can be computed as pc.get_nr_points() * max_dist;
		and max_dist is (pc.box().get_max_pnt() - pc.box().get_min_pnt()).length();

*/
/// one step growing with bfs, grow_with_queue not used 
/// only one point is grown in this step 
bool point_cloud_interactable::grow_one_step_bfs(bool grow_with_queue, int which_group) {
	// bfs, simple approach, do not update normal currently 
	if (pc.get_nr_points() == 0)
		return false;
	if (!grow_with_queue) {
		if (queue_for_regions.size() == 0)
			return false;
		if (queue_for_regions[which_group].empty())
			return false;
	}
	else {
		if (suspend_queue_for_regions.size() == 0)
			return false;
		if (suspend_queue_for_regions[which_group].empty())
			return false;
	}

	point_priority_mapping to_visit;

	// find current to_visit 
	if (!grow_with_queue) {	
		if (check_the_queue_and_stop) {
			/*if (std::get<CURVATURE>(to_visit) > pc.curvinfo.coloring_threshold) { // do not do this
				return false;
			}*/

			// check the queue 
			bool every_point_in_queue_has_high_curvature = true;
			for (int i = 0; i < pc.get_nr_points(); i++) {
				// in queue and belongs to current group 
				if (pc.point_in_queue.at(i) == true && pc.point_in_queue_which_group.at(i) == which_group) {
					if (pc.curvature.at(i).mean_curvature < pc.curvinfo.coloring_threshold) {
						every_point_in_queue_has_high_curvature = false;
					}
				}
			}
			if (every_point_in_queue_has_high_curvature) // stop if every_point_in_queue_has_high_curvature
				return false;
		}

		// success stop: 1. ignore high curvature regions, 2. approperate searching radius 3. correct boundary touch (distance)
		// minimum_searching_radius
		// ignore high curvature regions and decrease searching radius, to be careful later on 
		to_visit = queue_for_regions[which_group].top(); // this will query a pid with minimal second value 
		if (ignore_high_curvature_regions) {
			while (std::get<CURVATURE>(to_visit) > pc.curvinfo.coloring_threshold) { // just ignore, do not visit  
				pc.face_id.at(std::get<ID>(to_visit)) = 19; // visual mark 
				suspend_queue_for_regions[which_group].push(to_visit); // save to an other queue, for final growing process 
				// mark as in queue 
				pc.point_in_queue.at(std::get<ID>(to_visit)) = true;
				pc.point_in_queue_which_group.at(std::get<ID>(to_visit)) = which_group;
				// pop 
				queue_for_regions[which_group].pop();
				// check and update 
				if (queue_for_regions[which_group].empty()) // check 
					return false;
				to_visit = queue_for_regions[which_group].top(); // query top of the queue as new candidate 
			}
		}
		// ignore points that are marked as not in queue 
		while (!pc.point_in_queue.at(std::get<ID>(to_visit))) {
			queue_for_regions[which_group].pop();
			if (queue_for_regions[which_group].empty()) // check 
				return false;
			to_visit = queue_for_regions[which_group].top();
		}
		// state: lower curvature found 
		queue_for_regions[which_group].pop();
	}
	else {
		to_visit = suspend_queue_for_regions[which_group].top();
		suspend_queue_for_regions[which_group].pop();
	}

	// restore attribute after dequeue
	pc.point_in_queue.at(std::get<ID>(to_visit)) = false;
	pc.point_in_queue_which_group.at(std::get<ID>(to_visit)) = 0;

	// visit if not visited: may already visited by an other group, we added to queue before, not queried by us 
	if (pc.point_visited.at(std::get<ID>(to_visit)))
		return false;

	// visit current 
	pc.face_id.at(std::get<ID>(to_visit)) = which_group;
	pc.point_visited.at(std::get<ID>(to_visit)) = true;
	points_grown++;
	num_of_points_curr_region[which_group]++;
	std::vector<int> which_neighbors_were_pushed;

	// expend queue by visiting children of the to_visit (knn)
	// not everything in queue in the final growing step 
	// query knn and add to queue conditionally  
	int num_of_children_expanded = 0;
	for (auto kid: pc.nearest_neighbour_indices.at(std::get<ID>(to_visit))) {
		num_of_children_expanded++;

		// with variable searching radius 
		if (decrease_searching_radius_on_high_curvature) {
			if (num_of_children_expanded > num_of_knn_used_for_each_group[which_group])
				continue;
		}
		else {
			if (num_of_children_expanded > k) // oin case k may adjusted to be smaller than 30
				continue;
		}

		// init varibles 
		bool add_to_queue = true;
		float property_scale = 1;

		// ignore visited 
		if (pc.point_visited.at(kid))
			continue;

		// ignore deleted, topo_id and face_id should work together, p, todo 
		if (pc.topo_id.at(kid) == point_cloud::TOPOAttribute::DEL && !pc.point_visited.at(kid)) {
			pc.point_visited.at(kid) = true;
			points_grown++;
		}
		if (pc.topo_id.at(kid) == point_cloud::TOPOAttribute::DEL && pc.point_visited.at(kid)) {
			continue;
		}

		//
		if (region_grow_check_normals) {
			vec3 parent_nml = normalize(pc.nml(std::get<ID>(to_visit)));
			vec3 child_nml = normalize(pc.nml(kid));
			float curr_normal_diff = dot(parent_nml, child_nml);
			if (curr_normal_diff < normal_threshold)
				continue;
		}

		// drop redundant points in queue, must belongs to current group 
		// points may be shared by diff. group queue? 
		if (pc.point_in_queue.at(kid) == true && pc.point_in_queue_which_group.at(kid) == which_group)
			if(use_property_scale)
				property_scale = 0.5f;
			else
				continue; // ok-todo: decrease property instead of just ignore 

		// basic computations for property computing 
		float dist = (pc.pnt(kid) - pc.pnt(std::get<ID>(to_visit))).length(); // smallest distance to S set first
		float accu_dist = dist + std::get<DIST>(to_visit);
		if (seed_for_regions[which_group] == -1) break;
		float dist_to_seed = (pc.pnt(kid) - pc.pnt(seed_for_regions[which_group])).length();

		//
		float curr_property = 0;
		float curr_curvature = pc.curvature.at(kid).mean_curvature;

		// pure distance based 
		if (gm == growing_mode::SEED_DISTANCE_BASED) {
			curr_property = dist_to_seed;
		}

		// accumulated distance based 
		if (gm == growing_mode::ACCU_DISTANCE_BASED) {
			curr_property = accu_dist;
		}

		// pure curvature based 
		if (gm == growing_mode::UNSIGNED_MEAN_CURVATURE_BASED) { // does work 
			curr_property = -pc.curvature.at(kid).mean_curvature;
		}

		// stop with bounary condition
		if (gm == growing_mode::STOP_ON_BOUNDARY) {
			curr_property = pc.curvature.at(kid).mean_curvature;
		}

		// dequeue first in low distance and low curvature regions. Just like fill water to a pool.
		if (gm == growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST) {
			float maxinum_accu_dist = pc.get_nr_points() * max_dist;
			float revertable_scaling_factor; 
			if(pc.curvinfo.minimum_curvature_difference>0)
				revertable_scaling_factor = maxinum_accu_dist / pc.curvinfo.minimum_curvature_difference;
			else 
				revertable_scaling_factor = pc.get_nr_points() * pc.get_nr_points();
			float scale_dist_by_curva = 1.0f + revertable_scaling_factor * pc.curvature.at(kid).mean_curvature; // ok-todo: find a upper bound 
			curr_property = scale_dist_by_curva * dist + std::get<DIST>(to_visit);
		}

		// dequeue first in low distance and high curvature regions.
		if (gm == growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_HIGHERFIRST) { // not working 
			float maxinum_accu_dist = pc.get_nr_points() * max_dist;
			float revertable_scaling_factor;
			if (pc.curvinfo.minimum_curvature_difference > 0)
				revertable_scaling_factor = maxinum_accu_dist / pc.curvinfo.minimum_curvature_difference;
			else
				revertable_scaling_factor = pc.get_nr_points() * pc.get_nr_points();
			// revertable_scaling_factor can also be scaled ... but do not have to do that 
			float scaled_mean_curvature = 
				(pc.curvature.at(kid).mean_curvature - pc.curvinfo.min_mean_curvature) 
					/ pc.curvinfo.max_mean_curvature;
			float scale_dist_by_curva = 1.0f - revertable_scaling_factor * pc.curvature.at(kid).mean_curvature;
			curr_property = scale_dist_by_curva * dist + std::get<DIST>(to_visit);
		}

		// lower normal dist will be dequeued first 
		if (gm == growing_mode::NORMAL_BASED) {
			float a_very_large_factor = pc.get_nr_points() * pc.get_nr_points();
			float scale_dist_by_nml_diff = 1.0f + a_very_large_factor * (1 - dot(pc.nml(kid), pc.nml(seed_for_regions[which_group]))) / 2.0f;
			curr_property = scale_dist_by_nml_diff * dist + std::get<DIST>(to_visit);
		}

		// queue visualization, diff color for diff groups 
		pc.face_id.at(kid) = 26 - which_group;

		//
		if (curr_curvature > pc.curvinfo.coloring_threshold && !grow_with_queue && decrease_searching_radius_on_high_curvature) {
			if(is_residual_grow)
				std::cout << "num_of_knn_used_for_each_group of " << which_group << " is: " 
					<< num_of_knn_used_for_each_group[which_group] << std::endl;
			if (num_of_knn_used_for_each_group[which_group] > minimum_searching_neighbor_points)
				num_of_knn_used_for_each_group[which_group]--; // lower searching redius, more careful 
			//growing_latency++;
			//std::cout << "growing_latency " << growing_latency << std::endl;
		}

		// push to queue. store addtional value, can be quired when dequeue 
		which_neighbors_were_pushed.push_back(kid);
		if (!grow_with_queue) 
			queue_for_regions[which_group].push(std::make_tuple(kid, property_scale * curr_property, curr_curvature));
		else 
			suspend_queue_for_regions[which_group].push(std::make_tuple(kid, property_scale * curr_property, curr_curvature));
		pc.point_in_queue.at(kid) = true;
		pc.point_in_queue_which_group.at(kid) = which_group;
	}
	growing_history_element e;
	e.curr_point_id = std::get<ID>(to_visit);
	e.curr_neighbors_pushed_to_queue = std::move(which_neighbors_were_pushed);
	growing_history_for_region[which_group].push_back(e);
	return true;
}
/// check if all points growed 
bool point_cloud_interactable::all_points_growed() {
	for (auto ps : pc.face_id) {
		if (ps == 0)
			return false;
	}
	return true;
}
/// ensure all points to be growed 
void point_cloud_interactable::grow_one_step_bfs_ensure() {
}
/// for now, we only set confirmed to true 
void point_cloud_interactable::mark_points_inside_selection_tool(Pnt p, float r, bool confirmed, int objctive)
{
}
/// only need to compute once after record the current frame
bool point_cloud_interactable::assign_pc_and_ensure_source_tree_ds(point_cloud& _pc) {
	if (_pc.get_nr_points()) {
		if (tree_ds_source_pc)
			delete tree_ds_source_pc;
		tree_ds_source_pc = new ann_tree;
		tree_ds_source_pc->build(_pc);

		pc_to_be_append = _pc;
		return true;
	}
	else {
		// err: called on empty cloud 
		return false;
	}
}
/// make sure that the point cloud has selection
void point_cloud_interactable::ensure_point_selection_pc() {
}
/// inner function 
void point_cloud_interactable::ensure_point_selection() {
}
/// subsampled_target will be computed acc to p, r information 
void point_cloud_interactable::subsampling_target(Pnt& p, float& r, bool confirmed) {
}
/// subsample the source point cloud 
void point_cloud_interactable::subsampling_source(Pnt& p, float& r, bool confirmed) {
}
/// 
void point_cloud_interactable::collect_to_subsampled_pcs() {
}
///
void point_cloud_interactable::scale_points_to_desk() {
	vec3 point_center = pc.box().get_center();
	float max_extent = 0;
	vec3 point_extent = pc.box().get_extent();
	max_extent = point_extent.x();
	if (point_extent.y() > max_extent)
		max_extent = point_extent.y();
	if (point_extent.z() > max_extent)
		max_extent = point_extent.z();
	float factor = 1.0f / max_extent;
	pc.translate(-point_center);
	mat3 scale_matrix; 
	scale_matrix.identity();
	scale_matrix(0, 0) = factor;
	scale_matrix(1, 1) = factor;
	scale_matrix(2, 2) = factor;
	pc.transform(scale_matrix);
}
///
void point_cloud_interactable::update_scan_index_visibility_test() {
	for (int i = 0; i < pc.scan_index_visibility.size(); i++) {
		if (i == 0 || i == 1)
			pc.scan_index_visibility.at(i) = true;
		else
			pc.scan_index_visibility.at(i) = false;
	}
	pc.update_scan_index_visibility();
}
///
void point_cloud_interactable::set_src_and_target_scan_idx_as_test() {
	src_scan_idx = 1;
	target_scan_idx = 0;
}
///
void point_cloud_interactable::render_select_src_cloud_only() {
	for (int i = 0; i < pc.scan_index_visibility.size(); i++) {
		if (i == src_scan_idx)
			pc.scan_index_visibility.at(i) = true;
		else
			pc.scan_index_visibility.at(i) = false;
	}
	pc.update_scan_index_visibility();
}
///
void point_cloud_interactable::render_select_target_cloud_only() {
	for (int i = 0; i < pc.scan_index_visibility.size(); i++) {
		if (i == target_scan_idx)
			pc.scan_index_visibility.at(i) = true;
		else
			pc.scan_index_visibility.at(i) = false;
	}
	pc.update_scan_index_visibility();
}
///
void point_cloud_interactable::render_both_src_target_clouds() {
	for (int i = 0; i < pc.scan_index_visibility.size(); i++) {
		if ((i == src_scan_idx)||(i == target_scan_idx))
			pc.scan_index_visibility.at(i) = true;
		else
			pc.scan_index_visibility.at(i) = false;
	}
	pc.update_scan_index_visibility();
}
/// extract to pc_src and pc_target
void point_cloud_interactable::extract_point_clouds_for_icp() {
	pc_src.clear_all();
	pc_target.clear_all();
	// colloect points, todo: support more features 
	for (int Idx = 0; Idx < pc.get_nr_points(); Idx++) {
		if (pc.point_scan_index.at(Idx) == src_scan_idx)
			pc_src.add_point(pc.pnt(Idx), pc.clr(Idx), pc.nml(Idx));
		if (pc.point_scan_index.at(Idx) == target_scan_idx)
			pc_target.add_point(pc.pnt(Idx), pc.clr(Idx), pc.nml(Idx));
	}
	pc_src.box_out_of_date = true;
	pc_target.box_out_of_date = true;
	// state: tree ds not build
}
/// support for feature selection and icp just within feature points 
void point_cloud_interactable::extract_point_clouds_for_icp_marked_only() {
	pc_src.clear_all();
	pc_target.clear_all();
	// colloect points, todo: support more features 
	for (int Idx = 0; Idx < pc.get_nr_points(); Idx++) {
		if (pc.point_scan_index.at(Idx) == src_scan_idx)
			if(pc.topo_id.at(Idx) == point_cloud::TOPOAttribute::ICP_SOURCE_A)
				pc_src.add_point(pc.pnt(Idx), pc.clr(Idx), pc.nml(Idx));
		if (pc.point_scan_index.at(Idx) == target_scan_idx)
			if (pc.topo_id.at(Idx) == point_cloud::TOPOAttribute::ICP_TARGET_A)
				pc_target.add_point(pc.pnt(Idx), pc.clr(Idx), pc.nml(Idx));
	}
	pc_src.box_out_of_date = true;
	pc_target.box_out_of_date = true;
	// state: tree ds not build
}
/// register 
void point_cloud_interactable::perform_icp_and_acquire_matrices() {
	// align pc_src to pc_target, target is fixed 
	if (pc_target.get_nr_points() && pc_src.get_nr_points()) {
		// set sources 
		icp.set_source_cloud(pc_src);
		icp.set_target_cloud(pc_target);
		// init matrices 
		rmat.identity();
		tvec.zeros();
		// setup pdarameters 
		icp.set_iterations(icp_iterations); // shall set to 1 for now, avoid numerical problem ... but we have effecency problem 
		icp.set_eps(1e-8);
		icp.set_num_random(icp_samples);
		// pc_to_be_append has been changed during the registration 
		icp.reg_icp_get_matrices(&pc_src, &pc_target, S, Q, rmat, tvec); // rmat will be corrept if icp_iterations>1, precision error. S doesnt have 
		// update feature points for rendering 
		// update feature_points_src from S: selected points in source cloud 
		feature_points_src.resize(S.get_nr_points());
		feature_point_colors_src.resize(S.get_nr_points());
		for (int Idx = 0; Idx < S.get_nr_points(); Idx++) {
			feature_points_src.at(Idx) = S.pnt(Idx);
			feature_point_colors_src.at(Idx) = rgb(0, 1, 0);
		}
		// update feature_points_target from Q: selected points in target cloud 
		feature_points_target.resize(Q.get_nr_points());
		feature_point_colors_target.resize(S.get_nr_points());
		for (int Idx = 0; Idx < Q.get_nr_points(); Idx++) {
			feature_points_target.at(Idx) = Q.pnt(Idx);
			feature_point_colors_target.at(Idx) = rgb(1, 0, 0);
		}
	}
	else { std::cout << "icp: error point cloud size" << std::endl; }
}
///
void point_cloud_interactable::perform_icp_given_four_pair_points() {
	icp.reg_icp_get_matrices_from_4pair_points(&icp_clicking_points_src, &icp_clicking_points_target, rmat, tvec);
}
///
void perform_icp_1Iter() { // impl. outside 

}
// manually clicking bring the point clouds closer and perfrom traditional ICP with ease 
void point_cloud_interactable::perform_icp_manual_clicking() {
	perform_icp_given_four_pair_points();
	apply_register_matrices_for_the_original_point_cloud();
	// apply to four clicking points src 
	for (auto& cps : icp_clicking_points_src)
		cps = rmat * cps + tvec;
}
/// apply transformations to the global pc 
void point_cloud_interactable::apply_register_matrices_for_the_original_point_cloud() {
	for (int Idx = 0; Idx < pc.get_nr_points(); Idx++) {
		if (pc.point_scan_index.at(Idx) == src_scan_idx) {
			pc.pnt(Idx) = rmat * pc.pnt(Idx) + tvec;
		}
	}
	// state: point position updated, will be passed to gpu with set_array() in pc drawable automatically 
}
/// register without subsampling 
void point_cloud_interactable::register_cur_and_last_pc_if_present() {
	// align pc_src to pc_target, target is fixed 
	if (pc_target.get_nr_points() && pc_src.get_nr_points()) {
		// set sources 
		icp.set_source_cloud(pc_src);
		icp.set_target_cloud(pc_target);
		// init matrices 
		rotation.identity();
		translation.zeros();
		// setup pdarameters 
		icp.set_iterations(10);
		icp.set_eps(1e-8);
		icp.set_num_random(40);
		icp.build_target_ann_tree();
		// pc_to_be_append has been changed during the registration 
		icp.reg_icp(rotation, translation, pc_src, pc_target, icp_filter_type, this->get_context());
	}
	else { std::cout << "icp: error point cloud size" << std::endl; }
}
///
void point_cloud_interactable::register_with_subsampled_pcs(point_cloud& _pc) {
	if(!pc_last_subsampled.get_nr_points())
		collect_to_subsampled_pcs();
	if (pc_last_subsampled.get_nr_points() && pc_to_be_append_subsampled.get_nr_points()) {
		// we just need the rotation and translation from this subsampled pcs 
		rotation.identity();
		translation.zeros();
		icp.set_iterations(10);
		icp.set_eps(1e-8);
		icp.set_num_random(40);

		icp.set_source_cloud(pc_to_be_append_subsampled);
		icp.set_target_cloud(pc_last_subsampled);
		icp.build_target_ann_tree();

		// pc_to_be_append has been changed during the registration 
		icp.reg_icp(rotation, translation, crs_srs_pc, crs_tgt_pc, icp_filter_type, this->get_context());
	
		// will be added to pc 
		pc_to_be_append.rotate(cgv::math::quaternion<float>(rotation));
		pc_to_be_append.translate(translation);

		// as visual feedback 
		_pc.rotate(cgv::math::quaternion<float>(rotation));
		_pc.translate(translation);
	}
}
///
void point_cloud_interactable::highlight_last_pc() {
	//if (num_of_pcs > 0) {
	//	tmppc = pc;
	//	pc = pc_last;
	//	ensure_face_id();
	//	for (auto& v : pc_last.face_id) {
	//		v = point_cloud::TOPOAttribute::ICP_TARGET_HIGHLIGHT;
	//	}
	//	use_these_point_color_indices = &pc_last.face_id;
	//	is_highlighting = true;
	//}
	//for (auto& v : pc_to_be_append.face_id) {
	//	v = point_cloud::TOPOAttribute::ICP_SOURCE_HIGHLIGHT;
	//}
}
///
void point_cloud_interactable::reset_highlighted_last_pc() {
	//pc = tmppc;
	//ensure_point_selection_pc();
	//for (auto& v : pc_last.face_id) {
	//	v = point_cloud::TOPOAttribute::ORI;
	//}
	//use_these_point_color_indices = &pc.face_id;
	//is_highlighting = false;
}
/// quick test 
void point_cloud_interactable::fill_subsampled_pcs_with_cur_pc_as_a_test() {
	pc_last_subsampled = pc_last;
	pc_to_be_append_subsampled = pc_to_be_append;
}
///
void point_cloud_interactable::pc_changed() {
	ng.clear();
	tree_ds_out_of_date = true;
}
///
void point_cloud_interactable::append_frame(point_cloud& _pc, bool registered) {
	if (is_highlighting)
		reset_highlighted_last_pc();
	if (registered) {
		pc.append(pc_to_be_append);
		if (pc_to_be_append.has_cam_posi)
			frame_cam_posi.push_back(pc_to_be_append.cam_posi_list.front()); // may prob.
		pc_last = pc_to_be_append;
		tree_ds_target_pc_last_frame = tree_ds_source_pc;
	}
	else {
		pc.append(_pc);
		if (_pc.has_cam_posi)
			frame_cam_posi.push_back(_pc.cam_posi_list.front()); // may prob.
		pc_last = _pc;
	}
	num_of_pcs++;
	frame_pointers.push_back(pc.get_nr_points());

	pc_changed();
}
///
void point_cloud_interactable::clear_all() {
	pc.clear_all();
	tmppc.clear_all();
	pc_last.clear_all();
	pc_to_be_append.clear_all();
	oripc.clear_all();

	render_camposes = false;
	num_of_pcs = 0;
	marked = false;
	tree_ds_out_of_date = true;
	is_highlighting = false;
	crs_srs_pc.clear_all();
	crs_tgt_pc.clear_all();
	pc_last_subsampled.clear_all();
	pc_to_be_append_subsampled.clear_all();
	frame_pointers.clear();
	frame_cam_posi.clear();
	rotation.identity();
	translation.zeros();
	icp_filter_type = cgv::pointcloud::ICP::RANDOM_SAMPLING;
	queue_for_regions.clear();

	// todo: reset vars here ...
	//use_these_point_colors = 0;
	use_these_point_palette = 0;
	use_these_point_color_indices = 0; 
	use_these_component_colors = 0;
}
///
void point_cloud_interactable::interact_callback(double t, double dt)
{

	if (interact_state == IS_FULL_FRAME || interact_state == IS_DRAW_FULL_FRAME)
		return;

	if (interact_state == IS_INTERMEDIATE_FRAME)
		interact_state = IS_WAIT_INTERACTION_TO_STOP;
	else {
		interact_state = IS_DRAW_FULL_FRAME;
		post_redraw();
	}

}
///
void point_cloud_interactable::ensure_tree_ds()
{
	if (tree_ds_out_of_date) {
		if (tree_ds)
			delete tree_ds;
		tree_ds = new ann_tree;
		tree_ds->build(pc);
		tree_ds_out_of_date = false;
	}
}
///
void point_cloud_interactable::ensure_neighbor_graph() {
	if (ng.empty())
		build_neighbor_graph();
}
///
void point_cloud_interactable::build_neighbor_graph()
{
	std::cout << "building neighbor graph..." << std::endl;
	// use component wise implementation if we do have more than one component 
	if (pc.has_components() && pc.get_nr_components() > 1) {
		build_neighbor_graph_componentwise();
		return;
	}

	ng.clear();
	ensure_tree_ds();
	cgv::utils::statistics he_stats;
	ng.build(pc.get_nr_points(), k, *tree_ds, &he_stats);
	if (do_symmetrize)
		ng.symmetrize();
	on_point_cloud_change_callback(PCC_NEIGHBORGRAPH_CREATE);

	std::cout << "half edge statistics " << he_stats << std::endl;
	std::cout << "v " << pc.get_nr_points()
		<< ", he = " << ng.nr_half_edges
		<< " ==> " << (float)ng.nr_half_edges / ((unsigned)(pc.get_nr_points())) << " half edges per vertex" << std::endl;
}
///
void point_cloud_interactable::build_neighbor_graph_componentwise()
{
	// prepare neighbor graph data structure
	ng.clear();
	ng.resize(pc.get_nr_points());

	// iterate components
	std::cout << "build_neighbor_graph_componentwise(" << pc.get_nr_components() << "):"; std::cout.flush();
	for (Idx ci = 0; ci < (Idx)pc.get_nr_components(); ++ci) {
		std::cout << " " << ci << ":"; std::cout.flush();
		ann_tree* T = new ann_tree;
		std::vector<Idx> C(1, Idx(ci));
		T->build(pc, C);
		Idx n = Idx(pc.component_point_range(ci).nr_points);
		Idx offset = Idx(pc.component_point_range(ci).index_of_first_point);
		for (Idx l = 0; l < n; ++l) {
			Idx i = l + offset;
			std::vector<Idx>& Ni = ng[i];
			T->extract_neighbors(i, k, Ni);
			for (auto& ni : Ni) 
				ni += offset;
			ng.nr_half_edges += k;
		}
		delete T;
		std::cout << "*"; std::cout.flush();

		if (do_symmetrize) {
			for (Idx l = 0; l < n; ++l) {
				Idx i = l + offset;
				std::vector<Idx>& Ni = ng[i];
				for (size_t o = 0; o < Ni.size(); ++o) {
					std::vector<Idx>& Nj = ng[Ni[o]];
					if (std::find(Nj.begin(), Nj.end(), i) == Nj.end()) {
						Nj.push_back(i);
						++ng.nr_half_edges;
					}
				}
			}
			std::cout << "s"; std::cout.flush();
		}
	}
	std::cout << std::endl;

	on_point_cloud_change_callback(PCC_NEIGHBORGRAPH_CREATE);
}
///
bool point_cloud_interactable::get_picked_point(int x, int y, unsigned& index)
{
	cgv::math::fvec<double, 3> world_location;
	if (!get_world_location(x, y, *find_view_as_node(), world_location))
		return false;
	//  unproject to world coordinates with smaller (closer to eye) z-value one	
	Pnt p_pick_world = world_location;

	// find closest point
	int i_closest = -1;
	if (accelerate_picking) {
		ensure_tree_ds();
		i_closest = tree_ds->find_closest(p_pick_world);
	}
	else {
		int n = (int)pc.get_nr_points();
		float smallest_sqr_dist = 0;
		for (int i = 0; i < n; ++i) {
			if (i_closest == -1) {
				i_closest = i;
				smallest_sqr_dist = (pc.pnt(i) - p_pick_world).sqr_length();
			}
			else {
				float new_sqr_dist = (pc.pnt(i) - p_pick_world).sqr_length();
				if (new_sqr_dist < smallest_sqr_dist) {
					i_closest = i;
					smallest_sqr_dist = new_sqr_dist;
				}
			}
		}
	}
	if (i_closest == -1)
		return false;
	index = i_closest;
	return true;
}
///
void point_cloud_interactable::compute_normals()
{
	// already has nmls or too few points are given 
	if (pc.has_normals())
		return;
	if (pc.get_nr_points() < 31) {
		std::cout << "too few points for nml computing!" << std::endl;
		return;
	}
	// rebuild ds for nml computation 
	// normal estimation
	ng.clear();
	tree_ds_out_of_date = true;
	if (ng.empty())
		build_neighbor_graph();
	ne.compute_weighted_normals(reorient_normals && pc.has_normals());
	// orient to camera position if cam position present 
	if (pc.has_cam_posi)
		orient_normals_to_view_point_vr(pc.cam_posi_list.back()); // orient to just updated camera position 
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}
///
void point_cloud_interactable::recompute_normals()
{
	if (ng.empty())
		build_neighbor_graph();
	if (!pc.has_normals())
		compute_normals();
	//	ne.compute_bilateral_weighted_normals(reorient_normals);
	ne.compute_plane_bilateral_weighted_normals(reorient_normals);
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}
///
void point_cloud_interactable::toggle_normal_orientations()
{
	if (!pc.has_normals())
		return;
	for (Idx i = 0; i < Idx(pc.get_nr_points()); ++i)
		pc.nml(i) = -pc.nml(i);
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}
///
void point_cloud_interactable::orient_normals()
{
	if (ng.empty())
		build_neighbor_graph();
	if (!pc.has_normals())
		compute_normals();
	ne.orient_normals();
	on_point_cloud_change_callback(PCC_NORMALS);
	post_redraw();
}
///
void point_cloud_interactable::orient_normals_to_view_point()
{
	if (ensure_view_pointer()) {
		if (ng.empty())
			build_neighbor_graph();
		Pnt view_point = view_ptr->get_eye();
		ne.orient_normals(view_point);
		on_point_cloud_change_callback(PCC_NORMALS);
		post_redraw();
	}
}
///
void point_cloud_interactable::orient_normals_to_view_point_vr(vec3 cam_posi)
{
	//if (ensure_view_pointer()) {
		if (ng.empty())
			build_neighbor_graph();
		ne.orient_normals(cam_posi);
		on_point_cloud_change_callback(PCC_NORMALS);
		post_redraw();
	//}
}
/// call this before using the view ptr for the first time
bool point_cloud_interactable::ensure_view_pointer()
{
	return cgv::base::ensure_by_find(this, view_ptr);
}
///
point_cloud_interactable::point_cloud_interactable() : ne(pc, ng)
{
	set_name("Point Cloud Viewer");

	view_ptr = 0;

	accelerate_picking = true;
	tree_ds_out_of_date = true;
	tree_ds = 0;
	tree_ds_target_pc_last_frame = 0;
	tree_ds_source_pc = 0;

	do_append = false;
	do_auto_view = true;

	show_nmls = false;
	interact_point_step = 1;
	show_point_count = 0;
	show_point_start = 0;
	interact_delay = 0.15;

	interact_state = IS_INTERMEDIATE_FRAME;

	cgv::signal::connect(interact_trigger.shoot, this, &point_cloud_interactable::interact_callback);
	interact_trigger.schedule_recuring(interact_delay);

	show_neighbor_graph = false;
	k = 30; // init k to 30 
	gm = growing_mode::UNSIGNED_MEAN_CURVATURE_BASED;
	do_symmetrize = false;
	reorient_normals = true;

	frame_pointers.push_back(0);

	icp_clicking_points_src.resize(4);
	icp_clicking_point_colors_src.resize(4);
	icp_clicking_points_target.resize(4);
	icp_clicking_point_colors_target.resize(4);
}
///
void point_cloud_interactable::auto_set_view()
{
	if (pc.get_nr_points() == 0)
		return;

	std::vector<cgv::render::view*> view_ptrs;
	cgv::base::find_interface<cgv::render::view>(get_node(), view_ptrs);
	if (view_ptrs.empty()) {
		cgv::gui::message("could not find a view to adjust!!");
		return;
	}
	cgv::gui::animate_with_rotation(view_ptrs[0]->ref_view_up_dir(), dvec3(0, 1, 0), 0.5)->set_base_ptr(this);
	cgv::gui::animate_with_geometric_blend(view_ptrs[0]->ref_y_extent_at_focus(), 1.5*pc.box().get_extent()(1), 0.5)->set_base_ptr(this);
	cgv::gui::animate_with_linear_blend(view_ptrs[0]->ref_focus(), dvec3(pc.box().get_center()), 0.5)->set_base_ptr(this);
	post_redraw();
}
///
std::string point_cloud_interactable::get_type_name() const
{
	return "point_cloud_interactable";
}
///
bool point_cloud_interactable::self_reflect(cgv::reflect::reflection_handler& srh)
{
	if (srh.reflect_member("do_append", do_append) &&
		srh.reflect_member("use_component_colors", use_component_colors) &&
		srh.reflect_member("use_component_transformations", use_component_transformations) &&
		srh.reflect_member("do_auto_view", do_auto_view) &&
		srh.reflect_member("data_path", data_path) &&
		srh.reflect_member("file_name", new_file_name) &&
		srh.reflect_member("directory_name", directory_name) &&
		srh.reflect_member("interact_point_step", interact_point_step) &&
		srh.reflect_member("surfel_style", surfel_style) &&
		srh.reflect_member("normal_style", normal_style) &&
		srh.reflect_member("box_style", box_style) &&
		srh.reflect_member("box_wire_style", box_wire_style) &&
		srh.reflect_member("show_points", show_points) &&
		srh.reflect_member("show_nmls", show_nmls) &&
		srh.reflect_member("show_boxes", show_boxes) &&
		srh.reflect_member("show_box", show_box) &&
		srh.reflect_member("show_neighbor_graph", show_neighbor_graph) &&
		srh.reflect_member("k", k) &&
		srh.reflect_member("do_symmetrize", do_symmetrize) &&
		srh.reflect_member("reorient_normals", reorient_normals))
		return true;
	return false;
}
///
void point_cloud_interactable::stream_help(std::ostream& os)
{
	os << "PC: open (Ctrl-O), append (Ctrl-A), toggle <p>oints, <n>ormals, <b>ox, <g>graph, <i>llum" << std::endl;
}
///
void point_cloud_interactable::stream_stats(std::ostream& os)
{
	os << "PC: #P=" << pc.get_nr_points()
		<< ", #N=" << (pc.has_normals() ? pc.get_nr_points() : 0)
		<< ", #C=" << (pc.has_colors() ? pc.get_nr_points() : 0) 
		<< ", B=" << pc.box().get_center() << "<" << pc.box().get_extent() << ">" << std::endl;
}
///
void point_cloud_interactable::draw_edge_color(unsigned int vi, unsigned int j, bool is_symm, bool is_start) const
{
	if (is_symm)
		glColor3f(0.5f, 0.5f, 0.5f);
	else {
		if (is_start)
			glColor3f(1, 0.5f, 0.5f);
		else
			glColor3f(1, 1, 0.5f);
	}
}
/// basic functions
void point_cloud_interactable::draw_graph(cgv::render::context& ctx)
{
	if (!show_neighbor_graph)
		return;

	glDisable(GL_LIGHTING);
	glColor3f(0.5f, 0.5f, 0.5f);
	glLineWidth(normal_style.line_width);
	glBegin(GL_LINES);
	for (unsigned int vi = 0; vi<ng.size(); ++vi) {
		const std::vector<Idx> &Ni = ng[vi];
		for (unsigned int j = 0; j<Ni.size(); ++j) {
			unsigned int vj = Ni[j];
			// check for symmetric case and only draw once
			if (ng.is_directed_edge(vj, vi)) {
				if (vi < vj) {
					draw_edge_color(vi, j, true, true);
					glArrayElement(vi);
					draw_edge_color(vi, j, true, false);
					glArrayElement(vj);
				}
			}
			else {
				draw_edge_color(vi, j, false, true);
				glArrayElement(vi);
				draw_edge_color(vi, j, false, false);
				glArrayElement(vj);
			}
		}
	}
	glEnd();
	glEnable(GL_LIGHTING);
}
/// basic functions
bool point_cloud_interactable::init(cgv::render::context& ctx)
{
	if (!gl_point_cloud_drawable::init(ctx))
		return false;
	return true;
}
/// basic functions
void point_cloud_interactable::init_frame(cgv::render::context& ctx)
{
	/*static bool my_tab_selected = false;
	if (!my_tab_selected) {
		my_tab_selected = true;
		cgv::gui::gui_group_ptr gg = ((provider*)this)->get_parent_group();
		if (gg) {
			cgv::gui::gui_group_ptr tab_group = gg->get_parent()->cast<cgv::gui::gui_group>();
			if (tab_group) {
				cgv::base::base_ptr c = gg;
				tab_group->select_child(c, true);
			}
		}
	}*/
	gl_point_cloud_drawable::init_frame(ctx);
}
/// draw call
void point_cloud_interactable::draw(cgv::render::context& ctx)
{
	if (pc.get_nr_points() != 0) {
		glVertexPointer(3, GL_FLOAT, 0, &(pc.pnt(0).x()));
		glEnableClientState(GL_VERTEX_ARRAY);
		draw_graph(ctx);
		glDisableClientState(GL_VERTEX_ARRAY);
	}

	/*if (interact_state != IS_DRAW_FULL_FRAME)
		std::swap(show_point_step, interact_point_step);*/

	gl_point_cloud_drawable::draw(ctx);

	//if (interact_state != IS_DRAW_FULL_FRAME) {
	//	std::swap(show_point_step, interact_point_step);
	//	interact_state = IS_INTERMEDIATE_FRAME;
	//}
	//else
	//	interact_state = IS_FULL_FRAME;

	// render the cameras with information read from .campose file 
	//if (pc.render_cams) {
	//	for (auto render_kit : image_renderer_list) {
	//		render_kit.draw(ctx);
	//	}
	//}

	//// render feature points 
	//if (feature_points.size() > 0 && fea_box.size() > 0) {
	//	cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
	//	renderer.set_render_style(fea_style);
	//	renderer.set_box_array(ctx, fea_box);
	//	renderer.set_color_array(ctx, fea_box_color);
	//	renderer.set_translation_array(ctx, fea_box_trans);
	//	renderer.set_rotation_array(ctx, fea_box_rot);
	//	renderer.render(ctx, 0, fea_box.size());
	//}

	//// render feature points for ICP 
	//srs_icp_feature_points.radius = surfel_style.point_size / 100.0f;
	//if (feature_points_src.size() > 0) {
	//	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	//	sr.set_render_style(srs_icp_feature_points);
	//	sr.set_position_array(ctx, feature_points_src);
	//	sr.set_color_array(ctx, feature_point_colors_src);
	//	sr.render(ctx, 0, feature_points_src.size());
	//}
	//if (feature_points_target.size() > 0) {
	//	auto& sr = cgv::render::ref_sphere_renderer(ctx);
	//	sr.set_render_style(srs_icp_feature_points);
	//	sr.set_position_array(ctx, feature_points_target);
	//	sr.set_color_array(ctx, feature_point_colors_target);
	//	sr.render(ctx, 0, feature_points_target.size());
	//}

	// 
	/*if (icp_clicking_points_src.size() > 0) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_render_style(srs_icp_feature_points);
		sr.set_position_array(ctx, icp_clicking_points_src);
		sr.set_color_array(ctx, icp_clicking_point_colors_src);
		sr.render(ctx, 0, icp_clicking_points_src.size());
	}
	if (icp_clicking_points_target.size() > 0) {
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_render_style(srs_icp_feature_points);
		sr.set_position_array(ctx, icp_clicking_points_target);
		sr.set_color_array(ctx, icp_clicking_point_colors_target);
		sr.render(ctx, 0, icp_clicking_points_target.size());
	}*/

	// quick test, higher effecency rendering 
	//cgv::render::clod_point_renderer& cp_renderer = ref_clod_point_renderer(ctx);
	//cp_renderer.set_render_style(cp_style);

	//if (pc.get_nr_points() > 0) {
	//	if (renderer_out_of_date) {
	//		std::vector<LODPoint> V(pc.get_nr_points());
	//		for (int i = 0; i < pc.get_nr_points(); ++i) {
	//			V[i].position() = pc.pnt(i);
	//			V[i].color() = pc.clr(i);
	//		}
	//		points_with_lod = std::move(lod_generator.generate_lods(V));

	//		cp_renderer.set_points(ctx, &points_with_lod.data()->position(),
	//			&points_with_lod.data()->color(), &points_with_lod.data()->level(), points_with_lod.size(), sizeof(LODPoint));
	//		
	//		renderer_out_of_date = false;
	//	}

	//	if (cp_renderer.enable(ctx))
	//		cp_renderer.draw(ctx, 0, (size_t)pc.get_nr_points());
	//}
}
///
void point_cloud_interactable::configure_subsample_controls()
{
	if (find_control(show_point_begin)) {
		find_control(show_point_begin)->set("max", show_point_end);
		find_control(show_point_end)->set("min", show_point_begin);
		find_control(show_point_end)->set("max", pc.get_nr_points());
		find_control(show_point_count)->set("max", pc.get_nr_points());
		find_control(show_point_start)->set("max", pc.get_nr_points() - show_point_count);
	}
}
/// basic functions
void point_cloud_interactable::handle_args(std::vector<std::string>& args)
{
	for (unsigned ai = 0; ai < args.size(); ++ai) {
		if (cgv::utils::file::exists(args[ai])) {
			if (open_and_append(args[ai])) {
				args.erase(args.begin() + ai);
				--ai;
			}
		}
	}
}
/// basic functions
bool point_cloud_interactable::handle(cgv::gui::event& e)
{
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::key_event& ke = static_cast<cgv::gui::key_event&>(e);
		if (ke.get_action() == cgv::gui::KA_PRESS || ke.get_action() == cgv::gui::KA_REPEAT) {
			switch (ke.get_key()) {
			case '=' :
			case '+' :
				surfel_style.point_size += 1;
				on_set(&surfel_style.point_size); 
				return true;
			case '-' :
				if (surfel_style.point_size > 0) {
					surfel_style.point_size -= 1;
					if (surfel_style.point_size < 1)
						surfel_style.point_size = 1;
					on_set(&surfel_style.point_size); 
				}
				return true;
			case 'P' :
				show_points = !show_points;
				on_set(&show_points); 
				return true;
			case 'C' :
				if (ke.get_modifiers() == 0) {
					if (surfel_style.map_color_to_material == cgv::render::MS_FRONT_AND_BACK)
						surfel_style.map_color_to_material = cgv::render::CM_NONE;
					else
						++(int&)surfel_style.map_color_to_material;
					on_set(&surfel_style.map_color_to_material);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					if (surfel_style.culling_mode == cgv::render::CM_FRONTFACE)
						surfel_style.culling_mode = cgv::render::CM_OFF;
					else
						++(int&)surfel_style.culling_mode;
					on_set(&surfel_style.culling_mode);
				}
				return true;
			case 'B' :
				if (ke.get_modifiers() == 0) {
					surfel_style.blend_points = !surfel_style.blend_points;
					on_set(&surfel_style.blend_points);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					show_boxes = !show_boxes;
					on_set(&show_boxes);
				}
				return true;
			case 'N' :
				if (ke.get_modifiers() == 0) {
					show_nmls = !show_nmls;
					on_set(&show_nmls);
				}
				else if(ke.get_modifiers() == cgv::gui::EM_CTRL+ cgv::gui::EM_ALT) {
					if (!pc.has_normals()) {
						pc.create_normals();
						compute_normals();
					}
					else 
						recompute_normals();

					on_point_cloud_change_callback(PCC_NORMALS);
					post_redraw();
				}
				return true;
			case 'I' :
				if (surfel_style.illumination_mode == cgv::render::IM_TWO_SIDED)
					surfel_style.illumination_mode = cgv::render::IM_OFF;
				else
					++(int&)surfel_style.illumination_mode;
				on_set(&surfel_style.illumination_mode);
				return true;
			case 'G' :
				show_neighbor_graph = !show_neighbor_graph;
				on_set(&show_neighbor_graph); 
				return true;
			case 'O' :
				if (ke.get_modifiers() == 0) {
					surfel_style.orient_splats = !surfel_style.orient_splats;
					on_set(&surfel_style.orient_splats);
				}
				else if (ke.get_modifiers() == cgv::gui::EM_SHIFT) {
					orient_normals();
					on_point_cloud_change_callback(PCC_NORMALS);
					post_redraw();
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_ALT) {
					orient_normals_to_view_point();
					on_point_cloud_change_callback(PCC_NORMALS);
					post_redraw();
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_CTRL) {
					std::string fn = cgv::gui::file_open_dialog(FILE_OPEN_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						open(fn);
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_CTRL + cgv::gui::EM_SHIFT) {
					std::string fn = cgv::gui::file_open_dialog("Open Component Transformations", "Text Files (txt):*.txt|All Files:*.*");
					if (!fn.empty()) {
						if (!pc.read_component_transformations(fn)) {
							std::cerr << "error reading component transformation file " << fn << std::endl;
						}
						post_redraw();
					}
					return true;
				}
				return false;
			case 'A' :
				if (ke.get_modifiers() == cgv::gui::EM_CTRL) {
					std::string fn = cgv::gui::file_open_dialog(FILE_APPEND_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						open_and_append(fn);
					return true;
				}
				return false;
			case 'S' :
				if (ke.get_modifiers() == cgv::gui::EM_CTRL + cgv::gui::EM_SHIFT) {
					std::string fn = cgv::gui::file_save_dialog(FILE_SAVE_TITLE, FILE_OPEN_FILTER);
					if (!fn.empty())
						save(fn);
					return true;
				}
				else if (ke.get_modifiers() == cgv::gui::EM_ALT) {
					sort_points = !sort_points;
					on_set(&sort_points);
					return true;
				}
				return false;
			case cgv::gui::KEY_Space :
				if (pc.get_nr_points() == 0)
					return false;
				auto_set_view();
				return true;
			}
		}
	}

	if (e.get_kind() == cgv::gui::EID_MOUSE) {
		//cgv::gui::mouse_event& me = (cgv::gui::mouse_event&) e;
		//if (me.get_action() == cgv::gui::MA_PRESS) {
		//	/*if (me.get_modifiers() != cgv::gui::EM_CTRL)
		//		return false;*/
		//	unsigned picked_index;
		//	if (get_picked_point(me.get_x(), me.get_y(), picked_index)) {
		//		std::cout << pc.pnt(picked_index) << std::endl;
		//	}
		//	return true;
		//}
	}

	return false;
}
///
void point_cloud_interactable::on_point_cloud_change_callback(PointCloudChangeEvent pcc_event)
{
	if (use_component_transformations != pc.has_component_transformations()) {
		use_component_transformations = pc.has_component_transformations();
		on_set(&use_component_transformations);
	}
	if (use_component_colors != pc.has_component_colors()) {
		use_component_colors = pc.has_component_colors();
		on_set(&use_component_colors);
	}
	if (!pc.has_normals() != surfel_style.illumination_mode == cgv::render::IM_OFF) {
		if (pc.has_normals())
			surfel_style.illumination_mode = cgv::render::IM_ONE_SIDED;
		else
			surfel_style.illumination_mode = cgv::render::IM_OFF;
		update_member(&surfel_style.illumination_mode);
	}
	// after new points are added, very fast 
	if (((pcc_event & PCC_POINTS_MASK) == PCC_POINTS_RESIZE) || ((pcc_event & PCC_POINTS_MASK) == PCC_NEW_POINT_CLOUD)) {
		// reconstruct colors if not present 
		if (!pc.has_colors()) {
			pc.C.resize(pc.get_nr_points());
			for (auto& pc : pc.C) pc = rgba(0.4,0.4,0.4,1);
		}
		// reconstruct normals if not present 
		if (!pc.has_normals() && compute_normal_after_read) {
			compute_normals();
			orient_normals();
		}
		// create per-point face id  if not present, fast 
		if (pc.face_id.size() == 0 || reading_from_raw_scan) {
			pc.face_id.resize(pc.get_nr_points());
			for (auto& fi : pc.face_id) fi = 0; // 0 means non-selected faces 
			pc.has_face_selection = true;
		}
		// create per-point topo id if not present 
		if (pc.topo_id.size() == 0 || reading_from_raw_scan) {
			pc.topo_id.resize(pc.get_nr_points());
			for (auto& ti : pc.topo_id) ti = 0; // 0 means non-topological information present 
			pc.has_topo_selection = true; // not used actually? 
		}
		// create per-point scan index if not present, 0 by default 
		// the first is for one scan, later for raw scans 
		// mainly used for point appending 
		if (pc.point_scan_index.size() == 0 || reading_from_raw_scan) {
			int append_starting_index = pc.point_scan_index.size();
			pc.point_scan_index.resize(pc.get_nr_points());
			for (int Idx = append_starting_index; Idx < pc.point_scan_index.size(); Idx++) {
				pc.point_scan_index.at(Idx) = pc.currentScanIdx_Recon;
			}
			pc.has_scan_index = true;
		}
		reading_from_raw_scan = false;
		// prepare scan indices visibility, create if not present: not used currently 
		/*pc.scan_index_visibility.resize(pc.num_of_scan_indices);
		for (auto& siv : pc.scan_index_visibility) {siv = true;}
		pc.update_scan_index_visibility();*/
		// box is used to adjust correct size of the normals, surfel size... and for selection estimation 
		// also used for rendering of the boxes 
		pc.box_out_of_date = true;
		// tree ds will be 
		tree_ds_out_of_date = true;
		if (tree_ds) {
			delete tree_ds;
			tree_ds = 0;
		}
		// neighbor_graph is used for normal and feature computation, built upon tree ds 
		ng.clear();
		// configure point cloud rendering, step...
		show_point_end = pc.get_nr_points();
		show_point_begin = 0;
		show_point_count = pc.get_nr_points();
		interact_point_step = std::max((unsigned)(show_point_count / 1000000), 1u);
		nr_draw_calls = std::max((unsigned)(show_point_count / 1000000), 1u);
		// update gui, not used currently 
		update_member(&show_point_begin);
		update_member(&show_point_end);
		update_member(&show_point_count);
		update_member(&interact_point_step);
		update_member(&nr_draw_calls);
		configure_subsample_controls();
	}
	if ((pcc_event & PCC_POINTS_MASK) == PCC_NEW_POINT_CLOUD && do_auto_view) {
		auto_set_view();
	}
	post_redraw();
}
///
void point_cloud_interactable::on_set(void* member_ptr)
{
	if (member_ptr == &ne.localization_scale || member_ptr == &ne.normal_sigma || member_ptr == &ne.bw_type || member_ptr == &ne.plane_distance_scale) {
		on_point_cloud_change_callback(PCC_WEIGHTS);
	}
	if (member_ptr == &new_file_name) {
		if (ref_tree_node_visible_flag(new_file_name)) {
			save(new_file_name);
		}
		else {
			if (!do_append)
				open(new_file_name);
			else
				open_and_append(new_file_name);
		}
	}
	if (member_ptr == &directory_name) {
		open_directory(directory_name);
	}
	if (member_ptr == &show_point_start) {
		show_point_begin = show_point_start;
		show_point_end = show_point_start + show_point_count;
		update_member(&show_point_begin);
		update_member(&show_point_end);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_count) {
		if (show_point_start + show_point_count > pc.get_nr_points()) {
			show_point_start = pc.get_nr_points() - show_point_count;
			show_point_begin = show_point_start;
			update_member(&show_point_begin);
			update_member(&show_point_start);
		}
		show_point_end = show_point_start + show_point_count;
		update_member(&show_point_end);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_begin) {
		show_point_start = show_point_begin;
		show_point_count = show_point_end - show_point_begin;
		update_member(&show_point_start);
		update_member(&show_point_count);
		configure_subsample_controls();
	}
	if (member_ptr == &show_point_end) {
		show_point_count = show_point_end - show_point_begin;
		update_member(&show_point_count);
		configure_subsample_controls();
	}
	if (member_ptr == &interact_delay) {
		interact_trigger.stop();
		interact_trigger.schedule_recuring(interact_delay);
	}
	if (member_ptr == &use_component_colors) {
		surfel_style.use_group_color = use_component_colors;
		update_member(&surfel_style.use_group_color);
	}
	if (member_ptr == &use_component_transformations) {
		surfel_style.use_group_transformation = use_component_transformations;
		update_member(&surfel_style.use_group_transformation);
	}
	update_member(member_ptr);
	post_redraw();
}
///
void point_cloud_interactable::create_gui()
{
	add_decorator(get_name(), "heading", "level=2");
	bool show = begin_tree_node("IO", data_path, true, "level=3;options='w=40';align=' '");
	add_member_control(this, "append", do_append, "toggle", "w=60", " ");
	add_member_control(this, "auto_view", do_auto_view, "toggle", "w=80");

	if (show) {
		align("\a");
		add_gui("data_path", data_path, "directory", "w=170;title='Select Data Directory'");
		add_gui("file_name", new_file_name, "file_name",
			"w=130;"
			"open=true;open_title='" FILE_OPEN_TITLE "';open_filter='" FILE_OPEN_FILTER "';"
			"save=true;save_title='" FILE_SAVE_TITLE "';save_filter='" FILE_SAVE_FILTER "'"
		);
		add_gui("directory_name", directory_name, "directory", "w=170;title='Select Data Directory';tooltip='read all point clouds from a directory'");
		align("\b");
		end_tree_node(data_path);
	}
	show = begin_tree_node("points", show_points, false, "level=3;options='w=70';align=' '");
	add_member_control(this, "show", show_points, "toggle", "w=40", " ");
	add_member_control(this, "sort", sort_points, "toggle", "w=40", " ");
	add_member_control(this, "blnd", surfel_style.blend_points, "toggle", "w=40");
	if (show) {
		align("\a");
		add_gui("surfel_style", surfel_style);
		if (begin_tree_node("subsample", show_point_step, false, "level=3")) {
			align("\a");
			add_member_control(this, "nr_draw_calls", nr_draw_calls, "value_slider", "min=1;max=100;log=true;ticks=true");
			add_member_control(this, "interact step", interact_point_step, "value_slider", "min=1;max=100;log=true;ticks=true");
			add_member_control(this, "interact delay", interact_delay, "value_slider", "min=0.01;max=1;log=true;ticks=true");
			add_member_control(this, "show step", show_point_step, "value_slider", "min=1;max=20;log=true;ticks=true");
			add_decorator("range control", "heading", "level=3");
			add_member_control(this, "begin", show_point_begin, "value_slider", "min=0;max=10;ticks=true");
			add_member_control(this, "end", show_point_end, "value_slider", "min=0;max=10;ticks=true");
			add_decorator("window control", "heading", "level=3");
			add_member_control(this, "start", show_point_start, "value_slider", "min=0;max=10;ticks=true");
			add_member_control(this, "width", show_point_count, "value_slider", "min=0;max=10;ticks=true");
			configure_subsample_controls();
			align("\b");
			end_tree_node(show_point_step);
		}
		add_member_control(this, "accelerate_picking", accelerate_picking, "check");
		align("\b");
		end_tree_node(show_points);
	}
	show = begin_tree_node("components", pc.components, false, "level=3;options='w=140';align=' '");
	add_member_control(this, "clr", use_component_colors, "toggle", "w=30", " ");
	add_member_control(this, "tra", use_component_transformations, "toggle", "w=30");
	if (show) {
		align("\a");
		if (begin_tree_node("component colors", pc.component_colors, false, "level=3")) {
			align("\a");
			for (unsigned i = 0; i < pc.component_colors.size(); ++i) {
				add_member_control(this, std::string("C") + cgv::utils::to_string(i), pc.component_colors[i]);
			}
			align("\b");
			end_tree_node(pc.component_colors);
		}
		if (begin_tree_node("group transformations", pc.component_translations, false, "level=3")) {
			align("\a");
			for (unsigned i = 0; i < pc.component_translations.size(); ++i) {
				add_gui(std::string("T") + cgv::utils::to_string(i), pc.component_translations[i]);
				add_gui(std::string("Q") + cgv::utils::to_string(i), (HVec&)pc.component_rotations[i], "direction");
			}
			align("\b");
			end_tree_node(pc.component_translations);
		}
		align("\b");
		end_tree_node(pc.components);
	}
	show = begin_tree_node("neighbor graph", show_neighbor_graph, false, "level=3;options='w=160';align=' '");
	add_member_control(this, "show", show_neighbor_graph, "toggle", "w=50");
	if (show) {
		add_member_control(this, "k", k, "value_slider", "min=3;max=50;log=true;ticks=true");
		add_member_control(this, "symmetrize", do_symmetrize, "toggle");
		cgv::signal::connect_copy(add_button("build")->click, cgv::signal::rebind(this, &point_cloud_interactable::build_neighbor_graph));
		end_tree_node(show_neighbor_graph);
	}

	show = begin_tree_node("normals", show_nmls, false, "level=3;options='w=160';align=' '");
	add_member_control(this, "show", show_nmls, "toggle", "w=50");
	if (show) {
		cgv::signal::connect_copy(add_button("toggle orientation")->click, cgv::signal::rebind(this, &point_cloud_interactable::toggle_normal_orientations));
		add_member_control(this, "bilateral weight approach", ne.bw_type, "dropdown", "enums='normal,plane'");
		add_member_control(this, "localization_scale", ne.localization_scale, "value_slider", "min=0.1;max=2;log=true;ticks=true");
		add_member_control(this, "normal_sigma", ne.normal_sigma, "value_slider", "min=0.01;max=2;log=true;ticks=true");
		add_member_control(this, "plane_distance_scale", ne.plane_distance_scale, "value_slider", "min=0.01;max=2;log=true;ticks=true");
		add_member_control(this, "reorient", reorient_normals, "toggle");
		cgv::signal::connect_copy(add_button("compute")->click, cgv::signal::rebind       (this, &point_cloud_interactable::compute_normals));
		cgv::signal::connect_copy(add_button("recompute")->click, cgv::signal::rebind     (this, &point_cloud_interactable::recompute_normals));
		cgv::signal::connect_copy(add_button("orient")->click, cgv::signal::rebind        (this, &point_cloud_interactable::orient_normals));
		cgv::signal::connect_copy(add_button("orient to view")->click, cgv::signal::rebind(this, &point_cloud_interactable::orient_normals_to_view_point));
		add_gui("normal_style", normal_style);
		end_tree_node(show_nmls);
	}

/*	show = begin_tree_node("surfrec", show_surfrec, false, "level=3;w=100;align=' '");
	add_member_control(this, "show", show_surfrec, "toggle", "w=50");
	if (show) {
		add_member_control(this, "debug_mode", SR.debug_mode, "enums='none,symmetrize,make_consistent,cycle_filter'");
		end_tree_node(show_surfrec);
	}
	*/
	show = begin_tree_node("box", show_box, false, "level=3;options='w=140';align=' '");
	add_member_control(this, "main", show_box, "toggle", "w=30", " ");
	add_member_control(this, "com", show_boxes, "toggle", "w=30");
	if (show) {
		add_gui("color", box_color);
		add_gui("box_style", box_style);
		add_gui("box_wire_style", box_wire_style);
		end_tree_node(show_box);
	}
}
///
void point_cloud_interactable::compute_feature_points_and_colorize()
{
	ensure_tree_ds();
	ensure_neighbor_graph();
	pc.F.resize(pc.get_nr_points()); // ensure feature array size  
	for (int i = 0; i < pc.get_nr_points(); i++) {
		float d[3] = { 0,0,0 };
		{
			std::vector<Crd> weights;
			std::vector<Pnt> points;
			ne.compute_weights(i, weights, &points);
			Nml new_nml;
			cgv::math::estimate_normal_wls((unsigned)points.size(), points[0], &weights[0], new_nml, &d[0]);
		}
		Pnt eigen_val_vec = Pnt(d[0], d[1], d[2]);
		pc.fea(i) = eigen_val_vec;
		//std::cout << eigen_val_vec << std::endl;
		if (d[0] > 0.1 && d[1] > 0.1 && d[2] > 0.1)
			feature_points.push_back(pc.pnt(i));
	}
	std::cout << "feature points computed " << std::endl;

	// put to boxes 
	//vec3 box_ext = vec3(0.01);
	//for (auto& p : feature_points) {
	//	fea_box.push_back(box3(-box_ext,box_ext));
	//	fea_box_color.push_back(rgb(0.6));
	//	fea_box_trans.push_back(p);
	//	fea_box_rot.push_back(quat());
	//}

	// normalize fea and map to color 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		vec3 fea_vec = pc.fea(i);
		fea_vec.normalize();
		pc.clr(i) = Clr(float_to_color_component(fea_vec.x()), float_to_color_component(fea_vec.y()), float_to_color_component(fea_vec.z()));
	}
	on_point_cloud_change_callback(PCC_COLORS);
}
///
void point_cloud_interactable::compute_principal_curvature_unsigned() {
	// ensure point structures and properties 
	ensure_tree_ds();
	ensure_neighbor_graph();
	pc.curvature.resize(pc.get_nr_points()); // ensure writting space 

	// define temp helper varibles 
	std::vector<point_cloud::point_cloud_types::Idx> neighbor_points; 
	std::vector<point_cloud::point_cloud_types::Nml> projected_normals; // for each neighbor points 

	// loop over to compute 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		projected_normals.clear();
		if (!pc.has_normals()) {
			compute_normals();
			orient_normals();
		}
		const Dir& point_normal = pc.nml(i);
		tree_ds->extract_neighbors(i, k, neighbor_points);

		// projection_matrix 
		Mat projection_matrix;
		projection_matrix.identity();
		projection_matrix -= Mat(point_normal, point_normal);

		// collect projected normals 
		for (int j = 0; j < neighbor_points.size(); ++j) {
			// Project normals into the tangent plane
			Dir projected_normal = pc.nml(neighbor_points[j]) * projection_matrix; //  right multiply? 
			projected_normals.push_back(projected_normal);
		}

		cgv::pointcloud::pca3<float> analysis = cgv::pointcloud::pca3<float>(projected_normals.data(), k);
		auto* eigenvalues = analysis.eigen_values_ptr();
		auto* eigenvectors = analysis.eigen_vectors_ptr();
		//store the two largest eigenvalues as prinicpal curvature
		double kf = 1.0 / k;
		float curv1 = eigenvalues[0] * kf;
		float curv2 = eigenvalues[1] * kf;

		// save to structure 
		principal_curvature tmpcurva;
		tmpcurva.principal_curvature_vector = *eigenvectors; // derefernce to get a vec3, assign to a Dir, they have the same def.
		tmpcurva.kmax = curv1;//store the two largest eigenvalues as prinicpal curvature
		tmpcurva.kmin = curv2;
		pc.curvature.at(i) = tmpcurva; // pc.curvature already reserved, resized 

		/*
			auto* eigenvalues = analysis.eigen_values_ptr();
			auto* eigenvectors = analysis.eigen_vectors_ptr();
			//store the two largest eigenvalues as prinicpal curvature
			double kf = 1.0 / k;
			float curv1 = eigenvalues[0] * kf;
			float curv2 = eigenvalues[1] * kf;
			pce[i] = principal_curvature_estimation(curv1, curv2, eigenvectors[0], eigenvectors[1]);

			const float& principal_max, const float& principal_min, const Dir& principal_tangent_max, const Dir& principal_tangent_min
		*/
	}

}
/// and recolor at the same time, one can observe from colors 
void point_cloud_interactable::smooth_curvature_and_recolor() {
	compute_smoothed_curvature();
	apply_smoothed_curvature();
	compute_and_print_curvature_computing_info();
	auto_cluster_kmeans();
	colorize_with_computed_curvature_unsigned();
}
///
void point_cloud_interactable::compute_smoothed_curvature() {
	pc.smoothed_mean_curvature.resize(pc.get_nr_points());
	for (int i = 0; i < pc.get_nr_points(); i++) {
		pc.smoothed_mean_curvature.at(i) = 0;
		// compute for each point a new curvature 
		float weighted_average_curvature = 0;
		float sum_dist = 0;
		for (int j = 0; j < pc.nearest_neighbour_indices.at(i).size(); j++) { // 0 is the poitn itself 
			int neighbor_index = pc.nearest_neighbour_indices.at(i).at(j);
			float curr_dist = (pc.pnt(i) - pc.pnt(neighbor_index)).length();
			if (curr_dist > 0) { // float version of != 0
				curr_dist = 1.0f / curr_dist;
				weighted_average_curvature += 
					pc.curvature.at(neighbor_index).mean_curvature * curr_dist;
				sum_dist += curr_dist;
			}
		}
		if (sum_dist > 0)
			weighted_average_curvature = weighted_average_curvature / sum_dist;
		pc.smoothed_mean_curvature.at(i) = weighted_average_curvature;
	}
}
///
void point_cloud_interactable::apply_smoothed_curvature() {
	for (int i = 0; i < pc.get_nr_points(); i++) 
		pc.curvature.at(i).mean_curvature = pc.smoothed_mean_curvature.at(i);
}
/// recolor point cloud with curvature
void point_cloud_interactable::colorize_with_computed_curvature_unsigned() {

	// loop over to assign 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		//std::cout << "pc.curvature.at(i).gaussian_curvature: " << pc.curvature.at(i).gaussian_curvature << std::endl;
		if (pc.curvature.at(i).mean_curvature > pc.curvinfo.coloring_threshold)
			pc.clr(i) = rgb(102.0f / 255, 0, 102.0f / 255);
		else
			pc.clr(i) = rgb(153.0f / 255, 204.0f / 255, 255.0f / 255);
	}
}
// describes a surface defined by a quadratic equation with two parameters
struct quadric {
	using vec3 = cgv::math::fvec<float, 3>;

	/// coefficients for ax²+by²+c*xy+dx+ey
	double a, b, c, d, e;

	quadric() = default;
	quadric(double pa, double pb, double pc, double pd, double pe) : a(pa), b(pb), c(pc), d(pd), e(pe) {}

	double evaluate(double u, double v)
	{
		return a * u * u + b * u * v + c * v * v + d * u + e * v;
	}

	double du(const double u, const double v) {
		return 2.0 * a * u + b * v + d;
	}

	double duu(const double u, const double v) {
		return 2.0 * a;
	}

	double dv(const double u, const double v) {
		return 2.0 * c * v + b * u + e;
	}

	double dvv(const double u, const double v) {
		return 2.0 * c;
	}

	double duv(const double u, const double v) {
		return b;
	}

	static quadric fit(const std::vector<vec3>& pnts)
	{
		assert(pnts.size() >= 5);
		size_t num_pnts = pnts.size();
		cgv::math::mat<double> A(num_pnts, 5);
		cgv::math::mat<double> b(num_pnts, 1);
		cgv::math::mat<double> x(5, 1); //solution vector

		//copy points to A and b
		for (int c = 0; c < num_pnts; ++c)
		{
			double u = pnts[c][0];
			double v = pnts[c][1];
			double y = pnts[c][2];

			A(c, 0) = u * u;
			A(c, 1) = u * v;
			A(c, 2) = v * v;
			A(c, 3) = u;
			A(c, 4) = v;

			b(c, 0) = y;
		}

		//cgv::math::mat<double> U, V;
		//cgv::math::diag_mat<double> sigma;
		//cgv::math::svd(A, U, sigma, V, true);
		//solve for b

		auto pseudo_inverse_A = cgv::math::pseudo_inv(A);
		x = pseudo_inverse_A * b;
		return quadric(x(0, 0), x(1, 0), x(2, 0), x(3, 0), x(4, 0));
	}
};
///
void point_cloud_interactable::compute_principal_curvature_signed() {
	// ensure point structures and properties 
	ensure_tree_ds();
	ensure_neighbor_graph();
	pc.curvature.resize(pc.get_nr_points()); // ensure writting space 

	// define temp helper varibles 
	std::vector<point_cloud::point_cloud_types::Idx> neighbor_points;
	std::vector<point_cloud::point_cloud_types::Nml> projected_normals; // for each neighbor points 
	std::vector<point_cloud::point_cloud_types::Pnt> plane_projected_points;

	// loop over to compute 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		const Pnt& selected_point = pc.pnt(i);
		Mat projection_matrix; // point to plane projection
		Mat plane_reference_matrix; // point to plane projection

		//build quadratics
		auto point_quadtratics = cgv::math::mat<float>(5, k);
		tree_ds->extract_neighbors(i, k, neighbor_points);

		// project point into normal plane
		Dir& point_normal = pc.nml(i);
		projection_matrix.identity();
		projection_matrix -= Mat(point_normal, point_normal);

		Dir y_axis = cgv::math::normalize(projection_matrix * pc.pnt(neighbor_points[neighbor_points.size() - 1]));
		Dir x_axis = cgv::math::cross(y_axis, point_normal);
		plane_reference_matrix.set_row(0, x_axis);
		plane_reference_matrix.set_row(1, y_axis);
		plane_reference_matrix.set_row(2, point_normal);
		Mat inv_plane_reference_matrix = cgv::math::inv(plane_reference_matrix);

		// change reference system, z is distance to plane, and x,y span the tangent space
		plane_projected_points.clear();
		for (const Idx& point_idx : neighbor_points) {
			plane_projected_points.push_back(plane_reference_matrix * (pc.pnt(point_idx) - selected_point));
		}
		// fit quadric
		quadric q = quadric::fit(plane_projected_points);

		// compute first fundamental form of the quadric
		double inv_sqrt_ab = 1.0 / (sqrt(q.a * q.a + q.b * q.b + 1.0));
		double E = 1.0 + q.a * q.a;
		double F = q.a * q.b;
		double G = 1.0 + q.b * q.b;
		// compute secound fundamental form
		double L = (2.0 * q.c) * inv_sqrt_ab;
		double M = q.d * inv_sqrt_ab;
		double N = (2.0 * q.e) * inv_sqrt_ab;
		// reshape
		cgv::math::mat<double> FFF = cgv::math::mat<double>(2, 2);
		cgv::math::mat<double> SFF = cgv::math::mat<double>(2, 2);
		FFF(0, 0) = E;
		FFF(0, 1) = FFF(1, 0) = F;
		FFF(1, 1) = G;

		SFF(0, 0) = L;
		SFF(0, 1) = SFF(1, 0) = M;
		SFF(1, 1) = N;

		// calculate the shape operator of the quadric
		cgv::math::mat<double> S = -SFF * cgv::math::inv_22(FFF);

		// compute principal curvatures
		cgv::math::mat<double> U, V;
		cgv::math::diag_mat<double> s;
		cgv::math::svd(S, U, s, V, false);

		std::array<Dir, 2> eigen_vectors;
		// columns of U contain vectors spanning the eigenspace and sigma contains the eigen values
		for (int i = 0; i < 2; ++i) {
			eigen_vectors[i] = Dir(0.0, 0.0, 0.0);
			for (int j = 0; j < 2; ++j) {
				eigen_vectors[i](j) = (float)U(j, i);
			}
		}
		// transform back to world coordinates
		for (auto& v : eigen_vectors) {
			v = inv_plane_reference_matrix * v;
		}
		//evaluate q at point p
		if (q.duu(0, 0) < 0) {
			s[0] = -s[0];
		}
		if (q.dvv(0, 0) < 0) {
			s[1] = -s[1];
		}
		int min_curv_ix = (s[0] < s[1]) ? 0 : 1;
		int max_curv_ix = (s[0] < s[1]) ? 1 : 0;

		/*
			pce[i] = principal_curvature_estimation(s[max_curv_ix], s[min_curv_ix], eigen_vectors[max_curv_ix], eigen_vectors[min_curv_ix]);

			const float& principal_max, const float& principal_min, const Dir& principal_tangent_max, const Dir& principal_tangent_min
		*/

		// save to structure 
		principal_curvature tmpcurva;
		tmpcurva.principal_tangent_vectors_max = eigen_vectors[max_curv_ix]; // 0 is the max 
		tmpcurva.principal_tangent_vectors_min = eigen_vectors[min_curv_ix];
		tmpcurva.kmax = s[max_curv_ix];//store the two largest eigenvalues as prinicpal curvature
		tmpcurva.kmin = s[min_curv_ix];
		pc.curvature.at(i) = tmpcurva; // pc.curvature already reserved, resized 
	}

}
///
void point_cloud_interactable::fill_curvature_structure() {
	// loop over to compute 
	for (auto& c: pc.curvature) {
		c.gaussian_curvature = c.kmax * c.kmin;
		c.mean_curvature = (c.kmax + c.kmin) * 0.5f;
	}
}
/// recolor point cloud with curvature
void point_cloud_interactable::colorize_with_computed_curvature_signed() {
	// loop over to assign 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		//std::cout << "pc.curvature.at(i).gaussian_curvature: " << pc.curvature.at(i).gaussian_curvature << std::endl;
		if (pc.curvature.at(i).gaussian_curvature < 0)
			pc.clr(i) = rgb(1, 0, 0);
		if (pc.curvature.at(i).gaussian_curvature > 0)
			pc.clr(i) = rgb(0, 0, 1);
		if (abs(pc.curvature.at(i).gaussian_curvature) < 1e-6) // == 0, higher priority, overwrite 
			pc.clr(i) = rgb(0, 1, 0);
	}
}
///
void point_cloud_interactable::compute_and_print_curvature_computing_info() {
	pc.curvinfo.minimum_curvature_difference = std::numeric_limits<float>::max();
	pc.curvinfo.max_mean_curvature = std::numeric_limits<float>::min();
	pc.curvinfo.min_mean_curvature = std::numeric_limits<float>::max();
	for (int i = 0; i < pc.get_nr_points(); i++) {
		if (pc.curvature.at(i).mean_curvature > pc.curvinfo.max_mean_curvature)
			pc.curvinfo.max_mean_curvature = pc.curvature.at(i).mean_curvature;
		if (pc.curvature.at(i).mean_curvature < pc.curvinfo.min_mean_curvature) 
			pc.curvinfo.min_mean_curvature = pc.curvature.at(i).mean_curvature;
	}

	for (int i = 0; i < pc.get_nr_points(); i++) {
		float curr_curvatire = pc.curvature.at(i).mean_curvature;
		for (int j = 0; j < pc.get_nr_points() && (j != i); j++) {
			float compairing_curvature = pc.curvature.at(j).mean_curvature;
			float curr_diff = abs(curr_curvatire - compairing_curvature);
			if (curr_diff < pc.curvinfo.minimum_curvature_difference && curr_diff>0)
				pc.curvinfo.minimum_curvature_difference = curr_diff;
		}
	}

	pc.curvinfo.max_gaussian_curvature = std::numeric_limits<float>::min();
	pc.curvinfo.min_gaussian_curvature = std::numeric_limits<float>::max();
	for (int i = 0; i < pc.get_nr_points(); i++) {
		if (pc.curvature.at(i).gaussian_curvature > pc.curvinfo.max_gaussian_curvature)
			pc.curvinfo.max_gaussian_curvature = pc.curvature.at(i).gaussian_curvature;
		if (pc.curvature.at(i).gaussian_curvature < pc.curvinfo.min_gaussian_curvature)
			pc.curvinfo.min_gaussian_curvature = pc.curvature.at(i).gaussian_curvature;
	}

	std::cout << "pc.curvinfo.curvinfo.minimum_curvature_difference = " << pc.curvinfo.minimum_curvature_difference << std::endl;
	std::cout << "pc.curvinfo.max_mean_curvature = " << pc.curvinfo.max_mean_curvature << std::endl;
	std::cout << "pc.curvinfo.min_mean_curvature = " << pc.curvinfo.min_mean_curvature << std::endl;
	std::cout << "pc.curvinfo.max_gaussian_curvature = " << pc.curvinfo.max_gaussian_curvature << std::endl;
	std::cout << "pc.curvinfo.min_gaussian_curvature = " << pc.curvinfo.min_gaussian_curvature << std::endl;
}
/*
	algorithm:
		a. assign initial centroid A and centroid B as maximal and minimal value 
		b. loop over all points, compute and assign to nearest clusters 
		c. recompute centers
		d. if centers does not move any more (compared to last iteration, with a threshold), done
		f. else, goto b.

*/
/// just for visualize, computing is no problem now
/// goal: find a good threshold
void point_cloud_interactable::auto_cluster_kmeans() {
	// init centroids
	float centroid_A = pc.curvinfo.max_mean_curvature;
	float centroid_B = pc.curvinfo.min_mean_curvature;

	std::vector<int> points_belongs_to_A;
	std::vector<int> points_belongs_to_B;

	int ounter = 0;
	std::cout << "auto_cluster_kmeans: starts" << std::endl;
	while (true) {
		points_belongs_to_A.clear();
		points_belongs_to_B.clear();

		// try classify, either A or B 
		for (int i = 0; i < pc.get_nr_points(); i++) {
			if (abs(pc.curvature.at(i).mean_curvature - centroid_A) < abs(pc.curvature.at(i).mean_curvature - centroid_B)) {
				points_belongs_to_A.push_back(i);
			}
			else {
				points_belongs_to_B.push_back(i);
			}
		}

		// compute new center 
		float new_centroid_A = 0;
		for (auto& pid_A : points_belongs_to_A) {
			if (!isnan(pc.curvature.at(pid_A).mean_curvature))
				new_centroid_A += pc.curvature.at(pid_A).mean_curvature;
		}
		assert(points_belongs_to_A.size() > 0);
		new_centroid_A = new_centroid_A / points_belongs_to_A.size();

		float new_centroid_B = 0;
		for (auto& pid_B : points_belongs_to_B) {
			if(!isnan(pc.curvature.at(pid_B).mean_curvature))
				new_centroid_B += pc.curvature.at(pid_B).mean_curvature;
		}
		assert(points_belongs_to_B.size() > 0);
		new_centroid_B = new_centroid_B / points_belongs_to_B.size();


		// break when we can not go any further 
		if ((abs(new_centroid_A - centroid_A)<1e-6) && (abs(new_centroid_B - centroid_B) < 1e-6)) {
			break;
		}

		std::cout << "centroid_A " << centroid_A << std::endl;
		std::cout << "new_centroid_A " << new_centroid_A << std::endl;
		std::cout << "centroid_B " << centroid_B << std::endl;
		std::cout << "new_centroid_B " << new_centroid_B << std::endl;

		// update centroid_A
		centroid_A = new_centroid_A;
		centroid_B = new_centroid_B;
	}
	std::cout << "auto_cluster_kmeans: done" << std::endl;
	float max_value_in_B = std::numeric_limits<float>::min();
	for (auto& pid_b: points_belongs_to_B) {
		if (pc.curvature.at(pid_b).mean_curvature > max_value_in_B) {
			max_value_in_B = pc.curvature.at(pid_b).mean_curvature;
		}
	}
	pc.curvinfo.coloring_threshold = max_value_in_B;
	std::cout << "threshold selected by kmeans: " << pc.curvinfo.coloring_threshold << std::endl;
	pc.has_curv_information = true;
}
/// the entry point 
void point_cloud_interactable::ep_compute_principal_curvature_and_colorize_signed() {
	compute_principal_curvature_signed();
	fill_curvature_structure();
	compute_and_print_curvature_computing_info();
	colorize_with_computed_curvature_signed(); // vis with gaussian_curvature
}
/// the entry point 
void point_cloud_interactable::ep_compute_principal_curvature_and_colorize_unsigned() {
	std::cout << "computing principal curvature..." << std::endl;
	compute_principal_curvature_unsigned();
	fill_curvature_structure();
	smooth_curvature_and_recolor();
	//print_curvature_computing_info();
	//auto_cluster_kmeans();
	//colorize_with_computed_curvature_unsigned(); 
	std::cout << "done!" << std::endl;
}
///
void point_cloud_interactable::ep_force_recolor() {
	colorize_with_computed_curvature_unsigned();
}

