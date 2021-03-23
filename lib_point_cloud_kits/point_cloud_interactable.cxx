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

#define FILE_OPEN_TITLE "Open Point Cloud"
#define FILE_APPEND_TITLE "Append Point Cloud"
#define FILE_OPEN_FILTER "Point Clouds (apc,bpc):*.apc;*.bpc|Mesh Files (obj,ply,pct):*.obj;*.ply;*.pct|All Files:*.*"

#define FILE_SAVE_TITLE "Save Point Cloud"
#define FILE_SAVE_FILTER "Point Clouds (apc,bpc):*.apc;*.bpc|Mesh Files (obj,ply):*.obj;*.ply|All Files:*.*"

void point_cloud_interactable::update_file_name(const std::string& ffn, bool append)
{
	std::string new_path = cgv::utils::file::get_path(ffn);
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
	update_member(&new_file_name);
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
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	//update_file_name(fn);
	store_original_pc();
	//auto_downsampling();
	return true;
}
/// procedure point cloud generation, a hemisphere
bool point_cloud_interactable::generate_pc_hemisphere() {
	pc.clear_all();
	int samples_per_row = 200;
	int nr_rows = 400;
	Pnt extent = Pnt(1, 1, 1);
	Rgba c = Rgba(0.5, 0.5, 0.5, 1);
	float R = 1.0f;
	Pnt offset = Pnt(0, 1, 0);
	pc.clear();
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
			Idx pi = pc.add_point(extent * p, c);
			//pc.nml(pi) = normalize(n/extent);
		}
	}
	on_point_cloud_change_callback(PCC_NEW_POINT_CLOUD);
	return true;
}
/// procedure point cloud generation, a cube 
bool point_cloud_interactable::generate_pc_cube() {
	pc.clear_all();
	int samples_per_row = 200;
	int nr_rows = 400;
	vec3 extent = vec3(1, 1, 1);
	rgba c = rgba(0.5, 0.5, 0.5, 1);
	pc.clear();
	pc.create_colors();
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
						pc.add_point(extent * p, c);
					}
				}
				else {
					p = vec3(x, 0.25f, y);
					pc.add_point(extent * p, c);
				}
			}
			else {
				p = vec3(x, 0, y);
				pc.add_point(extent * p, c);
			}
		}
	}
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
	std::string f = cgv::gui::file_open_dialog("Open", "Point Cloud:*");
	if (!append) 
		clear_all();
	open(f);
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
	pc.write(f);
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
	bool succ = pc.cam_posi.x() != -1000
				&& pc.cam_posi.y() != -1000
				&& pc.cam_posi.z() != -1000
				&& pc.get_nr_points() == pc.num_of_points_in_campose
				&& pc.has_cam_posi;
	if (!succ) {
		std::cout << "invalid! check following vars:" << std::endl;
		std::cout << "pc.cam_posi: " << pc.cam_posi << std::endl;
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
void point_cloud_interactable::prepare_marking(std::vector<rgba>* psc) {
	pc.point_selection.resize(pc.get_nr_points());
	for (auto& v : pc.point_selection) v = 0;
	pc.point_selection_visited.resize(pc.get_nr_points());
	for (auto& w : pc.point_selection_visited) w = false;

	use_these_point_palette = psc;
	use_these_point_color_indices = &pc.point_selection;

}

///
void point_cloud_interactable::new_history_recording() {
	std::vector<pointHistoryEntry> current_selection;
	point_marking_history.push_back(current_selection);
}

///
void point_cloud_interactable::reset_last_marking_non_processed_part(int which_is_marked_and_not_used) {
	// iterate all point indices recorded last time 
	for (int i = 0; i < point_marking_history.back().size(); i++) {
		// reset if not changed marking, that is, which_is_marked_and_not_used
		// current point index: point_marking_history.back().at(i).point_index
		// current selection: pc.point_selection.at(point_marking_history.back().at(i))
		if (pc.point_selection.at(point_marking_history.back().at(i).point_index) == (int)which_is_marked_and_not_used) {
			pc.point_selection.at(point_marking_history.back().at(i).point_index) 
				= point_cloud::PointSelectiveAttribute::ORI;
		}
	}
}

/// not finished -> to be tested
void point_cloud_interactable::reset_last_marked_points() {
	// a list of current changing: point_marking_history.back()
	// element, .at(i), is pointHistoryEntry
	// set the value to current selection bundle -> to_	
	// step too much 
	if (history_indexer > (point_marking_history.size() - 1))
		return;
	for (int i = 0; i < point_marking_history.at(point_marking_history.size() - 1 - history_indexer).size(); i++) {
		pc.point_selection.at(point_marking_history.at(point_marking_history.size() - history_indexer - 1)
			.at(i).point_index)
			= point_marking_history.at(point_marking_history.size() - 1 - history_indexer).at(i).from_selection;
	}
	// step back virtually
	history_indexer++;
}

// errors may occour when step too much 
void point_cloud_interactable::step_back_last_selection() {
	reset_last_marked_points();
}

// not finished -> to be tested
// errors may occour when step too much 
void point_cloud_interactable::step_forward_selection() {
	// step too much 
	if (history_indexer < 1)
		return;
	// set the value to current selection bundle -> to_
	for (int i = 0; i < point_marking_history.at(point_marking_history.size() - history_indexer - 1).size(); i++) {
		pc.point_selection.at(point_marking_history.at(point_marking_history.size() - history_indexer - 1)
			.at(i).point_index)
			= point_marking_history.at(point_marking_history.size() - history_indexer - 1).at(i).to_selection;
	}
	// step next 
	history_indexer--;
}

///
void point_cloud_interactable::mark_all_points_and_push_to_tmp_pointcloud_test(Pnt p, float r, int ignore_id = -1) {
	bool some_points_copied = false;
	float scan_index = 0;
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		//if ((pc.pnt(i) - p).length() < r) {
			if (pc.point_selection.at(i) != ignore_id) {
				if (pc.point_scan_index.size() > 0)
					scan_index = pc.point_scan_index.at(i);
				else
					scan_index = 0;
				to_be_copied_pointcloud.add_point(pc.pnt(i), pc.clr(i), pc.nml(i), scan_index, pc.point_selection.at(i));
				some_points_copied = true;
			}
		//}
	}
	if (some_points_copied) {
		// require properties 
		to_be_copied_pointcloud.create_colors();
		to_be_copied_pointcloud.create_normals();
		to_be_copied_pointcloud.has_scan_index = true;
		to_be_copied_pointcloud.has_selection = true;
		to_be_copied_pointcloud.box_out_of_date = true;
	}
}

///
void point_cloud_interactable::mark_points_and_push_to_tmp_pointcloud(Pnt p, float r, int ignore_id = -1) {
	bool some_points_copied = false;
	float scan_index = 0;
	for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
		if ((pc.pnt(i) - p).length() < r) {
			if (pc.point_selection.at(i) != ignore_id) {
				if (pc.point_scan_index.size() > 0)
					scan_index = pc.point_scan_index.at(i);
				else
					scan_index = 0;
				to_be_copied_pointcloud.add_point(pc.pnt(i), pc.clr(i), pc.nml(i), scan_index, pc.point_selection.at(i));
				some_points_copied = true;
			}
		}
	}
	if (some_points_copied) {
		// require properties 
		to_be_copied_pointcloud.create_colors();
		to_be_copied_pointcloud.create_normals();
		to_be_copied_pointcloud.has_scan_index = true;
		to_be_copied_pointcloud.has_selection = true;
		to_be_copied_pointcloud.box_out_of_date = true;
	}
}

/// currently, we set confirmed to true. The visual feedback is better to be impl. in shaders.
void point_cloud_interactable::mark_points_with_conroller(Pnt p, float r, bool confirmed, int objctive,int ignore_id) {
	new_history_recording();
	if (pc.get_nr_points()) {
		ensure_tree_ds();
		float closest_dist = tree_ds->find_closest_and_its_dist(p);
		//std::cout << "closest_dist = "<< closest_dist << std::endl;
		if (closest_dist > r) {
			if (marked == true) {
				for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
					if (pc.point_selection.at(i) == point_cloud::PointSelectiveAttribute::VISUAL_MARK)
						pc.point_selection.at(i) = point_cloud::PointSelectiveAttribute::ORI;
				}
				marked = false;
			}
		}
		else {
			//std::cout << "some points inside the sphere!" << std::endl;
			Box pc_bbox = pc.box();
			float vol_pc = pc_bbox.get_extent().x() * pc_bbox.get_extent().y() * pc_bbox.get_extent().z();
			int max_points_estimated = pc.get_nr_points() * pow((2 * r), 3) / vol_pc;
			std::vector<int> knn;
			std::vector<float> dist_list;
			tree_ds->find_closest_points(p, max_points_estimated, knn, dist_list);
			// if the last point is in outside of the ball on hand, 
			// all wanted points are included
			if (dist_list.at(dist_list.size() - 1) > r) {
				// start at minimal dist 
				for (int i = 0; i < knn.size(); i++) {
					// check if is smaller than r 
					if (dist_list.at(i) < r) {
						if (pc.point_selection.at(knn.at(i)) != ignore_id) {
							if (confirmed) {
									// record tracing information
									pointHistoryEntry phe;
									phe.point_index = knn.at(i);
									phe.from_selection = pc.point_selection.at(knn.at(i));
									phe.to_selection = objctive;
									point_marking_history.back().push_back(phe);

									// perfrom 
									pc.point_selection.at(knn.at(i)) = objctive;
							}
							else {
								pc.point_selection.at(knn.at(i)) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
								marked = true;
							}
							pc.has_selection = true;
						}
					}
				}
			}
			else {
				// too few points are estimated, iter the entire cloud 
				for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
					if ((pc.pnt(i) - p).length() < r) {
						if (pc.point_selection.at(i) != ignore_id) {
							if (confirmed) {
								// record tracing information
								pointHistoryEntry phe;
								phe.point_index = i;
								phe.from_selection = pc.point_selection.at(i);
								phe.to_selection = objctive;
								point_marking_history.back().push_back(phe);
								// perform real operations 
								pc.point_selection.at(i) = objctive;
							}
							else {
								pc.point_selection.at(i) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
								marked = true;
							}
							pc.has_selection = true;
						}
					}
				}
			}
		}
		on_point_cloud_change_callback(PCC_COLORS);
		//post_redraw();
	}

	// only one point will be marked as seed 
	//for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
	//	if ((pc.pnt(i) - p).length() < r) {
	//		pc.point_selection.at(i) = objctive;
	//		return;
	//	}
	//}
	//on_point_cloud_change_callback(PCC_COLORS);
}
/// mark_all_points_as_given_group, still, you have to prepare first, typically done in reading process
void point_cloud_interactable::marking_test_mark_all_points_as_given_group(int objective) {
	for (int i = 0; i < pc.get_nr_points(); i++) {
		// int -> uint8
		pc.point_selection[i] = (cgv::type::uint8_type)objective;
	}
}

/*operations after marked */
/// 
void point_cloud_interactable::selective_subsampling_cpu() {
	std::default_random_engine g;
	std::uniform_real_distribution<float> d(0, 1);
	// iterate all points 
	for (int i = 0; i < pc.get_nr_points(); i++) {
		// if marked as TO_BE_SUBSAMPLED 
		if (point_cloud::PointSelectiveAttribute::TO_BE_SUBSAMPLED == pc.point_selection[i]) {
			// if random condition fullfilled, the larger the radio, more points will be deleted
			if (d(g) < selective_subsampling_radio) {
				// mark it as deleted 
				pc.point_selection[i] = point_cloud::PointSelectiveAttribute::DEL;
			}
		}
	}
}

/*point addition */
void point_cloud_interactable::spawn_points_in_the_handhold_quad(quat controllerq, vec3 controllert, vec3 quadextent) {
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
		cgv::type::uint8_type newSelectionIndex = point_cloud::PointSelectiveAttribute::NEWLY_GENERATED;

		// add to pc 
		pc.add_point(newP, newC, newN, newScanIndex, newSelectionIndex);
	}

	// require properties 
	pc.create_colors();
	pc.create_normals();
	pc.has_scan_index = true;
	pc.has_selection = true;
	pc.box_out_of_date = true;

	// rebuild ann tree, managing rendering range..., crutial task when point cloud changes
	on_point_cloud_change_callback(PCC_POINTS_RESIZE);
}

/*region growing after marked*/
/// max_num_regions is not used 
void point_cloud_interactable::prepare_grow(bool read_from_file, std::vector<rgba>* psc, int max_num_regions) {
	// atomic operation 
	can_parallel_grow = false;

	// if no selection present, clear point_selection
	if (!(read_from_file && pc.has_selection)) {
		pc.point_selection.resize(pc.get_nr_points());
		// initialize to 1
		for (auto& v : pc.point_selection) v = 1;
		pc.has_selection = true;
	}

	// reset rendering properties 
	show_nmls = false;
	use_these_point_palette = psc;
	use_these_point_color_indices = &pc.point_selection;

	// reset region growing properties 
	pc.point_selection_visited.resize(pc.get_nr_points());
	for (auto& v : pc.point_selection_visited) v = false;

	// replaced by functions 
	region_id_and_seeds.clear();
	region_id_and_nmls.clear();
	std::queue<int> empty_indexqueue;
	std::queue<vec3> empty_nmlqueue;
	region_id_and_seeds.resize(pc.max_num_of_selections, empty_indexqueue);
	region_id_and_nmls.resize(pc.max_num_of_selections, empty_nmlqueue);

	// enable again  
	can_parallel_grow = true;
}
/// reset region growing regions 
void point_cloud_interactable::reset_all_grows() {
	// todo 
}
/// reset the seed lists 
void point_cloud_interactable::reset_region_growing_seeds() {
	region_id_and_seeds.clear();
	region_id_and_nmls.clear();
	std::queue<int> empty_indexqueue;
	std::queue<vec3> empty_nmlqueue;
	region_id_and_seeds.resize(pc.max_num_of_selections, empty_indexqueue);
	region_id_and_nmls.resize(pc.max_num_of_selections, empty_nmlqueue);
}
/// push to queue operation, as an init to RG 
void point_cloud_interactable::init_region_growing_by_collecting_group_and_seeds_vr(int current_selecting_idx) {
	// old version:
		//for (int gi = pc.num_of_functional_selections; gi< pc.max_num_of_selections; gi++) {
		//	for (int idx = 0; idx < pc.get_nr_points(); idx++) {
		//		if (pc.point_selection.at(idx) == gi) {
		//			region_id_and_seeds[gi].push(idx);
		//		}
		//	}
		//	region_id_and_nml[gi] = pc.nml(region_id_and_seeds[gi].front()); // the first nml, ranked in index order
		//}
	// collect all related idx and push to queue, ready to grow after this 
	// only region selection are accepted
	// unsigned int shoud cast to int! -> only when comparing bet. them 
	for (int pi = 0; pi < pc.get_nr_points(); pi++) {
		if ((int)pc.point_selection.at(pi) == current_selecting_idx) {
			// which group? which point?
			region_id_and_seeds[pc.point_selection.at(pi)].emplace(pi);
			region_id_and_nmls[pc.point_selection.at(pi)].emplace(pc.nml(pi));
			pc.point_selection_visited[pi] = true;
		}
	}
}
/// init growing group
void point_cloud_interactable::init_region_growing_by_setting_group_and_seeds(int growing_group, std::queue<int> picked_id_list) {
	//region_id_and_seeds[growing_group] = picked_id_list;
	////region_id_and_seeds.insert(std::make_pair(growing_group, picked_id_list));
	//// use the first seed's normal for test 
	//region_id_and_nml[growing_group] = pc.nml(picked_id_list.front());
	//pc.has_selection = true;
}
/// perform region grow in timer event 
void point_cloud_interactable::do_region_growing_timer_event(double t, double dt) {
	//if (!can_sleep && do_region_growing_directly) {
	//	int i = 0;
	//	bool can_not_sleep = false;
	//	while (i < steps_per_event_as_speed) {
	//		// grow marked regions, not functional ones 
	//		for (int gi = pc.num_of_functional_selections; gi < pc.max_num_of_selections; gi++)
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
/// one step growing with bfs 
bool point_cloud_interactable::grow_one_step_bfs(bool check_nml, int which_group) {
	// bfs, simple approach, do not update normal currently 
	if (pc.get_nr_points() == 0)
		return false;
	if (pc.N.size() == 0)
		return false;
	if (region_id_and_seeds.size() == 0)
		return false;

	std::vector<int> knn;
	ensure_tree_ds();
	if (region_id_and_seeds[which_group].size()) {
		int to_be_visit = region_id_and_seeds[which_group].front();
		region_id_and_seeds[which_group].pop();
		// 3 * 8 + 2 = 26
		if (to_be_visit > 0 && to_be_visit < pc.get_nr_points()) {
			tree_ds->find_closest_points(pc.pnt(to_be_visit), 8, knn);
			for (auto k : knn) {
				if (!pc.point_selection_visited.at(k)) {
					// compare nml
					vec3 cur_k_nml = pc.nml(k);
					//std::cout << "dot of nmls: " << dot(cur_nml, cur_k_nml) << std::endl;
					// compare curvature
					if (check_nml) {
						bool pass_nml_check = false;
						// iterate over the queue to check 
						std::queue<vec3> tmp_q = region_id_and_nmls[which_group];
						while (!tmp_q.empty()) {
							if (dot(tmp_q.front(), cur_k_nml) > 0.97) {
								pass_nml_check = true;
								break;
							}
							tmp_q.pop();
						}
						/*if (dot(region_id_and_nmls[which_group].front(), cur_k_nml) > 0.97) {
							pass_nml_check = true;
						}*/
						// if pass check, update and mark as new seed 
						if (pass_nml_check) {
							// update 
							pc.point_selection.at(k) = which_group;
							pc.point_selection_visited.at(k) = true;
							region_id_and_seeds[which_group].emplace(k);
						}
					}
					else {
						// update 
						pc.point_selection.at(k) = which_group;
						pc.point_selection_visited.at(k) = true;
						region_id_and_seeds[which_group].emplace(k);
					}
					//std::cout << "is growing" << std::endl;
				}
			}
		}
		return true;
	}
	else {
		//std::cout << "seed empty, stop." << std::endl;
		return false;
	}
}
/// check if all points growed 
bool point_cloud_interactable::all_points_growed() {
	for (auto ps : pc.point_selection) {
		if (ps == 0)
			return false;
	}
	return true;
}
/// ensure all points to be growed 
void point_cloud_interactable::grow_one_step_bfs_ensure() {
	//// bfs simple approach
	//std::vector<int> knn;

	//if (seeds.size()) {
	//	int to_be_visit = seeds.front();
	//	seeds.pop();
	//	// extract normal
	//	vec3 cur_nml = pc.nml(to_be_visit);
	//	int num_of_real_knn = 0;
	//	int increm_search = 0; // search 5 points at least 
	//	// ensure the num of not visited points, used to break boundaries 
	//	while (num_of_real_knn < 4 && !all_points_growed()) {
	//		increm_search += 4 - num_of_real_knn;
	//		tree_ds->find_closest_points(pc.pnt(to_be_visit), 1 + increm_search, knn);
	//		// check 
	//		// reset num_of_real_knn
	//		num_of_real_knn = 0;
	//		for (auto k : knn) {
	//			if (!pc.point_selection_visited.at(k)) {
	//				num_of_real_knn++;
	//			}
	//		}
	//		if (num_of_real_knn < 4) {
	//			//std::cout << "failed, retry, num_of_real_knn = " << num_of_real_knn << std::endl;
	//		}
	//	}
	//	//
	//	if (num_of_real_knn >= 4) {
	//		for (auto k : knn) {
	//			// compare nml
	//			// compare curvature
	//			// update 
	//			pc.point_selection.at(k) = 1;
	//			pc.point_selection_visited.at(k) = true;
	//			seeds.push(k);
	//			//std::cout << "is growing" << std::endl;
	//		}
	//	}
	//	post_redraw();
	//}
}
/// for now, we only set confirmed to true 
void point_cloud_interactable::mark_points_inside_selection_tool(Pnt p, float r, bool confirmed, int objctive)
{
	if (pc.get_nr_points()) {
		ensure_tree_ds();
		float closest_dist = tree_ds->find_closest_and_its_dist(p);
		//std::cout << "closest_dist = "<< closest_dist << std::endl;
		// restore the color marked visually, this is better to be done with gpu
		if (closest_dist > r) {
			if (marked == true){
				for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
					if (pc.point_selection.at(i) == point_cloud::PointSelectiveAttribute::VISUAL_MARK)
						pc.point_selection.at(i) = point_cloud::PointSelectiveAttribute::ORI;
				}
				marked = false;
			}
		}
		else {
			//std::cout << "some points inside the sphere!" << std::endl;
			Box pc_bbox = pc.box();
			float vol_pc = pc_bbox.get_extent().x() * pc_bbox.get_extent().y() * pc_bbox.get_extent().z();
			int max_points_estimated = pc.get_nr_points() *pow((2 * r),3) / vol_pc;
			std::vector<int> knn;
			std::vector<float> dist_list;
			tree_ds->find_closest_points(p, max_points_estimated, knn, dist_list);
			// if the last point is in outside of the ball on hand, 
			// all wanted points are included
			if (dist_list.at(dist_list.size() - 1) > r) {
				// start at minimal dist 
				for (int i = 0; i < knn.size(); i++) {
					// check if is smaller than r 
					if (dist_list.at(i) < r) {
						if(confirmed)
							pc.point_selection.at(knn.at(i)) = objctive;
						else {
							pc.point_selection.at(knn.at(i)) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
							marked = true;
						}
						pc.has_selection = true;
					}
				}
			}
			else {
				// too few points are estimated, iter the entire cloud 
				for (Idx i = 0; i < (Idx)pc.get_nr_points(); ++i) {
					if ((pc.pnt(i) - p).length() < r) {
						if (confirmed)
							pc.point_selection.at(i) = objctive;
						else {
							pc.point_selection.at(i) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
							marked = true;
						}
						pc.has_selection = true;
					}
				}
			}
			
		}
	}
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
	if (pc.get_nr_points()) {
		if (!pc.point_selection.size() ||
			pc.point_selection.size() != pc.get_nr_points())
		{
			pc.point_selection.resize(pc.get_nr_points());
			for (auto& v : pc.point_selection) {
				v = point_cloud::PointSelectiveAttribute::ORI;
			}
		}
	}
}
/// inner function 
void point_cloud_interactable::ensure_point_selection() {
	if (pc_last.get_nr_points()) {
		if (!pc_last.point_selection.size()|| 
			pc_last.point_selection.size()!= pc_last.get_nr_points()) 
		{
			pc_last.point_selection.resize(pc_last.get_nr_points());
			for (auto& v : pc_last.point_selection) {
				v = point_cloud::PointSelectiveAttribute::ORI;
			}
		}
	}
	if (pc_to_be_append.get_nr_points()) {
		if (!pc_to_be_append.point_selection.size() || 
			pc_to_be_append.point_selection.size() != pc_to_be_append.get_nr_points()) 
		{
			pc_to_be_append.point_selection.resize(pc_to_be_append.get_nr_points());
			for (auto& v : pc_to_be_append.point_selection) {
				v = point_cloud::PointSelectiveAttribute::ORI;
			}
		}
	}
}
/// subsampled_target will be computed acc to p, r information 
void point_cloud_interactable::subsampling_target(
	Pnt& p, float& r, bool confirmed) {
	if (pc_last.get_nr_points() && tree_ds_target_pc_last_frame) {
		//ensure_tree_ds(); this is only for pc 
		ensure_point_selection();
		float closest_dist = tree_ds_target_pc_last_frame->find_closest_and_its_dist(p);
		//std::cout << "closest_dist = "<< closest_dist << std::endl;
		if (closest_dist > r) {
			if (marked == true) {
				for (Idx i = 0; i < (Idx)pc_last.get_nr_points(); ++i) {
					if (pc_last.point_selection.at(i) == point_cloud::PointSelectiveAttribute::VISUAL_MARK)
						pc_last.point_selection.at(i) = point_cloud::PointSelectiveAttribute::ORI;
				}
				marked = false;
			}
		}
		else {
			//std::cout << "some points inside the sphere!" << std::endl;
			Box pc_bbox = pc_last.box();
			float vol_pc = pc_bbox.get_extent().x() * pc_bbox.get_extent().y() * pc_bbox.get_extent().z();
			int max_points_estimated = pc_last.get_nr_points() * pow((2 * r), 3) / vol_pc;
			std::vector<int> knn;
			std::vector<float> dist_list;
			tree_ds_target_pc_last_frame->find_closest_points(p, max_points_estimated, knn, dist_list);
			// if the last point is in outside of the ball on hand, 
			// all wanted points are included
			if (dist_list.at(dist_list.size() - 1) > r) {
				// start at minimal dist 
				for (int i = 0; i < knn.size(); i++) {
					// check if is smaller than r 
					if (dist_list.at(i) < r) {
						if (confirmed)
							pc_last.point_selection.at(knn.at(i)) = point_cloud::PointSelectiveAttribute::ICP_TARGET;
						else {
							pc_last.point_selection.at(knn.at(i)) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
							marked = true;
						}
						pc_last.has_selection = true;
					}
				}
			}
			else {
				// too few points are estimated, iter the entire cloud 
				for (Idx i = 0; i < (Idx)pc_last.get_nr_points(); ++i) {
					if ((pc_last.pnt(i) - p).length() < r) {
						if (confirmed)
							pc_last.point_selection.at(i) = point_cloud::PointSelectiveAttribute::ICP_TARGET;
						else {
							pc_last.point_selection.at(i) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
							marked = true;
						}
						pc_last.has_selection = true;
					}
				}
			}

		}
	}
}
/// subsample the source point cloud 
void point_cloud_interactable::subsampling_source(
	Pnt& p, float& r, bool confirmed) {
	//subsampled_source, pc changed, treeds changed, obj changed 
	if (pc_to_be_append.get_nr_points() && tree_ds_source_pc) {
		//ensure_tree_ds(); this is only for pc 
		ensure_point_selection();
		float closest_dist = tree_ds_source_pc->find_closest_and_its_dist(p);
		//std::cout << "closest_dist = "<< closest_dist << std::endl;
		if (closest_dist > r) {
			if (marked == true) {
				for (Idx i = 0; i < (Idx)pc_to_be_append.get_nr_points(); ++i) {
					if (pc_to_be_append.point_selection.at(i) == point_cloud::PointSelectiveAttribute::VISUAL_MARK)
						pc_to_be_append.point_selection.at(i) = point_cloud::PointSelectiveAttribute::ORI;
				}
				marked = false;
			}
		}
		else {
			//std::cout << "some points inside the sphere!" << std::endl;
			Box pc_bbox = pc_to_be_append.box();
			float vol_pc = pc_bbox.get_extent().x() * pc_bbox.get_extent().y() * pc_bbox.get_extent().z();
			int max_points_estimated = pc_to_be_append.get_nr_points() * pow((2 * r), 3) / vol_pc;
			std::vector<int> knn;
			std::vector<float> dist_list;
			tree_ds_source_pc->find_closest_points(p, max_points_estimated, knn, dist_list);
			// if the last point is in outside of the ball on hand, 
			// all wanted points are included
			if (dist_list.at(dist_list.size() - 1) > r) {
				// start at minimal dist 
				for (int i = 0; i < knn.size(); i++) {
					// check if is smaller than r 
					if (dist_list.at(i) < r) {
						if (confirmed)
							pc_to_be_append.point_selection.at(knn.at(i)) = point_cloud::PointSelectiveAttribute::ICP_SOURCE;
						else {
							pc_to_be_append.point_selection.at(knn.at(i)) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
							marked = true;
						}
						pc_to_be_append.has_selection = true;
					}
				}
			}
			else {
				// too few points are estimated, iter the entire cloud 
				for (Idx i = 0; i < (Idx)pc_to_be_append.get_nr_points(); ++i) {
					if ((pc_to_be_append.pnt(i) - p).length() < r) {
						if (confirmed)
							pc_to_be_append.point_selection.at(i) = point_cloud::PointSelectiveAttribute::ICP_SOURCE;
						else {
							pc_to_be_append.point_selection.at(i) = point_cloud::PointSelectiveAttribute::VISUAL_MARK;
							marked = true;
						}
						pc_to_be_append.has_selection = true;
					}
				}
			}

		}
	}
}
/// 
void point_cloud_interactable::collect_to_subsampled_pcs() {
	pc_last_subsampled.clear_all();
	pc_to_be_append_subsampled.clear_all();
	for (int i = 0; i < pc_last.get_nr_points(); i++) {
		if (pc_last.point_selection.at(i) == point_cloud::PointSelectiveAttribute::ICP_TARGET) {
			pc_last_subsampled.add_point_subsampling(pc_last.pnt(i), pc_last.nml(i));
		}
	}
	for (int i = 0; i < pc_to_be_append.get_nr_points(); i++) {
		if (pc_to_be_append.point_selection.at(i) == point_cloud::PointSelectiveAttribute::ICP_SOURCE) {
			pc_to_be_append_subsampled.add_point_subsampling(pc_to_be_append.pnt(i), pc_to_be_append.nml(i));
		}
	}
	
}
///
void point_cloud_interactable::highlight_last_pc() {
	if (num_of_pcs > 0) {
		tmppc = pc;
		pc = pc_last;
		ensure_point_selection();
		for (auto& v : pc_last.point_selection) {
			v = point_cloud::PointSelectiveAttribute::ICP_TARGET_HIGHLIGHT;
		}
		use_these_point_color_indices = &pc_last.point_selection;
		is_highlighting = true;
	}
	for (auto& v : pc_to_be_append.point_selection) {
		v = point_cloud::PointSelectiveAttribute::ICP_SOURCE_HIGHLIGHT;
	}
}
///
void point_cloud_interactable::reset_highlighted_last_pc() {
	pc = tmppc;
	ensure_point_selection_pc();
	for (auto& v : pc_last.point_selection) {
		v = point_cloud::PointSelectiveAttribute::ORI;
	}
	use_these_point_color_indices = &pc.point_selection;
	is_highlighting = false;
}
/// quick test 
void point_cloud_interactable::fill_subsampled_pcs_with_cur_pc_as_a_test() {
	pc_last_subsampled = pc_last;
	pc_to_be_append_subsampled = pc_to_be_append;
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
		icp.build_ann_tree();

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
/// register without subsampling 
void point_cloud_interactable::register_cur_and_last_pc_if_present() {
	// align pc_to_be_append to last_pc 
	if (pc_last.get_nr_points() && pc_to_be_append.get_nr_points()) {
		rotation.identity();
		translation.zeros();
		icp.set_iterations(10);
		icp.set_eps(1e-8);
		icp.set_num_random(40);

		icp.set_source_cloud(pc_to_be_append);
		icp.set_target_cloud(pc_last);
		icp.build_ann_tree();

		// pc_to_be_append has been changed during the registration 
		icp.reg_icp(rotation, translation, crs_srs_pc, crs_tgt_pc, icp_filter_type, this->get_context());
	}
	else {
		//err: empty pcs 
	}
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
			frame_cam_posi.push_back(pc_to_be_append.cam_posi);
		pc_last = pc_to_be_append;
		tree_ds_target_pc_last_frame = tree_ds_source_pc;
	}
	else {
		pc.append(_pc);
		if (_pc.has_cam_posi)
			frame_cam_posi.push_back(_pc.cam_posi);
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
	region_id_and_seeds.clear();
	region_id_and_nmls.clear();

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
void point_cloud_interactable::build_neighbor_graph()
{
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
		orient_normals_to_view_point_vr(pc.cam_posi);
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
///
void point_cloud_interactable::compute_feature_points()
{
	feature_points.clear();
	ensure_tree_ds();
	if (ng.empty())
		build_neighbor_graph();
	pc.create_features();
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
		if (d[0]>0.1 && d[1]>0.1 && d[2]>0.1)
			feature_points.push_back(pc.pnt(i));
	}
	std::cout << "num of feature points computed: " << feature_points.size() << std::endl;

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
	k = 30;
	do_symmetrize = false;
	reorient_normals = true;

	frame_pointers.push_back(0);

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

	//get_root()->set("bg_index", 4);

	return true;
}
/// basic functions
void point_cloud_interactable::init_frame(cgv::render::context& ctx)
{
	static bool my_tab_selected = false;
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
	}
	gl_point_cloud_drawable::init_frame(ctx);
}
/// draw call
void point_cloud_interactable::draw(cgv::render::context& ctx)
{
	//if (pc.get_nr_points() != 0) {
	//	glVertexPointer(3, GL_FLOAT, 0, &(pc.pnt(0).x()));
	//	glEnableClientState(GL_VERTEX_ARRAY);
	//	draw_graph(ctx);
	//	glDisableClientState(GL_VERTEX_ARRAY);
	//}

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
	if (pc.render_cams) {
		for (auto render_kit : image_renderer_list) {
			render_kit.draw(ctx);
		}
	}

	// render feature points 
	if (feature_points.size() > 0 && fea_box.size() > 0) {
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(fea_style);
		renderer.set_box_array(ctx, fea_box);
		renderer.set_color_array(ctx, fea_box_color);
		renderer.set_translation_array(ctx, fea_box_trans);
		renderer.set_rotation_array(ctx, fea_box_rot);
		renderer.render(ctx, 0, fea_box.size());
	}
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
	if (((pcc_event & PCC_POINTS_MASK) == PCC_POINTS_RESIZE) || ((pcc_event & PCC_POINTS_MASK) == PCC_NEW_POINT_CLOUD)) {
		tree_ds_out_of_date = true;
		if (tree_ds) {
			delete tree_ds;
			tree_ds = 0;
		}
		ng.clear();
		show_point_end = pc.get_nr_points();
		show_point_begin = 0;

		update_member(&show_point_begin);
		update_member(&show_point_end);

		show_point_count = pc.get_nr_points();
		update_member(&show_point_count);

		interact_point_step = std::max((unsigned)(show_point_count / 1000000), 1u);
		nr_draw_calls = std::max((unsigned)(show_point_count / 1000000), 1u);
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

