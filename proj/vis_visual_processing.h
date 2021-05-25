#pragma once
#include <cgv/base/node.h>
#include <cgv/render/drawable.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/gl/mesh_render_info.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/rounded_cone_renderer.h>
#include <cgv/render/frame_buffer.h>
#include <chrono>
#include <cgv/base/import.h>
#include <cgv\gui\file_dialog.h>

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>

#include "point_cloud_interactable.h"
#include "vr_kit_boxgui.h"
#include "vr_kit_skybox.h"
//#include "vr_kit_image_renderer.h"
#include "vr_kit_teleportation.h"
#include "vr_kit_roller_coaster.h"
#include "vr_kit_draw.h"
#include "vr_kit_motioncap.h"
#include "vis_kit_datastore.h"
#include "vr_kit_manipulation.h"
#include "vr_kit_imagebox.h"
#include <vr_kit_light.h>
#include "vis_kit_selection.h"
#include "vr_kit_handhold_near_gui.h"
#include <vr_kit_tmpfixed_gui.h>
#include <vis_kit_meshes.h>
#include "vis_parametric_surface_kit.h"
#include "vr_kit_env_renderer.h"

class visual_processing :
	public cgv::base::node,
	public cgv::render::drawable,
	public cgv::gui::event_handler,
	public cgv::gui::provider {
protected:
	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};
	
	// store the scene as colored boxes
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;

	// rendering styles
	cgv::render::box_render_style style;
	cgv::render::rounded_cone_render_style cone_style;

	// sample for rendering a mesh
	double mesh_scale;
	dvec3 mesh_location;
	dquat mesh_orientation;

	// render information for mesh
	cgv::render::mesh_render_info MI;

	// sample for rendering text labels
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;

	// for batch operations 
	std::vector<std::string> f_names;

	// parallel tasks 
	std::thread* timer_thread;

private:
	bool label_outofdate; // whether label texture is out of date
protected:
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;

	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;

	// manage controller input configuration for left controller
	std::vector<vr::controller_input_config> left_inp_cfg;

	// store handle to vr kit of which left deadzone and precision is configured
	void* last_kit_handle;

	// length of to be rendered rays
	float ray_length;

	// keep reference to vr_view_interactor
	vr_view_interactor* vr_view_ptr;

	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;

	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	// state of current interaction with boxes for all controllers
	InteractionState state[4];

	// render style for interaction
	cgv::render::sphere_render_style srs;
	cgv::render::box_render_style movable_style;

	int nr_cameras;
	int frame_width, frame_height;
	int frame_split;
	float seethrough_gamma;
	mat4 camera_to_head_matrix[2];
	cgv::math::fmat<float, 4, 4> camera_projection_matrix[4];
	vec2 focal_lengths[4];
	vec2 camera_centers[4];
	cgv::render::texture camera_tex;
	cgv::render::shader_program seethrough;
	GLuint camera_tex_id;
	bool undistorted;
	bool shared_texture;
	bool max_rectangle;
	float camera_aspect;
	bool show_seethrough;
	bool use_matrix;
	float background_distance;
	float background_extent;
	vec2 extent_texcrd;
	vec2 center_left;
	vec2 center_right;

	// for seelction tool 
	int pick_point_index = -1;
	cgv::render::sphere_render_style sphere_style;
	bool in_picking = false;
	cgv::render::view* view_ptr = nullptr;

	// alignment
	quat initial_cam_alinmentq = quat(vec3(0, 1, 0), -115 * M_PI / 180);
	quat addi_alignq = quat();

	point_cloud_interactable* one_shot_360pc = nullptr;
		//= new point_cloud_interactable();
	point_cloud_interactable* stored_cloud = nullptr;
		//= new point_cloud_interactable();
	bool render_pc = true;
	bool render_skybox = true;
	bool camera_ready = false;
	bool render_handhold_gui = false;
	bool render_env = true;
	bool force_correct_num_pcs = true;
	bool direct_write = false;
	bool render_img = false;
	int step = 1;
	int num_of_points_wanted = 1;
	int strategy = 1;
	bool auto_growing_prepare_thesis = true;
	bool overwrite_face_id = false;
	bool instance_redraw = true;
	bool put_points_to_table = true;
	bool backward_grow = false;
	int curr_face_selecting_id;
	float height_offset = 1;
	float delay = 0, speed = 30;
	bool animate_boundary_loop = false;

	// cam rendering 
	std::vector<vec3> point_and_cam;
	std::vector<rgb> point_and_cam_colors;

	// all necessary kits 
	vis_kit_data_store_shared* data_ptr = new vis_kit_data_store_shared();
	vr_kit_tmpfixed_gui* tmpfixed_gui_kit = new vr_kit_tmpfixed_gui();
	vr_kit_light* light_kit = new vr_kit_light();
	vr_kit_teleportation* teleportation_kit = new vr_kit_teleportation();
	// optional kits 
		//= new boxgui_interactable();
		//= new vr_kit_handhold_near_gui();
		//= nullptr;
	vr_kit_skybox* skybox_kit = new vr_kit_skybox();
	env_renderer* env_render_kit = new env_renderer();
	boxgui_interactable* b_interactable = nullptr;
	vis_kit_meshes* mesh_kit = nullptr;
	vis_kit_meshes* mesh_kit_2 = nullptr;
	vr_kit_handhold_near_gui* handhold_near_kit = nullptr;
	vr_kit_roller_coaster_1* roller_coaster_kit_1 = nullptr; 
	vr_kit_draw* draw_kit = nullptr; 
	vr_kit_motioncap* motioncap_kit = nullptr; 
	vr_kit_manipulation* manipulation_kit = nullptr;
	vr_kit_imagebox* imagebox_kit = nullptr;
	vis_kit_selection* selection_kit = nullptr;
	parametric_surface* parametric_surface_kit = nullptr;
public:
	///
	void init_cameras(vr::vr_kit* kit_ptr);
	///
	void start_camera();
	///
	void stop_camera();
	/// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	/// keep track of status changes
	void on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status);
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach);
	void parallel_region_growing();
	void parallel_region_growing_finialize_grow();
	void timer_event(double t, double dt);
public:
	visual_processing();
	~visual_processing();
	///
	std::string get_type_name() { return "visual_processing"; }
	///
	void stream_help(std::ostream& os);
	///
	//bool self_reflect(cgv::reflect::reflection_handler& rh);
	///
	void on_set(void* member_ptr);
	///
	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("pick_point_index", pick_point_index) &&
			rh.reflect_member("curr_region", selection_kit->curr_face_selecting_id) &&
			rh.reflect_member("highlight_edge_rank_within_loop", data_ptr->point_cloud_kit->highlight_edge_rank_within_loop);
	}
	///
	void init_6_points_picking() {
		data_ptr->pick_points.push_back(vec3(1));
		data_ptr->pick_colors.push_back(rgb(0, 1, 0));
		data_ptr->pick_points.push_back(vec3(1));
		data_ptr->pick_colors.push_back(rgb(1, 1, 0));

		data_ptr->pick_points.push_back(vec3(1));
		data_ptr->pick_colors.push_back(rgb(0, 1, 0));
		data_ptr->pick_points.push_back(vec3(1));
		data_ptr->pick_colors.push_back(rgb(1, 1, 0));

		data_ptr->pick_points.push_back(vec3(1));
		data_ptr->pick_colors.push_back(rgb(0, 1, 0));
		data_ptr->pick_points.push_back(vec3(1));
		data_ptr->pick_colors.push_back(rgb(1, 1, 0));
		pick_point_index = 0;

		point_and_cam.push_back(vec3(0, 0, 0));
		point_and_cam.push_back(vec3(1));

		point_and_cam.push_back(vec3(0, 0, 0));
		point_and_cam.push_back(vec3(1));

		point_and_cam.push_back(vec3(0, 0, 0));
		point_and_cam.push_back(vec3(1));

		point_and_cam.push_back(vec3(0, 0, 0));
		point_and_cam.push_back(vec3(1));


		point_and_cam_colors.push_back(rgb(0, 1, 0));
		point_and_cam_colors.push_back(rgb(0, 1, 0));

		point_and_cam_colors.push_back(rgb(1, 0, 0));
		point_and_cam_colors.push_back(rgb(1, 0, 0));

		point_and_cam_colors.push_back(rgb(1, 0, 1));
		point_and_cam_colors.push_back(rgb(1, 0, 1));

		point_and_cam_colors.push_back(rgb(1, 1, 0));
		point_and_cam_colors.push_back(rgb(1, 1, 0));
	}
	/// selection tool, pick points from 0-6
	bool on_pick_face(const cgv::gui::mouse_event& me)
	{
		if (!view_ptr)
			return false;
		if (!data_ptr)
			return false;
		if (pick_point_index == -1)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z))
			return false; //window_z > 0.999
		mesh_kit->pick_face(pick_point);
		return true;
	}
	/// selection tool, pick points from 0-6
	bool on_pick(const cgv::gui::mouse_event& me)
	{
		if (!view_ptr)
			return false;
		if (!data_ptr)
			return false;		
		if (pick_point_index == -1)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z))
			return false; //window_z > 0.999
		//pick_point_index indicates which to pick from 0-6 eg.
		bool already_picked = false;
		double pick_dist = 0;
		double pick_dist_threshold = 1.2 * sphere_style.radius * sphere_style.radius_scale;
		for (int i = 0; i < (int)data_ptr->pick_points.size(); ++i) {
			double dist = (data_ptr->pick_points[i] - vec3(pick_point)).length();
			if (dist < pick_dist_threshold) {
				/*if (pick_point_index == -1 || dist < pick_dist) {
					pick_dist = dist;
					pick_point_index = i;
				}*/
				already_picked = true;
			}
		}
		if (!already_picked) { 
			// move the points 
			/*pick_point_index = data_ptr->pick_points.size();
			data_ptr->pick_points.push_back(pick_point);
			data_ptr->pick_colors.push_back(rgb(0, 1, 0));*/
			data_ptr->pick_points[pick_point_index] = pick_point;
			point_and_cam.at(pick_point_index * 2 + 1) = pick_point;
			//pick_point_index++;
			return true;
		}
		return false;

	}
	/// some prob. with the depth information, TODO 
	bool on_drag(const cgv::gui::mouse_event& me)
	{
		if (!view_ptr)
			return false;
		if (!data_ptr)
			return false;
		if (pick_point_index == -1)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z) ||
			window_z > 0.999) {
			data_ptr->pick_colors[pick_point_index] = rgb(0, 1, 0);
			pick_point_index = -1;
			post_redraw();
			return false;
		}
		data_ptr->pick_points[pick_point_index] = pick_point;
		data_ptr->pick_colors[pick_point_index] = rgb(1, 0, 0);
		post_redraw();
		return true;
	}
	///
	bool init(cgv::render::context& ctx);
	///
	void clear(cgv::render::context& ctx);
	///
	bool handle(cgv::gui::event& e);
	///
	void init_frame(cgv::render::context& ctx);
	///
	void draw(cgv::render::context& ctx);
	///
	void finish_draw(cgv::render::context& ctx);
	///
	thread* parallel_reading_thread = nullptr;
	thread* parallel_writting_thread = nullptr;
	thread* parallel_region_growing_thread = nullptr;
	bool parallel_reading = false;
	/// wrappers 
	void prepare_grow_ourmethod();
	void read_pc();
	void read_pc_parallel();
	void start_reading_pc_parallel();
	void start_parallel_region_growing();
	void residual_grow_dist_and_curvature_based();
	void sync_grow_dist_curvature_based();
	void sync_grow_dist_based();
	void sync_grow_curv_based();
	void undo_sync_grow();
	void residual_grow_curvature_based();
	void grow_with_dist_and_lowest_curvature();
	void grow_with_dist_and_highest_curvature();
	void grow_with_accu_dist();
	void grow_with_highest_curvature_only();
	void backward_growing();
	void force_start_grow();
	void ep_start_grow_auto_curv_direction();
	void pause_continue_parallel_region_growing();
	void stop_parallel_region_growing();
	void read_pc_queue();
	void read_pc_append();
	void downsampling();
	void add_reflectance();
	void write_pc_parallel();
	void start_writting_pc_parallel();
	void read_campose();
	void show_camposes();
	void apply_further_transformation();
	void align_leica_scans_with_cgv();
	void rotate_x();
	void higher_y();
	void rotate_z();
	bool load_next_shot();
	void compute_nmls_if_is_required();
	void force_nml_computing();
	void append_current_shot_to_stored_cloud();
	void write_stored_pc_to_file();
	void write_stored_pc_to_file_direct();
	void print_cloud_info();
	void auto_conduct_nml_estimation_leica();
	void add_to_file_list();
	void clean_file_list();
	bool batch_compute_nmls_given_file_list();
	bool batch_read_pc_queue_and_downsampling();
	void clear_all_pcs();
	void load_image_from_bin_files();
	void switch_rendering_mode_quad_based();
	void switch_rendering_mode_point_based();
	void switch_rendering_mode_surfel_based();
	void switch_rendering_mode_clod_based();
	void batch__load_and_convert_add_lod_info();
	void batch__load_and_convert_add_lod_single_file();
	void single_hit__load_point_cloud_and_render_with_clod();
	void mark_all_points_as_tobedownsampled();
	void mark_all_active_points_as_tobedownsampled();
	void del_clipping_btn_press();
	void del_menu_btn_press();
	void del_menu_btn_release();
	void selective_downsampling_menu_btn_press();
	void selective_downsampling_menu_btn_release();
	void quad_addition_menu_btn_press();
	void quad_addition_menu_btn_release();
	void send_updated_point_cloud_to_gpu();
	void step_back_selection();
	void step_forward_selection();
	void enlarge_tube_length();
	void schrink_tube_length();
	void point_copy_btn_pressed();
	void point_copy_btn_release();
	void release_controller_pc_binding();
	void on_rendering_settings_changed();
	void download_points_from_gpu_to_memory();
	void reset_marking();
	void compute_lods();
	void load_sample_seeds_default();
	void load_sample_seeds_with_dialog();
	void load_seeds_with_dialog_without_recover(); 
	void save_sample_seeds_with_dialog();
	void save_sample_seeds_default();
	void single_hit__prepare_region_grow(bool overwrite_face_id);
	void single_hit__prepare_region_grow_worker(bool overwrite_face_id);
	void single_hit__regrow_accu_distance_based();
	void single_hit__regrow_seed_distance_based();
	void single_hit__regrow_unsigned_mean_curvature_based();
	void single_hit__regrow_distance_and_curvature_based();
	void single_hit__regrow_stop_at_high_curvature();
	void find_pointcloud();
	bool find_next_and_increase_curr_region();
	void drop_unwanted_regions();
	void automatic_region_extraction();
	/// embeded for vr handlers 
	void down_scale_model_one_step();
	/// embeded for vr handlers 
	void upscale_model_one_step();
	void undo_curr_region();

	void quiet_save();

	void apply_transfrom_to_pc();
	void next_topo_ranking();
	void prev_topo_ranking();

	void visualize_boundary_loop();

	void next_edge_within_curr_boundary();

	void prev_edge_within_curr_boundary();
	///
	void next_face();
	///
	void prev_face();
	///
	void find_first_he_curr_settings();
	///
	void find_next_he();
	///
	void update_halfedge_visulization();
	///
	void next_percentage_progress();

	void create_gui();
	void write_trajectory() { draw_kit->write_trajectory(); }
	void read_trajectory() { draw_kit->read_trajectory(); }
	void clear_drawing() { draw_kit->clear_drawing(); }
	void start_replay_all() { motioncap_kit->start_replay_all(); }
	void save_to_tj_file() { motioncap_kit->save_to_tj_file(); }
	void stop_and_clear_mocap_data() { motioncap_kit->stop_and_clear_mocap_data(); }
	void read_tj_file() { 
		motioncap_kit->read_tj_file(data_ptr->default_tj_file);
		post_recreate_gui(); 
	}
	void compute_coordinates_with_rot_correction() { 
		quat rotq;
		mat3 rotation_mat;
		vec3 translation_vec;

		vec3 p1 = data_ptr->pick_points.at(0);
		vec3 p2 = data_ptr->pick_points.at(2);
		vec3 p3 = data_ptr->pick_points.at(4);

		vec3 q1 = data_ptr->pick_points.at(1);
		vec3 q2 = data_ptr->pick_points.at(3);
		vec3 q3 = data_ptr->pick_points.at(5);

		vec3 source_center = vec3(0);
		source_center += p1;
		source_center += p2;
		source_center += p3;
		source_center /= 3;

		vec3 target_center = vec3(0);
		target_center += q1;
		target_center += q2;
		target_center += q3;
		target_center /= 3;

		mat3 fA;
		fA.zeros();			
		cgv::math::mat<float> U, V;
		cgv::math::diag_mat<float> Sigma;
		U.zeros();
		V.zeros();
		Sigma.zeros();
		fA += mat3(q1 - target_center, p1 - source_center);
		fA += mat3(q2 - target_center, p2 - source_center);
		fA += mat3(q3 - target_center, p3 - source_center);

		///cast fA to A
		cgv::math::mat<float> A(3, 3, &fA(0, 0));
		cgv::math::svd(A, U, Sigma, V);
		mat3 fU(3, 3, &U(0, 0)), fV(3, 3, &V(0, 0));

		///get new R and t
		rotation_mat = fU * cgv::math::transpose(fV);
		cgv::math::mat<float> R(3, 3, &rotation_mat(0, 0));
		if (cgv::math::det(R) < 0) {
			// multiply the 1,1...-1 diag matrix 
			mat3 fS;
			fS.zeros();
			fS(0, 0) = 1;
			fS(1, 1) = 1;
			fS(2, 2) = -1;
			rotation_mat = fU * fS * cgv::math::transpose(fV);
		}
		rotq = quat(rotation_mat);
		translation_vec = target_center - rotation_mat * source_center;

		//vec3 view_up_dir = vec3(0, 1, 0);
		//cgv::render::render_types::dmat3 R = cgv::math::build_orthogonal_frame(p1, view_up_dir);
		//R.transpose();
		//R = cgv::math::build_orthogonal_frame(p2, view_up_dir) * R;
		//cgv::render::render_types::dvec3 daxis;
		//double dangle;
		//int res = cgv::math::decompose_rotation_to_axis_and_angle(R, daxis, dangle);
		//addi_alignq = quat(daxis, dangle);

		//initial_cam_alinmentq
		mesh_kit->compute_coordinates_with_rot_correction(rotq, translation_vec);
	}
	void compute_feature_points() { 
		//data_ptr->point_cloud_kit->compute_feature_points(); post_redraw(); 
	}
	void render_with_fullpc() { data_ptr->point_cloud_kit->render_with_fullpc(); }
	void auto_downsampling() { data_ptr->point_cloud_kit->auto_downsampling(); }
	void supersampling_with_bbox() { data_ptr->point_cloud_kit->supersampling_with_bbox(data_ptr->supersampling_bbox); }
	void restore_supersampling() { data_ptr->point_cloud_kit->restore_supersampling(); }
	void prepare_grow() {
		data_ptr->point_cloud_kit->prepare_grow(false);
	}
	void clear_face_id_and_topo_id() {
		for (auto& fi : data_ptr->point_cloud_kit->pc.face_id) fi = 0; // 0 reserved
		for (auto& ti : data_ptr->point_cloud_kit->pc.topo_id) ti = 0; // 0 reserved
	}
	void convert_to_int_face_selection_representation() {
		data_ptr->point_cloud_kit->pc.convert_to_int_face_selection_representation();
	}
	void mark_sample_seed() {
		//data_ptr->point_cloud_kit->pc.face_id.at(0) = 1; // mark index 0 as face 1 
		//data_ptr->point_cloud_kit->init_region_growing_by_collecting_group_and_seeds_vr(1); // collect face seed with index 

		//data_ptr->point_cloud_kit->pc.face_id.at(data_ptr->point_cloud_kit->pc.get_nr_points()-1) = 2; // mark index 0 as face 1 
		//data_ptr->point_cloud_kit->init_region_growing_by_collecting_group_and_seeds_vr(2); // collect face seed with index 
		data_ptr->point_cloud_kit->seed_for_regions[1] = 0; // face_id -> pid 
		data_ptr->point_cloud_kit->add_seed_to_queue(1);
		//data_ptr->point_cloud_kit->seed_for_regions[2] = data_ptr->point_cloud_kit->pc.get_nr_points() - 1; // face_id -> pid 
		std::cout << "marked! seed_for_regions updated" << std::endl;
	}
	int current_seed_group = 1;
	void mark_next_seed() {
		if (data_ptr->point_cloud_kit->loaded_seeds_for_regions.empty()) {
			std::cout << "empty seed list! have you loaded them? " << std::endl;
			return;
		}
		std::cout << "loading seed group: "<< current_seed_group << std::endl;
		data_ptr->point_cloud_kit->seed_for_regions[current_seed_group] = 
			data_ptr->point_cloud_kit->loaded_seeds_for_regions[current_seed_group]; 
		data_ptr->point_cloud_kit->add_seed_to_queue(current_seed_group);
		current_seed_group++;
	}
	void generate_pc_hemisphere() { 
		data_ptr->point_cloud_kit->generate_pc_hemisphere();
		force_nml_computing();
		prepare_grow();
		post_redraw();
	}
	void generate_pc_cube() { 
		data_ptr->point_cloud_kit->generate_pc_cube(); 
		force_nml_computing(); 
		prepare_grow();
		post_redraw(); 
	}
	void generate_testing_plane() {
		data_ptr->point_cloud_kit->generate_testing_plane();
		post_redraw();
	}
	void generate_pc_init_sphere() {
		data_ptr->point_cloud_kit->generate_pc_init_sphere();
		post_redraw();
	}
	void generate_pc_random_sphere() {
		data_ptr->point_cloud_kit->generate_pc_random_sphere();
		post_redraw();
	}
	void generate_pc_unit_sylinder() {
		data_ptr->point_cloud_kit->generate_pc_unit_cylinder();
		post_redraw();
	}
	void generate_pc_unit_torus() {
		data_ptr->point_cloud_kit->generate_pc_unit_torus();
		post_redraw();
	}
	void toggle_normal_orientations() {
		data_ptr->point_cloud_kit->toggle_normal_orientations();
		post_redraw();
	}
	void print_pc_information() {
		data_ptr->point_cloud_kit->print_pc_information();
	}
	void boundary_extraction() {
		//data_ptr->point_cloud_kit->extract_all();
	}
	void extract_connectivity_graph() {
		//data_ptr->point_cloud_kit->extract_connectivity_graph();
	}
	void fitting_render_control_points_test() {
		data_ptr->point_cloud_kit->
			fitting_render_control_points_test();
		parametric_surface_kit->
			fetch_from_point_cloud_kit_demo();
	}
	void build_connectivity_graph_fitting_and_render_control_points() {
		data_ptr->point_cloud_kit->connectivity_extraction();
	}
	void read_cgvcad_with_dialog() {
		data_ptr->point_cloud_kit->read_cgvcad_with_dialog();
	}
	void triangulation_of_the_points() {
		data_ptr->point_cloud_kit->pc.triangulation_of_the_points();
	}
	void update_scan_index_visibility_test() {
		data_ptr->point_cloud_kit->update_scan_index_visibility_test();
	}
	void scale_points_to_desk(){
		data_ptr->point_cloud_kit->scale_points_to_desk();
		post_redraw();
	}
	void extract_point_clouds_for_icp() {
		data_ptr->point_cloud_kit->extract_point_clouds_for_icp();
	}
	void point_visibility_vis_all_points() {
		std:vector<bool>* visibility_array = &data_ptr->point_cloud_kit->pc.scan_index_visibility;
		for (int i = 0; i < visibility_array->size(); i++) {
			visibility_array->at(i) = true;
		}
		data_ptr->point_cloud_kit->pc.update_scan_index_visibility();
	}
	void perform_icp_and_acquire_matrices() {
		data_ptr->point_cloud_kit->perform_icp_and_acquire_matrices();
	}
	void set_src_and_target_scan_idx_as_test() {
		data_ptr->point_cloud_kit->set_src_and_target_scan_idx_as_test();
	}
	void apply_register_matrices_for_the_original_point_cloud() {
		data_ptr->point_cloud_kit->apply_register_matrices_for_the_original_point_cloud();
	}
	void export_to_an_obj_file() {
		std::string f = cgv::gui::file_save_dialog("Save", "OBJ Mesh:*");
		data_ptr->point_cloud_kit->pc.export_to_an_obj_file(f);
	}
	void do_icp_once() {
		perform_icp_and_acquire_matrices();
		apply_register_matrices_for_the_original_point_cloud();
		//for (int i = 0; i < data_ptr->point_cloud_kit->icp_iterations; i++) {
		//}
	}
	void randomize_current_pc() {
		data_ptr->point_cloud_kit->pc.randomize_position(data_ptr->point_cloud_kit->src_scan_idx);
	}
	void rotate_right(){ data_ptr->active_off_rotation -= 30; }
	void rotate_left() { data_ptr->active_off_rotation += 30; }
	void drop_other_info_points_only() {
		data_ptr->point_cloud_kit->pc.N.clear();
		data_ptr->point_cloud_kit->pc.has_nmls = false;
		data_ptr->point_cloud_kit->pc.point_visited.clear();
		data_ptr->point_cloud_kit->pc.face_id.clear();
		data_ptr->point_cloud_kit->pc.point_scan_index.clear();
		std::cout << "dropped!" << std::endl;
	}
	void ep_compute_principal_curvature_and_colorize_signed() {
		data_ptr->point_cloud_kit->ep_compute_principal_curvature_and_colorize_signed();
	}
	void ep_compute_principal_curvature_and_colorize_unsigned() {
		data_ptr->point_cloud_kit->ep_compute_principal_curvature_and_colorize_unsigned();
	}
	void ep_force_recolor() {
		data_ptr->point_cloud_kit->ep_force_recolor();
	}
	void compute_feature_points_and_colorize() {
		data_ptr->point_cloud_kit->compute_feature_points_and_colorize();
	}
	void debug_region_growing_step_by_step_test() {
		data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::ACCU_DISTANCE_BASED;
		data_ptr->point_cloud_kit->grow_one_step_bfs(false, 1);
	}
	void direct_buffer_loading() { // very special case 
		data_ptr->point_cloud_kit->direct_buffer_loading();
	}
	void direct_buffer_saving() {
		data_ptr->point_cloud_kit->direct_buffer_saving();
	}
};

///@}
