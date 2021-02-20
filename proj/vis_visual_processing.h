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
#include "vis_kit_meshes.h"
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
#include "vr_kit_hmbgui.h"

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

	point_cloud_interactable* one_shot_360pc = new point_cloud_interactable();
	point_cloud_interactable* stored_cloud = new point_cloud_interactable();
	bool render_pc = true;
	bool render_skybox = true;
	bool force_correct_num_pcs = true;
	bool direct_write = false;
	bool render_img = false;
	int step = 1;
	int num_of_points_wanted = 1;
	int strategy = 1;

	// cam rendering 
	std::vector<vec3> point_and_cam;
	std::vector<rgb> point_and_cam_colors;

	// all necessary kits 
	boxgui_interactable* b_interactable = new boxgui_interactable();
	vr_kit_hmbgui* hmbgui_kit = new vr_kit_hmbgui();
	vr_kit_light* light_kit = new vr_kit_light();
	vr_kit_skybox* skybox_kit = new vr_kit_skybox();
	vr_kit_teleportation* teleportation_kit = new vr_kit_teleportation();
	vis_kit_data_store_shared* data_ptr = new vis_kit_data_store_shared();

	// optional kits 
	vr_kit_roller_coaster_1* roller_coaster_kit_1 = nullptr; 
	vis_kit_meshes* mesh_kit = nullptr;
	vis_kit_meshes* mesh_kit_2 = nullptr;
	vr_kit_draw* draw_kit = nullptr; 
	vr_kit_motioncap* motioncap_kit = nullptr; 
	vr_kit_manipulation* manipulation_kit = nullptr;
	vr_kit_imagebox* imagebox_kit = nullptr;
	vis_kit_selection* selection_kit = nullptr;

public:

	void init_cameras(vr::vr_kit* kit_ptr);

	void start_camera();

	void stop_camera();

	/// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color);
	/// keep track of status changes
	void on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status);
	/// register on device change events
	void on_device_change(void* kit_handle, bool attach);
public:
	visual_processing();

	std::string get_type_name() { return "visual_processing"; }

	void stream_help(std::ostream& os);

	void on_set(void* member_ptr);

	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("pick_point_index", pick_point_index);
	}

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
	// some prob. with the depth information, TODO 
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

	bool init(cgv::render::context& ctx);

	void clear(cgv::render::context& ctx);

	//
	bool handle(cgv::gui::event& e);

	void init_frame(cgv::render::context& ctx);

	void draw(cgv::render::context& ctx);

	void finish_draw(cgv::render::context& ctx);

	///
	void read_pc();

	void read_pc_queue();

	void read_pc_append();

	///
	void downsampling();

	void add_reflectance();

	void write_read_pc_to_file();

	void read_campose();

	void show_camposes();

	void apply_further_transformation();

	void align_leica_scans_with_cgv();

	void rotate_x();

	void rotate_z();

	bool load_next_shot();

	void compute_nmls_if_is_required();

	void append_current_shot_to_stored_cloud();

	void write_stored_pc_to_file();

	void write_stored_pc_to_file_direct();
	///
	void print_cloud_info();
	///
	void auto_conduct_nml_estimation_leica();

	void add_to_file_list();

	void clean_file_list();

	bool batch_compute_nmls_given_file_list();

	bool batch_read_pc_queue_and_downsampling();

	void clean_all_pcs();

	void load_image_from_bin_files();

	void create_gui();

	/// wrappers 
	void write_trajectory() { draw_kit->write_trajectory(); }
	void read_trajectory() { draw_kit->read_trajectory(); }
	void clear_drawing() { draw_kit->clear_drawing(); }

	void start_replay_all() { motioncap_kit->start_replay_all(); }
	void save_to_tj_file() { motioncap_kit->save_to_tj_file(); }
	void read_tj_file() { motioncap_kit->read_tj_file(); }
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

	void compute_feature_points() { data_ptr->point_cloud_kit->compute_feature_points(); post_redraw(); }

	void render_with_fullpc() { data_ptr->point_cloud_kit->render_with_fullpc(); }
	void auto_downsampling() { data_ptr->point_cloud_kit->auto_downsampling(); }
	void supersampling_with_bbox() { data_ptr->point_cloud_kit->supersampling_with_bbox(data_ptr->supersampling_bbox); }
	void restore_supersampling() { data_ptr->point_cloud_kit->restore_supersampling(); }
};

///@}
