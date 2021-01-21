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

	boxgui_interactable* b_interactable = new boxgui_interactable();
	point_cloud_interactable* point_cloud_kit = new point_cloud_interactable();
	point_cloud_interactable* one_shot_360pc = new point_cloud_interactable();
	point_cloud_interactable* stored_cloud = new point_cloud_interactable();
	bool render_pc = false;
	bool force_correct_num_pcs = true;

	typedef cgv::media::mesh::simple_mesh<float> mesh_type;
	mesh_type M;
	cgv::render::mesh_render_info mesh_info;
	bool have_new_mesh = false;
	bool show_face = false;
	bool direct_write = false;
	bool show_wireframe = false;
	bool render_img = false;

	int step = 1;
	int num_of_points_wanted = 1;
	int strategy = 1;

	rgb surface_color = rgb(0.4);
	CullingMode cull_mode;

	vr_kit_skybox* skybox_kit = new vr_kit_skybox();
	//vr_kit_image_renderer* image_renderer_kit = new vr_kit_image_renderer();

	vr_kit_teleportation* teleportation_kit = new vr_kit_teleportation();
	vr_kit_roller_coaster_1* roller_coaster_kit_1 = nullptr;
		//= new vr_kit_roller_coaster_1();

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

	void clean_all_pcs();

	void read_mesh();

	void randomize_texcoordi();

	void write_mesh();

	void load_image_from_bin_files();

	void update_paras_roller_coaster_kit_1();

	void create_gui();
};

///@}