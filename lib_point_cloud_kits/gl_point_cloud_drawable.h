#pragma once

#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/view.h>

#include "point_cloud.h"
#include "octree.h"

#include <cgv_gl/surfel_renderer.h>
#include <cgv_gl/normal_renderer.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/box_wire_renderer.h>
#include <libs/cgv_gl/clod_point_renderer.h>

#include "lib_begin.h"

enum class LoDMode {
	OCTREE = 1,
	RANDOM_POISSON = 2,
	INVALID = -1
};

// should add functions here, should be a manager class 
/** drawable for a point cloud that manages a neighbor graph and a normal estimator and supports rendering of point cloud and bounding box. */
class CGV_API gl_point_cloud_drawable : public cgv::render::drawable, public point_cloud_types
{
	struct VertexAttributeBinding {
		vec3 position;
		rgb8 color;
		vec3 normal;
		int index; // per vertex index used for marking 
		int scanindex;
		float point_size;
	};
public:
	gl_point_cloud_drawable();

	/*direct buffer storage */
	std::vector<cgv::render::clod_point_renderer::Point> point_buffer_storage;
	///
	bool using_directly_buffer_loading = false;

	/*storage*/
	point_cloud pc;
	
	/*
		1 - quad rendering 
		2 - point rendering 
		3 - surfel rendering 
		4 - clod rendering 
	*/
	int RENDERING_STRATEGY = 3;
	bool is_switching = false;
	// for small point clouds 
	bool continus_redraw = false; 

	/*raw rendering*/
	GLuint raw_vao = -1;
	GLuint raw_vbo_position = -1;
	GLuint raw_vbo_color = -1;
	GLuint raw_vbo_normal = -1;
	bool raw_renderer_out_of_date = true;
	cgv::render::shader_program raw_prog;
	std::vector<VertexAttributeBinding> input_buffer_data;
	float point_size = 0.1;
	float percentual_halo_width = 0.3f;

	/*surfel rendering*/
	cgv::render::surfel_renderer s_renderer;
	cgv::render::surfel_render_style surfel_style;
	cgv::render::attribute_array_manager sl_manager;
	bool highlight_unmarked_points = false;
	bool highlight_boundaries = true;
	bool force_render_with_original_color = false;

	bool render_with_topo_selction = false;
	bool render_with_topo_selctions_only = false;
	bool colorize_with_face_selection = true;
	bool colorize_with_face_selection_only = false;
	bool colorize_with_corner_edge_id = true;

	int highlight_topo_id = 4;
	int highlight_topo_ranking = 0;
	bool enable_topo_highlight = false;
	bool enable_point_visible_conn = true;
	bool enable_lighting = true;

	int highlight_fid = 1;
	int highlight_which_loop = 0;
	int highlight_edge_rank_within_loop = 0;

	/*point rendering */
	

	/*clod rendering*/
	//using LODPoint = cgv::pointcloud::SimpleLODPoint;
	using LODPoint = cgv::render::clod_point_renderer::LODPoint;
	//int lod_mode = (int)LoDMode::OCTREE;
	bool use_octree_sampling = false;
	bool renderer_out_of_date = true;
	bool color_based_on_lod = false;
	static constexpr float min_level_hue = 230.0 / 360.0;
	static constexpr float max_level_hue = 1.0;
	cgv::render::clod_point_render_style cp_style; 
	cgv::render::clod_point_renderer cp_renderer;
	cgv::pointcloud::octree_lod_generator<LODPoint> lod_generator;
	std::vector<LODPoint> points_with_lod; // pass points from main mem to here, then send to shader 
	bool is_triggering = false;
	float clod_render_scale = 0.3;
	bool test_shader_selection = false;

	/*normal rendering*/ 
	cgv::render::normal_renderer n_renderer;
	cgv::render::normal_render_style normal_style;

	/*bbox rendering*/
	cgv::render::surface_render_style box_style;
	cgv::render::line_render_style box_wire_style;
	cgv::render::box_renderer b_renderer;
	cgv::render::box_wire_renderer bw_renderer;

	/*render with culling*/
	vec3 headset_position = vec3(0);
	vec3 headset_direction = vec3(0);
	vec3 left_controller_position = vec3(0);
	vec3 right_controller_position = vec3(0);
	float headset_culling_range = 2;
	float controller_effect_range = 0.05;
	bool enable_acloud_effect = true;
	bool enable_headset_culling = false;
	int which_effect_righthand = -1;
	int which_effect_lefthand = -1;
	int which_effect_headset = -1;

	/*visual effects, shader control*/
	bool visual_delete = true;
	bool render_with_original_color = false;
	float collapse_tantheta = 0.577;
	bool colorize_with_scan_index = false;
	// uniforms used to toggle point cloud rendering from different scans (scan index)
	bool renderScan0 = true;
	bool renderScan1 = true;
	bool renderScan2 = true;
	bool renderScan3 = true;
	bool renderScan4 = true;
	bool renderScan5 = true;

	/*point addition */
	// they are used for realtime rendering
	// this mat is not changing pc at runtime, but sill can be used when we need the transform
	mat4 current_model_matrix;
	// those will be passed to shader as uniforms, otherwise, use last_model_matrix
	bool use_current_matrix = false;
	// relates controllers and point cloud, recorded when copy key press. used to "keep" the current relation 
	mat4 relative_model_matrix_controller_to_pc; 
	// 
	mat4 last_model_matrix;
	//
	bool binded_to_controller;

	/*palette rendering, render with color palletes*/
	std::vector<Clr>* use_these_point_colors;
	std::vector<RGBA>* use_these_point_palette;
	std::vector<int>* use_these_point_color_indices;
	std::vector<RGBA>* use_these_component_colors;

	/*cpu reduction*/
	unsigned show_point_step;
	std::size_t show_point_begin, show_point_end;
	unsigned nr_draw_calls;
	cgv::render::view* view_ptr;
	bool ensure_view_pointer();

	/*controlling varibles*/ 
	bool show_points;
	bool show_box;
	bool show_boxes;
	bool show_nmls;
	bool sort_points;
	bool use_component_colors;
	bool use_component_transformations;
	rgba box_color; 
	bool finished_loading_points = false; // for parallel loading or procedural generation  
	
	/*drawables */
	void render_boxes(cgv::render::context& ctx, cgv::render::group_renderer& R, cgv::render::group_render_style& RS);
	void draw_box(cgv::render::context& ctx, const Box& box, const rgba& clr);
	void draw_boxes(cgv::render::context& ctx);
	void draw_points_surfel(cgv::render::context& ctx);
	void destruct_prog_and_buffers_when_switching(cgv::render::context& ctx);
	void draw_raw(cgv::render::context& ctx);
	void draw_points_quad(cgv::render::context& ctx); 
	void switch_to_quad_rendering();
	void draw_points_point_rendering(cgv::render::context& ctx);
	void compute_lods();
	void draw_points_clod(cgv::render::context& ctx);
	void direct_buffer_loading();
	void direct_buffer_saving();
	void on_rendering_settings_changed();
	void draw_normals(cgv::render::context& ctx);
	void download_points_from_gpu_to_memory();
	void reset_marking();

public:
	/*IO*/
	bool read(const std::string& file_name);
	bool append(const std::string& file_name, bool add_component = true);
	bool write(const std::string& file_name);
	void set_arrays(cgv::render::context& ctx, size_t offset = 0, size_t count = -1);

	/*interfaces*/ 
	bool init(cgv::render::context& ctx);
	void draw(cgv::render::context& ctx);
	void clear(cgv::render::context& ctx);
};

#include <cgv/config/lib_end.h>