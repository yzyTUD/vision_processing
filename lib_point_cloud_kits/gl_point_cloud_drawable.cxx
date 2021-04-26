#include <cgv/base/base.h>
#include <cgv/base/import.h>
#include <cgv/render/shader_program.h>
#include "gl_point_cloud_drawable.h"
#include <cgv/utils/file.h>
#include <cgv/utils/scan.h>
#include <cgv_gl/gl/wgl.h>
#include <cgv_gl/gl/gl_tools.h>
///
using namespace std;
using namespace cgv::render;
using namespace cgv::utils::file;
///
#include <cgv/base/find_action.h>
#include <cgv/render/view.h>
///
#ifdef REGISTER_SHADER_FILES
#include <cgv/base/register.h>
//#include <point_cloud_shader_inc.h>
#endif
///
gl_point_cloud_drawable::gl_point_cloud_drawable() 
{
	view_ptr = 0;

	nr_draw_calls = 1;

	show_point_step = 1;
	show_point_begin = 0;
	show_point_end = 0;

	sort_points = false;
	show_points = true;
	show_nmls = true;
	show_boxes = false;
	show_box = true;

	surfel_style.blend_points = false;
	surfel_style.measure_point_size_in_pixel = false;
	surfel_style.blend_width_in_pixel = 0.0f;

	box_color = rgba(0.5f, 0.5f, 0.5f, 1.0f);
	box_style.illumination_mode = cgv::render::IM_TWO_SIDED;
	box_style.culling_mode = cgv::render::CM_FRONTFACE;

	use_these_point_colors = 0;
	use_these_component_colors = 0;
	use_these_point_palette = 0;
	use_these_point_color_indices = 0;

	use_component_colors = false;
	use_component_transformations = false;

	relative_model_matrix_controller_to_pc.identity();
	last_model_matrix.identity();
}
///
bool gl_point_cloud_drawable::read(const std::string& _file_name)
{
	std::string fn = cgv::base::find_data_file(_file_name, "cpD");
	if (fn.empty()) {
		cerr << "point cloud " << _file_name << " not found" << endl;
		return false;
	}
	if (!pc.read(fn)) {
		cerr << "could not read point cloud " << fn << endl;
		return false;
	}
	// these are managed by interactable, do not use drawable saparately for me 
	//show_point_begin = 0;
	//show_point_end = pc.get_nr_points();
	std::cout << "points read: " << pc.get_nr_points() << std::endl;
	post_redraw();
	return true;
}
///
bool gl_point_cloud_drawable::append(const std::string& _file_name, bool add_component)
{
	std::string fn = cgv::base::find_data_file(_file_name, "cpD");
	if (fn.empty()) {
		cerr << "point cloud " << _file_name << " not found" << endl;
		return false;
	}
	point_cloud pc1;
	if (!pc1.read(fn)) {
		cerr << "could not read point cloud " << fn << endl;
		return false;
	}

	// construct component if necessary
	if (add_component) {
		if (!pc.has_components()) {
			pc.create_components();
			pc.create_component_colors();
			pc.create_component_tranformations();
		}
		if (!pc1.has_components())
			pc.add_component();
	}
	pc.append(pc1);
	pc.component_name(cgv::type::int32_type(pc.get_nr_components() - 1)) = 
		cgv::utils::file::drop_extension(cgv::utils::file::get_file_name(_file_name));
	show_point_begin = 0;
	show_point_end = pc.get_nr_points();
	return true;
}
///
bool gl_point_cloud_drawable::write(const std::string& fn)
{
	if (!pc.write(fn)) {
		cerr << "could not write point cloud " << fn << endl;
		return false;
	}
	return true;
}
///
void gl_point_cloud_drawable::render_boxes(context& ctx, group_renderer& R, cgv::render::group_render_style& RS)
{
	R.set_position_array(ctx, &pc.box(0).get_min_pnt(), pc.get_nr_components(), sizeof(Box));
	if (use_component_colors)
		R.set_color_array(ctx, &pc.component_color(0), pc.get_nr_components());
	R.render(ctx, 0, pc.get_nr_components());
}
///
void gl_point_cloud_drawable::draw_box(cgv::render::context& ctx, const Box& box, const rgba& clr)
{
	bool tmp_use_color = box_style.use_group_color;
	bool tmp_use_transformation = box_style.use_group_transformation;
	box_style.use_group_color = false;
	box_style.use_group_transformation = false;
	b_renderer.set_render_style(box_style);
	b_renderer.set_box_array(ctx, &box, 1);
	b_renderer.set_color_array(ctx, &clr, 1);
	b_renderer.render(ctx, 0, 1);

	box_style.use_group_color = tmp_use_color;
	box_style.use_group_transformation = tmp_use_transformation;

	tmp_use_color = box_wire_style.use_group_color;
	tmp_use_transformation = box_wire_style.use_group_transformation;
	box_wire_style.use_group_color = false;
	box_wire_style.use_group_transformation = false;
	bw_renderer.set_render_style(box_wire_style);
	bw_renderer.set_box_array(ctx, &box, 1);
	bw_renderer.set_color_array(ctx, &clr, 1);
	bw_renderer.render(ctx, 0, 1);
	box_wire_style.use_group_color = tmp_use_color;
	box_wire_style.use_group_transformation = tmp_use_transformation;
}
///
void gl_point_cloud_drawable::draw_boxes(context& ctx)
{

	if (show_box)
		draw_box(ctx, pc.box(), box_color);

	if (!show_boxes)
		return;

	if (pc.has_components()) {
		Box b;
		for (unsigned ci = 0; ci < pc.get_nr_components(); ++ci)
			b.add_axis_aligned_box(pc.box(ci));

		b_renderer.set_render_style(box_style);
		if (use_component_transformations) {
			b_renderer.set_rotation_array(ctx, &static_cast<HVec&>(pc.component_rotation(0)), pc.get_nr_components(), sizeof(HVec));
			b_renderer.set_translation_array(ctx, &pc.component_translation(0), pc.get_nr_components(), sizeof(Dir));
		}
		b_renderer.set_extent_array(ctx, &pc.box(0).get_max_pnt(), pc.get_nr_components(), sizeof(Box));
		render_boxes(ctx, b_renderer, box_style);

		bw_renderer.set_render_style(box_wire_style);
		if (use_component_transformations) {
			bw_renderer.set_rotation_array(ctx, &static_cast<HVec&>(pc.component_rotation(0)), pc.get_nr_components(), sizeof(HVec));
			bw_renderer.set_translation_array(ctx, &pc.component_translation(0), pc.get_nr_components(), sizeof(Dir));
		}
		
		bw_renderer.set_extent_array(ctx, &pc.box(0).get_max_pnt(), pc.get_nr_components(), sizeof(Box));
		render_boxes(ctx, bw_renderer, box_wire_style);
	}
}
///
bool gl_point_cloud_drawable::ensure_view_pointer()
{
	/*cgv::base::base_ptr bp(dynamic_cast<cgv::base::base*>(this));
	if (bp) {
		vector<cgv::render::view*> views;
		cgv::base::find_interface<cgv::render::view>(bp, views);
		if (!views.empty()) {
			view_ptr = views[0];
			return true;
		}
	}*/
	return false;
}
///
void gl_point_cloud_drawable::on_rendering_settings_changed() {
	renderer_out_of_date = true;
	raw_renderer_out_of_date = true;
	finished_loading_points = true; // prepared to upload data to gpu 
}
///
void gl_point_cloud_drawable::draw_normals(context& ctx)
{
	if (!show_nmls || !pc.has_normals())
		return;
	n_renderer.set_normal_scale(pc.box().get_extent().length() / sqrt(float(pc.get_nr_points())));
	n_renderer.set_position_array(ctx, &pc.pnt(0), pc.get_nr_points(), sizeof(Pnt)*show_point_step);
	if (pc.has_colors())
		n_renderer.set_color_array(ctx, &pc.clr(0), pc.get_nr_points(), sizeof(Clr)*show_point_step);
	if (pc.has_normals())
		n_renderer.set_normal_array(ctx, &pc.nml(0), pc.get_nr_points(), sizeof(Nml)*show_point_step);
	std::size_t n = (show_point_end - show_point_begin) / show_point_step;
	GLint offset = GLint(show_point_begin / show_point_step);
	n_renderer.render(ctx, offset,n);
}
///
bool gl_point_cloud_drawable::init(cgv::render::context& ctx)
{
	// this step is simplified with the cgv ref_renderer api 
	/*if (!cp_renderer.init(ctx))
		return false;
	cp_renderer.set_render_style(cp_style);*/
	cgv::render::ref_clod_point_renderer(ctx, 1);
	if (!sl_manager.init(ctx))
		return false;
	if (!s_renderer.init(ctx))
		return false;
	// here to control use cpu mem or gpu mem 
	s_renderer.enable_attribute_array_manager(ctx, sl_manager);
	s_renderer.set_render_style(surfel_style);

	if (!n_renderer.init(ctx))
		return false;
	n_renderer.set_render_style(normal_style);
	if (!b_renderer.init(ctx))
		return false;
	b_renderer.set_render_style(box_style);
	b_renderer.set_position_is_center(false);
	if (!bw_renderer.init(ctx))
		return false;
	bw_renderer.set_render_style(box_wire_style);
	bw_renderer.set_position_is_center(false);
	return true;
}
///
void gl_point_cloud_drawable::clear(cgv::render::context& ctx)
{
	s_renderer.clear(ctx);
	sl_manager.destruct(ctx);
	n_renderer.clear(ctx);
	b_renderer.clear(ctx);
	bw_renderer.clear(ctx);
}
/// helper functions to switch rendering modes: destruct other vaos 
void gl_point_cloud_drawable::destruct_prog_and_buffers_when_switching(context& ctx) {
	raw_prog.destruct(ctx);
	raw_renderer_out_of_date = true;
	glDeleteVertexArrays(1, &raw_vao);
	glDeleteBuffers(1, &raw_vbo_position);
}
/// helper functions to switch rendering modes: set to quad rendering mode 
void gl_point_cloud_drawable::switch_to_quad_rendering() {
	RENDERING_STRATEGY = 1;
	is_switching = true;
	on_rendering_settings_changed();
}
///
void gl_point_cloud_drawable::draw(context& ctx)
{
	// a quick check 
	if (pc.get_nr_points() == 0)
		return;
	// perhaps the size of the points is already not 0 but not finished loading points 
	if (!finished_loading_points)
		return;

	// 
	draw_boxes(ctx);
	draw_normals(ctx);

	// diff rendering strategies
	if (RENDERING_STRATEGY == 1)
		draw_points_quad(ctx);
	if (RENDERING_STRATEGY == 2) 
		draw_points_point_rendering(ctx);
	if (RENDERING_STRATEGY == 3) 
		draw_points_surfel(ctx);
	if (RENDERING_STRATEGY == 4) 
		draw_points_clod(ctx);
}
/// render with gl points 
void gl_point_cloud_drawable::draw_points_point_rendering(context& ctx) {
	if (pc.get_nr_points() == 0)
		return;
	if (!show_points)
		return;
	if (raw_renderer_out_of_date) {
		if (!raw_prog.is_linked())
			raw_prog.build_program(ctx, "point_vr.glpr", true);
		if (raw_vao == -1)
			glGenVertexArrays(1, &raw_vao);
		if (raw_vbo_position == -1)
			glGenBuffers(1, &raw_vbo_position);

		if (is_switching) {
			raw_prog.destruct(ctx);
			raw_prog.build_program(ctx, "point_vr.glpr", true);
			glDeleteVertexArrays(1, &raw_vao);
			glDeleteBuffers(1, &raw_vbo_position);
			glGenVertexArrays(1, &raw_vao);
			glGenBuffers(1, &raw_vbo_position);
			is_switching = false;
		}

		// prepare data, * 
		input_buffer_data.clear();
		for (int i = 0; i < pc.P.size(); i++) {
			VertexAttributeBinding p;
			p.position = pc.P[i];
			p.color = pc.C[i];
			p.normal = pc.N[i];
			input_buffer_data.push_back(p);
		}

		// bind vao 
		glBindVertexArray(raw_vao);
		// upload all data chunk 
		glBindBuffer(GL_ARRAY_BUFFER, raw_vbo_position);
		glBufferData(GL_ARRAY_BUFFER, input_buffer_data.size() * sizeof(VertexAttributeBinding), input_buffer_data.data(), GL_STATIC_DRAW);
		// find all locations to be used, *
		int strip_size = sizeof(VertexAttributeBinding);
		int color_var_size = 3;
		int color_loc = raw_prog.get_attribute_location(ctx, "color");
		int position_var_size = 3;
		int position_loc = raw_prog.get_attribute_location(ctx, "position");
		int normal_var_size = 3;
		int normal_loc = raw_prog.get_attribute_location(ctx, "normal");
		// specify their format, * 
		glVertexAttribPointer(color_loc, color_var_size, GL_UNSIGNED_BYTE, GL_TRUE, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, color));
		glVertexAttribPointer(position_loc, position_var_size, GL_FLOAT, GL_FALSE, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, position));
		glVertexAttribPointer(normal_loc, normal_var_size, GL_FLOAT, GL_FALSE, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, normal));
		// enable them, * 
		glEnableVertexAttribArray(color_loc);
		glEnableVertexAttribArray(position_loc);
		if(normal_loc) glEnableVertexAttribArray(normal_loc);
		// unbind
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		raw_renderer_out_of_date = false;
	}
	glDisable(GL_CULL_FACE);
	raw_prog.enable(ctx);
	glBindVertexArray(raw_vao);
	glPointSize(5);

	raw_prog.set_uniform(ctx, "enable_headset_culling", enable_headset_culling);
	raw_prog.set_uniform(ctx, "headset_position", headset_position);
	raw_prog.set_uniform(ctx, "headset_direction", headset_direction);
	raw_prog.set_uniform(ctx, "headset_culling_range", headset_culling_range);
	// setup uniforms 
	//int y_view_angle = 45;
	//float pixel_extent_per_depth = (float)(2.0 * tan(0.5 * 0.0174532925199 * y_view_angle) / ctx.get_height());
	//raw_prog.set_uniform(ctx, "pixel_extent_per_depth", pixel_extent_per_depth);
	//raw_prog.set_uniform(ctx, "orient_splats", true);
	//raw_prog.set_uniform(ctx, "point_size", point_size);
	//raw_prog.set_uniform(ctx, "map_color_to_material", 3); // must have 
	//raw_prog.set_uniform(ctx, "blend_points", true);
	//raw_prog.set_uniform(ctx, "percentual_halo_width", percentual_halo_width);

	//raw_prog.set_uniform(ctx, "enable_headset_culling", enable_headset_culling);
	//raw_prog.set_uniform(ctx, "headset_position", headset_position);
	//raw_prog.set_uniform(ctx, "headset_direction", headset_direction);
	//raw_prog.set_uniform(ctx, "headset_culling_range", headset_culling_range);
	//raw_prog.set_uniform(ctx, "enable_acloud_effect", enable_acloud_effect);
	//raw_prog.set_uniform(ctx, "left_controller_position", left_controller_position);
	//raw_prog.set_uniform(ctx, "right_controller_position", right_controller_position);
	//raw_prog.set_uniform(ctx, "controller_effect_range", controller_effect_range);
	glDrawArrays(GL_POINTS, 0, input_buffer_data.size());
	glBindVertexArray(0);
	raw_prog.disable(ctx);
}
/// render with clod rendering 
void gl_point_cloud_drawable::draw_points_clod(context& ctx) {
	if (pc.get_nr_points() == 0)
		return;
	if (!show_points)
		return;
	cgv::render::clod_point_renderer& cp_renderer = ref_clod_point_renderer(ctx);
	cp_style.draw_circles = true;
	cp_renderer.set_render_style(cp_style);
	if (renderer_out_of_date) {
		// cast points from pc to V
		std::vector<LODPoint> V(pc.get_nr_points());
		for (int i = 0; i < pc.get_nr_points(); ++i) {
			V[i].position() = pc.pnt(i);
			V[i].color() = pc.clr(i);
		}
		std::cout << "draw_points_clod: computing lods with octree..." << std::endl;
		points_with_lod = std::move(lod_generator.generate_lods(V));
		std::cout << "draw_points_clod: lods generated" << std::endl;
		// state: lods generated 
		// pass points from points_with_lod to renderer 
		std::cout << "draw_points_clod: uploading points with lods to renderer" << std::endl;
		if (color_based_on_lod) {
			std::vector<LODPoint> pnts = points_with_lod;
			int num_points = pnts.size();
			int max_lod = 0;
			for (int i = 0; i < pc.get_nr_points(); ++i) {
				max_lod = max((int)pnts[i].level(), max_lod);
			}

			std::vector<rgb8> col_lut;
			for (int lod = 0; lod <= max_lod; ++lod) {
				cgv::media::color<float, cgv::media::HLS> col;
				col.L() = 0.5f;
				col.S() = 1.f;
				col.H() = min_level_hue + (max_level_hue - min_level_hue) * ((float)lod / (float)max_lod);
				col_lut.push_back(col);
			}
			for (int i = 0; i < num_points; ++i) {
				pnts[i].color() = col_lut[pnts[i].level()];
			}
			//cp_renderer.set_points(ctx, pnts.data(), pnts.size());
			cp_renderer.set_points(ctx, &pnts.data()->position(), &pnts.data()->color(), &pnts.data()->level(), pnts.size(), sizeof(LODPoint));
		}
		else {
			cp_renderer.set_points(ctx, 
				&points_with_lod.data()->position(), 
				&points_with_lod.data()->color(), 
				&points_with_lod.data()->level(), 
				points_with_lod.size(), sizeof(LODPoint));
		}
		std::cout << "draw_points_clod: done." << std::endl;
		renderer_out_of_date = false;
	}
	if (cp_renderer.enable(ctx))
		cp_renderer.draw(ctx, 0, pc.get_nr_points());
}
/// render test 
void gl_point_cloud_drawable::draw_raw(context& ctx) { // quick test 
	if (pc.get_nr_points() == 0)
		return;
	if (!show_points)
		return;
	if (raw_renderer_out_of_date) {
		raw_prog.build_program(ctx, "default.glpr", true);
		float vertices[] = {
			// positions         // colors
			 0.5f, -0.5f, 0.0f,1,  1.0f, 0.0f, 0.0f,1,   // bottom right
			-0.5f, -0.5f, 0.0f,1,  0.0f, 1.0f, 0.0f,1,   // bottom left
			 0.0f,  0.5f, 0.0f,1,  0.0f, 0.0f, 1.0f,1    // top 
		};

		glGenVertexArrays(1, &raw_vao);
		glGenBuffers(1, &raw_vbo_position);

		glBindVertexArray(raw_vao);
		glBindBuffer(GL_ARRAY_BUFFER, raw_vbo_position);
		glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);
		glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(4 * sizeof(float)));
		glEnableVertexAttribArray(1);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		raw_renderer_out_of_date = false;
	}
	raw_prog.enable(ctx);
	glBindVertexArray(raw_vao);
	glPointSize(5);
	glDrawArrays(GL_POINTS, 0, 3);
	glBindVertexArray(0);
	raw_prog.disable(ctx);
}
/// render with quads 
void gl_point_cloud_drawable::draw_points_quad(context& ctx) {
	if (pc.get_nr_points() == 0)
		return;
	if (!show_points)
		return;
	if (raw_renderer_out_of_date || continus_redraw) {
		if (!raw_prog.is_linked())
			raw_prog.build_program(ctx, "quad_vr.glpr", true);
		if (raw_vao == -1)
			glGenVertexArrays(1, &raw_vao);
		if (raw_vbo_position == -1)
			glGenBuffers(1, &raw_vbo_position);

		if (is_switching) {
			raw_prog.destruct(ctx);
			raw_prog.build_program(ctx, "quad_vr.glpr", true);
			glDeleteVertexArrays(1, &raw_vao);
			glDeleteBuffers(1, &raw_vbo_position);
			glGenVertexArrays(1, &raw_vao);
			glGenBuffers(1, &raw_vbo_position);
			is_switching = false;
		}

		// prepare data, * 
		input_buffer_data.clear();
		for (int i = 0; i < pc.P.size(); i++) {
			VertexAttributeBinding p;
			p.position = pc.P[i];
			p.color = pc.C[i];

			if (pc.has_normals())
				p.normal = pc.N[i];
			else
				p.normal = vec3(0, 1, 0);

			if (pc.has_face_selections())
				// from uint8 -> int 
				p.index = (int)pc.face_id[i]; 
			else
				p.index = 1;

			if (pc.has_scan_index)
				// from 0 -> inf // from float -> int 
				p.scanindex = (int)pc.point_scan_index[i]; 
			else
				p.scanindex = 0;

			input_buffer_data.push_back(p);
		}

		// bind vao 
		glBindVertexArray(raw_vao);
		// upload all data chunk 
		glBindBuffer(GL_ARRAY_BUFFER, raw_vbo_position);
		glBufferData(GL_ARRAY_BUFFER, input_buffer_data.size() * sizeof(VertexAttributeBinding), input_buffer_data.data(), GL_STATIC_DRAW);
		// find all locations to be used, *
		int strip_size = sizeof(VertexAttributeBinding);
		int color_var_size = 3;
		int color_loc = raw_prog.get_attribute_location(ctx, "color");
		int position_var_size = 3;
		int position_loc = raw_prog.get_attribute_location(ctx, "position");
		int normal_var_size = 3;
		int normal_loc = raw_prog.get_attribute_location(ctx, "normal");
		int index_var_size = 1;
		int index_loc = raw_prog.get_attribute_location(ctx, "index");
		int scanindex_var_size = 1;
		int scanindex_loc = raw_prog.get_attribute_location(ctx, "scanindex");
		// specify their format, * 
		glVertexAttribPointer(color_loc,color_var_size, GL_UNSIGNED_BYTE, GL_TRUE, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, color));
		glVertexAttribPointer(position_loc, position_var_size, GL_FLOAT, GL_FALSE, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, position));
		glVertexAttribPointer(normal_loc, normal_var_size, GL_FLOAT, GL_FALSE, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, normal));
		glVertexAttribIPointer(index_loc, index_var_size, GL_INT, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, index));
		glVertexAttribIPointer(scanindex_loc, scanindex_var_size, GL_INT, strip_size,
			(void*)offsetof(struct VertexAttributeBinding, scanindex));
		// enable them, * 
		glEnableVertexAttribArray(color_loc);
		glEnableVertexAttribArray(position_loc);
		glEnableVertexAttribArray(normal_loc);
		glEnableVertexAttribArray(index_loc);
		glEnableVertexAttribArray(scanindex_loc);
		// unbind
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

		raw_renderer_out_of_date = false;
	}
	raw_prog.enable(ctx);
	glBindVertexArray(raw_vao);
	// setup uniforms 
	glDisable(GL_CULL_FACE);
	int y_view_angle = 45;
	float pixel_extent_per_depth = (float)(2.0 * tan(0.5 * 0.0174532925199 * y_view_angle) / ctx.get_height());
	raw_prog.set_uniform(ctx, "pixel_extent_per_depth", pixel_extent_per_depth);
	raw_prog.set_uniform(ctx, "orient_splats", true);
	raw_prog.set_uniform(ctx, "point_size", point_size);
	raw_prog.set_uniform(ctx, "map_color_to_material", 3); // must have this 
	/*raw_prog.set_uniform(ctx, "halo_width_in_pixel", 10);
	* //
	*/
	raw_prog.set_uniform(ctx, "blend_points", true);
	raw_prog.set_uniform(ctx, "percentual_halo_width", percentual_halo_width);

	raw_prog.set_uniform(ctx, "enable_headset_culling", enable_headset_culling);
	raw_prog.set_uniform(ctx, "headset_position", headset_position);
	raw_prog.set_uniform(ctx, "headset_direction", headset_direction);
	raw_prog.set_uniform(ctx, "headset_culling_range", headset_culling_range);
	raw_prog.set_uniform(ctx, "enable_acloud_effect", enable_acloud_effect);
	raw_prog.set_uniform(ctx, "left_controller_position", left_controller_position);
	raw_prog.set_uniform(ctx, "right_controller_position", right_controller_position);
	raw_prog.set_uniform(ctx, "controller_effect_range", controller_effect_range);

	/*shading effects*/
	raw_prog.set_uniform(ctx, "which_effect_righthand", which_effect_righthand);
	raw_prog.set_uniform(ctx, "which_effect_lefthand", which_effect_lefthand);
	raw_prog.set_uniform(ctx, "which_effect_headset", which_effect_headset);

	/*adjustable effects */
	raw_prog.set_uniform(ctx, "collapse_tantheta", collapse_tantheta);
	raw_prog.set_uniform(ctx, "colorize_with_scan_index", colorize_with_scan_index);

	/*scan index */
	raw_prog.set_uniform(ctx, "renderScan0", renderScan0);
	raw_prog.set_uniform(ctx, "renderScan1", renderScan1);
	raw_prog.set_uniform(ctx, "renderScan2", renderScan2);
	raw_prog.set_uniform(ctx, "renderScan3", renderScan3);
	raw_prog.set_uniform(ctx, "renderScan4", renderScan4);
	raw_prog.set_uniform(ctx, "renderScan5", renderScan5);

	/*additional model matrix*/
	//additional_model_matrix.identity();
	// todo: modi. the shader. always use 
	raw_prog.set_uniform(ctx, "use_additional_model_matrix", true);
	//
	if(use_current_matrix)
		raw_prog.set_uniform(ctx, "additional_model_matrix", current_model_matrix);
	else
		raw_prog.set_uniform(ctx, "additional_model_matrix", last_model_matrix);
		

	glDrawArrays(GL_POINTS, 0, input_buffer_data.size());
	glBindVertexArray(0);
	raw_prog.disable(ctx);
}
/// set arrays for surfel renderer 
void gl_point_cloud_drawable::set_arrays(context& ctx, size_t offset, size_t count)
{
	if (count == -1)
		count = pc.get_nr_points();

	// upload vertices, one must have, others are optional  
	s_renderer.set_position_array(ctx, &pc.pnt(unsigned(offset)), pc.get_nr_points(), unsigned(sizeof(Pnt))*show_point_step);
	
	// upload colors 
	if (pc.has_colors()) {
		s_renderer.set_color_array(ctx, &pc.clr(unsigned(offset)), count, unsigned(sizeof(Clr)) * show_point_step);
	}

	// upload normals 
	if (pc.has_normals())
		s_renderer.set_normal_array(ctx, &pc.nml(unsigned(offset)), count, unsigned(sizeof(Nml))*show_point_step);

	// upload point selection, point_color_indices indicates face id or topo info id 
	if (pc.has_face_selections()) {
		s_renderer.set_attribute_array_renderer(ctx,
			"face_id",
			&pc.face_id.at(unsigned(offset)),
			count,
			unsigned(sizeof(int)) * show_point_step
		);
		// do not forget to set renderer parameters... private varible init to true. so we do not have to do this for now: 
		// s_renderer.has_indexed_colors = true; (adjust renderer later )
	}

	// upload topological selection, indicates corner/ edge/ faces...
	if (pc.has_topo_selections()) {
		s_renderer.set_attribute_array_renderer(ctx,
			"topo_id",
			&pc.topo_id.at(unsigned(offset)),
			count,
			unsigned(sizeof(int)) * show_point_step
		);
		// do not forget to set renderer parameters... private varible init to true. so we do not have to do this for now: 
		// s_renderer.has_indexed_colors = true; (adjust renderer later )
	}

	// upload scan indices 
	// a good refernce when adding point attributes 
	// actually, this will always be true, I fill the fields with some default values 
	if (pc.has_scan_indices()) {
		s_renderer.set_attribute_array_renderer(ctx,
			"scan_index", 
			&pc.point_scan_index.at(unsigned(offset)), 
			count, 
			unsigned(sizeof(float)) * show_point_step
		);
	}

	
}
/// render with surfel 
void gl_point_cloud_drawable::draw_points_surfel(context& ctx)
{
	if (!show_points)
		return;

	// do this when switching renderers 
	if (raw_prog.is_linked())
		destruct_prog_and_buffers_when_switching(ctx);

	// upload/ update attributes to shaders 
	set_arrays(ctx);

	// adjust renderer parameters 
	surfel_style.use_group_color = false;
	s_renderer.validate_and_enable(ctx);
	s_renderer.ref_prog().set_uniform(ctx, "enable_headset_culling", enable_headset_culling);
	s_renderer.ref_prog().set_uniform(ctx, "headset_position", headset_position);
	s_renderer.ref_prog().set_uniform(ctx, "headset_direction", headset_direction);
	s_renderer.ref_prog().set_uniform(ctx, "headset_culling_range", headset_culling_range);

	s_renderer.ref_prog().set_uniform(ctx, "enable_acloud_effect", enable_acloud_effect);
	s_renderer.ref_prog().set_uniform(ctx, "left_controller_position", left_controller_position);
	s_renderer.ref_prog().set_uniform(ctx, "right_controller_position", right_controller_position);
	s_renderer.ref_prog().set_uniform(ctx, "controller_effect_range", controller_effect_range);

	s_renderer.ref_prog().set_uniform(ctx, "visual_delete", visual_delete);
	s_renderer.ref_prog().set_uniform(ctx, "render_with_original_color", render_with_original_color);
	s_renderer.ref_prog().set_uniform(ctx, "colorize_with_scan_index", colorize_with_scan_index);

	s_renderer.ref_prog().set_uniform(ctx, "renderScan0", renderScan0);
	s_renderer.ref_prog().set_uniform(ctx, "renderScan1", renderScan1);
	s_renderer.ref_prog().set_uniform(ctx, "renderScan2", renderScan2);
	s_renderer.ref_prog().set_uniform(ctx, "renderScan3", renderScan3);
	s_renderer.ref_prog().set_uniform(ctx, "renderScan4", renderScan4);
	s_renderer.ref_prog().set_uniform(ctx, "renderScan5", renderScan5);

	s_renderer.ref_prog().set_uniform(ctx, "render_with_functional_ids_only", render_with_functional_ids_only);
	s_renderer.ref_prog().set_uniform(ctx, "highlight_unmarked_points", highlight_unmarked_points);
	s_renderer.ref_prog().set_uniform(ctx, "force_render_with_original_color", force_render_with_original_color);
	
	std::size_t n = pc.get_nr_points();
		//(show_point_end - show_point_begin) / show_point_step;
	GLint offset = GLint(show_point_begin / show_point_step);

	if (sort_points && ensure_view_pointer()) {
		struct sort_pred {
			const point_cloud& pc;
			const Pnt& view_dir;
			bool operator () (GLuint i, GLuint j) const {
				return dot(pc.pnt(i), view_dir) > dot(pc.pnt(j), view_dir);
			}
			sort_pred(const point_cloud& _pc, const Pnt& _view_dir) : pc(_pc), view_dir(_view_dir) {}
		};
		Pnt view_dir = view_ptr->get_view_dir();
		std::vector<GLuint> indices;
		if (pc.has_components() && use_these_component_colors) {
			unsigned nr = 0;
			for (unsigned ci = 0; ci < pc.get_nr_components(); ++ci) {
				if ((*use_these_component_colors)[ci][3] > 0.0f) {
					unsigned off = unsigned(pc.components[ci].index_of_first_point);
					for (unsigned i = 0; i < pc.components[ci].nr_points; ++i)
						indices.push_back((GLuint)(off + i));
				}
			}
		}
		else {
			indices.resize(n);
			size_t i;
			for (i = 0; i < indices.size(); ++i)
				indices[i] = (GLuint)(show_point_step*i) + offset;
		}
		std::sort(indices.begin(), indices.end(), sort_pred(pc, view_dir));

		glDepthFunc(GL_LEQUAL);
		size_t nn = indices.size() / nr_draw_calls;
		for (unsigned i = 1; i<nr_draw_calls; ++i)
			glDrawElements(GL_POINTS, GLsizei(nn), GL_UNSIGNED_INT, &indices[(i - 1)*nn]);
		glDrawElements(GL_POINTS, GLsizei(indices.size() - (nr_draw_calls - 1)*nn), GL_UNSIGNED_INT, &indices[(nr_draw_calls - 1)*nn]);
		glDepthFunc(GL_LESS);
	}
	else {
		if (pc.has_components() && use_these_component_colors) {
			for (unsigned ci = 0; ci < pc.get_nr_components(); ++ci) {
				if ((*use_these_component_colors)[ci][3] > 0.0f) {
					set_arrays(ctx, pc.components[ci].index_of_first_point, pc.components[ci].nr_points);
					glDrawArrays(GL_POINTS, 0, GLsizei(pc.components[ci].nr_points));
				}
			}
		}
		else {
			size_t nn = n / nr_draw_calls;
			for (unsigned i=1; i<nr_draw_calls; ++i)
				glDrawArrays(GL_POINTS, GLint(offset + (i-1)*nn), GLsizei(nn));
			glDrawArrays(GL_POINTS, GLint(offset + (nr_draw_calls - 1)*nn), GLsizei(n-(nr_draw_calls - 1)*nn));
		}
	}
	s_renderer.disable(ctx);
}