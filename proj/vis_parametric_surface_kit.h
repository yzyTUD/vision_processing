#pragma once
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/base/register.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/key_event.h>
#include <cgv/utils/ostream_printf.h>
#include <cgv/gui/provider.h>
#include <cgv/math/ftransform.h>
#include <cgv/media/illum/surface_material.h>
#include "vis_kit_datastore.h"

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>
#include "vr_kit_intersection.h"

class SurfacePatch {
	typedef cgv::media::color<float, cgv::media::RGB> rgb;
	typedef typename cgv::math::fvec<float, 2> vec2;

	//
	cgv::render::shader_program parametric_surface_prog;
	sphere_render_style ctrlpoint_rendering_style;
	int nr_quads_per_row = 100; // not related to delay
	bool prepared = false;
	std::vector<vec2> texcoord;

	//
	vec3 control_point_array[16]; // easier uploading to gpu 
	std::vector<vec3> control_points; // local, 16 elements
	std::vector<rgb> control_point_colors; // local, 16 elements
public:
	SurfacePatch() {
		ctrlpoint_rendering_style.radius = 0.01;
		control_points.resize(16);
		control_point_colors.resize(16);
	}
	//void build_demo() {
	//	// for bezier patch rendering 
	//	// write control_point_array
	//	control_point_array[0] = vec3(0, 0, 0);
	//	control_point_array[1] = vec3(0, 1.01, 1);
	//	control_point_array[2] = vec3(0, 1.08, 2);
	//	control_point_array[3] = vec3(0, 0, 3);

	//	control_point_array[4] = vec3(1, 1.2, 0);
	//	control_point_array[5] = vec3(1, 1.3, 1);
	//	control_point_array[6] = vec3(1, 1.2, 2);
	//	control_point_array[7] = vec3(1, 1.1, 3);
	//
	//	control_point_array[8] = vec3(2, 1.4, 0);
	//	control_point_array[9] = vec3(2, 1.6, 1);
	//	control_point_array[10] = vec3(2, 1.1, 2);
	//	control_point_array[11] = vec3(2, 1.2, 3);

	//	control_point_array[12] = vec3(3, 0, 0);
	//	control_point_array[13] = vec3(3, 1.3, 1);
	//	control_point_array[14] = vec3(3, 1.4, 2);
	//	control_point_array[15] = vec3(3, 0, 3);

	//	// for control points rendering 
	//	// write to control_points
	//	for (int i = 0; i < 16; i++) 
	//		control_points.at(i) = control_point_array[i];
	//	// write to control_point_colors 
	//	for (int i = 0; i < 16; i++) 
	//		control_point_colors.at(i) = rgb(0,0,1);
	//}
	void update(std::vector<vec3>* cpgptr,std::vector<rgb>* cpcptr, std::vector<int>* cpi) {
		// write to control_points
		for (int i = 0; i < cpi->size(); i++) 
			control_points.at(i) = cpgptr->at(cpi->at(i));
		// write to control_point_colors 
		for (int i = 0; i < cpi->size(); i++) 
			control_point_colors.at(i) = cpcptr->at(cpi->at(i));
	}
	void render_surface_patch_instanced(context& ctx) {
		// lazy check and build prog for the loop
		if (!prepared) {
			if (!parametric_surface_prog.build_program(ctx, "parametric_surface.glpr", true)) {
				std::cerr << "could not build height_field shader program" << std::endl;
			}
			//parametric_surface_prog.set_attribute();
			prepared = true;
		}
		if (control_points.size() != 16)
			return;
		// render one parametric surface patch with 16 control points 
		// small quads are generated
		// ranging from (-0.5,0.5) in xz plane for each, centered in origin 
		glDisable(GL_CULL_FACE);
		parametric_surface_prog.enable(ctx);// currently, quad = texel, transform
		parametric_surface_prog.set_uniform(ctx, "nr_quads_per_row", nr_quads_per_row);
		parametric_surface_prog.set_uniform(ctx, "transform", vec4(0, 0, 0, 0));
		parametric_surface_prog.set_uniform(ctx, "texel_extent", 1.0f / float(nr_quads_per_row));
		parametric_surface_prog.set_uniform(ctx, "quad_extent", 1.0f / float(nr_quads_per_row));
		parametric_surface_prog.set_uniform_array(ctx, "control_points", control_points); // not related
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glEnableClientState(GL_VERTEX_ARRAY);
		// send points to vertex shader/ geo shader, in uv space 
		glDrawArraysInstanced(GL_POINTS, 0, 1, nr_quads_per_row * nr_quads_per_row);
		glDisableClientState(GL_VERTEX_ARRAY);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		parametric_surface_prog.disable(ctx);
	}
	void render_surface_patch_point_based(context& ctx) {
		// lazy check and build prog for the loop
		if (!prepared) {
			if (!parametric_surface_prog.build_program(ctx, "parametric_surface_pointbased.glpr", true)) {
				std::cerr << "could not build height_field shader program" << std::endl;
			}
			for (int i = 0; i < nr_quads_per_row; i++) {
				for (int j = 0; j < nr_quads_per_row; j++) {
					texcoord.push_back(
						vec2((float)i/ nr_quads_per_row,(float)j / nr_quads_per_row));
				}
			}
			
			prepared = true;
		}
		if (control_points.size() != 16)
			return;
		cgv::render::attribute_array_binding::set_global_attribute_array(
				ctx, parametric_surface_prog.get_attribute_location(ctx, "inTexcoord"), texcoord);
		// render one parametric surface patch with 16 control points 
		// small quads are generated
		// ranging from (-0.5,0.5) in xz plane for each, centered in origin 
		parametric_surface_prog.enable(ctx);// currently, quad = texel, transform
		parametric_surface_prog.set_uniform_array(ctx, "control_points", control_points); // not related
		glDrawArrays(GL_POINTS,0, texcoord.size());
		parametric_surface_prog.disable(ctx);
	}
	void render_control_points(context& ctx) {
		if (control_points.size() > 0) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(ctrlpoint_rendering_style);
			sr.set_position_array(ctx, control_points);
			sr.set_color_array(ctx, control_point_colors);
			sr.render(ctx, 0, control_points.size());
		}
	}
};

class parametric_surface :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	std::vector<SurfacePatch> surface_patches; // global 
public:
	/// initialize rotation angle
	parametric_surface()
	{
		surface_patches.push_back(*(new SurfacePatch()));
	}
	/// external api 
	void fetch_from_point_cloud_kit() {
		// abubrive some vars 
		std::vector<mFace>* mf_ptr = &data_ptr->point_cloud_kit->pc.modelFace;
		// resize surface patches 
		surface_patches.resize(mf_ptr->size());
		// update surface patches 
		std::vector<vec3>* control_points_g_ptr = &data_ptr->point_cloud_kit->pc.control_points;
		std::vector<rgb>* control_point_colors_ptr = &data_ptr->point_cloud_kit->pc.control_point_colors;
		// for each modelFace, build a surface patch for rendering 
		for (int i = 0; i < mf_ptr->size(); i++) {
			// ignore patches that is not ready 
			if (!mf_ptr->at(i).ready_for_rendering)
				continue;
			//
			std::vector<int>* control_point_indices_ptr = &((mf_ptr->at(i)).control_point_indices);
			surface_patches.at(i).update(control_points_g_ptr,
				control_point_colors_ptr, control_point_indices_ptr);
		}
	}
	/// download control points to local representation 
	void fetch_from_point_cloud_kit_demo() {
		surface_patches.resize(1);
		std::vector<vec3>* control_points_g_ptr = &data_ptr->point_cloud_kit->pc.control_points;
		std::vector<rgb>* control_point_colors_ptr = &data_ptr->point_cloud_kit->pc.control_point_colors;
		std::vector<int>* control_point_indices_ptr = &data_ptr->point_cloud_kit->pc.demo_surface;
		surface_patches.at(0).update(control_points_g_ptr,
			control_point_colors_ptr, control_point_indices_ptr);
	}
	/// directly load 
	void load_from_file() {

	}
	/// external api 
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
	}
	/// 
	void on_set(void* member_ptr)
	{
		update_member(member_ptr);
		post_redraw();
	}
	/// self reflection allows to change values in the config file
	bool self_reflect(reflection_handler& rh)
	{
		return true;
	}
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "parametric_surface";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}
	/// overload to handle events, return true if event was processed
	/// external api 
	bool handle(event& e)
	{
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	/// setting the view transform yourself
	/// external api 
	void draw(context& ctx)
	{
		// update patches, to support real time modification
		fetch_from_point_cloud_kit_demo();
		//
		if (data_ptr->render_parametric_surface) {
			for (auto& sp : surface_patches) {
				sp.render_surface_patch_instanced(ctx);
			}
		}

		// bbox quick test 
		/*auto& prog = ctx.ref_surface_shader_program();
		prog.enable(ctx);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4(0.5,0.5,0.5));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();
		prog.disable(ctx);*/

		// render spheres for control points 
		if (data_ptr->render_control_points) {
			for (auto& sp : surface_patches) {
				sp.render_control_points(ctx);
			}
		}
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
