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

class parametric_surface :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	cgv::render::shader_program parametric_surface_prog;
	int nr_quads_per_row = 100;
public:
	/// initialize rotation angle
	parametric_surface()
	{
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
		// check and build prog for the first enter 
		if (!parametric_surface_prog.is_linked()) {
			if (!parametric_surface_prog.build_program(ctx, "parametric_surface.glpr", true)) {
				std::cerr << "could not build height_field shader program" << std::endl;
			}
		}

		// render one parametric surface patch with 16 control points 
		// small quads are generated
		// ranging from (-0.5,0.5) in xz plane for each, centered in origin 
		glDisable(GL_CULL_FACE);
		parametric_surface_prog.enable(ctx);// currently, quad = texel, transform
		parametric_surface_prog.set_uniform(ctx, "nr_quads_per_row", nr_quads_per_row);
		parametric_surface_prog.set_uniform(ctx, "transform", vec4(0,0,0,0));
		parametric_surface_prog.set_uniform(ctx, "texel_extent", 1.0f / float(nr_quads_per_row));
		parametric_surface_prog.set_uniform(ctx, "quad_extent", 1.0f / float(nr_quads_per_row));
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		glEnableClientState(GL_VERTEX_ARRAY);
		// send points to vertex shader/ geo shader, in uv space 
		glDrawArraysInstanced(GL_POINTS, 0, 1, nr_quads_per_row * nr_quads_per_row);
		glDisableClientState(GL_VERTEX_ARRAY);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		parametric_surface_prog.disable(ctx);

		// bbox quick test 
		/*auto& prog = ctx.ref_surface_shader_program();
		prog.enable(ctx);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4(0.5,0.5,0.5));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();
		prog.disable(ctx);*/
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
