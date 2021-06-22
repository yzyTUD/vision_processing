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

class renderable_octree : public render_types {
	//
	std::vector<box3> boxes;
	std::vector<rgb> colors;

	//
	box_render_style box_style;
	cgv::render::box_wire_render_style box_wire_style;
	bool wireframe = true;
public:
	renderable_octree() {
		boxes.push_back(box3(vec3(0, 0, 0), vec3(1, 1, 1)));
		boxes.push_back(box3(vec3(1, 1, 1), vec3(2, 2, 2)));
		colors.push_back(rgb(0, 0, 1));
		colors.push_back(rgb(0, 0, 1));
	}

	void render(context& ctx) {
		glEnable(GL_CULL_FACE);
		cgv::render::ref_box_renderer(ctx, 1);
		cgv::render::ref_box_wire_renderer(ctx, 1);
		if (wireframe) {
			cgv::render::box_wire_renderer& bw_renderer = cgv::render::ref_box_wire_renderer(ctx);
			bw_renderer.set_render_style(box_wire_style);
			bw_renderer.set_box_array(ctx, boxes);
			bw_renderer.set_color_array(ctx, colors);
			bw_renderer.render(ctx, 0, boxes.size());
		}
		else {
			cgv::render::box_renderer& b_renderer = cgv::render::ref_box_renderer(ctx);
			b_renderer.set_render_style(box_style);		
			b_renderer.set_box_array(ctx, boxes);
			b_renderer.set_color_array(ctx, colors);
			b_renderer.render(ctx, 0, boxes.size());
		}
	}
};

class vis_pw_renderer :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	renderable_octree rd_octree;
public:
	/// initialize rotation angle
	vis_pw_renderer()
	{
		connect(get_animation_trigger().shoot, this, &vis_pw_renderer::timer_event);
	}
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
		return "vis_pw_renderer";
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
	bool handle(event& e)
	{
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	///
	void render_octree(context& ctx) {

	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		rd_octree.render(ctx);
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
