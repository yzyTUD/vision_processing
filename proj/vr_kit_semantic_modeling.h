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

class semantic_modeling :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
public:
	/// initialize rotation angle
	semantic_modeling()
	{
		connect(get_animation_trigger().shoot, this, &semantic_modeling::timer_event);
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
		return "semantic_modeling";
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
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		auto& prog = ctx.ref_surface_shader_program();
		prog.enable(ctx);
		ctx.tesselate_unit_cube();
		prog.disable(ctx);
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
