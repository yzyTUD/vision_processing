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

class vr_kit_light :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;

	bool light_changed = false; 
public:
	/// initialize rotation angle
	vr_kit_light()
	{
		connect(get_animation_trigger().shoot, this, &vr_kit_light::timer_event);
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
		return "vr_kit_light";
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
		if (!light_changed && ctx.get_nr_light_sources()>0) {
			//cgv::media::illum::light_source ls = ctx.get_light_source(ctx.get_enabled_light_source_handle(0));
			//ctx.disable_light_source(ctx.get_enabled_light_source_handle(0));

			auto lss = ctx.ref_light_sources();
			for (auto it = lss->begin(); it != lss->end(); ++it) {
				it->second.first.set_local_to_eye(false);
				it->second.first.set_ambient_scale(0.1);
			}

			ctx.on_lights_changed();

			auto& prog = ctx.ref_surface_shader_program();
			prog.enable(ctx);
			ctx.tesselate_unit_cube();
			prog.disable(ctx);

			light_changed = true;
		}
	}
	/// overload the create gui method
	void create_gui()
	{
	}

};
