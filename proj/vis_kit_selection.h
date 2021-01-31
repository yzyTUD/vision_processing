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
#include <cgv/gui/mouse_event.h>
#include <cgv/render/context.h>

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
#include "vis_kit_datastore.h"
#include "vis_visual_processing.h"

class vis_kit_selection :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	stereo_view_interactor* view_ptr = nullptr;
	cgv::render::context* ctx_ptr = nullptr;

	int pick_point_index = -1;
	cgv::render::sphere_render_style sphere_style;
	bool in_picking = false;

public:
	/// initialize rotation angle
	vis_kit_selection()
	{
		connect(get_animation_trigger().shoot, this, &vis_kit_selection::timer_event); 
		sphere_style.map_color_to_material = CM_COLOR_AND_OPACITY;
		sphere_style.surface_color = rgb(0.8f, 0.4f, 0.4f);
		sphere_style.radius = 0.1;
		//sphere_style.radius = float(0.05*sqrt(B.get_extent().sqr_length() / vertex_count));
		
	}
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
	}
	void set_view_ptr(stereo_view_interactor* vp) {
		view_ptr = vp;
	}
	void set_context_str(cgv::render::context* cptr) {
		ctx_ptr = cptr;
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
		return "vis_kit_selection";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}

	bool on_pick(const cgv::gui::mouse_event & me)
	{
		if (!view_ptr)
			return false;
		if (!data_ptr)
			return false;
		if (ctx_ptr == nullptr)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location_ctx(*ctx_ptr, me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z) ||
			window_z > 0.999)
			return false;
		// check whether to pick a previously defined point
		pick_point_index = -1;
		double pick_dist = 0;
		double pick_dist_threshold = sphere_style.radius * sphere_style.radius_scale;
		for (int i = 0; i < (int)data_ptr->pick_points.size(); ++i) {
			double dist = (data_ptr->pick_points[i] - vec3(pick_point)).length();
			if (dist < pick_dist_threshold) {
				if (pick_point_index == -1 || dist < pick_dist) {
					pick_dist = dist;
					pick_point_index = i;
				}
			}
		}
		if (pick_point_index == -1) {
			pick_point_index = data_ptr->pick_points.size();
			data_ptr->pick_points.push_back(pick_point);
			data_ptr->pick_colors.push_back(rgb(0, 1, 0));
			return true;
		}
		return false;
		
	}
	
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
		post_redraw();
		return true;
	}

	/// overload to handle events, return true if event was processed
	/// call this 
	bool handle(event& e)
	{
		if (e.get_kind() == EID_MOUSE) {
			auto& me = static_cast<cgv::gui::mouse_event&>(e);
			switch (me.get_action()) {
				case MA_PRESS:
					if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_CTRL) {
						if (!view_ptr)
							return false;
						in_picking = true;
						on_pick(me);
						post_redraw();
						return true;
					}
					break;
				case MA_DRAG:
					if (in_picking) {
						on_drag(me);
						return true;
					}
					break;
			}
		}
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		
	}
	/// call this 
	void finish_frame(context& ctx) {
		if (!view_ptr)
			return;
		if (!data_ptr)
			return;

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		sphere_renderer& sr = ref_sphere_renderer(ctx);
		if (view_ptr)
			sr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
		sr.set_render_style(sphere_style );

		glDepthMask(GL_FALSE);
		if (!data_ptr->pick_points.empty()) {
			sr.set_position_array(ctx, data_ptr->pick_points);
			sr.set_color_array(ctx, data_ptr->pick_colors);
			sr.validate_and_enable(ctx);
			glDrawArrays(GL_POINTS, 0, (GLsizei)data_ptr->pick_points.size());
			sr.disable(ctx);
		}
		if (pick_point_index != -1) {
			glDisable(GL_BLEND);
			glDisable(GL_DEPTH_TEST);
			vec3 p = data_ptr->pick_points[pick_point_index];
			if (view_ptr)
				p += 1.5f * sphere_style.radius * sphere_style.radius_scale * vec3(view_ptr->get_view_up_dir());
			std::stringstream ss;
			ss << "[" << p << "]";
			ss.flush();

			ctx.set_color(rgb(0.1f, 0.1f, 0.1f));
			ctx.set_cursor(p.to_vec(), ss.str(), (TextAlignment)TA_BOTTOM, 0, 0);
			ctx.output_stream() << ss.str();
			ctx.output_stream().flush();

			ctx.set_color(rgb(0.9f, 0.9f, 0.9f));
			ctx.set_cursor(p.to_vec(), ss.str(), (TextAlignment)TA_BOTTOM, 1, -1);
			ctx.output_stream() << ss.str();
			ctx.output_stream().flush();

			glEnable(GL_DEPTH_TEST);
			glEnable(GL_BLEND);
		}
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
