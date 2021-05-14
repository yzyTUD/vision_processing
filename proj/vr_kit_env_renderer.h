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

class env_renderer :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	std::vector<box3> boxes;
	std::vector<rgb> box_colors;
	cgv::render::box_render_style style;
	constexpr static float table_height = 0.7f;
public:
	bool show_boxes_environment = true;
	/// initialize rotation angle
	env_renderer()
	{
		connect(get_animation_trigger().shoot, this, &env_renderer::timer_event);
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
		return "env_renderer";
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
	void setup_scene() {
		build_scene(5, 7, 3, 0.2f, 1.6f, 0.8f, table_height, 0.03f);
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(style);
		renderer.set_box_array(ctx, boxes);
		renderer.set_color_array(ctx, box_colors);
		renderer.render(ctx, 0, boxes.size());
	}
	/// overload the create gui method
	void create_gui()
	{

	}
	/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
	void construct_table(float tw, float td, float th, float tW) {
		// construct table
		rgb table_clr(0.3f, 0.2f, 0.0f);
		boxes.push_back(box3(
			vec3(-0.5f * tw - 2 * tW, th - tW, -0.5f * td - 2 * tW),
			vec3(0.5f * tw + 2 * tW, th, 0.5f * td + 2 * tW)));
		box_colors.push_back(table_clr);

		boxes.push_back(box3(vec3(-0.5f * tw, 0, -0.5f * td), vec3(-0.5f * tw - tW, th - tW, -0.5f * td - tW)));
		boxes.push_back(box3(vec3(-0.5f * tw, 0, 0.5f * td), vec3(-0.5f * tw - tW, th - tW, 0.5f * td + tW)));
		boxes.push_back(box3(vec3(0.5f * tw, 0, -0.5f * td), vec3(0.5f * tw + tW, th - tW, -0.5f * td - tW)));
		boxes.push_back(box3(vec3(0.5f * tw, 0, 0.5f * td), vec3(0.5f * tw + tW, th - tW, 0.5f * td + tW)));
		box_colors.push_back(table_clr);
		box_colors.push_back(table_clr);
		box_colors.push_back(table_clr);
		box_colors.push_back(table_clr);
	}

	/// construct boxes that represent a room of dimensions w,d,h and wall width W
	void construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
		// construct floor
		boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d), vec3(0.5f * w, 0, 0.5f * d)));
		box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

		if (walls) {
			// construct walls
			boxes.push_back(box3(vec3(-0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w, h, -0.5f * d)));
			box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
			boxes.push_back(box3(vec3(-0.5f * w, -W, 0.5f * d), vec3(0.5f * w, h, 0.5f * d + W)));
			box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

			boxes.push_back(box3(vec3(0.5f * w, -W, -0.5f * d - W), vec3(0.5f * w + W, h, 0.5f * d + W)));
			box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
		}
		if (ceiling) {
			// construct ceiling
			boxes.push_back(box3(vec3(-0.5f * w - W, h, -0.5f * d - W), vec3(0.5f * w + W, h + W, 0.5f * d + W)));
			box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
		}
	}

	void construct_environment(float s, float ew, float ed, float w, float d, float h) {
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0, 1);
		unsigned n = unsigned(ew / s);
		unsigned m = unsigned(ed / s);
		float ox = 0.5f * float(n) * s;
		float oz = 0.5f * float(m) * s;
		for (unsigned i = 0; i < n; ++i) {
			float x = i * s - ox;
			for (unsigned j = 0; j < m; ++j) {
				float z = j * s - oz;
				if (fabsf(x) < 0.5f * w && fabsf(x + s) < 0.5f * w && fabsf(z) < 0.5f * d && fabsf(z + s) < 0.5f * d)
					continue;
				float h = 0.2f * (std::max(abs(x) - 0.5f * w, 0.0f) + std::max(abs(z) - 0.5f * d, 0.0f)) * distribution(generator) + 0.1f;
				boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
				constexpr float hue_radius = 0.3;
				constexpr float hue_center = 0.4;
				rgb color = cgv::media::color<float, cgv::media::HLS>(fmod(hue_center + hue_radius * distribution(generator), 1.f), 0.1f * distribution(generator) + 0.15f, 0.6f);
				box_colors.push_back(color);
			}
		}
	}

	/// construct a scene with a table
	void build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
	{
		construct_room(w, d, h, W, false, false);
		construct_table(tw, td, th, tW);
		if (show_boxes_environment)
			construct_environment(0.30f, 3 * w, 3 * d, w, d, h);
	}
};
