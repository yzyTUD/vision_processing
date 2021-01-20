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
#include <libs/cgv_gl/sphere_renderer.h>
#include <libs/cgv_gl/box_renderer.h>
#include <random>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

// muster class 
class vr_kit_roller_coaster :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	cgv::render::sphere_render_style sphere_style;
	cgv::render::rounded_cone_render_style rounded_cone_style;

	std::vector<float> indicator_radii;
	std::vector<rgb> indicator_color;
	std::vector<vec3> indicator_trans;

	std::vector<vec3> para_curve_posi_list;
	std::vector<rgb> color_para_curve;
public:
public:
	/// initialize rotation angle
	vr_kit_roller_coaster()
	{
		generate_para_curve();
		connect(get_animation_trigger().shoot, this, &vr_kit_roller_coaster::timer_event);
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
		return "vr_kit_roller_coaster";
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
		return true;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		indicator_trans.front() = get_posi_para_curve(t,true);
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		render_para_curves_with_cones(ctx);
	}
	/*
		render para curves
	*/
	vec3 get_posi_para_curve(float t,bool from_timer_event) {
		if(from_timer_event)
			t = t / 10;
		float x = 0, y = 0, z = 0;
		x = cos(t) * (3 + sin(8 * t));
		z = sin(t) * (3 + sin(8 * t));
		y = cos(8 * t);
		return vec3(x, y, z);
	}
	void generate_para_curve() {
		std::default_random_engine g;
		std::uniform_real_distribution<float> d(0, 1);
		float t = 0;
		float x = 0, y = 0, z = 0;
		float step = 2 * M_PI / 1000;
		while (t < 2 * M_PI) {
			para_curve_posi_list.push_back(get_posi_para_curve(t,false));
			color_para_curve.push_back(rgb(d(g), d(g), d(g)));
			t += step;
		}
		rounded_cone_style.radius = 0.01f;
		generate_one_sphere_to_indicate();
	}
	void generate_one_sphere_to_indicate() {
		indicator_trans.push_back(get_posi_para_curve(0, false));
		indicator_color.push_back(rgb(0,1,0));
		sphere_style.radius = 0.5;
	}
	void render_para_curves_with_cones(context& ctx) {
		if (para_curve_posi_list.size()) {
			auto& rc_renderer = cgv::render::ref_rounded_cone_renderer(ctx);
			rc_renderer.set_render_style(rounded_cone_style);
			rc_renderer.set_position_array(ctx, &para_curve_posi_list.front(), para_curve_posi_list.size(), sizeof(vec3));
			rc_renderer.set_color_array(ctx, &color_para_curve.front(), color_para_curve.size(), sizeof(rgb));
			rc_renderer.render(ctx, 0, para_curve_posi_list.size());

			rc_renderer.set_position_array(ctx, &para_curve_posi_list.at(1), para_curve_posi_list.size(), sizeof(vec3));
			rc_renderer.set_color_array(ctx, &color_para_curve.at(1), color_para_curve.size(), sizeof(rgb));
			rc_renderer.render(ctx, 0, para_curve_posi_list.size() - 1);
			
			std::vector<vec3> tmp_posi;
			std::vector<rgb> tmp_color;
			tmp_posi.push_back(para_curve_posi_list.back());
			tmp_posi.push_back(para_curve_posi_list.front());
			tmp_color.push_back(color_para_curve.back());
			tmp_color.push_back(color_para_curve.front());
			rc_renderer.set_position_array(ctx, tmp_posi);
			rc_renderer.set_color_array(ctx, tmp_color);
			rc_renderer.render(ctx, 0, 2);
		}
		if (indicator_trans.size()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(sphere_style);
			sr.set_position_array(ctx, indicator_trans);
			sr.set_color_array(ctx, indicator_color);
			sr.render(ctx, 0, indicator_trans.size());
		}
	}

	/// overload the create gui method
	void create_gui()
	{

	}
};

// has its own drawable, construction methods 
class vr_kit_roller_coaster_1 :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	cgv::render::sphere_render_style sphere_style;
	cgv::render::rounded_cone_render_style rounded_cone_style;

	std::vector<float> indicator_radii;
	std::vector<rgb> indicator_color;
	std::vector<vec3> indicator_trans;

	std::vector<vec3> para_curve_posi_list;
	std::vector<rgb> color_para_curve;
public:
	float para_y = 8, para_x = 8, para_z = 8, para_x_0 = 3, para_z_0 = 3;
	int speed_factor = 5;
	int resolution = 1000;
public:
	/// initialize rotation angle
	vr_kit_roller_coaster_1()
	{
		generate_para_curve();
		connect(get_animation_trigger().shoot, this, &vr_kit_roller_coaster_1::timer_event);
	}
	/// 
	void on_set(void* member_ptr)
	{
		if (member_ptr == &para_x
			|| member_ptr == &para_y
			|| member_ptr == &para_z
			|| member_ptr == &para_x_0
			|| member_ptr == &para_z_0
			|| member_ptr == &resolution
			) {
			generate_para_curve();
		}
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
		return "vr_kit_roller_coaster_1";
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
		return true;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		indicator_trans.front() = get_posi_para_curve(t, true);
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		render_para_curves_with_cones(ctx);
	}
	/*
		render para curves
	*/
	vec3 get_posi_para_curve(float t, bool from_timer_event) {
		if (from_timer_event)
			t = t / speed_factor;
		float x = 0, y = 0, z = 0;
		x = cos(t) * (para_x_0 + sin(para_x * t));
		z = sin(t) * (para_z_0 + sin(para_z * t));
		y = cos(para_y * t);
		return vec3(x, y, z);
	}
	bool generate_para_curve() {
		para_curve_posi_list.clear();
		color_para_curve.clear();
		std::default_random_engine g;
		std::uniform_real_distribution<float> d(0, 1);
		float t = 0;
		float x = 0, y = 0, z = 0;
		float step = 2 * M_PI / resolution;
		while (t < 2 * M_PI) {
			para_curve_posi_list.push_back(get_posi_para_curve(t, false));
			color_para_curve.push_back(rgb(d(g), d(g), d(g)));
			t += step;
		}
		rounded_cone_style.radius = 0.01f;
		generate_one_sphere_to_indicate();
		return true;
	}
	void generate_one_sphere_to_indicate() {
		indicator_trans.clear();
		indicator_color.clear();
		indicator_trans.push_back(get_posi_para_curve(0, false));
		indicator_color.push_back(rgb(0, 1, 0));
		sphere_style.radius = 0.5;
	}
	void render_para_curves_with_cones(context& ctx) {
		if (para_curve_posi_list.size()) {
			auto& rc_renderer = cgv::render::ref_rounded_cone_renderer(ctx);
			rc_renderer.set_render_style(rounded_cone_style);
			rc_renderer.set_position_array(ctx, &para_curve_posi_list.front(), para_curve_posi_list.size(), sizeof(vec3));
			rc_renderer.set_color_array(ctx, &color_para_curve.front(), color_para_curve.size(), sizeof(rgb));
			rc_renderer.render(ctx, 0, para_curve_posi_list.size());

			rc_renderer.set_position_array(ctx, &para_curve_posi_list.at(1), para_curve_posi_list.size(), sizeof(vec3));
			rc_renderer.set_color_array(ctx, &color_para_curve.at(1), color_para_curve.size(), sizeof(rgb));
			rc_renderer.render(ctx, 0, para_curve_posi_list.size() - 1);

			std::vector<vec3> tmp_posi;
			std::vector<rgb> tmp_color;
			tmp_posi.push_back(para_curve_posi_list.back());
			tmp_posi.push_back(para_curve_posi_list.front());
			tmp_color.push_back(color_para_curve.back());
			tmp_color.push_back(color_para_curve.front());
			rc_renderer.set_position_array(ctx, tmp_posi);
			rc_renderer.set_color_array(ctx, tmp_color);
			rc_renderer.render(ctx, 0, 2);
		}
		if (indicator_trans.size()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(sphere_style);
			sr.set_position_array(ctx, indicator_trans);
			sr.set_color_array(ctx, indicator_color);
			sr.render(ctx, 0, indicator_trans.size());
		}
	}

	/// overload the create gui method
	void create_gui()
	{
	}
};