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

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

class render_test :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	/// flag used to represent the state of the extensible gui node
	bool toggle;
protected:
	/// whether animation is turned on
	bool animate;
	/// rotation angle around y-axis in degrees
	double angle;
	/// rotation speed in degrees per second
	double speed;
	/// recursion depth
	unsigned int rec_depth;
	/// resolution of smooth shapes
	int resolution;
	///
	cgv::media::illum::surface_material material;
	/// different shape types
	enum Shape { CUBE, PRI, TET, OCT, DOD, ICO, CYL, CONE, DISK, ARROW, SPHERE } shp;
public:
	/// initialize rotation angle
	render_test()
	{
		connect(get_animation_trigger().shoot, this, &render_test::timer_event);
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
		return "simple_cube";
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
		add_decorator("Simple Cube GUI", "heading", "level=1"); // level=1 is default and can be skipped
		add_member_control(this, "recursion depth", rec_depth, "value_slider", "min=1;max=8;ticks=true");
		/// use a selection gui element to directly manipulate the shape enum
		add_member_control(this, "shape", shp, "dropdown", "enums='CUBE,PRI,TET,OCT,DOD,ICO,CYL,CONE,DISK,ARROW,SPHERE'");
	}
};

#include <cgv/base/register.h>
cgv::base::object_registration<render_test> render_test_reg("render_test");
