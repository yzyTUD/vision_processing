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

class vr_kit_skybox :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	cgv::render::shader_program skyprog;
	cgv::render::texture img_tex;

	char* cgv_data = getenv("CGV_DATA");
	std::string data_dir = std::string(cgv_data);
public:
	/// initialize rotation angle
	vr_kit_skybox()
	{
	}
	/// 
	void on_set(void* member_ptr)
	{
	}
	/// self reflection allows to change values in the config file
	bool self_reflect(reflection_handler& rh)
	{
		return true;
	}
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "vr_kit_skybox";
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
	void timer_event(double, double dt)
	{
	}
	bool init(context& ctx) { 
		skyprog.build_program(ctx, "skycube.glpr");
		img_tex.create_from_images(ctx, data_dir + "/skybox/cm_{xp,xn,yp,yn,zp,zn}.jpg");
		return true;
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		float max_scene_extent = 100;
		glDepthMask(GL_FALSE);
		glDisable(GL_CULL_FACE);
		img_tex.enable(ctx, 1);
		skyprog.enable(ctx);
		skyprog.set_uniform(ctx, "img_tex", 1);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::scale4<double>(
			max_scene_extent, max_scene_extent, max_scene_extent));
		ctx.tesselate_unit_cube();
		ctx.pop_modelview_matrix();
		skyprog.disable(ctx);
		img_tex.disable(ctx);
		glEnable(GL_CULL_FACE);
		glDepthMask(GL_TRUE);
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
