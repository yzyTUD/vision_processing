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
#include <libs/cgv_gl/rectangle_renderer.h>

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
#include <cgv/math/geom.h>




class vr_kit_imagebox :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymores
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;

public:
	/// initialize rotation angle
	vr_kit_imagebox()
	{
		connect(get_animation_trigger().shoot, this, &vr_kit_imagebox::timer_event);
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
		return "vr_kit_imagebox";
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
	void sample_unit_hemi_sphere(vec3& p,quat& q,vec3 c,float r) {
		
	}
	void init_given_a_list_of_file_names(context& ctx,std::vector<std::string>* fnl_ptr) {
		std::default_random_engine g;
		std::uniform_real_distribution<float> d(0, 1);
		vec3 boxext = vec3(0.1, 0.1, 0.02);
		float r = 1;
		vec3 c = vec3(0,1,0);
		for (auto& fn : *fnl_ptr) {
			vec3 t_p;
			quat t_q;

			float min_phi = 0;
			float max_phi = M_PI * d(g);
			float min_theta = -M_PI * d(g);
			float max_theta = -M_PI * d(g);

			float c_phi = (max_phi - min_phi) * d(g) - min_phi;
			float c_theta = (max_theta - min_theta) * d(g) - min_theta;

			// Switch to cartesian coordinates
			float x = r * sin(c_theta) * cos(c_phi);
			float y = r * sin(c_theta) * sin(c_phi);
			float z = r * cos(c_theta);

			// upper hemi sphere 
			if (y < 0)
				y = -y;

			t_p = vec3(x, y, z) + c;
			vec3 target_n = c - t_p;
			target_n.normalize();
			vec3 cur_n = vec3(0, 0, -1);
			vec3 view_up_dir = vec3(0, 1, 0);

			dmat3 R = cgv::math::build_orthogonal_frame(cur_n, view_up_dir);
			R.transpose();
			R = cgv::math::build_orthogonal_frame(target_n, view_up_dir) * R;
			dvec3 daxis;
			double dangle;
			int res = cgv::math::decompose_rotation_to_axis_and_angle(R, daxis, dangle);
			t_q = quat(daxis, dangle);

			imagebox* ib = new imagebox(ctx, box3(-boxext, boxext), fn);
			ib->set_posi_ori(t_p, t_q);
			ib->set_color(rgb(d(g), d(g), d(g)));
			data_ptr->iba->imagebox_list.push_back(*ib);
		}
	}
	/// call this 
	bool init(context& ctx) {
		if (data_ptr==nullptr)
			return false;
		data_ptr->iba = new imagebox_array();

		std::vector<std::string> fnl;
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_1.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_2.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_3.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_4.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_5.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_6.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_7.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_8.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_9.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_10.jpg");
		fnl.push_back(data_ptr->data_dir + "/imir_images/image_11.jpg");
		init_given_a_list_of_file_names(ctx, &fnl);

		data_ptr->iba->prepare_rendering_once();
		data_ptr->imageboxes_init_to_trackable_list();
		return true;
	}
	/// call this 
	void init_frame(context& ctx) {
		if (data_ptr == nullptr)
			return;
		for (auto& imgboxlist : data_ptr->iba->imagebox_list) {
			imgboxlist.material.ensure_textures(ctx);
		}
	}
	/// setting the view transform yourself
	/// call this 
	void draw(context& ctx)
	{
		if (data_ptr == nullptr)
			return;
		if(data_ptr->iba) data_ptr->iba->render(ctx);
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
