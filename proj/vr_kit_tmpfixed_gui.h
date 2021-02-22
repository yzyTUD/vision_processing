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

class quadbtn {
public:
	label_texture* labeltex;
	bool use_label;
	bool use_icon;
	int group;
	float off_angle;
	int level;
	quadbtn(int which_group) {
		group = which_group;
		off_angle = 0;
		level = 0;
	}
	// accociated mesh 
	void set_label(std::string str, float font_size) {
		labeltex = new label_texture(str, font_size);
		use_label = true;
	}
	void set_angle_level(float agl, int l) {
		off_angle = agl;
		level = l;
	}
};

class vr_kit_tmpfixed_gui :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	std::vector<quadbtn> hmbmenubtns;
	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;
	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;

	vec2 start_touching_point = vec2(0);
	vec2 end_touching_point = vec2(0);
public:
	/// initialize rotation angle
	vr_kit_tmpfixed_gui()
	{
		//connect(get_animation_trigger().shoot, this, &vr_kit_tmpfixed_gui::timer_event);		
		// enum system fonts can assign font vars locally 
		cgv::media::font::enumerate_font_names(font_names);
		label_face_type = cgv::media::font::FFA_BOLD;
		for (unsigned i = 0; i < font_names.size(); ++i) {
			std::string fn(font_names[i]);
			if (cgv::utils::to_lower(fn) == "calibri") {
				label_font_face = cgv::media::font::find_font(fn)
					->get_font_face(label_face_type);
				for (auto& btn : hmbmenubtns) {
					if (btn.use_label)
						btn.labeltex->label_font_idx = i; // why i? pp
				}
			}
		}
	}
	// call me 
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
		for (int i = 0; i < data_ptr->gps.size(); i++) {
			int cur_group = i;
			int cur_angle = 0;
			int btnidx = 0;
			for (btnidx = 0; btnidx < data_ptr->gps.at(i).size() && btnidx < 12; btnidx++) {
				hmbmenubtns.push_back(*(new quadbtn(cur_group)));
				hmbmenubtns.back().set_label(data_ptr->gps.at(i).at(btnidx), 50);
				hmbmenubtns.back().set_angle_level(cur_angle, 0);
				cur_angle += 30;
			}
			while(btnidx<12){
				hmbmenubtns.push_back(*(new quadbtn(cur_group)));
				hmbmenubtns.back().set_label("#gp "+std::to_string(i), 50);
				hmbmenubtns.back().set_angle_level(cur_angle, 0);
				cur_angle += 30;
				btnidx++;
			}
		}
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
		return "vr_kit_tmpfixed_gui";
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
	// call me 
	bool handle(event& e)
	{
		switch (e.get_kind()) {
		case cgv::gui::EID_KEY:
		{
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			int ci = vrke.get_controller_index();

		}
		case cgv::gui::EID_STICK:
		{
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			int ci = vrse.get_controller_index();
			switch (vrse.get_action()) {
				//case cgv::gui::SA_TOUCH:
				//	if (ci == data_ptr->left_rgbd_controller_index) {
				//		float y_range = 1; float x_range = 1;
				//		if (vrse.get_y() > -y_range && vrse.get_y() < y_range && vrse.get_x() > 0)
				//			data_ptr->active_off_rotation -= 30; // switch to the right btn 
				//		else if(vrse.get_y() > -y_range && vrse.get_y() < y_range && vrse.get_x() < 0)
				//			data_ptr->active_off_rotation += 30; // move to the left btn
				//		return true;
				//	}

				//case cgv::gui::SA_TOUCH:
				//	if (ci == data_ptr->left_rgbd_controller_index) {
				//		start_touching_point = vec2(vrse.get_x(), vrse.get_y());
				//		return true;
				//	}
				////case cgv::gui::SA_RELEASE:
				////	if (ci == data_ptr->left_rgbd_controller_index) {
				////		end_touching_point = vec2(vrse.get_x(), vrse.get_y());
				////		std::cout << "ci = " << ci << " from: " << start_touching_point<<
				////			" to: " << end_touching_point << std::endl;
				////		vec2 diff = end_touching_point - start_touching_point;
				////		if (diff.x() < data_ptr->paratone_1)
				////			data_ptr->active_off_rotation -= 30; // switch to the right btn 
				////		else if (diff.x() > -data_ptr->paratone_1)
				////			data_ptr->active_off_rotation += 30; // move to the left btn
				////		return true;
				////	}

				//case cgv::gui::SA_MOVE:
				//	if (ci == data_ptr->left_rgbd_controller_index) {
				//		end_touching_point = vec2(vrse.get_x(), vrse.get_y());
				//		std::cout << "ci = " << ci << " from: " << start_touching_point<<
				//			" to: " << end_touching_point << std::endl;
				//		vec2 diff = end_touching_point - start_touching_point;
				//		if (diff.x() > 0.3) // todo: move with adjestable para. 
				//			data_ptr->active_off_rotation += data_ptr->paratone_4 * abs(diff.x()); // switch to the left btn 
				//		else if (diff.x() < -0.3) 
				//			data_ptr->active_off_rotation -= data_ptr->paratone_4 * abs(diff.x()); // move to the right btn

				//		return true;
				//	}

				case cgv::gui::SA_TOUCH:
					if (ci == data_ptr->left_rgbd_controller_index) {
						if(vrse.get_x()>0.3)
							data_ptr->active_off_rotation -= 30;
						else if(vrse.get_x() < -0.3)
							data_ptr->active_off_rotation += 30;
						return true;
					}
				}
		}

		}
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	///
	void init_frame(cgv::render::context& ctx) {
		for (auto& btn : hmbmenubtns) {
			if (btn.use_label) {
				if (btn.labeltex->label_fbo.get_width() != btn.labeltex->label_resolution) {
					btn.labeltex->label_tex.destruct(ctx);
					btn.labeltex->label_fbo.destruct(ctx);
				}
				if (!btn.labeltex->label_fbo.is_created()) {
					btn.labeltex->label_tex.create(ctx, cgv::render::TT_2D, btn.labeltex->label_resolution, btn.labeltex->label_resolution);
					btn.labeltex->label_fbo.create(ctx, btn.labeltex->label_resolution, btn.labeltex->label_resolution);
					btn.labeltex->label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
					btn.labeltex->label_tex.set_mag_filter(cgv::render::TF_LINEAR);
					btn.labeltex->label_fbo.attach(ctx, btn.labeltex->label_tex);
					btn.labeltex->label_outofdate = true;
				}
				if (btn.labeltex->label_outofdate && btn.labeltex->label_fbo.is_complete(ctx)) {
					glPushAttrib(GL_COLOR_BUFFER_BIT);
					btn.labeltex->label_fbo.enable(ctx);
					btn.labeltex->label_fbo.push_viewport(ctx);
					ctx.push_pixel_coords();
					glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
					glClear(GL_COLOR_BUFFER_BIT);

					glColor4f(btn.labeltex->label_color[0], btn.labeltex->label_color[1], btn.labeltex->label_color[2], 1);
					ctx.set_cursor(20, (int)ceil(btn.labeltex->label_size) + 20);
					ctx.enable_font_face(label_font_face, btn.labeltex->label_size);
					ctx.output_stream() << btn.labeltex->label_text << "\n";
					ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face
					ctx.output_stream().flush();

					ctx.pop_pixel_coords();
					btn.labeltex->label_fbo.pop_viewport(ctx);
					btn.labeltex->label_fbo.disable(ctx);
					glPopAttrib();
					btn.labeltex->label_outofdate = false;

					btn.labeltex->label_tex.generate_mipmaps(ctx);
				}
			}
		}
	}
	/// setting the view transform yourself
	// call me 
	void draw(context& ctx)
	{
		// render test with: 
		//for (auto btn : hmbmenubtns) {
		//	if (btn.use_label) {
		//		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		//		prog.enable(ctx);
		//		btn.labeltex->label_tex.enable(ctx);
		//		ctx.tesselate_flexible_disk(36, 2 * M_PI * 0.33, 2 * M_PI * 0.67, 0.2, false, false);
		//		btn.labeltex->label_tex.disable(ctx);
		//		prog.disable(ctx);
		//	}
		//}

		render_menu_bar_quads(ctx, 0.1, 0.06, 28);
		render_an_arrow_on_left_hand(ctx, rgb(0,0.4,0),0.1);
		
		// TEST
		//data_ptr->active_off_rotation = data_ptr->paratone_1;
	}
	/// overload the create gui method
	void create_gui()
	{
	}
	///
	void tesselete_PNT_fan(int lev, float r, float s, float theta_as_angle, quat rot, std::vector<vec3>* P, std::vector<vec3>* N, std::vector<vec2>* T) {

		float theta = 2 * M_PI * (theta_as_angle / 360.0f);
		// 2d shape flexible 
		/*P->push_back(vec3(0.2, 0, -0.4));
		P->push_back(vec3(-0.2, 0, -0.4));
		P->push_back(vec3(0.2, 0, -0.2));
		P->push_back(vec3(-0.2, 0, -0.2));*/

		float sums = 0;
		for (int i = 0; i < lev; i++) {
			sums += pow(1.2, i) * s;
		}
		r = 0.1 + lev * 0.01 + sums;
		if (lev > 0)
			s = lev * 1.2 * s;

		P->push_back(vec3(r * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));
		P->push_back(vec3(-r * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));
		P->push_back(vec3(r * sin(theta / 2.0), 0, -r * cos(theta / 2.0)));
		//
		P->push_back(vec3(-r * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));
		P->push_back(vec3(r * sin(theta / 2.0), 0, -r * cos(theta / 2.0)));
		P->push_back(vec3(-r * sin(theta / 2.0), 0, -r * cos(theta / 2.0)));
		//
		P->push_back(vec3(r * sin(theta / 2.0), 0, -r * cos(theta / 2.0)));
		P->push_back(vec3((s + r) * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));
		P->push_back(vec3(r * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));
		//
		P->push_back(vec3(-r * sin(theta / 2.0), 0, -r * cos(theta / 2.0)));
		P->push_back(vec3(-r * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));
		P->push_back(vec3(-(s + r) * sin(theta / 2.0), 0, -(r + s) * cos(theta / 2.0)));

		//T->push_back(vec2(1.0f, 1.0f));
		//T->push_back(vec2(0.0f, 1.0f));
		//T->push_back(vec2(1.0f, 0.0f));
		////
		//T->push_back(vec2(0.0f, 1.0f));
		//T->push_back(vec2(1.0f, 0.0f));
		//T->push_back(vec2(0.0f, 0.0f));

		T->push_back(vec2(0.0f, 0.0f));
		T->push_back(vec2(1.0f, 0.0f));
		T->push_back(vec2(0.0f, 1.0f));
		T->push_back(vec2(1.0f, 0.0f));
		T->push_back(vec2(0.0f, 1.0f));
		T->push_back(vec2(1.0f, 1.0f));

		// random? 
		T->push_back(vec2(-1, -1));
		T->push_back(vec2(-1, -1));
		T->push_back(vec2(-1, -1));
		T->push_back(vec2(-1, -1));
		T->push_back(vec2(-1, -1));
		T->push_back(vec2(-1, -1));

		vec3 normal = cross(P->at(0) - P->at(1), P->at(2) - P->at(1)); normal.normalize();
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		//
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);
		N->push_back(normal);

		for (int i = 0; i < P->size(); i++) {
			rot.rotate(P->at(i));
		}
	}
	/// default val: /*float r = 0.2; float s = 0.2;*/
	void render_menu_bar_quads(context& ctx, float r, float s, float theta_as_angle) {
		for (auto btn : hmbmenubtns) {
			if (btn.use_label && btn.group == data_ptr->active_group) {

				std::vector<vec3> P;
				std::vector<vec3> N;
				std::vector<vec2> T;

				quat rot;
				rot = quat(vec3(0, 1, 0), 2 * M_PI * (btn.off_angle / 360.0f));
				quat off_rot = quat(vec3(0, 1, 0), M_PI);
				quat global_rot = quat(vec3(0, 1, 0), 2 * M_PI * (data_ptr->active_off_rotation / 360.0f));
				tesselete_PNT_fan(btn.level, r, s, theta_as_angle, rot * off_rot * global_rot, &P, &N, &T);

				// fixed part 
				// handhold transformation
				vec3 offset = vec3(0,0.008,-0.2);
				if (data_ptr->check_btn_active_givenrot(btn.off_angle))
					offset.y() = 0;
				data_ptr->cur_left_hand_rot_quat.rotate(offset);
				for (auto& p : P) {
					data_ptr->cur_left_hand_rot_quat.rotate(p);
					p += data_ptr->cur_left_hand_posi + offset;
				}

				// enable texture 
				cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
				int pi = prog.get_position_index();
				int ni = prog.get_normal_index();
				int ti = prog.get_texcoord_index();

				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ni, N);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ni);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
				glDisable(GL_CULL_FACE);
				prog.enable(ctx);
				btn.labeltex->label_tex.enable(ctx);
				ctx.set_color(rgb(1, 1, 1));
				glDrawArrays(GL_TRIANGLES, 0, (GLsizei)P.size());
				btn.labeltex->label_tex.disable(ctx);
				prog.disable(ctx);
				glEnable(GL_CULL_FACE);
				/*cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ni);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ti);*/
			}
		}
	}
	///
	void render_an_arrow_on_left_hand(cgv::render::context& ctx, rgb c, float r, float l = 2.0) {
		if (data_ptr == nullptr)
			return;
		vec3 startingdir = vec3(0, 0, -0.06);
		data_ptr->cur_left_hand_rot_quat.rotate(startingdir);
		vec3 endposi = data_ptr->cur_left_hand_posi + startingdir;

		auto& prog = ctx.ref_surface_shader_program();
		prog.set_uniform(ctx, "map_color_to_material", 3);
		prog.enable(ctx);
		ctx.set_color(c);
		ctx.tesselate_arrow(data_ptr->cur_left_hand_posi, endposi, r, l, 0.5f);
		prog.disable(ctx);
	}
};
