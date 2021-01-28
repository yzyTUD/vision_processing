#pragma once

#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/signal/signal.h>
#include <libs/cgv_gl/gl/gl_tools.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/math/ftransform.h>
#include <random>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>

#include "vr_kit_intersection.h"

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers

class boxgui_label_texture : public cgv::render::render_types {
public:
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;
	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	boxgui_label_texture(std::string label, float font_size) {
		label_text = label;
		label_font_idx = 0;
		label_upright = false;
		label_size = font_size; // 150
		label_color = rgb(1, 1, 1);
		label_outofdate = true; // whether label texture is out of date
		label_resolution = 256; // resolution of label texture
		// 
		// this two will be initialized during init_frame() func.: label_tex; label_fbo;
	}
};

class boxgui_button :
	public cgv::render::render_types,
	public cgv::signal::tacker // still not work after adding this 
{
public:
	vec3 ext;
	vec3 local_trans;
	quat local_rot;
	vec3 trans;
	quat rot;
	rgb color;
	bool use_label;
	bool use_icon;
	//std::string labelstr;
	boxgui_label_texture* labeltex;
	unsigned int icontex;
	std::string iconpath;
	bool has_intersec;
	bool is_static;
	float radio = 19.0f / 4.0f;
	int group;
	void* f;
	// may not work due to the loss of tracker information 
	cgv::signal::signal<int> button_clicked;
	boxgui_button(vec3 e, vec3 t, quat r, rgb col, int which_group) {
		ext = e;
		trans = local_trans = t;
		rot = local_rot = r;
		color = col;
		use_label = false;
		use_icon = false;
		//labelstr = "";
		labeltex = nullptr;
		icontex = -1;
		iconpath = "";
		has_intersec = false;
		is_static = false;
		group = which_group;
	}
	void set_label(std::string str, float font_size) {
		//labelstr = str;
		labeltex = new boxgui_label_texture(str, font_size);
		use_label = true;
	}
	void front_append_info_to_label(std::string str) {
		//labelstr = str + "\n" + labelstr;
		if (labeltex)
			labeltex->label_text = str + "\n" + labeltex->label_text;
	}
	void set_icon(std::string path) {
		iconpath = path;
		double aspect_ptr;
		bool has_alpha_ptr;
		double aspect;
		icontex = cgv::render::gl::read_image_to_texture(iconpath, false, &aspect, &has_alpha_ptr);
		use_icon = true;
	}
	void set_static() {
		is_static = true;
	}
	std::string get_label_text() {
		return labeltex->label_text;
	}
};

class boxgui_interactable :
	public base,    // base class of all to be registered classes
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	/*
		boxgui managment 
	*/
	std::vector<int> intersection_boxgui_indices;
	std::vector<int> intersection_boxgui_controller_indices;
	std::vector<vec3> btnposilist;

	/*
		for handheld gui
	*/
	std::vector<box3> objpicklist;
	std::vector<rgb> objpicklist_colors;
	std::vector<vec3> objpicklist_translations;
	std::vector<quat> objpicklist_rotations;
	float obj_scale_factor = 3;
	cgv::render::shader_program cube_prog;
	int which_boxgui_group_is_going_to_be_rendered = 0;

	/*
		hand tracking
	*/
	int left_rgbd_controller_index = 0;
	int right_rgbd_controller_index = 1;
	vec3 cur_left_hand_posi;
	vec3 cur_right_hand_posi;
	vec3 cur_left_hand_dir;
	mat3 cur_left_hand_rot;
	mat3 cur_left_hand_rot_mat;

	/*
		font and boxes rendering 
	*/
	// general font information
	std::vector<const char*> font_names;
	std::string font_enum_decl;
	// current font face used
	cgv::media::font::font_face_ptr label_font_face;
	cgv::media::font::FontFaceAttributes label_face_type;
	cgv::render::box_render_style movable_style;
	bool construct_boxgui = false;

	std::vector<boxgui_button> boxguibtns;
	std::vector<cgv::signal::signal<int>> boxguibtn_signals;
	bool btn_keydown_boxgui = false;
	boxgui_button* info_btn;

public:
	cgv::signal::signal<int> s;
	/// initialize rotation angle
	boxgui_interactable() {
		// construct boxgui v3  
		//construct_handheld_gui();

		// enum system fonts can assign font vars locally 
		cgv::media::font::enumerate_font_names(font_names);
		label_face_type = cgv::media::font::FFA_BOLD;
		for (unsigned i = 0; i < font_names.size(); ++i) {
			std::string fn(font_names[i]);
			if (cgv::utils::to_lower(fn) == "calibri") {
				label_font_face = cgv::media::font::find_font(fn)
					->get_font_face(label_face_type);
				for (auto& btn : boxguibtns) {
					if (btn.use_label)
						btn.labeltex->label_font_idx = i; // why i? pp
				}
			}
		}
	}// compute intersection points of controller ray with movable boxes
	///
	bool init(cgv::render::context& ctx) {

		return true;
	}
	///
	void compute_intersections(const vec3& origin, const vec3& direction, bool moveboxes, bool boxgui)
	{
		if (boxgui)
			// compute intersec. with boxgui boxes 
			for (size_t i = 0; i < boxguibtns.size(); ++i) {
				if (boxguibtns.at(i).group == which_boxgui_group_is_going_to_be_rendered) {
					vec3 origin_box_i = origin - boxguibtns.at(i).trans;
					boxguibtns.at(i).rot.inverse_rotate(origin_box_i);
					vec3 direction_box_i = direction;
					boxguibtns.at(i).rot.inverse_rotate(direction_box_i);
					float t_result;
					vec3  p_result;
					vec3  n_result;
					if (cgv::media::ray_axis_aligned_box_intersection(
						origin_box_i, direction_box_i,
						box3(-0.5f * boxguibtns.at(i).ext, 0.5f * boxguibtns.at(i).ext),
						t_result, p_result, n_result, 0.000001f)) {

						// transform result back to world coordinates
						boxguibtns.at(i).rot.rotate(p_result);
						p_result += boxguibtns.at(i).trans;
						boxguibtns.at(i).rot.rotate(n_result);

						intersection_boxgui_indices.push_back((int)i);
						//intersection_boxgui_controller_indices.push_back(ci);
					}
				}
			}
	}
	///
	/*
		position doesnt matter, will be allocated gloablly, 
		
		btn_name
		btn_group
		signal related vars: X* x, void (X::* mp)(T1)

		todo: add group info, name
	*/
	template <class X, typename T1>
	void add_btn(std::string btn_name,int btn_group, X* x, void (X::* mp)(T1)) {
		boxgui_button test_btn =
			boxgui_button(
				vec3(0.2, 0.2, 0.1),
				vec3(0.3, 0, -0.2), // offset 
				quat(vec3(0, 1, 0), 0),
				rgb(0.2, 0.4, 0.2),
				btn_group
			);
		test_btn.set_label(btn_name, 50);
		cgv::signal::signal<int> btn_s;
		boxguibtns.push_back(test_btn);
		boxguibtn_signals.push_back(btn_s);
		connect(boxguibtn_signals.back(), x, mp);
	}
	/// the callback is strightforward 
	void add_main_panel_btn(std::string btn_name, int btn_group) {

	}
	/// no name, no callback function 
	void add_bkg_btn(int btn_group) {
	
	}
	/// 
	void add_info_btn() {
		
	}
	/// 
	void construct_handheld_gui() {
		//boxguibtns.clear();

		//std::default_random_engine generator;
		//std::uniform_real_distribution<float> distribution(0, 1);
		//std::uniform_real_distribution<float> signed_distribution(-1, 1);

		//float tmpboxsize = 0.02f * obj_scale_factor;
		//vec3 extent(tmpboxsize);
		//vec3 demoposi = vec3(0, 0, -0.2f);

		//std::vector<std::string> label_list;
		//std::vector<vec3> local_coordi_list;

		//// info panel
		//boxgui_button info_button =
		//	boxgui_button(
		//		vec3(1.2, 1.2, 0.1),
		//		vec3(0, -1, -0.4) + vec3(0, 0.2, 0), // offset 
		//		quat(vec3(1, 0, 0), -45),
		//		rgb(0.2, 0.4, 0.2),
		//		1
		//	);
		//info_button.set_static();
		//info_button.set_label("info label", 15);
		//boxguibtns.push_back(info_button);

		//// button group 0
		//label_list.push_back("pc_tools");
		//label_list.push_back("mesh_\ntools");
		//label_list.push_back("semantic\ntools");
		//label_list.push_back("interaction\n_tools");
		////label_list.push_back("gui_tools");
		//local_coordi_list.push_back(vec3(-0.3, 0, -0.2));
		//local_coordi_list.push_back(vec3(-0.3 * sqrt(2) / 2, 0.3 * sqrt(2) / 2, -0.2));
		//local_coordi_list.push_back(vec3(0, 0.3, -0.2));
		//local_coordi_list.push_back(vec3(0.3 * sqrt(2) / 2, 0.3 * sqrt(2) / 2, -0.2));
		//local_coordi_list.push_back(vec3(0.3, 0, -0.2));
		//for (int i = 0; i < label_list.size(); i++) {
		//	rgb tmpcol = rgb(
		//		0.4f * distribution(generator) + 0.1f,
		//		0.4f * distribution(generator) + 0.3f,
		//		0.4f * distribution(generator) + 0.1f
		//	);
		//	// store local coordi frames in button structure 
		//	boxgui_button tmpbtn = boxgui_button(
		//		vec3(0.2, 0.2, 0.1),
		//		local_coordi_list.at(i),
		//		quat(vec3(0, 1, 0), 0),
		//		tmpcol,
		//		0 // they blongs to group 0
		//	);
		//	tmpbtn.set_label(label_list.at(i), 50);// 8 char per line 
		//	boxguibtns.push_back(tmpbtn);
		//}
		//// clean up 
		//label_list.clear();
		//local_coordi_list.clear();

		//{
		//	// button group 1 , first pop-up buttons 
		//	float btn_offset_y = 0.2;
		//	label_list.push_back("back");//
		//	label_list.push_back("save_marked_points");
		//	label_list.push_back("read_properties");
		//	label_list.push_back("load_next_pc");
		//	label_list.push_back("load_all_pcs");
		//	label_list.push_back("del_last_pc");
		//	label_list.push_back("align_pc_\nwith_last\n_frame");
		//	/*for (int i = 0; i < 12; i++) {
		//		label_list.push_back("test");
		//	}*/
		//	vec3 start_point = vec3(-0.45, 0.45, -1);
		//	local_coordi_list.push_back(vec3(start_point.x(), start_point.y() + 0.3, start_point.z()));
		//	int num_per_line = 4;
		//	float len = num_per_line * 0.3;
		//	for (int i = 0; i < num_per_line * num_per_line; i++) {
		//		float tmp_x = start_point.x() + i * 0.3 - (i / num_per_line) * len;
		//		float tmp_y = start_point.y() - i / num_per_line * 0.3;
		//		local_coordi_list.push_back(vec3(tmp_x, tmp_y, start_point.z()));
		//	}
		//	for (int i = 0; i < label_list.size(); i++) {
		//		rgb tmpcol = rgb(
		//			0.4f * distribution(generator) + 0.1f,
		//			0.4f * distribution(generator) + 0.3f,
		//			0.4f * distribution(generator) + 0.1f
		//		);
		//		// store local coordi frames in button structure 
		//		boxgui_button tmpbtn = boxgui_button(
		//			vec3(0.2, 0.2, 0.1),
		//			local_coordi_list.at(i) + vec3(0, btn_offset_y, 0),
		//			quat(vec3(0, 1, 0), 0),
		//			tmpcol,
		//			1 // belongs to group 1
		//		);
		//		tmpbtn.set_label(label_list.at(i), 50);// 8 char per line 
		//		boxguibtns.push_back(tmpbtn);
		//	}
		//	boxgui_button tmpbtn =
		//		boxgui_button(
		//			vec3(1.2, 1.2, 0.1),
		//			vec3(0, 0, -1.2) + vec3(0, btn_offset_y, 0),
		//			quat(vec3(0, 1, 0), 0),
		//			rgb(0.2, 0.4, 0.2),
		//			1
		//		);
		//	tmpbtn.set_static();
		//	boxguibtns.push_back(tmpbtn);
		//	// clean up 
		//	label_list.clear();
		//	local_coordi_list.clear();
		//}

		//{
		//	// button group 2 , first pop-up buttons 
		//	label_list.push_back("back");
		//	label_list.push_back("l_up_group2");
		//	label_list.push_back("r_up");
		//	label_list.push_back("r_low");
		//	label_list.push_back("l_low");
		//	for (int i = 0; i < 12; i++) {
		//		label_list.push_back("test");
		//	}
		//	vec3 start_point = vec3(-0.45, 0.45, -1);
		//	local_coordi_list.push_back(vec3(start_point.x(), start_point.y() + 0.3, start_point.z()));
		//	int num_per_line = 4;
		//	float len = num_per_line * 0.3;
		//	for (int i = 0; i < num_per_line * num_per_line; i++) {
		//		float tmp_x = start_point.x() + i * 0.3 - (i / num_per_line) * len;
		//		float tmp_y = start_point.y() - i / num_per_line * 0.3;
		//		local_coordi_list.push_back(vec3(tmp_x, tmp_y, start_point.z()));
		//	}
		//	for (int i = 0; i < label_list.size(); i++) {
		//		rgb tmpcol = rgb(
		//			0.4f * distribution(generator) + 0.1f,
		//			0.4f * distribution(generator) + 0.3f,
		//			0.4f * distribution(generator) + 0.1f
		//		);
		//		// store local coordi frames in button structure 
		//		boxgui_button tmpbtn = boxgui_button(
		//			vec3(0.2, 0.2, 0.1),
		//			local_coordi_list.at(i),
		//			quat(vec3(0, 1, 0), 0),
		//			tmpcol,
		//			2 // belongs to group 2
		//		);
		//		tmpbtn.set_label(label_list.at(i), 50);// 8 char per line 
		//		boxguibtns.push_back(tmpbtn);
		//	}
		//	boxgui_button tmpbtn =
		//		boxgui_button(
		//			vec3(1.2, 1.2, 0.1),
		//			vec3(0, 0, -1.2),
		//			quat(vec3(0, 1, 0), 0),
		//			rgb(0.2, 0.4, 0.2),
		//			2
		//		);
		//	tmpbtn.set_static();
		//	boxguibtns.push_back(tmpbtn);
		//	// clean up 
		//	label_list.clear();
		//	local_coordi_list.clear();
		//}

		// colorful box  
		//objpicklist.push_back(box3(-0.5f * extent, 0.5f * extent));
		//objpicklist_colors.push_back(rgb(distribution(generator),
		//	distribution(generator),
		//	distribution(generator)));
		//quat rot(cur_right_hand_rot);
		//rot.normalize();
		//mat3 rot_mat;
		//rot.put_matrix(rot_mat);
		//vec3 addi_posi = rot_mat * demoposi;
		//vec3 modi_posi = cur_right_hand_posi + addi_posi; // addi direction vector should be rotated 
		//objpicklist_translations.push_back(modi_posi);
		//objpicklist_rotations.push_back(rot);

		construct_boxgui = true;
	}
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "boxgui_interactable";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}
	/// self reflection allows to change values in the config file
	bool self_reflect(reflection_handler& rh)
	{
		return true;
	}
	//
	void btn_callback_test(){}
	/// overload to handle events, return true if event was processed
	bool handle(event& e)
	{
		// check if vr event flag is not set and don't process events in this case
		if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;
		// check event id
		switch (e.get_kind()) {
			case cgv::gui::EID_KEY:
			{
				return true;
			}
			case cgv::gui::EID_THROTTLE:
			{
				return true;
			}
			case cgv::gui::EID_STICK:
			{
				cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
				switch (vrse.get_action()) {
					case cgv::gui::SA_TOUCH:
						if (vrse.get_controller_index() == 1)
						btn_keydown_boxgui = true;
						break;
				}
				return true;
			}
			case cgv::gui::EID_POSE:
				cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
				// check for controller pose events
				int ci = vrpe.get_trackable_index();
				// left hand event 
				if (ci == left_rgbd_controller_index) {
					// update positions 
					cur_left_hand_posi = vrpe.get_position();
					cur_left_hand_rot = vrpe.get_orientation();
					cur_left_hand_rot_mat = vrpe.get_rotation_matrix();
				}
				if (ci == right_rgbd_controller_index) {
					cur_right_hand_posi = vrpe.get_position();
				}
				if (ci != -1) {
					if (intersection_boxgui_indices.size() > 0 && btn_keydown_boxgui) {
						boxguibtn_signals.at(intersection_boxgui_indices.front())(1);
						btn_keydown_boxgui = false;
					}

					// updated with right hand controller, we do not use button local rot now
					vec3 local_z_offset = vec3(0, 0, -1);
					quat rot(cur_left_hand_rot);
					rot.normalize();
					mat3 rot_mat;
					rot.put_matrix(rot_mat);
					vec3 global_offset = rot_mat * local_z_offset;
					cur_left_hand_dir = global_offset;

					for (auto& btn : boxguibtns) {
						vec3 local_offset = btn.local_trans;
						quat rot(cur_left_hand_rot);
						rot.normalize();
						mat3 rot_mat;
						rot.put_matrix(rot_mat);
						vec3 global_offset = rot_mat * local_offset;
						// addi direction vector should be rotated 
						vec3 modi_posi = cur_left_hand_posi + global_offset;

						btn.trans = modi_posi;
						btn.rot = rot * btn.local_rot;
					}

					intersection_boxgui_indices.clear();
					for (auto& b : boxguibtns) { 
						b.has_intersec = false;
					}

					vec3 origin, direction;
					// get left hand position, cam position 
					vrpe.get_state().controller[right_rgbd_controller_index].put_ray(&origin(0), &direction(0));
					compute_intersections(origin, direction, false, true);
					/*vrpe.get_state().controller[0].put_ray(&origin(0), &direction(0));
					compute_intersections(origin, direction, true, false);*/
					if (intersection_boxgui_indices.size() > 0) {
						boxguibtns.at(intersection_boxgui_indices.front()).has_intersec = true;
					}
				}
				
				//if (ci != -1)
					//s(1);
				//if (ci != -1)
					//boxguibtn_signals.at(0)(1);
					/*for (int idx = 0; idx < boxguibtns.size(); idx++) {
						if (boxguibtns.at(idx).get_label_text()._Equal("btn test")) {
							boxguibtn_signals.at(idx)(1);
						}
					}*/
				return true;
		}
		return false;
	}
	///
	void init_frame(cgv::render::context& ctx) {
		for (auto& btn : boxguibtns) {
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
	void draw(context& ctx)
	{
		if (boxguibtns.size()) {
			cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
			std::vector<box3> movable_btn;
			std::vector<rgb> movable_btn_colors;
			std::vector<vec3> movable_btn_translations;
			std::vector<quat> movable_btn_rotations;
			for (auto btn : boxguibtns) {
				if (btn.group == which_boxgui_group_is_going_to_be_rendered) {
					movable_btn.push_back(box3(-0.5f * btn.ext, 0.5f * btn.ext));
					movable_btn_colors.push_back(btn.color);
					cur_left_hand_dir.normalize();
					if (btn.has_intersec && !btn.is_static)
						movable_btn_translations.push_back(btn.trans + cur_left_hand_dir * 0.1);
					else
						movable_btn_translations.push_back(btn.trans);
					movable_btn_rotations.push_back(btn.rot);
				}
			}
			renderer.set_render_style(movable_style);
			renderer.set_box_array(ctx, movable_btn);
			renderer.set_color_array(ctx, movable_btn_colors);
			renderer.set_translation_array(ctx, movable_btn_translations);
			renderer.set_rotation_array(ctx, movable_btn_rotations);
			if (renderer.validate_and_enable(ctx)) {
				glDrawArrays(GL_POINTS, 0, (GLsizei)movable_btn.size());
			}
			renderer.disable(ctx);
		}

		// render labels of boxgui 
		for (auto btn : boxguibtns) {
			if (btn.group == which_boxgui_group_is_going_to_be_rendered)
				if (btn.use_label) {
					// points for a label, y-z plane
					vec3 p1(0.5 * btn.ext.x(), 0.5 * btn.ext.y(), 0);
					vec3 p2(-0.5 * btn.ext.x(), 0.5 * btn.ext.y(), 0);
					vec3 p3(0.5 * btn.ext.x(), -0.5 * btn.ext.y(), 0);
					vec3 p4(-0.5 * btn.ext.x(), -0.5 * btn.ext.y(), 0);

					vec3 addi_offset = vec3(0);

					if (btn.has_intersec && !btn.is_static)
						addi_offset = vec3(0, 0, -0.1f); // todo

					p1 = p1 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;
					p2 = p2 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;
					p3 = p3 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;
					p4 = p4 + vec3(0, 0, 0.5 * btn.ext.z() + 0.01) + addi_offset;

					// rotate and translate according to the gui boxes
					btn.rot.rotate(p1);
					btn.rot.rotate(p2);
					btn.rot.rotate(p3);
					btn.rot.rotate(p4);

					p1 = p1 + btn.trans;
					p2 = p2 + btn.trans;
					p3 = p3 + btn.trans;
					p4 = p4 + btn.trans;

					cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
					int pi = prog.get_position_index();
					int ti = prog.get_texcoord_index();
					std::vector<vec3> P;
					std::vector<vec2> T;

					P.push_back(p1); T.push_back(vec2(1.0f, 1.0f));
					P.push_back(p2); T.push_back(vec2(0.0f, 1.0f));
					P.push_back(p3); T.push_back(vec2(1.0f, 0.0f));
					P.push_back(p4); T.push_back(vec2(0.0f, 0.0f));

					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
					cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
					cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
					prog.enable(ctx);
					btn.labeltex->label_tex.enable(ctx);
					ctx.set_color(rgb(1, 1, 1));
					glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
					btn.labeltex->label_tex.disable(ctx);
					prog.disable(ctx);
					cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
					cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
				}
		}
	}
};
