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
#include <chrono>
#include <fstream>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;
using namespace std;

#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>
#include <vr_kit_intersection.h>
#include "vis_kit_trackable.h"

class motion_storage_per_device:public cgv::render::render_types {
public:
	std::string device_name;
	int num_of_posi_rec = 0;
	vector<vec3> device_posi;
	vector<quat> device_orie;
	vector<std::string> time_stemp;
	motion_storage_per_device(std::string dn) {
		device_name = dn;
	}
};

class vr_kit_motioncap :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:

	std::vector<motion_storage_per_device> motion_storage;
	std::vector<motion_storage_per_device> motion_storage_read;
	std::vector<trackable> trackable_list;

	int left_rgbd_controller_index = 0;
	int right_rgbd_controller_index = 1;
	vec3 cur_left_hand_posi;
	vec3 cur_right_hand_posi;
	vec3 cur_hmd_posi;
	quat cur_left_hand_orientation_quat;
	quat cur_right_hand_orientation_quat;
	quat cur_hmd_orie;
	bool write_stemp = false;

	std::string data_dir = std::string(getenv("CGV_DATA"));
	bool have_new_mesh = false;
	int cur_frame = 0;

	// per device stuff 
	cgv::render::mesh_render_info MI_l_hand;
	mesh_type M_l_hand;
	vec3 read_l_hand_posi;
	mat4 read_l_hand_ori_mat;

	cgv::render::mesh_render_info MI_r_hand;
	mesh_type M_r_hand;
	vec3 read_r_hand_posi;
	mat4 read_r_hand_ori_mat;

	cgv::render::mesh_render_info MI_hmd;
	mesh_type M_hmd;
	vec3 read_hmd_posi;
	mat4 read_hmd_ori_mat;
public:
	bool rec_pose = false;
	bool replay = false;
	bool instanced_redraw = true;
	/// initialize rotation angle
	vr_kit_motioncap()
	{
		connect(get_animation_trigger().shoot, this, &vr_kit_motioncap::timer_event);	
		motion_storage.push_back(motion_storage_per_device("left_hand"));
		motion_storage.push_back(motion_storage_per_device("right_hand"));
		motion_storage.push_back(motion_storage_per_device("hmd"));	
		M_l_hand.read(data_dir + "/vr_controller_vive_1_5.obj");
		M_r_hand.read(data_dir + "/vr_controller_vive_1_5.obj");
		M_hmd.read(data_dir + "/generic_hmd.obj");
		have_new_mesh = true;

		trackable_mesh* tm = new trackable_mesh("left_hand", data_dir + "/vr_controller_vive_1_5.obj");
		trackable_list.push_back(*tm);
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
		return "vr_kit_motioncap";
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
	/// call this 
	bool handle(event& e)
	{
		if (e.get_kind() == cgv::gui::EID_POSE) {
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			int ci = vrpe.get_trackable_index();
			if (ci == left_rgbd_controller_index) {
				cur_left_hand_posi = vrpe.get_position();
				cur_left_hand_orientation_quat = vrpe.get_quaternion();
			}
			if (ci == right_rgbd_controller_index) {
				cur_right_hand_posi = vrpe.get_position();
				cur_right_hand_orientation_quat = vrpe.get_quaternion();
			}
			cur_hmd_posi = mat34(3, 4, vrpe.get_state().hmd.pose) * vec4(0, 0, 0, 1.0f);
			cur_hmd_orie = quat(reinterpret_cast<const mat3&>(vrpe.get_state().hmd.pose[0]));
			//rec_pose = true;
		}
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		if (rec_pose) {
			for (auto& d : motion_storage) {
				auto microsecondsUTC = std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count());
				if (d.device_name._Equal("left_hand")) {
					d.device_posi.push_back(cur_left_hand_posi);
					d.device_orie.push_back(cur_left_hand_orientation_quat);
					d.time_stemp.push_back(microsecondsUTC);
				}
				if (d.device_name._Equal("right_hand")) {
					d.device_posi.push_back(cur_right_hand_posi);
					d.device_orie.push_back(cur_right_hand_orientation_quat);
					d.time_stemp.push_back(microsecondsUTC);
				}
				if (d.device_name._Equal("hmd")) {
					d.device_posi.push_back(cur_hmd_posi);
					d.device_orie.push_back(cur_hmd_orie);
					d.time_stemp.push_back(microsecondsUTC);
				}
			}
		}
		if (replay) {
			if (cur_frame >= motion_storage_read.back().device_posi.size())
				cur_frame = 0;
			for (auto& d : motion_storage_read) {
				if (d.device_name._Equal("left_hand")) {
					read_l_hand_posi = d.device_posi.at(cur_frame);
					read_l_hand_ori_mat = d.device_orie.at(cur_frame).get_homogeneous_matrix();
				}
				if (d.device_name._Equal("right_hand")) {
					read_r_hand_posi = d.device_posi.at(cur_frame);
					read_r_hand_ori_mat = d.device_orie.at(cur_frame).get_homogeneous_matrix();
				}
				if (d.device_name._Equal("hmd")) {
					read_hmd_posi = d.device_posi.at(cur_frame);
					read_hmd_ori_mat = d.device_orie.at(cur_frame).get_homogeneous_matrix();
				}
				post_redraw();
			}
			std::cout << "cur_frame: " << cur_frame << std::endl;
			cur_frame++;
		}
	}
	/// setting the view transform yourself
	/// call this
	void draw(context& ctx)
	{
		if (have_new_mesh) {
			//
			if (!M_l_hand.has_normals())
				M_l_hand.compute_vertex_normals();
			MI_l_hand.destruct(ctx);
			MI_l_hand.construct(ctx, M_l_hand);
			MI_l_hand.bind(ctx, ctx.ref_surface_shader_program(true), true);
			//
			if (!M_r_hand.has_normals())
				M_r_hand.compute_vertex_normals();
			MI_r_hand.destruct(ctx);
			MI_r_hand.construct(ctx, M_r_hand);
			MI_r_hand.bind(ctx, ctx.ref_surface_shader_program(true), true);
			//
			if (!M_hmd.has_normals())
				M_hmd.compute_vertex_normals();
			MI_hmd.destruct(ctx);
			MI_hmd.construct(ctx, M_hmd);
			MI_hmd.bind(ctx, ctx.ref_surface_shader_program(true), true);

			have_new_mesh = false;
		}
		if (replay)
		if (MI_l_hand.is_constructed()) {
			glDisable(GL_CULL_FACE);
			shader_program& prog = ctx.ref_surface_shader_program(true);
			prog.set_uniform(ctx, "map_color_to_material", (int)cgv::render::ColorMapping::CM_COLOR);
			prog.set_attribute(ctx, prog.get_color_index(), rgb(0.4));
			//
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(translate4(read_l_hand_posi)*read_l_hand_ori_mat);
			MI_l_hand.draw_all(ctx, false, true);
			ctx.pop_modelview_matrix();
			//
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(translate4(read_r_hand_posi) * read_r_hand_ori_mat);
			MI_r_hand.draw_all(ctx, false, true);
			ctx.pop_modelview_matrix();
			//
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(translate4(read_hmd_posi) * read_hmd_ori_mat);
			MI_hmd.draw_all(ctx, false, true);
			ctx.pop_modelview_matrix();

			glEnable(GL_CULL_FACE);
		}
	}
	///
	bool save_to_tj_file() {
		rec_pose = false;
		auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		auto& fn = data_dir + "/mocap_trajectory/motioncap_" + std::to_string(microsecondsUTC) + ".tj";
		std::ofstream os(fn);
		if (os.fail())
			return false;
		for (auto m : motion_storage) {
			os << "d " << m.device_name << endl;
			os << "n " << m.device_posi.size() << endl;
			for (int i = 0; i < m.device_posi.size(); i++) {
				os << "p " << m.device_posi.at(i) << endl;
				os << "o " << m.device_orie.at(i) << endl;
				if(write_stemp)
					os << "stemp " << m.time_stemp.at(i) << endl;
			}
		}
		os.close();
		return true;
	}
	///
	bool read_tj_file() {
		std::string f = cgv::gui::file_open_dialog("Open", "trajectory files:*");
		std::ifstream is(f);
		if (is.fail())
			return false;
		std::string current_device_name;
		while (!is.eof()) {
			char buffer[2048];
			is.getline(&buffer[0], 2048);
			std::string line(buffer);
			if (line.empty())
				continue;
			std::stringstream ss(line, std::ios_base::in);
			string c;
			ss >> c;
			if (c._Equal("d")) {
				ss >> c;
				motion_storage_read.push_back(motion_storage_per_device(c));
				current_device_name = c;
			}
			if (c._Equal("p")) {
				vec3 tmp_vec3;
				ss >> tmp_vec3;
				motion_storage_read.back().device_posi.push_back(tmp_vec3);
			}
			if (c._Equal("o")) {
				quat tmp_ori;
				ss >> tmp_ori;
				motion_storage_read.back().device_orie.push_back(tmp_ori);
			}
		}
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
