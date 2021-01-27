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
	int num_of_posi_rec = 0;
	vector<vec3> device_posi;
	vector<quat> device_orie;
	vector<std::string> time_stemp;
};

class vr_kit_motioncap :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:

	std::map<std::string, motion_storage_per_device> motion_storage;
	std::map<std::string, motion_storage_per_device> motion_storage_read;
	std::vector<trackable_mesh> trackable_list;

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
	int num_of_frames = 0;
public:
	bool rec_pose = false;
	bool replay = false;
	bool instanced_redraw = true;
	/// initialize rotation angle
	vr_kit_motioncap()
	{
		connect(get_animation_trigger().shoot, this, &vr_kit_motioncap::timer_event);	
		trackable_mesh* tm;
		tm = new trackable_mesh("left_hand", data_dir + "/vr_controller_vive_1_5.obj");
		trackable_list.push_back(*tm);
		tm = new trackable_mesh("right_hand", data_dir + "/vr_controller_vive_1_5.obj");
		trackable_list.push_back(*tm);
		tm = new trackable_mesh("hmd", data_dir + "/generic_hmd.obj");
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

			for (auto& t : trackable_list) {
				if (t.get_name()._Equal("left_hand")) 
					t.set_position_orientation_write(cur_left_hand_posi, cur_left_hand_orientation_quat);
				if (t.get_name()._Equal("right_hand"))
					t.set_position_orientation_write(cur_right_hand_posi, cur_right_hand_orientation_quat);
				if (t.get_name()._Equal("hmd"))
					t.set_position_orientation_write(cur_hmd_posi, cur_hmd_orie);
			}
			//rec_pose = true;
		}
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		if (rec_pose) {
			for (auto& t : trackable_list) {
				std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage.find(t.get_name());
				if ( mt == motion_storage.end()) {
					// not found, init
					motion_storage_per_device* ms = new motion_storage_per_device();
					motion_storage.insert(std::pair<std::string, motion_storage_per_device>(t.get_name(),*ms));
				}
				else {
					vec3 posi;
					quat orie;
					t.get_position_orientation(posi,orie);
					mt->second.device_posi.push_back(posi);
					mt->second.device_orie.push_back(orie);
				}
			}
		}
		if (replay) {
			if (cur_frame >= num_of_frames)
				cur_frame = 0;
			for (auto& t : trackable_list) {
				std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage_read.find(t.get_name());
				if (mt == motion_storage_read.end()) {
					// not found
					continue;
				}
				else {
					t.set_position_orientation_read(mt->second.device_posi.at(cur_frame),mt->second.device_orie.at(cur_frame));
				}
			}
			std::cout << "cur_frame: " << cur_frame << std::endl;
			cur_frame++;
		}
	}
	/// setting the view transform yourself
	/// call this
	void draw(context& ctx)
	{
		for (auto& t : trackable_list) {
			//trackable* tt = &t;
			//trackable_mesh* tt = static_cast<trackable_mesh*>(&t);
			//t.draw(ctx);
			//TODO 
			t.draw(ctx);
		}
	}
	void start_replay_all() {
		replay = true;
		for (auto& t : trackable_list) 
			t.replay = true;
		num_of_frames = motion_storage_read.find(trackable_list.at(0).get_name())->second.device_posi.size();
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
		for (std::map<std::string, motion_storage_per_device>::iterator it = motion_storage.begin(); it != motion_storage.end(); ++it){
			os << "d " << it->first << endl;
			os << "n " << it->second.device_posi.size() << endl;
			for (int i = 0; i < it->second.device_posi.size(); i++) {
				os << "p " << it->second.device_posi.at(i) << endl;
				os << "o " << it->second.device_orie.at(i) << endl;
				if(write_stemp)
					os << "stemp " << it->second.time_stemp.at(i) << endl;
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
				motion_storage_read.insert(std::pair<std::string, motion_storage_per_device>(c,*(new motion_storage_per_device())));
				current_device_name = c;
			}
			if (c._Equal("p")) {
				vec3 tmp_vec3;
				ss >> tmp_vec3;
				motion_storage_read.find(current_device_name)->second.device_posi.push_back(tmp_vec3);
			}
			if (c._Equal("o")) {
				quat tmp_ori;
				ss >> tmp_ori;
				motion_storage_read.find(current_device_name)->second.device_orie.push_back(tmp_ori);
			}
		}
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
