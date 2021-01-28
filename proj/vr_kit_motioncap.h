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
#include "vis_kit_datastore.h"

class vr_kit_motioncap :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	// just store a pointer here 
	vis_kit_data_store_shared* data_ptr = nullptr;
	vr_view_interactor* vr_view_ptr;

	int left_rgbd_controller_index = 0;
	int right_rgbd_controller_index = 1;
	vec3 cur_left_hand_posi;
	vec3 cur_right_hand_posi;
	vec3 cur_hmd_posi;
	quat cur_left_hand_orientation_quat;
	quat cur_right_hand_orientation_quat;
	quat cur_hmd_orie;
	bool write_stemp = false;

	bool have_new_mesh = false;
	int cur_frame = 0;
	int num_of_frames = 0;
public:
	//bool rec_pose = false;
	//bool replay = false;
	bool instanced_redraw = true;
	/// initialize rotation angle
	vr_kit_motioncap()
	{
		connect(get_animation_trigger().shoot, this, &vr_kit_motioncap::timer_event);	
	}
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
	}
	void set_vr_view_ptr(vr_view_interactor* p) {
		vr_view_ptr = p;
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
		if (data_ptr==nullptr)
			return false;
		if (e.get_kind() == cgv::gui::EID_KEY) {
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			int ci = vrke.get_controller_index();
			if (ci == left_rgbd_controller_index && vrke.get_key() == vr::VR_GRIP) {
				if (vrke.get_action() == cgv::gui::KA_PRESS) {
					data_ptr->rec_pose = !data_ptr->rec_pose;
					if (!data_ptr->rec_pose)
						save_to_tj_file();
				}
			}
		}
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

			for (auto& t : data_ptr->trackable_list) {
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
		if (data_ptr==nullptr)
			return;
		if (data_ptr->rec_pose) {
			// this can maintain unchanged when adding more trackables
			data_ptr->upload_to_motion_storage();
		}
		if (data_ptr->is_replay) { 
			if (cur_frame >= num_of_frames)
				cur_frame = 0;
			// this can maintain unchanged when adding more trackables
			data_ptr->download_from_motion_storage_read_per_frame(cur_frame);
			data_ptr->download_from_trackable_list_per_frame();
			
			std::cout << "cur_frame: " << cur_frame << std::endl;
			cur_frame++;
		}
	}
	/// setting the view transform yourself
	/// call this
	void draw(context& ctx)
	{
		if (data_ptr) {
			if (!data_ptr->rec_pose)
				render_a_handhold_box(ctx);
			for (auto& t : data_ptr->trackable_list) {
				//trackable* tt = &t;
				//trackable_mesh* tt = static_cast<trackable_mesh*>(&t);
				//t.draw(ctx);
				//TODO 
				t.draw(ctx);
			}
			// add here
		}

	}
	///
	void render_a_handhold_box(cgv::render::context& ctx) {
		if (!vr_view_ptr)
			return;
		vr::vr_kit_state* state_ptr = (vr::vr_kit_state*)vr_view_ptr->get_current_vr_state();
		if (!state_ptr)
			return;
		vec3 p = mat34(3, 4, state_ptr->controller[right_rgbd_controller_index].pose) * vec4(0,0,-0.1, 1.0f);
		auto& prog = ctx.ref_surface_shader_program();
		prog.set_uniform(ctx, "map_color_to_material", 3);
		prog.enable(ctx);
		ctx.set_color(rgb(0.4));
		ctx.tesselate_box(box3(p- vec3(0.01), p + vec3(0.01)), false, false);
		prog.disable(ctx);
	}
	///
	void start_replay_all() {
		if (data_ptr==nullptr)
			return;
		// this can maintain unchanged when adding more trackables
		num_of_frames = data_ptr->motion_storage_read.find(data_ptr->trackable_list.at(0).get_name())
			->second.device_posi.size();
		data_ptr->enable_replay_all();
	}
	///
	bool save_to_tj_file() {
		if (data_ptr==nullptr)
			return false;
		data_ptr->rec_pose = false;
		auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		auto& fn = data_ptr->data_dir + "/mocap_trajectory/motioncap_" + std::to_string(microsecondsUTC) + ".tj";
		std::ofstream os(fn);
		if (os.fail())
			return false;
		for (std::map<std::string, motion_storage_per_device>::iterator it = data_ptr->motion_storage.begin(); it != data_ptr->motion_storage.end(); ++it){
			os << "d " << it->first << endl;
			if(it->second.has_box)
				os << "b " << it->second.b.get_min_pnt() <<" "<< it->second.b.get_max_pnt() << endl;
			if (it->second.has_color)
				os << "c " << it->second.color << endl;
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
		if (data_ptr==nullptr)
			return false;
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
				data_ptr->motion_storage_read.insert(std::pair<std::string, motion_storage_per_device>(c,*(new motion_storage_per_device())));
				current_device_name = c;
			}
			if (c._Equal("b")) {
				vec3 t_min_pnt;
				ss >> t_min_pnt;
				vec3 t_max_pnt;
				ss >> t_max_pnt;
				data_ptr->motion_storage_read.find(current_device_name)->second.b = box3(t_min_pnt,t_max_pnt);
			}
			if (c._Equal("c")) {
				vec3 tmp_col;
				ss >> tmp_col;
				data_ptr->motion_storage_read.find(current_device_name)->second.color = rgb(tmp_col.x(), tmp_col.y(), tmp_col.z());
			}
			if (c._Equal("p")) {
				vec3 tmp_vec3;
				ss >> tmp_vec3;
				data_ptr->motion_storage_read.find(current_device_name)->second.device_posi.push_back(tmp_vec3);
			}
			if (c._Equal("o")) {
				quat tmp_ori;
				ss >> tmp_ori;
				data_ptr->motion_storage_read.find(current_device_name)->second.device_orie.push_back(tmp_ori);
			}
		}
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
