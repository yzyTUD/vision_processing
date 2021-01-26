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
using namespace std;

class motion_storage_per_device:public cgv::render::render_types {
public:
	std::string device_name;
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

	int left_rgbd_controller_index = 0;
	int right_rgbd_controller_index = 1;
	vec3 cur_left_hand_posi;
	vec3 cur_right_hand_posi;
	vec3 cur_hmd_posi;
	quat cur_left_hand_orientation_quat;
	quat cur_right_hand_orientation_quat;
	quat cur_hmd_orie;
	bool has_pose = false;

	std::string data_dir = std::string(getenv("CGV_DATA"));
public:
	/// initialize rotation angle
	vr_kit_motioncap()
	{
		motion_storage.push_back(motion_storage_per_device("left_hand"));
		motion_storage.push_back(motion_storage_per_device("right_hand"));
		motion_storage.push_back(motion_storage_per_device("hmd"));
		connect(get_animation_trigger().shoot, this, &vr_kit_motioncap::timer_event);
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
			has_pose = true;
		}
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		if (has_pose) {
			for (auto& d : motion_storage) {
				auto microsecondsUTC = std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count());
				if (d.device_name == "left_hand") {
					d.device_posi.push_back(cur_left_hand_posi);
					d.device_orie.push_back(cur_left_hand_orientation_quat);
					d.time_stemp.push_back(microsecondsUTC);
				}
				if (d.device_name == "right_hand") {
					d.device_posi.push_back(cur_right_hand_posi);
					d.device_orie.push_back(cur_right_hand_orientation_quat);
					d.time_stemp.push_back(microsecondsUTC);
				}
				if (d.device_name == "hmd") {
					d.device_posi.push_back(cur_hmd_posi);
					d.device_orie.push_back(cur_hmd_orie);
					d.time_stemp.push_back(microsecondsUTC);
				}

			}
		}
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
	}
	///
	bool save_to_tj_file() {
		auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		auto& fn = data_dir + "/mocap_trajectory/motioncap_" + std::to_string(microsecondsUTC) + ".tj";
		std::ofstream os(fn);
		if (os.fail())
			return false;

		return true;
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
