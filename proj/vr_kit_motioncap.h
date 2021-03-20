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


////////////////////////////////////////////
//	take response for data uploading and downloading 
//	have to push to movable boxes
//	trackerbles are rendered separatly, we manipulate positions and 
//	orientations here 
//	objects asking for a position in timer event, if not present, do nothing 
//		query by name 
////////////////////////////////////////////
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


	//////////////////////////////render_a_timeline//////////////////////////////
	std::vector<vec3> P;
	std::vector<float> R;
	std::vector<rgb> C;
	cgv::render::rounded_cone_render_style rounded_cone_style;
	vec3 timeline_left_end = vec3(-0.1, 0.05, -0.02);
	vec3 timeline_right_end = vec3(0.1, 0.05, -0.02);
	bool render_a_timeline_configured = false;

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
	//
	int timer_count = 0;
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		timer_count++;
		if (data_ptr==nullptr)
			return;
		if (data_ptr->rec_pose) {
			// this can maintain unchanged when adding more trackables
			data_ptr->upload_to_motion_storage();
		}
		if (data_ptr->is_replay ) {
			if ((timer_count % data_ptr->frame_factor == 0)) {
				if (cur_frame >= num_of_frames)
					cur_frame = 0;
				// this can maintain unchanged when adding more trackables
				data_ptr->download_from_motion_storage_read_per_frame(cur_frame);
				data_ptr->download_from_trackable_list_per_frame();
			
				std::cout << "cur_frame: " << cur_frame << std::endl;
				cur_frame++;
				data_ptr->realtimeOffset = vec3(
					timeline_left_end.x() + ( timeline_right_end.x() - timeline_left_end.x())*(float)cur_frame/num_of_frames, 
					timeline_left_end.y(), 
					timeline_left_end.z());
				data_ptr->realtimeOffset = data_ptr->get_global_from_local_lefthand(data_ptr->realtimeOffset);
			}
			
		}
	}
	/// setting the view transform yourself
	/// call this
	void draw(context& ctx)
	{
		if (data_ptr) {
			if (!data_ptr->rec_pose) {
				render_a_handhold_box(ctx);
			}
			if (data_ptr->is_replay) {
				render_a_timeline(ctx);
				render_a_timelineBox(ctx);
			}
			for (auto& t : data_ptr->trackable_list) {
				//trackable* tt = &t;
				//trackable_mesh* tt = static_cast<trackable_mesh*>(&t);
				t.draw(ctx);
			}
			// add here
		}
	}

	///
	void render_a_timeline(cgv::render::context& ctx) {
		if (!render_a_timeline_configured) {
			P.push_back(timeline_left_end);
			P.push_back(timeline_right_end);
			R.push_back(0.002);
			R.push_back(0.002);
			C.push_back(0.6);
			C.push_back(0.6);
			render_a_timeline_configured = true;
		}

		// update positions, local to global at run time 
		P[0] = data_ptr->get_global_from_local_lefthand(timeline_left_end);
		P[1] = data_ptr->get_global_from_local_lefthand(timeline_right_end);

		//
		auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
		cr.set_render_style(rounded_cone_style);
		//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
		cr.set_position_array(ctx, P);
		cr.set_color_array(ctx, C);
		cr.set_radius_array(ctx, R);
		cr.render(ctx, 0, P.size());
	}

	////////////////////////////////////////////////////////////
	std::vector<box3> timelineBoxBase;
	std::vector<vec3> timelinePosi;
	std::vector<quat> timelineOri;
	std::vector<rgb> timelineRgb;
	cgv::render::box_render_style bs;
	bool render_a_timelineBox_configured = false;
	vec3 timelineBoxOff = vec3(0, 0.05, -0.02);
	///
	void render_a_timelineBox(cgv::render::context& ctx) {
		if (!render_a_timelineBox_configured) {
			timelineBoxBase.push_back(box3(vec3(-0.005), vec3(0.005)));
			timelinePosi.push_back(vec3(0));
			timelineOri.push_back(quat());
			timelineRgb.push_back(rgb(0.4));
			render_a_timelineBox_configured = true;
		}
		if (data_ptr->lefthand_object_positions.size() == 0)
			return;
		if (timelineBoxBase.size() == 0)
			return;

		// update to renderable structure 
		if (!data_ptr->is_replay) {
			vec3 updatedOffset = timelineBoxOff + data_ptr->realtimeOffset;
			timelinePosi[0] = data_ptr->get_global_from_local_lefthand(timelineBoxOff);
		}
		else {
			timelinePosi[0] = data_ptr->realtimeOffset;
		}

		timelineOri[0] = data_ptr->cur_left_hand_rot_quat;

		// apply to renderer 
		cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
		renderer.set_render_style(bs);
		renderer.set_box_array(ctx, timelineBoxBase);
		renderer.set_color_array(ctx, timelineRgb);
		renderer.set_translation_array(ctx, timelinePosi);
		renderer.set_rotation_array(ctx, timelineOri);
		renderer.render(ctx, 0, timelineBoxBase.size());
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
		if(data_ptr->trackable_box_list.size() &&
			data_ptr->motion_storage_read.find(data_ptr->trackable_box_list.at(0).get_name())
				!= data_ptr->motion_storage_read.end())
		num_of_frames = data_ptr->motion_storage_read.find(data_ptr->trackable_box_list.at(0).get_name())
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
		std::default_random_engine g;
		std::uniform_real_distribution<float> d(0, 1);
		float tw = 0.8f;
		float td = 0.8f;
		bool read_until_next_d = false;
		while (!is.eof()) {
			char buffer[2048];
			is.getline(&buffer[0], 2048);
			std::string line(buffer);
			if (line.empty())
				continue;
			std::stringstream ss(line, std::ios_base::in);
			
			// just a quick test error may occ. 
			bool no_bbox = true;
			bool no_color = true;
			string c;
			ss >> c;
			// ignore current d
			if (c._Equal("id")) {
				//while (ss){
				//	ss >> c;
				//	data_ptr->tj_rendering_ignore[stoi(c)] = true;
				//}
				read_until_next_d = true;
			}
			if (c._Equal("d")) {
				read_until_next_d = false;
				ss >> c;
				// c is the name of the device as string 
				data_ptr->motion_storage_read.insert(std::pair<std::string, motion_storage_per_device>(c,*(new motion_storage_per_device())));
				current_device_name = c;
				/*if(!(c._Equal("hmd") || c._Equal("left_hand") || c._Equal("right_hand")))
					data_ptr->names_tj_rendering.push_back(c);*/

				/*if ((c._Equal("Cube") ||
					c._Equal("rigRoot") ||
					c._Equal("Cube_(1)") || 
					c._Equal("Cube_(2)") || 
					c._Equal("Cube_(3)")
				))*/
					data_ptr->names_tj_rendering.push_back(c);
					//data_ptr->tj_rendering_ignore.push_back(false);
			}
			if (c._Equal("b")) {
				if (read_until_next_d)
					continue;
				vec3 t_min_pnt;
				ss >> t_min_pnt;
				vec3 t_max_pnt;
				ss >> t_max_pnt;
				data_ptr->motion_storage_read.find(current_device_name)->second.b = box3(t_min_pnt,t_max_pnt);
			}
			if (no_bbox)
			{
				if (read_until_next_d)
					continue;
				/*vec3 extent(d(g), d(g), d(g));
				extent += 0.01f;
				extent *= std::min(tw, td) * 0.1f;*/

				vec3 extent;
				if(!(current_device_name._Equal("Cube")|| 
					current_device_name._Equal("Cube_(1)")|| 
					current_device_name._Equal("Cube_(2)")|| 
					current_device_name._Equal("Cube_(3)")
				))
					extent = vec3(0.02);
				else {
					extent = vec3(0.08);
				}
				data_ptr->motion_storage_read.find(current_device_name)->second.b =
					box3(-0.5f * extent, 0.5f * extent);
			}
			if (no_color) {
				if (read_until_next_d)
					continue;
				data_ptr->motion_storage_read.find(current_device_name)->second.color
					= rgb(d(g), d(g), d(g));
			}
			if (c._Equal("c")) {
				if (read_until_next_d)
					continue;
				vec3 tmp_col;
				ss >> tmp_col;
				data_ptr->motion_storage_read.find(current_device_name)->second.color = rgb(tmp_col.x(), tmp_col.y(), tmp_col.z());
			}
			if (c._Equal("p")) {
				if (read_until_next_d)
					continue;
				vec3 tmp_vec3;
				ss >> tmp_vec3;
				data_ptr->motion_storage_read.find(current_device_name)->second.device_posi.push_back(tmp_vec3);
			}
			if (c._Equal("o")) {
				if (read_until_next_d)
					continue;
				quat tmp_ori;
				ss >> tmp_ori;
				data_ptr->motion_storage_read.find(current_device_name)->second.device_orie.push_back(tmp_ori);
			}
		}

		gen_random_trackables_after_reading_tj_files();
		start_replay_all();
	}

	// generate rendering stuff, num equals to the number of objects stored in file 
	// will ask for update by name matching 
	// initial positions are random generated  and size 
	void gen_random_trackables_after_reading_tj_files() {
		if (data_ptr == nullptr)
			return;
		float tw = 0.8f;
		float td = 0.8f;
		float th = 0.72f;
		float tW = 0.03f;

		int nr = data_ptr->names_tj_rendering.size();
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0, 1);
		std::uniform_real_distribution<float> signed_distribution(-1, 1);
		for (size_t i = 0; i < nr; ++i) {
			float x = distribution(generator);
			float y = distribution(generator);
			vec3 extent(distribution(generator), distribution(generator), distribution(generator));
			extent += 0.01f;
			extent *= std::min(tw, td) * 0.1f;

			vec3 center(-0.5f * tw + x * tw, th + tW, -0.5f * td + y * td);
			data_ptr->movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
			data_ptr->movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
			data_ptr->movable_box_translations.push_back(center);
			quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
			rot.normalize();
			data_ptr->movable_box_rotations.push_back(rot);

			// init trackable_box_list 
			// upload_to_trackable_list
			data_ptr->trackable_box_list.push_back(*(
				new trackable_box(data_ptr->names_tj_rendering.at(i),
					box3(-0.5f * extent, 0.5f * extent))));
			data_ptr->trackable_box_list.back().set_position_orientation_write(
				data_ptr->movable_box_translations[i], data_ptr->movable_box_rotations[i]);
			data_ptr->trackable_box_list.back().set_color(data_ptr->movable_box_colors[i]);
		}
	}

	void stop_and_clear_mocap_data() {
		/// main storage for motion data 
		data_ptr->motion_storage.clear();
		data_ptr->motion_storage_read.clear();
		data_ptr->is_replay = false;
		data_ptr->rec_pose = false;

		/// can be seen as an abstract data wrapper 
		//data_ptr->trackable_list.clear();
		data_ptr->trackable_box_list.clear();
		data_ptr->names_tj_rendering.clear();
		//data_ptr->trackable_imagebox_list.clear();

		data_ptr->movable_boxes.clear();
		data_ptr->movable_box_colors.clear();
		data_ptr->movable_box_translations.clear();
		data_ptr->movable_box_rotations.clear();
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
