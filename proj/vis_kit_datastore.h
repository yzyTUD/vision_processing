#pragma once
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/math/ftransform.h>
#include <random>
#include <cgv_gl/sphere_renderer.h>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;
using namespace std;

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>
#include "vis_kit_trackable.h"

class motion_storage_per_device :public cgv::render::render_types {
public:
	int num_of_posi_rec = 0;
	box3 b;
	rgb color;
	vector<vec3> device_posi;
	vector<quat> device_orie;
	vector<std::string> time_stemp;
	bool has_box = false;
	bool has_color = false;
};

// the storage of the whole scene data 
class vis_kit_data_store_shared :public cgv::render::render_types{
public:
	std::string data_dir = std::string(getenv("CGV_DATA"));

	/// main storage for motion data 
	std::map<std::string, motion_storage_per_device> motion_storage;
	std::map<std::string, motion_storage_per_device> motion_storage_read;

	/// can be seen as an abstract data wrapper 
	std::vector<trackable_mesh> trackable_list;	
	std::vector<trackable_box> trackable_box_list; 

	/// real interactables are stored here 
	// store the movable boxes
	std::vector<box3> movable_boxes;
	std::vector<rgb> movable_box_colors;
	std::vector<vec3> movable_box_translations;
	std::vector<quat> movable_box_rotations;
	cgv::render::box_render_style movable_style;	
	// intersection points
	std::vector<vec3> intersection_points;
	std::vector<rgb>  intersection_colors;
	std::vector<int>  intersection_box_indices;
	std::vector<int>  intersection_controller_indices;

	bool is_replay = false;
	bool rec_pose = false;

	vis_kit_data_store_shared() {
		initialize();
	}
	void initialize() {
		trackable_list.push_back(*(new trackable_mesh("hmd", data_dir + "/generic_hmd.obj")));
		trackable_list.push_back(*(new trackable_mesh("left_hand" , data_dir + "/vr_controller_vive_1_5.obj")));
		trackable_list.push_back(*(new trackable_mesh("right_hand", data_dir + "/vr_controller_vive_1_5.obj")));
	}
	/// 
	// this is updated partially ori. 
	void upload_to_trackable_list_ori() {
		for (int i = 0; i < trackable_box_list.size(); i++) {
			auto& tt = trackable_box_list.at(i);
			tt.set_ori_direct_manipulation(movable_box_rotations.at(i));
		}
	}
	/// from list to motion storage 
	void upload_to_motion_storage() {
		// saparated impl.
		// partial upload when intersection happends 
		for (auto& t : trackable_list) {
			std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage.find(t.get_name());
			if (mt == motion_storage.end()) {
				// not found, init
				motion_storage_per_device* ms = new motion_storage_per_device();
				ms->has_box = false;
				ms->has_color = false;
				motion_storage.insert(std::pair<std::string, motion_storage_per_device>(t.get_name(), *ms));
			}
			else {
				vec3 posi;
				quat orie;
				t.get_position_orientation(posi, orie);
				mt->second.device_posi.push_back(posi);
				mt->second.device_orie.push_back(orie);
			}
		}
		for (auto& t : trackable_box_list) {
			std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage.find(t.get_name());
			if (mt == motion_storage.end()) {
				// not found, init
				motion_storage_per_device* ms = new motion_storage_per_device();
				ms->b = t.get_box();
				ms->has_box = true;
				ms->color = t.get_color();
				ms->has_color = true;
				motion_storage.insert(std::pair<std::string, motion_storage_per_device>(t.get_name(), *ms));
			}
			else {
				vec3 posi;
				quat orie;
				t.get_position_orientation(posi, orie);
				mt->second.device_posi.push_back(posi);
				mt->second.device_orie.push_back(orie);
			}
		}
	}
	/// from motion storage read to list 
	void download_from_motion_storage_read_once() {
		for (auto& t : trackable_box_list) {
			t.set_color(motion_storage_read.find(t.get_name())->second.color);
			t.set_box(motion_storage_read.find(t.get_name())->second.b);
		}
	}
	void download_from_motion_storage_read_per_frame(int cur_frame) {
		for (auto& t : trackable_list) {
			std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage_read.find(t.get_name());
			if (mt == motion_storage_read.end()) {
				// not found
				continue;
			}
			else {
				t.set_position_orientation_read(mt->second.device_posi.at(cur_frame), mt->second.device_orie.at(cur_frame));
			}
		}
		for (auto& t : trackable_box_list) {
			std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage_read.find(t.get_name());
			if (mt == motion_storage_read.end()) {
				// not found
				continue;
			}
			else {
				t.set_position_orientation_read(mt->second.device_posi.at(cur_frame), mt->second.device_orie.at(cur_frame));
			}
		}
	}
	/// from list to drawable 
	void download_from_trackable_list() {
		// full download when reading tj files 
		for (int i = 0; i < trackable_box_list.size();i++) {
			movable_boxes.at(i) = trackable_box_list.at(i).b;
			trackable_box_list.at(i).get_position_orientation_read(movable_box_translations.at(i), movable_box_rotations.at(i));
			movable_box_colors.at(i) = trackable_box_list.at(i).get_color();
		}
	}
	///
	void enable_replay_all() {
		is_replay = true;
		for (auto& t : trackable_list)
			t.replay = true;
		for (auto& t : trackable_box_list)
			t.replay = true;
		download_from_motion_storage_read_once();
	}
	/// scene generation fucntions: 
	void gen_random_movable_boxes(std::string name) {
		//trackable_box_list.push_back(*(new trackable_box(name)));
	}
	void gen_random_movable_spheres(std::string name) {
		//trackable_box_list.push_back(*(new trackable_box(name)));
	}
	void construct_scene() {

	}
};
