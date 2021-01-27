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
	vector<vec3> device_posi;
	vector<quat> device_orie;
	vector<std::string> time_stemp;
};

// the storage of the whole scene data 
class vis_kit_data_store :public cgv::render::render_types{
public:
	std::string data_dir = std::string(getenv("CGV_DATA"));
	std::map<std::string, motion_storage_per_device> motion_storage;
	std::map<std::string, motion_storage_per_device> motion_storage_read;
	std::vector<trackable> trackable_list;	
	std::vector<trackable_box> trackable_box_list;

	vis_kit_data_store() {
		initialize();
	}
	void initialize() {
		trackable_list.push_back(*(new trackable_mesh("hmd", data_dir + "/generic_hmd.obj")));
		trackable_list.push_back(*(new trackable_mesh("left_hand" , data_dir + "/vr_controller_vive_1_5.obj")));
		trackable_list.push_back(*(new trackable_mesh("right_hand", data_dir + "/vr_controller_vive_1_5.obj")));
	}
	/// scene generation fucntions: 
	void gen_random_movable_boxes(std::string name) {
		trackable_box_list.push_back(*(new trackable_box(name)));
	}
	void gen_random_movable_spheres(std::string name) {
		//trackable_box_list.push_back(*(new trackable_box(name)));
	}
	void construct_scene() {

	}

};
