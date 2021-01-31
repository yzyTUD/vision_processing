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

class imagebox :cgv::render::render_types {
protected:
public:
	box3 bkg_box;
	vec3 ext;

	vec3 posi;
	quat ori;
	rgb color;

	texture tex;
	unsigned texture_resolution = 1024;
	cgv::render::textured_material material;
	float ratio;
	int tex_index;
	imagebox(context& ctx, box3 b, std::string fn) {
		bkg_box = b;
		int w, h;
		tex.create_from_image(ctx, fn, &w, &h);
		ratio = (float)w / h;

		tex_index = material.add_image_file(fn);
		material.set_diffuse_index(tex_index);

		ext = b.get_extent();
		bkg_box.ref_min_pnt().x() *= ratio;
		bkg_box.ref_max_pnt().x() *= ratio;
		ext.x() *= -ratio;
		posi = vec3(0);
		ori = quat();
		color = rgb(0.2, 0.6, 0.2);
	}
	void set_posi_ori(vec3 p, quat o) { posi = p; ori = o; }
	void set_color(rgb c) { color = c; }
	// render with self defined quad
	void render(context& ctx) {
		// 
		vec3 p1(0.5 * ext.x(), 0.5 * ext.y(), 0);
		vec3 p2(-0.5 * ext.x(), 0.5 * ext.y(), 0);
		vec3 p3(0.5 * ext.x(), -0.5 * ext.y(), 0);
		vec3 p4(-0.5 * ext.x(), -0.5 * ext.y(), 0);

		vec3 addi_offset = vec3(0);

		//if (has_intersec && !is_static)
		//	addi_offset = vec3(0, 0, -0.1f); 

		p1 = p1 + vec3(0, 0, -0.5 * ext.z() - 0.001) + addi_offset;
		p2 = p2 + vec3(0, 0, -0.5 * ext.z() - 0.001) + addi_offset;
		p3 = p3 + vec3(0, 0, -0.5 * ext.z() - 0.001) + addi_offset;
		p4 = p4 + vec3(0, 0, -0.5 * ext.z() - 0.001) + addi_offset;

		// rotate and translate according to the gui boxes
		ori.rotate(p1);
		ori.rotate(p2);
		ori.rotate(p3);
		ori.rotate(p4);

		p1 = p1 + posi;
		p2 = p2 + posi;
		p3 = p3 + posi;
		p4 = p4 + posi;

		cgv::render::shader_program& prog = ctx.ref_surface_shader_program(true);
		int pi = prog.get_position_index();
		int ni = prog.get_normal_index();
		int ti = prog.get_texcoord_index();
		std::vector<vec3> P;
		std::vector<vec3> N;
		std::vector<vec2> T;

		P.push_back(p1); T.push_back(vec2(1.0f, 1.0f));
		P.push_back(p2); T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p3); T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p4); T.push_back(vec2(0.0f, 0.0f));

		vec3 normal = cross(p1 - p2, p3 - p2); normal.normalize();
		N.push_back(normal);
		N.push_back(normal);
		N.push_back(normal);
		N.push_back(normal);

		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ni, N);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ni);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);
		prog.enable(ctx);
		material.enable_textures(ctx);
		ctx.enable_material(material);
		//tex.enable(ctx);
		glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
		ctx.disable_material(material);
		//tex.disable(ctx);
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ni);
		cgv::render::attribute_array_binding::disable_global_array(ctx, ti);

	}
};

class imagebox_array :cgv::render::render_types {
public:
	std::vector<imagebox> imagebox_list;

	std::vector<box3> boxarr;
	std::vector<vec3> posiarr;
	std::vector<quat> oriarr;
	std::vector<rgb> colorarr;
	cgv::render::box_render_style bs;

	void push_cur_imagebox_to_array(imagebox m) {
		imagebox_list.push_back(m);
	}

	void update_posi_ori_img_given_idx(vec3 p, quat o, int idx) {
		imagebox_list.at(idx).posi = p;
		imagebox_list.at(idx).ori = o;
	}

	void update_posi_ori_given_idx(vec3 p, quat o, int idx) {
		posiarr.at(idx) = p;
		imagebox_list.at(idx).posi = p;
		oriarr.at(idx) = o;
		imagebox_list.at(idx).ori = o;
	}

	void prepare_rendering_once() {
		for (auto& img : imagebox_list) {
			boxarr.push_back(img.bkg_box);
			posiarr.push_back(img.posi);
			oriarr.push_back(img.ori);
			colorarr.push_back(img.color);
		}
	}

	void render(context& ctx) {
		for (auto& img : imagebox_list) {
			img.render(ctx);
		}
		if (boxarr.size()) {
			cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
			renderer.set_render_style(bs);
			renderer.set_box_array(ctx, boxarr);
			renderer.set_color_array(ctx, colorarr);
			renderer.set_translation_array(ctx, posiarr);
			renderer.set_rotation_array(ctx, oriarr);
			renderer.render(ctx, 0, boxarr.size());
		}
	}
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
	std::vector<trackable_box> trackable_imagebox_list;

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

	// intersection points
	std::vector<vec3> ipimg;
	std::vector<rgb>  icimg;
	std::vector<int>  ibidximg;
	std::vector<int>  icidximg;

	imagebox_array* iba;

	std::vector<vec3> pick_points;
	std::vector<rgb> pick_colors;

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
	void imageboxes_init_to_trackable_list() {
		if (!iba)
			return;
		//
		for (int i = 0; i < iba->boxarr.size(); i++) {
			trackable_box* tb = new trackable_box("imagebox_" + std::to_string(i), iba->boxarr.at(i));
			tb->set_position_orientation_write(iba->posiarr.at(i),iba->oriarr.at(i));
			tb->set_color(iba->colorarr.at(i));
			trackable_imagebox_list.push_back(*tb);
		}
		// add here 
	}
	// for a quick test 
	void test_upload_to_trackable_list_ori() {
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
		for (auto& t : trackable_box_list) { // we assume that we have color and box in this list 
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
		for (auto& t : trackable_imagebox_list) { // we assume that we have color and box in this list 
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
		for (auto& t : trackable_imagebox_list) {
			t.set_color(motion_storage_read.find(t.get_name())->second.color);
			t.set_box(motion_storage_read.find(t.get_name())->second.b);
		}
		// add here 
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
		for (auto& t : trackable_imagebox_list) {
			std::map<std::string, motion_storage_per_device>::iterator mt = motion_storage_read.find(t.get_name());
			if (mt == motion_storage_read.end()) {
				// not found
				continue;
			}
			else {
				t.set_position_orientation_read(mt->second.device_posi.at(cur_frame), mt->second.device_orie.at(cur_frame));
			}
		}
		// add here 
	}
	/// from list to drawable 
	void download_from_trackable_list_per_frame() {
		// full download when reading tj files 
		for (int i = 0; i < trackable_box_list.size();i++) {
			movable_boxes.at(i) = trackable_box_list.at(i).b;
			trackable_box_list.at(i).get_position_orientation_read(movable_box_translations.at(i), movable_box_rotations.at(i));
			//movable_box_colors.at(i) = trackable_box_list.at(i).get_color();
		}
		for (int i = 0; i < trackable_imagebox_list.size(); i++) {
			iba->boxarr.at(i) = trackable_imagebox_list.at(i).b;
			vec3 tt; quat to;
			trackable_imagebox_list.at(i).get_position_orientation_read(tt, to);
			iba->update_posi_ori_given_idx(tt,to,i);
			//iba->colorarr.at(i) = trackable_box_list.at(i).get_color();
		}
		// add here 
	}
	///
	void enable_replay_all() {
		is_replay = true;
		for (auto& t : trackable_list)
			t.replay = true;
		for (auto& t : trackable_box_list)
			t.replay = true;
		for (auto& t : trackable_imagebox_list)
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
