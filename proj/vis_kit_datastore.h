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

		// default?
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

		/*vec3 normal = cross(p1 - p2, p3 - p2); normal.normalize();
		N.push_back(normal);
		N.push_back(normal);
		N.push_back(normal);
		N.push_back(normal);*/

		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
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
	std::string get_timestemp_for_filenames() {
		auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
		return std::to_string(microsecondsUTC);
	}
	
	point_cloud_interactable* point_cloud_kit = new point_cloud_interactable();
	point_cloud_interactable* point_cloud_in_hand = new point_cloud_interactable();
	bool render_handhold_pc = false;

	/// main storage for motion data 
	std::map<std::string, motion_storage_per_device> motion_storage;
	std::map<std::string, motion_storage_per_device> motion_storage_read;
	bool is_replay = false;
	bool rec_pose = false;

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
	imagebox_array* iba = nullptr;
	std::vector<vec3> pick_points;
	std::vector<rgb> pick_colors;

	// hand tracking
	int left_rgbd_controller_index = 0;
	int right_rgbd_controller_index = 1;
	vec3 cur_left_hand_posi;
	vec3 cur_left_hand_dir;
	quat cur_left_hand_rot_quat;
	vec3 cur_right_hand_posi;
	quat cur_right_hand_rot_quat = quat();
	vec3 cur_off_left;
	vec3 cur_off_right;
	//mat3 cur_left_hand_rot;
	//mat3 cur_left_hand_rot_mat;
	//mat3 cur_right_hand_rot_as_mat;

	vec3 cur_headset_position;

	// for point selection
	// superbox and interaction modes, righthand instead of boxgui for now 
	// decrate case, a little complicated when update tracking origin at each pose event 
	box3 supersampling_bbox;	
	//enum interaction_mode {
	//	None = 0,
	//	TELEPORT,
	//	DIRECTIONAL,
	//	CLIMB = 3, 
	//	SUPERSAMPLING_BOX = 4,
	//	SUPERSAMPLING_DRAW
	//}mode;
	int max_idx_num = 5;
	std::vector<vec3> righthand_posi_list;
	std::vector<vec3> righthand_dir_list;

	float paratone_1 = 0.1;
	float paratone_2 = 0.2;
	float paratone_3 = 2;
	float paratone_4 = 1;
	float paratone_5;
	float paratone_6;
	float paratone_7;

	int menu_theta = 28;

	int active_group = 1; // unlimited  
	int active_btnidx = 0; // from 0-12 
	float active_off_rotation = 0; // roulette
	int max_group_num = 10;

	// menu table: unified management: hard to iterate 
	//std::map<std::string, ivec2> menu_table;
	// control the orders easily! 
	std::vector<std::vector<std::string>> gps;
	std::vector<std::string> gp0_btns;
	std::vector<std::string> gp1_btns;
	std::vector<std::string> gp2_btns;
	std::vector<std::string> gp3_btns;

	// point cloud marking colors, the same in shaders 
	std::vector<rgba> point_selection_colors; 

	// 
	std::vector<vec3> headset_object_positions; // dynamic 
	vec3 headset_direction;

	//
	std::vector<vec3> righthand_object_positions; // dynamic, only one position, can be used globally
	std::vector<vec3> lefthand_object_positions;
	std::vector<rgb> righthand_object_colors; // 
	vec3 offset_right_global = vec3(0, 0, -0.2);
	vec3 offset_left_global = vec3(0, 0, -0.2);
	vec3 offset_headset_global = vec3(0, 0, -0.2);

	std::vector<vec3> palette_lefthand_object_positions; // dynamic positions used for rendering 
	std::vector<rgb> palette_lefthand_object_colors;
	std::vector<rgb> palette_righthand_object_colors;
	std::vector<vec3> palette_lefthand_palette_initialpose_positions;

	vec3 quad_addition_ext = vec3(0.2);

	// give/ read a name list, render their 
	std::vector<std::string> names_tj_rendering;

	//
	int frame_factor = 1;

	vis_kit_data_store_shared() {

		//initialize_trackable_list();
		//supersampling_bbox = box3(vec3(-2,0,0),vec3(2,1,1));
		//mode = interaction_mode::SUPERSAMPLING_DRAW;

		//////////////////////////////////////////////////////////////////
		//
		//////////////////////////////////////////////////////////////////
		gp0_btns.push_back("Teleport\nRotate");
		gp0_btns.push_back("Teleport\nDirectional");
		gp0_btns.push_back("Teleport\nLifting");
		gp0_btns.push_back("Teleport\nFineGrain");
		gp0_btns.push_back("Teleport\nTeleport");
		gps.push_back(gp0_btns);

		//////////////////////////////////////////////////////////////////
		// basic pc opereations, surfel, cpu side 
		//////////////////////////////////////////////////////////////////
		gp1_btns.push_back("PointCloud\nPointSize");
		gp1_btns.push_back("PointCloud\nGroupPicker");
		gp1_btns.push_back("PointCloud\nDelPoints\nTouchTo\nActivate");
		gp1_btns.push_back("PointCloud\nMarkAs\nOrig");
		gp1_btns.push_back("PointCloud\nPrepare\nMarking");
		gp1_btns.push_back("PointCloud\nToggle\npcColor");
		//gp1_btns.push_back("PointCloud\nFoldingPoints");
		//gp1_btns.push_back("PointCloud\nSuperSampling");
		gp1_btns.push_back("PointCloud\nAutoRegion\nGrowing");
		gp1_btns.push_back("PointCloud\nShowNml");
		gp1_btns.push_back("PointCloud\nToggle\nACloud");
		gp1_btns.push_back("PointCloud\nACloud\nCtrl\nRange");
		gp1_btns.push_back("PointCloud\nToggle\nCamera\nCulling");
		gp1_btns.push_back("PointCloud\nCulling\nRange");
		gps.push_back(gp1_btns);

		//
		std::vector<std::string> gp_btn_tmp;

		//////////////////////////////////////////////////////////////////
		// point cloud demo generator  
		//////////////////////////////////////////////////////////////////
		gp_btn_tmp.clear();
		gp_btn_tmp.push_back("PointCloud\nGenCube");
		gps.push_back(gp_btn_tmp);

		//////////////////////////////////////////////////////////////////
		// point cloud cleaning 
		// adjuest range with move event
		// press btn to confirm
		//////////////////////////////////////////////////////////////////
		// deletion 
		gp_btn_tmp.push_back("PCCleaning\nFake\nDel");
		gp_btn_tmp.push_back("PCCleaning\nFake\nSeleciton");
		gp_btn_tmp.push_back("PCCleaning\nSelective\nSubSampling");
		// addition 
		gp_btn_tmp.push_back("PCCleaning\nAddition\nQuad");  // maybe enough 
		gp_btn_tmp.push_back("PCCleaning\nAddition\nSphere");
		//
		gp_btn_tmp.push_back("PCCleaning\nStepBackWard");
		gp_btn_tmp.push_back("PCCleaning\nStepForward");
		gps.push_back(gp_btn_tmp);

		//////////////////////////////////////////////////////////////////
		// point cloud shading effect 
		//////////////////////////////////////////////////////////////////
		gp_btn_tmp.clear();
		gp_btn_tmp.push_back("PCShading\nLinearMelting");
		gp_btn_tmp.push_back("PCShading\nAdjustTheta");
		gp_btn_tmp.push_back("PCShading\nSphericalMelting"); 
		gps.push_back(gp_btn_tmp);

		//////////////////////////////////////////////////////////////////
		// point cloud shading effect 
		//////////////////////////////////////////////////////////////////
		gp2_btns.push_back("Meshing\nPickingPoints");
		gp2_btns.push_back("Meshing\nAdjestPointSize");
		gp2_btns.push_back("Meshing\nFaceCreation");
		gp2_btns.push_back("Meshing\nSaveMesh");
		gps.push_back(gp2_btns);

		//////////////////////////////////////////////////////////////////
		// animating 
		//////////////////////////////////////////////////////////////////
		gp_btn_tmp.clear();
		gp_btn_tmp.push_back("Animating\nPause");
		gp_btn_tmp.push_back("Animating\nContinue");
		gp_btn_tmp.push_back("Animating\nRecord");
		gp_btn_tmp.push_back("Animating\nStopRecording");
		gp_btn_tmp.push_back("Animating\nDiscrard\nRecording");
		gp_btn_tmp.push_back("Animating\nSaveRecoding");
		gp_btn_tmp.push_back("Animating\nReplay");
		/*gp_btn_tmp.push_back("Animating\");
		gp_btn_tmp.push_back("Animating\");*/
		gps.push_back(gp_btn_tmp);


		//std::default_random_engine generator;
		//std::uniform_real_distribution<float> distribution(0, 1);
		//float some_float = distribution(generator);
		//some_float = distribution(generator);
		//some_float = distribution(generator);

		/*this is replaced by fixed color initialization */
		//// first 7 colors are fixed while the others are random generated 
		//// 7 fixed color for easier visual recognization 
		//point_selection_colors.push_back(rgba(0.5f, 0.5f, 0.5f, 1.0f));
		//point_selection_colors.push_back(rgba(1.0f, 1.0f, 0.5f, 1.0f));
		//point_selection_colors.push_back(rgba(0.0f, 0.0f, 1.0f, 1.0f));
		//point_selection_colors.push_back(rgba(1.0f, 0.0f, 0.0f, 1.0f));
		//point_selection_colors.push_back(rgba(175.0f / 255, 109.0f / 255, 47.0f / 255, 1.0f));
		//point_selection_colors.push_back(rgba(148.0f / 255, 10.0f / 255, 161.0f / 255, 1.0f));
		//point_selection_colors.push_back(rgba(14.0f / 255, 100.0f / 255, 16.0f / 255, 1.0f));
		//// the rest of colors for functional selection 
		//for (int i = 0; i < point_cloud_kit->pc.num_of_functional_selections - 7; i++) {
		//	rgba tmpcol = rgba(
		//		0.9f * distribution(generator) + 0.1f,
		//		0.9f * distribution(generator) + 0.1f,
		//		0.9f * distribution(generator) + 0.1f,
		//		1.0f
		//	);
		//	point_selection_colors.push_back(tmpcol);
		//}
		//// region/ face/ edge selection, 25 for now as a very quick test 
		//for (int i = 0; i < point_cloud_kit->pc.num_of_regions; i++) {
		//	rgba tmpcol = rgba(
		//		0.9f * distribution(generator) + 0.1f,
		//		0.9f * distribution(generator) + 0.1f,
		//		0.9f * distribution(generator) + 0.1f,
		//		1.0f
		//	);
		//	point_selection_colors.push_back(tmpcol);
		//}

		/*colors for functional selection */
		point_selection_colors.push_back(rgba(1.0f, 1.0f, 0.5f, 1.0f)); 
		point_selection_colors.push_back(rgba(0.5f, 0.5f, 0.5f, 1.0f)); // ori, may not visible
		point_selection_colors.push_back(rgba(0.0f, 0.0f, 0.0f, 1.0f)); // del, may not visible, the same as ori? 
		point_selection_colors.push_back(rgba(1.0f, 0.0f, 0.0f, 1.0f));
		point_selection_colors.push_back(rgba(0.996815, 0.961756, 0.993593, 1));

		point_selection_colors.push_back(rgba(0.241852, 0.970925, 0.9684, 1));
		point_selection_colors.push_back(rgba(0.982999, 0.973534, 0.753255, 1));
		point_selection_colors.push_back(rgba(0.536838, 0.198876, 0.96145, 1));
		point_selection_colors.push_back(rgba(0.367326, 0.820252, 0.818295, 1));
		point_selection_colors.push_back(rgba(0.479585, 0.104305, 0.227698, 1));

		point_selection_colors.push_back(rgba(0.675787, 0.924162, 0.201218, 1));
		point_selection_colors.push_back(rgba(0.963543, 0.890588, 0.812987, 1));
		point_selection_colors.push_back(rgba(0.818136, 0.690167, 0.553296, 1));
		point_selection_colors.push_back(rgba(0.864216, 0.425165, 0.132141, 1));
		point_selection_colors.push_back(rgba(0.713224, 0.940594, 0.290732, 1));

		point_selection_colors.push_back(rgba(0.781966, 0.458865, 0.710862, 1));
		point_selection_colors.push_back(rgba(0.527283, 0.768819, 0.766583, 1));
		point_selection_colors.push_back(rgba(0.68993, 0.479879, 0.453004, 1));
		point_selection_colors.push_back(rgba(0.371722, 0.254068, 0.256479, 1));
		point_selection_colors.push_back(rgba(0.12865, 0.817552, 0.735442, 1));
		//
		/*colors for region selection */
		point_selection_colors.push_back(rgba(0.885186, 0.349231, 0.384895, 1));
		point_selection_colors.push_back(rgba(0.187419, 0.234203, 0.141554, 1));
		point_selection_colors.push_back(rgba(0.839713, 0.841112, 0.994662, 1));
		point_selection_colors.push_back(rgba(0.38539, 0.212664, 0.725346, 1));
		point_selection_colors.push_back(rgba(0.54153, 0.9552, 0.787375, 1));
		point_selection_colors.push_back(rgba(0.49487, 0.697245, 0.131001, 1));
		point_selection_colors.push_back(rgba(0.289188, 0.443403, 0.213307, 1));
		point_selection_colors.push_back(rgba(0.81568, 0.146095, 0.788965, 1));
		point_selection_colors.push_back(rgba(0.467858, 0.268185, 0.132797, 1));
		point_selection_colors.push_back(rgba(0.501028, 0.51219, 0.540788, 1));
		point_selection_colors.push_back(rgba(0.814578, 0.681682, 0.538812, 1));
		point_selection_colors.push_back(rgba(0.779218, 0.928787, 0.738428, 1));
		point_selection_colors.push_back(rgba(0.735197, 0.348423, 0.826778, 1));
		point_selection_colors.push_back(rgba(0.689588, 0.102537, 0.711732, 1));
		point_selection_colors.push_back(rgba(0.679565, 0.246351, 0.739634, 1));
		point_selection_colors.push_back(rgba(0.548528, 0.51043, 0.207098, 1));
		point_selection_colors.push_back(rgba(0.616379, 0.96377, 0.796525, 1));
		point_selection_colors.push_back(rgba(0.626741, 0.889082, 0.406347, 1));
		point_selection_colors.push_back(rgba(0.115997, 0.301431, 0.827358, 1));
		point_selection_colors.push_back(rgba(0.329586, 0.839121, 0.77614, 1));
		point_selection_colors.push_back(rgba(0.946067, 0.555361, 0.838757, 1));
		point_selection_colors.push_back(rgba(0.901813, 0.4714, 0.729169, 1));
		point_selection_colors.push_back(rgba(0.622861, 0.963362, 0.480849, 1));
		point_selection_colors.push_back(rgba(0.224762, 0.242252, 0.592494, 1));
		point_selection_colors.push_back(rgba(0.30714, 0.234365, 0.785558, 1));

		/*procedure color generation*/
		//for (int i = 0; i < point_cloud_kit->pc.max_num_of_selections; i++) {
		//	rgba tmpcol = rgba(
		//		0.9f * distribution(generator) + 0.1f,
		//		0.9f * distribution(generator) + 0.1f,
		//		0.9f * distribution(generator) + 0.1f,
		//		1.0f
		//	);
		//	point_selection_colors.push_back(tmpcol);
		//}
		//// print them out 
		//if (true) {
		//	int i = 0;
		//	for (auto c: point_selection_colors) {
		//		std::cout << "point_selection_colors.push_back(rgba(" 
		//			<< c[0] << "," << c[1] <<  ","  << c[2] << "," << c.alpha() << "));" << std::endl;
		//		i++;
		//		if (i == 20)std::cout << "//" << std::endl;
		//	}
		//}

		// to shader 
		/*
		vec3 COLOR_MASKS[8] = vec3[]( 
								vec3( 0.0, 0.0, 0.0 ),
                                vec3( 0.5, 0.5, 0.5 ),
                                ... 
                              );
		*/
		//if (true) {
		//	int i = 0;
		//	std::cout << "vec4 COLOR_MASKS[" << point_selection_colors.size() << "] = vec4[](" << std::endl;
		//	for (auto c: point_selection_colors) {
		//		std::cout << "vec4(" << c[0] << "," << c[1] <<  ","  << c[2] << "," << c.alpha() <<")," << std::endl;
		//		i++;
		//		if (i == 20)std::cout << "//" << std::endl;
		//	}
		//	std::cout << ");" << std::endl;
		//}

		// 
		righthand_object_positions.push_back(vec3(0));
		lefthand_object_positions.push_back(vec3(0));
		righthand_object_colors.push_back(rgb(0, 0, 1));
		vec3 offset_right_global = vec3(0, 0, -0.2);

		// pallete rendering: positions 
		for (int ix = -2; ix < 3; ix++) {
			for (int iz = 1; iz < 6; iz++) {
				palette_lefthand_object_positions.push_back(vec3(ix * 0.05, 0.1, -iz * 0.05));
				palette_lefthand_palette_initialpose_positions.push_back(vec3(ix * 0.05, 0.1, -iz * 0.05));
			}
		}

		// pallete rendering: colors  
		//init the colors rendering on left and right hand  
		for (int i = 0; i < point_cloud_kit->pc.num_of_regions; i++)
			palette_lefthand_object_colors.push_back(
				point_selection_colors[point_cloud_kit->pc.num_of_functional_selections + i]);
		palette_righthand_object_colors.push_back(
			point_selection_colors[point_cloud_kit->pc.num_of_functional_selections]);

		// acloud rendering 
		point_cloud_kit->enable_acloud_effect = false;
		point_cloud_kit->enable_headset_culling = true;

		// initialize a point cloud in your hand 
		point_cloud_in_hand->surfel_style.point_size = 0.1f;
		point_cloud_in_hand->surfel_style.halo_color_strength = 0.0f;
		point_cloud_in_hand->surfel_style.percentual_halo_width = 25.0f;
		point_cloud_in_hand->surfel_style.blend_points = true;
		point_cloud_in_hand->surfel_style.blend_width_in_pixel = 1.0f;
		point_cloud_in_hand->show_neighbor_graph = false;
		point_cloud_in_hand->show_box = false;
		point_cloud_in_hand->do_auto_view = false;
		point_cloud_in_hand->pc.create_colors();
		point_cloud_in_hand->RENDERING_STRATEGY = 1; // quad rendering 
		point_cloud_in_hand->continus_redraw = true;
		//point_cloud_in_hand->generate_pc_cube();
		//point_cloud_in_hand->compute_normals();
		//point_cloud_in_hand->orient_normals(); // Orientation with MST

	}

	vec2 get_id_with_name(string btn_name) {
		for (int i = 0; i < gps.size();i++) 
			for (int j = 0; j < gps.at(i).size(); j++) 
				if (gps.at(i).at(j)._Equal(btn_name)) 
					return vec2(i,j);
		return vec2(-1,-1);
	}

	bool check_roulette_selection(vec2 idxs) {
		int gpidx = idxs.x();
		int btnidx = idxs.y();
		if (!((btnidx >= 0) && (btnidx < 12)))
			return false;
		bool check_group = (gpidx == active_group);
		// judge with inversed && render with inversed 
		float inv_active_off_rotation = -active_off_rotation; 
		int projected_angle = ((int)inv_active_off_rotation) % 360;
		bool check_btn = (projected_angle > -menu_theta / 2.0f + 30 * btnidx)
			&& (projected_angle < menu_theta / 2.0f + 30 * btnidx);
		return check_group && check_btn;
	}

	bool check_btn_active_givenrot(float btn_off_angle) {
		float inv_active_off_rotation = -active_off_rotation;
		if (inv_active_off_rotation < 0)
			inv_active_off_rotation = 360 - ((int)-inv_active_off_rotation%360);
		int projected_angle = ((int)inv_active_off_rotation) % 360;
		bool check = (projected_angle > btn_off_angle - 14) && (projected_angle < btn_off_angle + 14);
		return check;
	}

	void supersampling_bounded_points_with_drawn_data() {
		//// at least 2 points 
		//if (righthand_posi_list.size() < 1)
		//	return;
		//if (point_cloud_kit->pc.get_nr_points() == 0)
		//	return;
		//point_cloud_kit->supersampling_within_clips(righthand_posi_list, righthand_dir_list);
	}

	void initialize_trackable_list() {
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
				if (cur_frame <= (mt->second.device_posi.size() - 1) && cur_frame <= (mt->second.device_orie.size() - 1))
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
				if(cur_frame<=(mt->second.device_posi.size()-1) && cur_frame <= (mt->second.device_orie.size() - 1))
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
				if (cur_frame <= (mt->second.device_posi.size() - 1) && cur_frame <= (mt->second.device_orie.size() - 1))
					t.set_position_orientation_read(mt->second.device_posi.at(cur_frame), mt->second.device_orie.at(cur_frame));
			}
		}
		// add here 
	}
	/// from list to drawable 
	void download_from_trackable_list_per_frame() {
		// full download when reading tj files 
		// be careful 
		for (int i = 0; i < trackable_box_list.size();i++) {
			movable_boxes.at(i) = trackable_box_list.at(i).b;
			trackable_box_list.at(i).get_position_orientation_read(movable_box_translations.at(i), movable_box_rotations.at(i));
			//movable_box_colors.at(i) = trackable_box_list.at(i).get_color();
			//vec3 extent = vec3(paratone_3, paratone_4, paratone_5);
			//trackable_box_list.at(i).b = box3(-0.5f * extent, 0.5f * extent);
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
