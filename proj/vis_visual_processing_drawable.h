#pragma once
#include "vis_visual_processing.h"

#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/dialog.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/media/mesh/simple_mesh.h>
#include <cg_vr/vr_events.h>

#include <random>

#include "vr_kit_intersection.h"

///
void visual_processing::init_cameras(vr::vr_kit* kit_ptr)
{
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	nr_cameras = camera_ptr->get_nr_cameras();
	frame_split = camera_ptr->get_frame_split();
	for (int i = 0; i < nr_cameras; ++i) {
		std::cout << "camera " << i << "(" << nr_cameras << "):" << std::endl;
		camera_ptr->put_camera_intrinsics(i, false, &focal_lengths[i](0), &camera_centers[i](0));
		camera_ptr->put_camera_intrinsics(i, true, &focal_lengths[2 + i](0), &camera_centers[2 + i](0));
		std::cout << "  fx=" << focal_lengths[i][0] << ", fy=" << focal_lengths[i][1] << ", center=[" << camera_centers[i] << "]" << std::endl;
		std::cout << "  fx=" << focal_lengths[2+i][0] << ", fy=" << focal_lengths[2+i][1] << ", center=[" << camera_centers[2+i] << "]" << std::endl;
		float camera_to_head[12];
		camera_ptr->put_camera_to_head_matrix(i, camera_to_head);
		kit_ptr->put_eye_to_head_matrix(i, camera_to_head);
		camera_to_head_matrix[i] = vr::get_mat4_from_pose(camera_to_head);
		std::cout << "  C2H=" << camera_to_head_matrix[i] << std::endl;
		camera_ptr->put_projection_matrix(i, false, 0.001f, 10.0f, &camera_projection_matrix[i](0, 0));
		camera_ptr->put_projection_matrix(i, true, 0.001f, 10.0f, &camera_projection_matrix[2+i](0, 0));
		std::cout << "  dP=" << camera_projection_matrix[i] << std::endl;
		std::cout << "  uP=" << camera_projection_matrix[2+i] << std::endl;
	}
	post_recreate_gui();
}
///
void visual_processing::start_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->start())
		cgv::gui::message(camera_ptr->get_last_error());
}
///
void visual_processing::stop_camera()
{
	if (!vr_view_ptr)
		return;
	vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
	if (!kit_ptr)
		return;
	vr::vr_camera* camera_ptr = kit_ptr->get_camera();
	if (!camera_ptr)
		return;
	if (!camera_ptr->stop())
		cgv::gui::message(camera_ptr->get_last_error());
}
/// compute intersection points of controller ray with movable boxes
void visual_processing::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
{
	for (size_t i = 0; i < movable_boxes.size(); ++i) {
		vec3 origin_box_i = origin - movable_box_translations[i];
		movable_box_rotations[i].inverse_rotate(origin_box_i);
		vec3 direction_box_i = direction;
		movable_box_rotations[i].inverse_rotate(direction_box_i);
		float t_result;
		vec3  p_result;
		vec3  n_result;
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin_box_i, direction_box_i,
			movable_boxes[i],
			t_result, p_result, n_result, 0.000001f)) {

			// transform result back to world coordinates
			movable_box_rotations[i].rotate(p_result);
			p_result += movable_box_translations[i];
			movable_box_rotations[i].rotate(n_result);

			// store intersection information
			intersection_points.push_back(p_result);
			intersection_colors.push_back(color);
			intersection_box_indices.push_back((int)i);
			intersection_controller_indices.push_back(ci);
		}
	}
}
/// keep track of status changes
void visual_processing::on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status)
{
	// ignore all but left controller changes
	if (ci != 0)
		return;
	vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
	// check for attaching of controller
	if (old_status == vr::VRS_DETACHED) {
		left_inp_cfg.resize(kit_ptr->get_device_info().controller[0].nr_inputs);
		for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
			left_inp_cfg[ii] = kit_ptr->get_controller_input_config(0, ii);
		post_recreate_gui();
	}
	// check for attaching of controller
	if (new_status == vr::VRS_DETACHED) {
		left_inp_cfg.clear();
		post_recreate_gui();
	}
}
/// register on device change events
void visual_processing::on_device_change(void* kit_handle, bool attach)
{
	if (attach) {
		if (last_kit_handle == 0) {
			vr::vr_kit* kit_ptr = vr::get_vr_kit(kit_handle);
			init_cameras(kit_ptr);
			if (kit_ptr) {
				last_kit_handle = kit_handle;
				// copy left controller input configurations from new device in order to make it adjustable
				left_inp_cfg.resize(kit_ptr->get_device_info().controller[0].nr_inputs);
				for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
					left_inp_cfg[ii] = kit_ptr->get_controller_input_config(0, ii);
				post_recreate_gui();
			}
		}
	}
	else {
		if (kit_handle == last_kit_handle) {
			last_kit_handle = 0;
			post_recreate_gui();
		}
	}
}
/// timer event, freq depends on rendering 
void visual_processing::timer_event(double t, double dt) {

}
/// timer event executes in an other parallel thread, fixed freq 
void visual_processing::parallel_region_growing() {
	if (!backward_grow)
		data_ptr->point_cloud_kit->grow_curr_region(selection_kit->curr_face_selecting_id);
	else
		data_ptr->point_cloud_kit->backward_grow_current_region(selection_kit->curr_face_selecting_id);
}
///
visual_processing::~visual_processing() {
	data_ptr = nullptr;
	/*if (timer_thread)
		timer_thread->join();
	if (parallel_region_growing_thread) 
		parallel_region_growing_thread->join();*/
}
///
visual_processing::visual_processing() 
{
	//draw_kit = new vr_kit_draw();
	mesh_kit = new vis_kit_meshes(); //mesh_kit->load_mesh();
	//mesh_kit_2 = new vis_kit_meshes();
	selection_kit = new vis_kit_selection(); // for 2d->3d selection, and 3d selection, point cloud selection included 
	parametric_surface_kit = new parametric_surface(); // parametric surface rendering 

	//imagebox_kit = new vr_kit_imagebox();
	//motioncap_kit = new vr_kit_motioncap(); //read_tj_file();
	//data_ptr->initialize_trackable_list();
	//manipulation_kit = new vr_kit_manipulation();
	//imagebox_kit = new vr_kit_imagebox();
	//roller_coaster_kit_1 = new vr_kit_roller_coaster_1();

	set_name("visual_processing");
	vr_view_ptr = 0;
	view_ptr = 0;
	ray_length = 2;		
	// for selection tool 
	sphere_style.map_color_to_material = CM_COLOR_AND_OPACITY;
	sphere_style.surface_color = rgb(0.8f, 0.4f, 0.4f);
	sphere_style.radius = 0.01;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &visual_processing::on_device_change);
	connect(cgv::gui::ref_vr_server().on_status_change, this, &visual_processing::on_status_change);
	//connect(get_animation_trigger().shoot, this, &visual_processing::timer_event);
	//timer_thread = new thread(&visual_processing::parallel_timer_event,this);
	register_object(base_ptr(light_kit), "");

	cone_style.radius = 0.01f;
	
	// register 
	register_object(base_ptr(mesh_kit), "");
	/*
	register_object(base_ptr(mesh_kit_2), "");*/
}
///	
void visual_processing::stream_help(std::ostream& os) {
	os << "visual_processing: no shortcuts defined" << std::endl;
}
///
bool visual_processing::self_reflect(cgv::reflect::reflection_handler& rh)
{
	return
		rh.reflect_member("curr_region", selection_kit->curr_face_selecting_id);
}
///		
void visual_processing::on_set(void* member_ptr)
{
	update_member(member_ptr);
	post_redraw();
}
///	
bool visual_processing::init(cgv::render::context& ctx)
{
	if(skybox_kit!=nullptr)skybox_kit->init(ctx);
	//image_renderer_kit->init(ctx);

	cgv::gui::connect_vr_server(true);
	view_ptr = find_view_as_node();
	if (view_ptr) {
		view_ptr->set_eye_keep_view_angle(dvec3(0, 4, -4));
		// if the view points to a vr_view_interactor
		vr_view_ptr = dynamic_cast<vr_view_interactor*>(view_ptr);
		if (vr_view_ptr) {
			// configure vr event processing
			vr_view_ptr->set_event_type_flags(
				cgv::gui::VREventTypeFlags(
					cgv::gui::VRE_DEVICE +
					cgv::gui::VRE_STATUS +
					cgv::gui::VRE_KEY +
					cgv::gui::VRE_ONE_AXIS_GENERATES_KEY +
					cgv::gui::VRE_ONE_AXIS +
					cgv::gui::VRE_TWO_AXES +
					cgv::gui::VRE_TWO_AXES_GENERATES_DPAD +
					cgv::gui::VRE_POSE
				));
			vr_view_ptr->enable_vr_event_debugging(false);
			// configure vr rendering
			vr_view_ptr->draw_action_zone(false);
			vr_view_ptr->draw_vr_kits(true);
			vr_view_ptr->enable_blit_vr_views(true);
			vr_view_ptr->set_blit_vr_view_width(200);
		}
	}


	cgv::render::ref_box_renderer(ctx, 1);
	cgv::render::ref_sphere_renderer(ctx, 1);
	cgv::render::ref_rounded_cone_renderer(ctx, 1);

	/*mesh_kit->show_wireframe = false;
	mesh_kit->use_texture = true;
	mesh_kit->cull_mode = CM_OFF;*/

	// config data_ptr->point_cloud_kit
	data_ptr->point_cloud_kit->surfel_style.point_size = 0.2f;
	data_ptr->point_cloud_kit->surfel_style.halo_color_strength = 0.0f;
	data_ptr->point_cloud_kit->surfel_style.percentual_halo_width = 25.0f;
	data_ptr->point_cloud_kit->surfel_style.blend_points = true;
	data_ptr->point_cloud_kit->surfel_style.blend_width_in_pixel = 1.0f;
	data_ptr->point_cloud_kit->show_neighbor_graph = false;
	data_ptr->point_cloud_kit->show_box = false;
	data_ptr->point_cloud_kit->do_auto_view = false;
	data_ptr->point_cloud_kit->pc.create_colors();

	/*one_shot_360pc->surfel_style.point_size = 0.2f;
	one_shot_360pc->surfel_style.halo_color_strength = 0.0f;
	one_shot_360pc->surfel_style.percentual_halo_width = 25.0f;
	one_shot_360pc->surfel_style.blend_points = true;
	one_shot_360pc->surfel_style.blend_width_in_pixel = 1.0f;
	one_shot_360pc->show_neighbor_graph = false;
	one_shot_360pc->show_box = false;
	one_shot_360pc->do_auto_view = false;
	one_shot_360pc->pc.create_colors();

	stored_cloud->surfel_style.point_size = 0.2f;
	stored_cloud->surfel_style.halo_color_strength = 0.0f;
	stored_cloud->surfel_style.percentual_halo_width = 25.0f;
	stored_cloud->surfel_style.blend_points = true;
	stored_cloud->surfel_style.blend_width_in_pixel = 1.0f;
	stored_cloud->show_neighbor_graph = false;
	stored_cloud->show_box = false;
	stored_cloud->do_auto_view = false;
	stored_cloud->pc.create_colors();*/

	// set the view ptrs 
	if (teleportation_kit != nullptr) teleportation_kit->set_vr_view_ptr(vr_view_ptr);
	if (draw_kit != nullptr) draw_kit->set_vr_view_ptr(vr_view_ptr);
	if (manipulation_kit != nullptr) manipulation_kit->set_vr_view_ptr(vr_view_ptr);
	if (motioncap_kit != nullptr) motioncap_kit->set_vr_view_ptr(vr_view_ptr);

	// set the data ptrs 
	if (teleportation_kit != nullptr) teleportation_kit->set_data_ptr(data_ptr);
	if (motioncap_kit != nullptr) motioncap_kit->set_data_ptr(data_ptr);
	if (manipulation_kit != nullptr)manipulation_kit->set_data_ptr(data_ptr);
	if (imagebox_kit != nullptr) imagebox_kit->set_data_ptr(data_ptr);
	if (selection_kit!=nullptr) selection_kit->set_data_ptr(data_ptr);
	if (handhold_near_kit != nullptr) handhold_near_kit->set_data_ptr(data_ptr);
	if (tmpfixed_gui_kit != nullptr) tmpfixed_gui_kit->set_data_ptr(data_ptr);
	if (mesh_kit != nullptr) mesh_kit->set_data_ptr(data_ptr);
	if (parametric_surface_kit!=nullptr) parametric_surface_kit->set_data_ptr(data_ptr);

	// set the context ptrs 
	if (selection_kit != nullptr) selection_kit->set_context_str(get_context());

	data_ptr->point_cloud_kit->init(ctx);
	data_ptr->point_cloud_in_hand->init(ctx);
	/*one_shot_360pc->init(ctx);
	stored_cloud->init(ctx);*/
	if (draw_kit != nullptr) draw_kit->init(ctx);
	if (imagebox_kit != nullptr) imagebox_kit->init(ctx);

	if(env_render_kit!=nullptr) env_render_kit->setup_scene();

	// light sources are not ava. now 
	//ctx.disable_light_source(ctx.get_enabled_light_source_handle(0));
	//std::map<void*, std::pair<cgv::media::illum::light_source, context::light_source_status> >* 
	//cgv::media::illum::light_source ls = ctx.get_light_source(ctx.get_enabled_light_source_handle(0));
	//ctx.disable_light_source(ctx.get_enabled_light_source_handle(0));

	//init_6_points_picking();

	return true;
}
/*interactive grow, test before using vr */
///
void visual_processing::prepare_grow_ourmethod() {
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
	//data_ptr->point_cloud_kit->reset_queue_with_seeds(); // reset growing parameters, faces are marked as origin 
}
///	
bool visual_processing::handle(cgv::gui::event& e)
{
	/* do not return explicitly */
	//if (b_interactable != nullptr)b_interactable->handle(e);
	if (handhold_near_kit != nullptr)handhold_near_kit->handle(e);
	if (tmpfixed_gui_kit != nullptr)tmpfixed_gui_kit->handle(e);
	if (teleportation_kit != nullptr)teleportation_kit->handle(e);
	if (draw_kit != nullptr)draw_kit->handle(e);
	if (motioncap_kit != nullptr) motioncap_kit->handle(e);
	if (manipulation_kit != nullptr) manipulation_kit->handle(e); 
	if (selection_kit != nullptr)selection_kit->handle(e);
	if (parametric_surface_kit != nullptr) parametric_surface_kit->handle(e);

	// main handler for gui 
	// adding a new function: key function -> render -> adjustment

	// toggle operations 
	if (e.get_kind() == cgv::gui::EID_THROTTLE) {
		auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
		float v = te.get_value();
		bool d = (v == 1); // event 
		if (te.get_controller_index() == data_ptr->right_rgbd_controller_index) {
			//
			if (d) {
				// click to toggle 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) {
					if (data_ptr->point_cloud_kit->which_effect_righthand == 0) {
						data_ptr->point_cloud_kit->which_effect_righthand = -1;
					}
					else {
						data_ptr->point_cloud_kit->which_effect_righthand = 0;
					}
				}
				// click to toggle 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nSeleciton"))) {
					if (data_ptr->point_cloud_kit->which_effect_righthand == 1) {
						data_ptr->point_cloud_kit->which_effect_righthand = -1;
					}
					else {
						data_ptr->point_cloud_kit->which_effect_righthand = 1;
					}
				}
				// click to toggle 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCShading\nLinearMelting"))) {
					if (data_ptr->point_cloud_kit->which_effect_righthand == 2) {
						data_ptr->point_cloud_kit->which_effect_righthand = -1;
					}
					else {
						data_ptr->point_cloud_kit->which_effect_righthand = 2;
					}
				}
			}

			// compute shader marking, is_triggering will continuely updated to gpu 
			if (v > 0) { 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("CSPCCleaning\nMarking"))) {
					data_ptr->point_cloud_kit->is_triggering = true;
				}
			}
			else {
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("CSPCCleaning\nMarking"))) {
					data_ptr->point_cloud_kit->is_triggering = false;
				}
			}
		}
	}
	// menu key 
	if (e.get_kind() == cgv::gui::EID_KEY) {
		cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
		if (vrke.get_key() == vr::VR_MENU) { //
			if (vrke.get_action() == cgv::gui::KA_PRESS) { //
				if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { //
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nDelPoints\nTouchTo\nActivate"))) { //
						// actual deletion 
						/*data_ptr->point_cloud_kit->pc.remove_deleted_points_impl();
						data_ptr->point_cloud_kit->on_point_cloud_change_callback(
							PointCloudChangeEvent(PCC_POINTS_RESIZE + PCC_COMPONENTS_RESIZE));
						post_redraw();*/
						data_ptr->point_cloud_kit->visual_delete = !data_ptr->point_cloud_kit->visual_delete;
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) {
						del_menu_btn_press();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nClipping"))) {
						del_clipping_btn_press();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nSelective\nSubSampling"))) {
						selective_downsampling_menu_btn_press();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nAddition\nQuad"))) {
						quad_addition_menu_btn_press();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nCopyPoints"))) {
						// the copied points will fillow the movemnt of users right hand 
						point_copy_btn_pressed();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nRenderSrcOnly"))) {
						data_ptr->point_cloud_kit->mark_face_id_with_controller( 
							data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
							data_ptr->point_cloud_kit->controller_effect_range, 
							point_cloud::TOPOAttribute::ICP_SOURCE_A
						);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nRenderTargetOnly"))) {
						data_ptr->point_cloud_kit->mark_face_id_with_controller(
							data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
							data_ptr->point_cloud_kit->controller_effect_range,
							point_cloud::TOPOAttribute::ICP_TARGET_A
						);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod"))) {
						data_ptr->point_cloud_kit->prepare_grow(true);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nMarkAsOrig"))) {
						//data_ptr->point_cloud_kit->prepare_grow(true);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nMarkQueue\nToOrig"))) {
						//data_ptr->point_cloud_kit->prepare_grow(true);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nMarkAsCurr"))) {
						//data_ptr->point_cloud_kit->prepare_grow(true);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nMarkSpawn\nQueue"))) {
						data_ptr->point_cloud_kit->clear_curr_queue_and_restore_attributes(selection_kit->curr_face_selecting_id);
					}
					
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nCurvatureViewer"))) {
						data_ptr->point_cloud_kit->force_render_with_original_color = !data_ptr->point_cloud_kit->force_render_with_original_color;
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nSaveToFile\nQuiet"))) {
						quiet_save();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nBindPCtoRHand"))) {
						data_ptr->point_cloud_kit->apply_transfrom_to_pc(); // flush current matrix to pc 
					}
				}
			}
			if (vrke.get_action() == cgv::gui::KA_PRESS) { //
				if (vrke.get_controller_index() == data_ptr->left_rgbd_controller_index) { //
					render_handhold_gui = !render_handhold_gui;
				}
			}
			if (vrke.get_action() == cgv::gui::KA_RELEASE) { //
				if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { //
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) {
						del_menu_btn_release();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nSelective\nSubSampling"))) {
						selective_downsampling_menu_btn_release();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nAddition\nQuad"))) {
						quad_addition_menu_btn_release();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nCopyPoints"))) {
						// update the copied point cloud once, then use mvp to update position 
						// disable handhold pc rendering, clean it 
						// after release, points will be added back to the original point cloud kit 
						point_copy_btn_release();
					}
				}
			}
		}

		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nScaleModel"))) {
			if (vrke.get_key() == vr::VR_DPAD_UP) {
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						upscale_model_one_step();
					}
				}
			}
			if (vrke.get_key() == vr::VR_DPAD_DOWN) {
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						down_scale_model_one_step();
					}
				}
			}
		}

		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) { // selection 
			if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
			{
				if (vrke.get_key() == vr::VR_DPAD_UP) {
					if (vrke.get_action() == cgv::gui::KA_PRESS) {
						data_ptr->point_cloud_kit->controller_effect_range += 0.001f;
					}
				}
				if (vrke.get_key() == vr::VR_DPAD_DOWN) {
					if (vrke.get_action() == cgv::gui::KA_PRESS) {
						data_ptr->point_cloud_kit->controller_effect_range -= 0.001f;
					}
				}
			}
		}
		/*press them clockwise */ 
		// 1 // prepare curr growing 
		if (vrke.get_key() == vr::VR_DPAD_UP) {
			if (vrke.get_action() == cgv::gui::KA_PRESS)
			{
				if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
				{
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod"))) {
						prepare_grow_ourmethod();
					}
				}
			}
		}

		// right: start growing 
		if (vrke.get_key() == vr::VR_DPAD_RIGHT) {
			if (vrke.get_action() == cgv::gui::KA_PRESS)
			{
				if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
				{
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod"))) {
						//force_start_grow();
						//pause_continue_parallel_region_growing(); 
						ep_start_grow_auto_curv_direction(); // automatic select which grow strategy 
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nBackGrow"))) {
						//force_start_grow();
						//pause_continue_parallel_region_growing(); 
						//ep_start_grow_auto_curv_direction(); // automatic select which grow strategy 
						backward_growing();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nSyncGrow"))) {
						sync_grow_dist_curvature_based();
					}
					
				}
			}
		}

		// down: pause, continue 
		if (vrke.get_key() == vr::VR_DPAD_DOWN) {
			if (vrke.get_action() == cgv::gui::KA_PRESS)
			{
				if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
				{
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod"))) {
						pause_continue_parallel_region_growing();
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nBackGrow"))) {
						pause_continue_parallel_region_growing();
					}
					
				}
			}
		}

		// left: undo_curr_region
		if (vrke.get_key() == vr::VR_DPAD_LEFT) {
			if (vrke.get_action() == cgv::gui::KA_PRESS)
			{
				if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
				{
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod"))) {
						data_ptr->point_cloud_kit->undo_curr_region(selection_kit->curr_face_selecting_id);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nBackGrow"))) {
						data_ptr->point_cloud_kit->undo_curr_region(selection_kit->curr_face_selecting_id);
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nSyncGrow"))) {
						undo_sync_grow();
					}
				}
			}
		}
		
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nClickPoints\nDpd\nSrc"))) {
			if (vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_src.at(0) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_src.at(0) = rgb(1, 0, 0);
					}
				}
			}			
			if (vrke.get_key() == vr::VR_DPAD_UP)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_src.at(1) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_src.at(1) = rgb(0, 1, 0);
					}
				}
			}
			if (vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_src.at(2) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_src.at(2) = rgb(0, 0, 1);
					}
				}
			}
			if (vrke.get_key() == vr::VR_DPAD_DOWN)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_src.at(3) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_src.at(3) = rgb(1, 1, 0);
					}
				}
			}
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nClickPoints\nDpd\nTarget"))) {
			if (vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_target.at(0) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_target.at(0) = rgb(1, 0, 0);
					}
				}
			}
			if (vrke.get_key() == vr::VR_DPAD_UP)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_target.at(1) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_target.at(1) = rgb(0, 1, 0);
					}
				}
			}
			if (vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_target.at(2) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_target.at(2) = rgb(0, 0, 1);
					}
				}
			}
			if (vrke.get_key() == vr::VR_DPAD_DOWN)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
				{
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index)
					{
						data_ptr->point_cloud_kit->icp_clicking_points_target.at(3) = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						data_ptr->point_cloud_kit->icp_clicking_point_colors_target.at(3) = rgb(1, 1, 0);
					}
				}
			}
		}

	}
	// touch/ move to adjest 
	if (e.get_kind() == cgv::gui::EID_STICK) {
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		if (vrse.get_action() == cgv::gui::SA_TOUCH) { // event 
			if (vrse.get_controller_index() == data_ptr->right_rgbd_controller_index) { // controller 
				/*basic point cloud operations */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nFoldingPoints"))) { // selection 
					if (vrse.get_y() > 0) {
						// supersampling 
						int num_points = data_ptr->point_cloud_kit->pc.get_nr_points();
						data_ptr->point_cloud_kit->downsampling(step, num_points * 2.0, 1);
					}
					else {
						int num_points = data_ptr->point_cloud_kit->pc.get_nr_points();
						data_ptr->point_cloud_kit->downsampling(step, num_points / 2.0, 1);
					}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nShowNml"))) { 
					data_ptr->point_cloud_kit->show_nmls = !data_ptr->point_cloud_kit->show_nmls;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nToggle\nACloud"))) {
					data_ptr->point_cloud_kit->enable_acloud_effect = 
						!data_ptr->point_cloud_kit->enable_acloud_effect;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nToggle\nCamera\nCulling"))) {
					data_ptr->point_cloud_kit->enable_headset_culling = 
						!data_ptr->point_cloud_kit->enable_headset_culling;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nDelPoints\nTouchTo\nActivate"))) {
					// this will be used in the throttle event 
					selection_kit->curr_face_selecting_id = point_cloud::TOPOAttribute::DEL;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nMarkAs\nOrig"))) {
					// this will be used in the throttle event 
					selection_kit->curr_face_selecting_id = point_cloud::TOPOAttribute::ORI;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nToggle\npcColor"))) {
					data_ptr->point_cloud_kit->render_with_original_color = !data_ptr->point_cloud_kit->render_with_original_color;
				}
				/*VR ICP: touch to activate */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nSetSrcAndTarget\nScanIndex(Via GUI)"))) {
					data_ptr->point_cloud_kit->src_scan_idx = 1;
					data_ptr->point_cloud_kit->target_scan_idx = 0;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nHighlightSrcAndTarget"))) {
					data_ptr->point_cloud_kit->colorize_with_scan_index = !data_ptr->point_cloud_kit->colorize_with_scan_index;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nRenderBoth"))) {
					data_ptr->point_cloud_kit->render_both_src_target_clouds();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nRenderSrcOnly"))) {
					data_ptr->point_cloud_kit->render_select_src_cloud_only();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nRenderTargetOnly"))) {
					data_ptr->point_cloud_kit->render_select_target_cloud_only();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nExtractPointClouds\nMarkedOnly"))) {
					data_ptr->point_cloud_kit->extract_point_clouds_for_icp_marked_only();
					// visual feedback 
					data_ptr->point_cloud_kit->render_both_src_target_clouds();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nExtractPointClouds"))) {
					data_ptr->point_cloud_kit->extract_point_clouds_for_icp();
					// visual feedback 
					data_ptr->point_cloud_kit->render_both_src_target_clouds();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nPerformICP\n_8CorrespPoints"))) {
					// state: you need to click 4 pair points before this 
					data_ptr->point_cloud_kit->perform_icp_manual_clicking();
					// state: matrix will be applied to point cloud after this 
				}
				// this has been moved to the move event 
					/*if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nPerformICP_10Iters"))) {
						data_ptr->point_cloud_kit->perform_icp_and_acquire_matrices();
						data_ptr->point_cloud_kit->apply_register_matrices_for_the_original_point_cloud();
					}*/
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nRandomizeSource"))) {
					data_ptr->point_cloud_kit->pc.randomize_position(data_ptr->point_cloud_kit->src_scan_idx);
				}
				/*region growing */
				//
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nBindPCtoRHand"))) {
					data_ptr->point_cloud_kit->use_controller_transformations = true;
					data_ptr->point_cloud_kit->bind_point_cloud_to_rhand = !data_ptr->point_cloud_kit->bind_point_cloud_to_rhand;
					if (data_ptr->point_cloud_kit->bind_point_cloud_to_rhand) {
						data_ptr->last_fixed_mat = data_ptr->point_cloud_kit->pc.last_additional_model_matrix;
						data_ptr->fixed_poseMat = data_ptr->poseMat_rhand;
					}
					else {
						data_ptr->point_cloud_kit->apply_transfrom_to_pc(); // apply after release
					}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nRenderTopoSel"))) {
					data_ptr->point_cloud_kit->render_with_topo_selction = !data_ptr->point_cloud_kit->render_with_topo_selction;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod\nToggleIgnore\nHighCurvatureRegions"))) {
					//data_ptr->point_cloud_kit->undo_curr_region(selection_kit->curr_face_selecting_id);
					data_ptr->point_cloud_kit->ignore_high_curvature_regions = !data_ptr->point_cloud_kit->ignore_high_curvature_regions;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nReset\nMarking"))) {
					data_ptr->point_cloud_kit->prepare_grow(true);
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nAutoRegion\nGrowing"))) {
					//timer_thread = new thread(&visual_processing::parallel_region_growing, this);
					//timer_thread->exit();
					data_ptr->point_cloud_kit->do_region_growing_directly = !data_ptr->point_cloud_kit->do_region_growing_directly;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nToggle\nCheckNmls"))) {
					data_ptr->point_cloud_kit->region_grow_check_normals = !data_ptr->point_cloud_kit->region_grow_check_normals;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nHighlightUnmarked"))) {
					data_ptr->point_cloud_kit->highlight_unmarked_points = !data_ptr->point_cloud_kit->highlight_unmarked_points;
				}
				/*topology extraction */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("TopologyExtraction\nBoundary\nExtraction"))) {
					data_ptr->point_cloud_kit->point_classification();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("TopologyExtraction\nStepBack"))) {
					data_ptr->point_cloud_kit->step_back_last_selection();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("TopologyExtraction\nToggle\nOnlyRender\nFunctionIdx"))) {
					//data_ptr->point_cloud_kit->render_with_functional_ids_only = !data_ptr->point_cloud_kit->render_with_functional_ids_only;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("TopologyExtraction\nTopology\nExtraction"))) {
					//data_ptr->point_cloud_kit->extract_connectivity_graph();
				}
				/*point cleaning */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nStepBackWard"))) {
					data_ptr->point_cloud_kit->step_back_last_selection();
					send_updated_point_cloud_to_gpu();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nStepForWard"))) {
					data_ptr->point_cloud_kit->step_forward_selection();
					send_updated_point_cloud_to_gpu();
				}
				/*compute shader point cleaning */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("CSPCCleaning\nResetMarking"))) {
					data_ptr->point_cloud_kit->reset_marking();
				}
				//
				/*animating */ 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Animating\nPause"))) {
					data_ptr->is_replay = false;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Animating\nContinue"))) {
					data_ptr->is_replay = true;
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Animating\nRenderAnAnimating\nTube"))) {
					data_ptr->render_an_animating_tube = !data_ptr->render_an_animating_tube;
				}
				/*demo point cloud loading */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nGenCube"))) {
					generate_pc_cube(); // well prepared to be used 
					send_updated_point_cloud_to_gpu();
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nGenPlane"))) {
					data_ptr->point_cloud_kit->generate_testing_plane();
					send_updated_point_cloud_to_gpu();
				}
			}
		}
		if (vrse.get_action() == cgv::gui::SA_MOVE) { // event 
			if (vrse.get_controller_index() == data_ptr->right_rgbd_controller_index) { // controller 
				/*point cloud rendering */ 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nPointSize"))) {
					if (vrse.get_y() > 0) {
						// larger point size 
						data_ptr->point_cloud_kit->surfel_style.point_size += 0.1f;
						data_ptr->point_cloud_kit->point_size += 0.1f;
					}
					else {
						data_ptr->point_cloud_kit->surfel_style.point_size -= 0.1f;
						data_ptr->point_cloud_kit->point_size -= 0.1f;
					}
				}
				/*region grow */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nOurMethod"))) {
					// vrse.get_y() : -1, 1 ... -vrse.get_y()+1 : 0 , 2 ... (-vrse.get_y()+1) * 100 : 200,0
					data_ptr->point_cloud_kit->growing_latency = int((-vrse.get_y() + 1) * 300) + 1;
					// UPPER means faster 
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nCurvatureViewer"))) {
					//ep_force_recolor();
					//prepare_grow_ourmethod();
					if (vrse.get_y() > 0) {
						data_ptr->point_cloud_kit->pc.curvinfo.coloring_threshold += 0.0001f;
						data_ptr->point_cloud_kit->ep_force_recolor();
					}
					else {
						data_ptr->point_cloud_kit->pc.curvinfo.coloring_threshold -= 0.0001f;
						data_ptr->point_cloud_kit->ep_force_recolor();
					}
				}
				/*vr icp */ 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("VRICP\nPerformICP_1Iter"))) {
					data_ptr->point_cloud_kit->perform_icp_and_acquire_matrices();
					data_ptr->point_cloud_kit->apply_register_matrices_for_the_original_point_cloud();
				}
				/*artistic cloud rendering */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nCulling\nRange"))) {
					if (vrse.get_y() > 0) {
						// larger point size 
						data_ptr->point_cloud_kit->headset_culling_range += 0.1f;
					}
					else {
						data_ptr->point_cloud_kit->headset_culling_range -= 0.1f;
					}
				} 
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nACloud\nCtrl\nRange"))) {
					if (vrse.get_y() > 0) {
						// larger point size 
						data_ptr->point_cloud_kit->controller_effect_range += 0.01f;
					}
					else {
						data_ptr->point_cloud_kit->controller_effect_range -= 0.01f;
					}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCShading\nAdjustTheta"))) {
					if (vrse.get_y() > 0) {
						// larger point size 
						data_ptr->point_cloud_kit->collapse_tantheta += 0.01f;
					}
					else {
						data_ptr->point_cloud_kit->collapse_tantheta -= 0.01f;
					}
				}
				/*point cloud cleaning */
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nSelective\nSubSampling"))) {
					if (vrse.get_y() > 0) {
						// larger point size 
						data_ptr->point_cloud_kit->controller_effect_range += 0.01f;
					}
					else {
						data_ptr->point_cloud_kit->controller_effect_range -= 0.01f;
					}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) {
					//if (vrse.get_y() > 0) {
					//	// larger point size 
					//	data_ptr->point_cloud_kit->controller_effect_range += 0.01f;
					//}
					//else {
					//	data_ptr->point_cloud_kit->controller_effect_range -= 0.01f;
					//}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nAddition\nQuad"))) {
					if (vrse.get_y() > 0 && abs(vrse.get_x()) < 0.1) {
						data_ptr->quad_addition_ext.y() += 0.01f;
					}
					else if(vrse.get_y() < 0 && abs(vrse.get_x()) < 0.1){
						data_ptr->quad_addition_ext.y() -= 0.01f;
					}

					if (vrse.get_x() > 0 && abs(vrse.get_y()) < 0.1) {
						data_ptr->quad_addition_ext.x() += 0.01f;
					}
					else if (vrse.get_x() < 0 && abs(vrse.get_y()) < 0.1) {
						data_ptr->quad_addition_ext.x() -= 0.01f;
					}
				}
			}
		}

	}

	if (data_ptr == nullptr)
		return false;
	if (e.get_kind() == EID_MOUSE) {
		auto& me = static_cast<cgv::gui::mouse_event&>(e);
		//on_pick(me);
		switch (me.get_action()) {
			case MA_PRESS:
				if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_CTRL) {
					if (!view_ptr)
						return false;
					in_picking = true;
					on_pick(me);
					post_redraw();
					return true;
				}

				if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_ALT) {
					on_pick_face(me);
					post_redraw();
					return true;
				}
				break;
			case MA_DRAG:
				if (in_picking) {
					on_pick(me);
					post_redraw();
				}
				break;
			case MA_RELEASE:
				if (me.get_button() == MB_LEFT_BUTTON && me.get_modifiers() == EM_CTRL) {
					/*if (in_picking && pick_point_index != -1) {
						data_ptr->pick_colors[pick_point_index] = rgb(0, 1, 0);
						pick_point_index = -1;
						post_redraw();
					}*/
					in_picking = false;
					return true;
				}
				break;
		}
	}
	if (e.get_kind() == EID_KEY) {
		auto& ke = static_cast<key_event&>(e);
		if (ke.get_action() != KA_RELEASE) {
			switch (ke.get_key()) {
			// next and previous point
			case 'N': pick_point_index++;  on_set(&pick_point_index); return true;
			case 'P': pick_point_index--;  on_set(&pick_point_index); return true;
			}
		}
	}
	return false;
}
///	
void visual_processing::clear(cgv::render::context& ctx)
{
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
	data_ptr->point_cloud_kit->clear(ctx);
	//one_shot_360pc->clear(ctx);
	//stored_cloud->clear(ctx);
}
///	
void visual_processing::init_frame(cgv::render::context& ctx)
{
	if (tmpfixed_gui_kit != nullptr)tmpfixed_gui_kit->init_frame(ctx);
	data_ptr->point_cloud_kit->init_frame(ctx);
	if(imagebox_kit!=nullptr)imagebox_kit->init_frame(ctx);
	//if (b_interactable != nullptr)b_interactable->init_frame(ctx);
	//if (handhold_near_kit != nullptr) handhold_near_kit->init_frame(ctx);
	/*one_shot_360pc->init_frame(ctx);
	stored_cloud->init_frame(ctx);*/
}
///	
void visual_processing::draw(cgv::render::context& ctx)
{
	
	if (camera_ready) {
		ctx.set_bg_color(1, 1, 1, 1);
	}
	else {
		if (render_skybox) {
			if (skybox_kit != nullptr)skybox_kit->draw(ctx);
		}
		else {
			ctx.set_bg_color(0, 0, 0, 1);
		}
	}
	if (render_env) if (env_render_kit != nullptr) env_render_kit->draw(ctx);
	if (render_handhold_gui) if (tmpfixed_gui_kit != nullptr) tmpfixed_gui_kit->draw(ctx);
	if (data_ptr->render_handhold_pc) data_ptr->point_cloud_in_hand->draw(ctx);
	if (roller_coaster_kit_1) roller_coaster_kit_1->draw(ctx);
	if (draw_kit!=nullptr) draw_kit->render_trajectory(ctx);
	if (motioncap_kit!=nullptr) motioncap_kit->draw(ctx);
	if (manipulation_kit!=nullptr) manipulation_kit->draw(ctx);
	if (imagebox_kit != nullptr)imagebox_kit->draw(ctx);
	if (selection_kit != nullptr)selection_kit->draw(ctx);
	if (parametric_surface_kit != nullptr) parametric_surface_kit->draw(ctx);
	ctx.push_modelview_matrix();
	if (!data_ptr->point_cloud_kit->use_controller_transformations) {
		data_ptr->point_cloud_kit->pc.curr_additional_model_matrix = data_ptr->point_cloud_kit->pc.last_additional_model_matrix;
	}	
	else {
		if (data_ptr->point_cloud_kit->bind_point_cloud_to_rhand) {
			data_ptr->point_cloud_kit->pc.curr_additional_model_matrix = data_ptr->poseMat_rhand * inv(data_ptr->fixed_poseMat) * data_ptr->last_fixed_mat;
			data_ptr->point_cloud_kit->pc.last_additional_model_matrix = data_ptr->point_cloud_kit->pc.curr_additional_model_matrix; // updating last matrix
		}
		else {
			data_ptr->point_cloud_kit->pc.curr_additional_model_matrix = data_ptr->point_cloud_kit->pc.last_additional_model_matrix; // unbind, last_additional_model_matrix will not be updated 
		}
	}
	ctx.set_modelview_matrix(ctx.get_modelview_matrix() * data_ptr->point_cloud_kit->pc.curr_additional_model_matrix);
	if (render_pc) data_ptr->point_cloud_kit->draw(ctx);
	ctx.pop_modelview_matrix();
	//if (b_interactable != nullptr) b_interactable->draw(ctx);
	//if (handhold_near_kit!=nullptr) handhold_near_kit->draw(ctx);

	//if (motioncap_kit != nullptr)
	//	if (motioncap_kit->instanced_redraw)
	//		post_redraw();

	// point and camera visualization 
	if (point_and_cam.size() > 0) {
		cgv::render::rounded_cone_renderer& sr = cgv::render::ref_rounded_cone_renderer(ctx);
		sr.set_render_style(cone_style);
		sr.set_position_array(ctx, point_and_cam);
		sr.set_color_array(ctx, point_and_cam_colors);
		sr.render(ctx, 0, point_and_cam.size());
	}

	if (instance_redraw)
		post_redraw();
}
///	
void visual_processing::finish_draw(cgv::render::context& ctx)
{
	// selection tool
	if (!view_ptr)
		return;
	if (!data_ptr)
		return;
	if (teleportation_kit) teleportation_kit->finish_draw(ctx);  // check gui selection, not fast enough  
	if (selection_kit) selection_kit->finish_draw(ctx);

	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	//sphere_style.radius = float(0.1 * sqrt(mesh_kit->B.get_extent().sqr_length() / mesh_kit->M.get_nr_positions()));
	if (!data_ptr->pick_points.empty()) {
		sphere_renderer& sr = ref_sphere_renderer(ctx);
		if (view_ptr)
			sr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
		sr.set_render_style(sphere_style);
		sr.set_position_array(ctx, data_ptr->pick_points);
		sr.set_color_array(ctx, data_ptr->pick_colors);
		sr.validate_and_enable(ctx);
		glDrawArrays(GL_POINTS, 0, (GLsizei)data_ptr->pick_points.size());
		sr.disable(ctx);
	}
	//glDepthMask(GL_FALSE); 
	//if (pick_point_index != -1) {
	//	glDisable(GL_BLEND);
	//	glDisable(GL_DEPTH_TEST);
	//	vec3 p = data_ptr->pick_points[pick_point_index];
	//	if (view_ptr)
	//		p += 1.5f * sphere_style.radius * sphere_style.radius_scale * vec3(view_ptr->get_view_up_dir());
	//	std::stringstream ss;
	//	ss << "[" << p << "]";
	//	ss.flush();

	//	ctx.set_color(rgb(0.1f, 0.1f, 0.1f));
	//	ctx.set_cursor(p.to_vec(), ss.str(), (TextAlignment)TA_BOTTOM, 0, 0);
	//	ctx.output_stream() << ss.str();
	//	ctx.output_stream().flush();

	//	ctx.set_color(rgb(0.9f, 0.9f, 0.9f));
	//	ctx.set_cursor(p.to_vec(), ss.str(), (TextAlignment)TA_BOTTOM, 1, -1);
	//	ctx.output_stream() << ss.str();
	//	ctx.output_stream().flush();

	//	glEnable(GL_DEPTH_TEST);
	//	glEnable(GL_BLEND);
	//}
	//glDepthMask(GL_TRUE);
	//glDisable(GL_BLEND);
}
///	
void visual_processing::read_campose() {
	data_ptr->point_cloud_kit->read_pc_campose(*get_context(), quat());
	//render_pc = true;
}
///	
void visual_processing::show_camposes() {
	for (int i = 0; i < data_ptr->point_cloud_kit->pc.list_cam_translation.size(); i++) {
		std::cout << "---w x y z: \nrot: " << 
			data_ptr->point_cloud_kit->pc.list_cam_rotation.at(i).w() << " " <<
			data_ptr->point_cloud_kit->pc.list_cam_rotation.at(i).x() << " " <<
			data_ptr->point_cloud_kit->pc.list_cam_rotation.at(i).y() << " " <<
			data_ptr->point_cloud_kit->pc.list_cam_rotation.at(i).z() << " " <<
			std::endl;
		std::cout << "trans: " << data_ptr->point_cloud_kit->pc.list_cam_translation.at(i) << std::endl;
	}
}
///	for a visual feedback 
void visual_processing::apply_further_transformation() {
	data_ptr->point_cloud_kit->apply_further_transformation(0, quat(), vec3(1));
}
/*pc reading and data_ptr->point_cloud_kit processing tool */
///  read the whole pc to data_ptr->point_cloud_kit
void visual_processing::read_pc() {
	if (data_ptr == nullptr)
		return;

	// the original pc will be automatically stored 
	data_ptr->point_cloud_kit->read_pc_with_dialog(false);
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	data_ptr->point_cloud_kit->using_directly_buffer_loading = false;
	
	if(auto_growing_prepare_thesis)
		single_hit__prepare_region_grow(overwrite_face_id);
	else
		data_ptr->point_cloud_kit->prepare_grow(overwrite_face_id); // reading from file, do not overwrite, this will very fast 
	
	if(put_points_to_table)
		data_ptr->point_cloud_kit->auto_scale_after_read_points();
}
///  read the whole pc to data_ptr->point_cloud_kit
void visual_processing::read_pc_parallel() {
	// wait until flag avaliable
	while (data_ptr->point_cloud_kit->can_parallel_edit == false) {}
	// can perform operations to the cloud 
	// lock if you want to change ds, rebuild tree 
	data_ptr->point_cloud_kit->can_parallel_edit = false;
	// the original pc will be automatically stored 
	if (!data_ptr->point_cloud_kit->read_pc_with_dialog(false)) {
		data_ptr->point_cloud_kit->can_parallel_edit = true;
		return;
	}
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	data_ptr->point_cloud_kit->prepare_grow(false); //  reading from file, do not overwrite point face selection  
	data_ptr->point_cloud_kit->can_parallel_edit = true;
	std::cout << "reading done" << std::endl;
}
///
void visual_processing::start_reading_pc_parallel() {
	if (parallel_reading)
		parallel_reading_thread = new thread(&visual_processing::read_pc_parallel, this);
	else
		read_pc();
	data_ptr->point_cloud_kit->can_parallel_edit = true;
}
///
void visual_processing::start_parallel_region_growing() {
	if (parallel_region_growing_thread == nullptr) {
		data_ptr->point_cloud_kit->pause_growing = false;
		parallel_region_growing_thread = new thread(&visual_processing::parallel_region_growing, this);
	}
	//parallel_region_growing();
}
///
void visual_processing::residual_grow_dist_and_curvature_based() {
	// re-set some parameters, flexible for switching between 
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
	data_ptr->point_cloud_kit->growing_latency = 200;
	data_ptr->point_cloud_kit->ignore_high_curvature_regions = false;
	data_ptr->point_cloud_kit->is_residual_grow = true;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::residual_grow_curvature_based() {
	// re-set some parameters, flexible for switching between 
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::UNSIGNED_MEAN_CURVATURE_BASED;
	data_ptr->point_cloud_kit->growing_latency = 200;
	data_ptr->point_cloud_kit->ignore_high_curvature_regions = false;
	data_ptr->point_cloud_kit->is_residual_grow = true;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::sync_grow_dist_curvature_based() {
	data_ptr->point_cloud_kit->record_current_state_before_sync_grow();
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
	data_ptr->point_cloud_kit->ignore_high_curvature_regions = false;
	data_ptr->point_cloud_kit->is_synchronous_growth = true;
	data_ptr->point_cloud_kit->growing_latency = 0;
	data_ptr->point_cloud_kit->region_grow_check_normals = true;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::sync_grow_dist_based() {
	data_ptr->point_cloud_kit->record_current_state_before_sync_grow();
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::ACCU_DISTANCE_BASED;
	data_ptr->point_cloud_kit->ignore_high_curvature_regions = false;
	data_ptr->point_cloud_kit->is_synchronous_growth = true;
	//data_ptr->point_cloud_kit->growing_latency = 0;
	//data_ptr->point_cloud_kit->region_grow_check_normals = true;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::sync_grow_curv_based() {
	data_ptr->point_cloud_kit->record_current_state_before_sync_grow();
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::UNSIGNED_MEAN_CURVATURE_BASED;
	data_ptr->point_cloud_kit->ignore_high_curvature_regions = false;
	data_ptr->point_cloud_kit->is_synchronous_growth = true;
	//data_ptr->point_cloud_kit->growing_latency = 0;
	//data_ptr->point_cloud_kit->region_grow_check_normals = true;
	backward_grow = false;
	force_start_grow();
}
/// will grow after hitting this button
void visual_processing::grow_with_dist_and_lowest_curvature() {
	// re-set some parameters, flexible for switching between 
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
	data_ptr->point_cloud_kit->is_residual_grow = false;
	//data_ptr->point_cloud_kit->growing_latency = 0;
	//data_ptr->point_cloud_kit->ignore_high_curvature_regions = true;
	backward_grow = false;
	force_start_grow();
}
/// will grow after hitting this button, not working 
void visual_processing::grow_with_dist_and_highest_curvature() {
	// re-set some parameters, flexible for switching between 
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_HIGHERFIRST;
	data_ptr->point_cloud_kit->is_residual_grow = false;
	//data_ptr->point_cloud_kit->growing_latency = 0;
	//data_ptr->point_cloud_kit->ignore_high_curvature_regions = true;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::grow_with_accu_dist() {
	// re-set some parameters, flexible for switching between 
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::ACCU_DISTANCE_BASED;
	//data_ptr->point_cloud_kit->growing_latency = 0;
	//data_ptr->point_cloud_kit->ignore_high_curvature_regions = true;
	data_ptr->point_cloud_kit->is_residual_grow = false;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::grow_with_highest_curvature_only() {
	// re-set some parameters, flexible for switching between 
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::UNSIGNED_MEAN_CURVATURE_BASED;
	//data_ptr->point_cloud_kit->growing_latency = 0;
	//data_ptr->point_cloud_kit->ignore_high_curvature_regions = true;
	data_ptr->point_cloud_kit->is_residual_grow = false;
	backward_grow = false;
	force_start_grow();
}
///
void visual_processing::backward_growing() {
	backward_grow = true;
	force_start_grow();
}
/// check, for it will access queue 
void visual_processing::undo_sync_grow() {
	stop_parallel_region_growing(); // stop to avoid conflict visiting queue 
	data_ptr->point_cloud_kit->undo_sync_grow();
}
///
//void visual_processing::final_grow_dist_and_curvature_based() {
//	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
//	data_ptr->point_cloud_kit->final_grow = true;
//	//data_ptr->point_cloud_kit->submit_face(); // move points from queue to final queue, all faces 
//	force_start_grow(); // but, only curr region now 
//}
///
//void visual_processing::final_grow_accu_dist() {
//	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::ACCU_DISTANCE_BASED;
//	data_ptr->point_cloud_kit->final_grow = true;
//	data_ptr->point_cloud_kit->submit_face(); // move points from queue to final queue, all faces 
//	force_start_grow();
//}
///
void visual_processing::force_start_grow() {
	if (parallel_region_growing_thread != nullptr) {
		data_ptr->point_cloud_kit->pause_growing = true;
		parallel_region_growing_thread->join();
		parallel_region_growing_thread = nullptr;
	}

	data_ptr->point_cloud_kit->pause_growing = false;
	parallel_region_growing_thread = new thread(&visual_processing::parallel_region_growing, this);
}
///
void visual_processing::ep_start_grow_auto_curv_direction() {
	int curr_seed_pid = data_ptr->point_cloud_kit->seed_for_regions[selection_kit->curr_face_selecting_id];
	float curr_seed_curvature = data_ptr->point_cloud_kit->pc.curvature.at(curr_seed_pid).mean_curvature;
	if (curr_seed_curvature > data_ptr->point_cloud_kit->pc.curvinfo.coloring_threshold) {
		//grow_with_dist_and_highest_curvature();
		grow_with_highest_curvature_only();
	}
	else {
		grow_with_dist_and_lowest_curvature();
	}
}
///
void visual_processing::pause_continue_parallel_region_growing() {
	if (parallel_region_growing_thread != nullptr) {
		data_ptr->point_cloud_kit->pause_growing = true;
		parallel_region_growing_thread->join();
		parallel_region_growing_thread = nullptr;
	}else{
		data_ptr->point_cloud_kit->pause_growing = false;
		parallel_region_growing_thread = new thread(&visual_processing::parallel_region_growing, this);
	}
}
///
void visual_processing::stop_parallel_region_growing() {
	if (parallel_region_growing_thread != nullptr) {
		data_ptr->point_cloud_kit->pause_growing = true;
		parallel_region_growing_thread->join();
		parallel_region_growing_thread = nullptr;
	}
}
///
void visual_processing::read_pc_queue() {
	data_ptr->point_cloud_kit->read_pc_with_dialog_queue(false);
}
///
void visual_processing::read_pc_append() {
	data_ptr->point_cloud_kit->read_pc_with_dialog(true);
	data_ptr->point_cloud_kit->pc.currentScanIdx_Recon++; // increase current scan index after reading points  
	on_set(&data_ptr->point_cloud_kit->pc.currentScanIdx_Recon);
}
/// 
void visual_processing::downsampling() {
	data_ptr->point_cloud_kit->downsampling(step, num_of_points_wanted, strategy);
}
///
void visual_processing::add_reflectance() {
	// = true;
}
///
void visual_processing::write_pc_parallel() {
	data_ptr->point_cloud_kit->write_pc_to_file();
}
///
void visual_processing::start_writting_pc_parallel() {
	parallel_reading_thread = new thread(&visual_processing::write_pc_parallel, this);
}
///
void  visual_processing::align_leica_scans_with_cgv() {
	data_ptr->point_cloud_kit->align_leica_scans_with_cgv();
}
///
void  visual_processing::rotate_x() {
	data_ptr->point_cloud_kit->pc.rotate(quat(vec3(1, 0, 0), 5 * M_PI / 180));
	std::cout << "rotate 5 degree around x!" << std::endl;
}
///
void  visual_processing::rotate_z() {
	data_ptr->point_cloud_kit->pc.rotate(quat(vec3(0, 0, 1), 5 * M_PI / 180));
	std::cout << "rotate 5 degree around z!" << std::endl;
}
/// load one shot from data_ptr->point_cloud_kit according to .campose file  
bool visual_processing::load_next_shot() {
	one_shot_360pc->pc.clear_all_for_get_next_shot();
	return one_shot_360pc->pc.get_next_shot(data_ptr->point_cloud_kit->pc);
}
///
void visual_processing::compute_nmls_if_is_required() {
	one_shot_360pc->compute_normals();
}
///
void visual_processing::force_nml_computing() {
	data_ptr->point_cloud_kit->compute_normals();
	data_ptr->point_cloud_kit->orient_normals();
	post_redraw();
}
///
void visual_processing::append_current_shot_to_stored_cloud() {
	stored_cloud->append_frame(one_shot_360pc->pc, false);
	/// it is logical to have multiple clean functions 
	one_shot_360pc->pc.clear_all_for_get_next_shot();
}
///
void visual_processing::write_stored_pc_to_file() {
	stored_cloud->write_pc_to_file();
}
///
void visual_processing::write_stored_pc_to_file_direct() {
	stored_cloud->pc.write(cgv::base::ref_data_path_list()[0] + "/output.txt");
}
///
void visual_processing::print_cloud_info() {
	std::cout << "data_ptr->point_cloud_kit: " << data_ptr->point_cloud_kit->pc.get_nr_points() << std::endl;
	std::cout << "one_shot_360pc: " << one_shot_360pc->pc.get_nr_points() << std::endl;
	std::cout << "stored_cloud: " << stored_cloud->pc.get_nr_points() << std::endl;
}
/// you should have points and camposes loaded first! 
/// sample save directly, subsample when required is simple 
void visual_processing::auto_conduct_nml_estimation_leica() {
	int loop_num = data_ptr->point_cloud_kit->pc.num_of_shots;
	int i = 0;
	while (i < loop_num) {
		bool succ = load_next_shot();
		if (!succ && force_correct_num_pcs)
			return;
		compute_nmls_if_is_required();
		append_current_shot_to_stored_cloud();
		i++;
	}
	//
	align_leica_scans_with_cgv();
	print_cloud_info();
	if (direct_write)
		write_stored_pc_to_file_direct();
	else
		write_stored_pc_to_file();
}
/// deprecated 
void visual_processing::add_to_file_list() {
	std::vector<string> tmpfilelist;
	string tmppath;
	cgv::gui::files_open_dialog(tmpfilelist, "Open", "Point Cloud:*", tmppath);
	for (auto f : tmpfilelist) {
		f_names.push_back(tmppath + f);
	}
	for (auto f : f_names) {
		std::cout << "file: " << f <<" in list" << std::endl;
	}
}
/// deprecated 
void visual_processing::clean_file_list() {
	f_names.clear();
}
/// must have a corresp. campose file ! 
bool visual_processing::batch_compute_nmls_given_file_list() {
	f_names.clear();
	cgv::gui::files_open_dialog(f_names, "Open", "Point Cloud:*");
	for (auto& f : f_names) {
		one_shot_360pc->clear_all();
		one_shot_360pc->open(f);
		one_shot_360pc->pc.read_campose(cgv::utils::file::drop_extension(f) + ".campose");
		// check if campose is corresp. to the point cloud, print err when necess. 
		if (one_shot_360pc->check_valid_pc_and_campose()) {
			one_shot_360pc->compute_normals();
			one_shot_360pc->align_leica_scans_with_cgv();
			one_shot_360pc->save(cgv::utils::file::drop_extension(f) + "_nml.txt");
		}
		else {
			return false;
		}
	}
	return true;
}
///
bool visual_processing::batch_read_pc_queue_and_downsampling() {
	return true;
}
///
void visual_processing::clear_all_pcs() {
	data_ptr->point_cloud_kit->clear_all();
	one_shot_360pc->clear_all();
	stored_cloud->clear_all();
	std::cout << "all pcs cleared" << std::endl;
}
///
void visual_processing::load_image_from_bin_files() {
	//std::string f = cgv::gui::file_open_dialog("Open", "Images:*");
	//image_renderer_kit->bind_image_to_camera_position(*get_context(),f,quat(),vec3(0,1,0));
	//render_img = true;
	//post_redraw();
}
/*switch rendering modes */
///
void visual_processing::switch_rendering_mode_quad_based() {
	data_ptr->point_cloud_kit->RENDERING_STRATEGY = 1;
	data_ptr->point_cloud_kit->is_switching = true;
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::switch_rendering_mode_point_based() {
	data_ptr->point_cloud_kit->RENDERING_STRATEGY = 2;
	data_ptr->point_cloud_kit->is_switching = true;
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::switch_rendering_mode_surfel_based() {
	data_ptr->point_cloud_kit->RENDERING_STRATEGY = 3;
	data_ptr->point_cloud_kit->is_switching = true;
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::switch_rendering_mode_clod_based() {
	data_ptr->point_cloud_kit->RENDERING_STRATEGY = 4;
	data_ptr->point_cloud_kit->is_switching = true;
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
}
///
void visual_processing::single_hit__load_point_cloud_and_render_with_clod() {
	read_pc();
	switch_rendering_mode_clod_based();
}
///
void visual_processing::batch__load_and_convert_add_lod_single_file() {
	// disable rendering 
	render_pc = false;

	// R
	std::string fn = cgv::gui::file_open_dialog("Open", "Point Cloud:*");	
	data_ptr->point_cloud_kit->clear_all();
	if (!data_ptr->point_cloud_kit->open(fn))
		return;

	// M
	compute_lods();

	// W
	std::string fn_without_extension =
		cgv::utils::file::get_file_name(cgv::utils::file::drop_extension(fn));
	data_ptr->point_cloud_kit->write_pc_to_file_with_given_dir(fn_without_extension + ".ypc");
	std::cout << "finished converting: " << fn << std::endl;
}
///
void visual_processing::batch__load_and_convert_add_lod_info() {

	// disable rendering 
	render_pc = false;

	std::vector<std::string> fnames;
	cgv::gui::files_open_dialog(fnames, "Open", "Point Cloud:*");

	for (auto& f : fnames) {
		// R 
		data_ptr->point_cloud_kit->clear_all();
		if (!data_ptr->point_cloud_kit->open(f))
			return;
		// M 
		compute_lods();

		// W
		std::string fn_without_extension = 
			cgv::utils::file::get_file_name(cgv::utils::file::drop_extension(f));
		data_ptr->point_cloud_kit->write_pc_to_file_with_given_dir(fn_without_extension + ".ypc");
		std::cout << "finished converting: " << f << std::endl;
	}
}
///
void visual_processing::mark_all_points_as_tobedownsampled() {
	data_ptr->point_cloud_kit->marking_test_mark_all_points_as_given_group(
		(int)point_cloud::TOPOAttribute::TO_BE_SUBSAMPLED);
}
///
void visual_processing::mark_all_active_points_as_tobedownsampled() {
	data_ptr->point_cloud_kit->new_history_recording();
	//
	for (int i = 0; i < data_ptr->point_cloud_kit->pc.get_nr_points(); i++) {
		if (data_ptr->point_cloud_kit->pc.face_id[i] != point_cloud::TOPOAttribute::DEL)
			data_ptr->point_cloud_kit->pc.face_id[i] = 
				point_cloud::TOPOAttribute::TO_BE_SUBSAMPLED;
	}
}
/*quick test and then, integrate*/
///
void visual_processing::del_clipping_btn_press() {
	data_ptr->point_cloud_kit->mark_points_with_clipping_plane(
		data_ptr->cur_right_hand_posi,
		data_ptr->normal_clipping_plane_RHand,
		point_cloud::TOPOAttribute::DEL
	);
	// do not have to upload to gpu since we must use the surfel renderer for now 
	// it will continuesly update 
	// just do this for consistant 
	data_ptr->point_cloud_in_hand->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::del_menu_btn_press() {
	// marking on cpu side, mark as deleted 
	data_ptr->point_cloud_kit->mark_topo_id_with_controller(
		data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
		data_ptr->point_cloud_kit->controller_effect_range, point_cloud::TOPOAttribute::DEL);
}
///
void visual_processing::del_menu_btn_release() {
	// update to gpu 
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::selective_downsampling_menu_btn_press() {
	// mark -> TO_BE_SUBSAMPLED, to test, ignore points marked as deleted
	data_ptr->point_cloud_kit->mark_face_id_with_controller(
		data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
			data_ptr->point_cloud_kit->controller_effect_range, 
				point_cloud::TOPOAttribute::TO_BE_SUBSAMPLED);
	// TO_BE_SUBSAMPLED -> DEL
	data_ptr->point_cloud_kit->selective_subsampling_cpu();
	// reset unused marks (some time needs)
	// do not keep them as TO_BE_SUBSAMPLED, unwanted effect 
	data_ptr->point_cloud_kit->reset_last_marking_non_processed_part(
		point_cloud::TOPOAttribute::TO_BE_SUBSAMPLED);
}
///
void visual_processing::selective_downsampling_menu_btn_release() {
	// update to gpu 
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::quad_addition_menu_btn_press() {
	// wait until flag avaliable
	while(data_ptr->point_cloud_kit->can_parallel_edit == false){}
	// can perform operations to the cloud 
	// lock if you want to change ds, rebuild tree 
	data_ptr->point_cloud_kit->can_parallel_edit = false;
	data_ptr->point_cloud_kit->spawn_points_in_the_handhold_quad(
		data_ptr->cur_right_hand_rot_quat, 
			data_ptr->righthand_object_positions[0], 
				data_ptr->quad_addition_ext);
	data_ptr->point_cloud_kit->can_parallel_edit = true;
}
///
void visual_processing::quad_addition_menu_btn_release() {
	// update to gpu 
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::send_updated_point_cloud_to_gpu() {
	// update to gpu 
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::step_back_selection() {
	data_ptr->point_cloud_kit->reset_last_marked_points();
}
///
void visual_processing::step_forward_selection() {
	data_ptr->point_cloud_kit->step_forward_selection();
}
///
void visual_processing::enlarge_tube_length() { data_ptr->enlarge_tube_length(); }
///
void visual_processing::schrink_tube_length() { data_ptr->schrink_tube_length(); }
///
void visual_processing::point_copy_btn_pressed() {
	// copy points to tmp handhold cloud 
	data_ptr->point_cloud_kit->mark_points_and_push_to_tmp_pointcloud(
		data_ptr->cur_right_hand_posi + data_ptr->cur_off_right, data_ptr->point_cloud_kit->controller_effect_range,-1);
	data_ptr->point_cloud_in_hand->pc = data_ptr->point_cloud_kit->to_be_copied_pointcloud;

	// enable controller binding 
	// compute for point movement: compute "fixed" relative model matrix, multiply to additional model matrix automatically 
	data_ptr->point_cloud_in_hand->relative_model_matrix_controller_to_pc = 
			inv(data_ptr->point_cloud_in_hand->current_model_matrix) * data_ptr->point_cloud_in_hand->last_model_matrix;
	data_ptr->point_cloud_in_hand->use_current_matrix = true;

	// enable render point in hand 
	data_ptr->render_handhold_pc = true;
	// take effect directly
	data_ptr->point_cloud_in_hand->on_rendering_settings_changed();
	post_redraw();
}
///
void visual_processing::point_copy_btn_release() {
	// lock parallel safty flag 
	// wait until flag avaliable
	while (data_ptr->point_cloud_kit->can_parallel_edit == false) {}
	// can perform operations to the cloud 
	// lock if you want to change ds, rebuild tree 
	data_ptr->point_cloud_kit->can_parallel_edit = false;

	// append points to point_cloud_kit and clean all tmp points 
	data_ptr->point_cloud_kit->pc.append_with_mat4(data_ptr->point_cloud_in_hand->pc, 
		data_ptr->point_cloud_in_hand->current_model_matrix);

	// clean tmp pcs x2 
	data_ptr->point_cloud_kit->to_be_copied_pointcloud.clear_all();
	data_ptr->point_cloud_in_hand->pc.clear_all(); // clean point in hand 

	// reset matrices 
	data_ptr->point_cloud_in_hand->relative_model_matrix_controller_to_pc.identity();
	data_ptr->point_cloud_in_hand->last_model_matrix.identity();

	// disable render point in hand 
	data_ptr->render_handhold_pc = false;

	// update point cloud kit pc 
	data_ptr->point_cloud_kit->on_point_cloud_change_callback(PCC_POINTS_RESIZE);
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
	post_redraw();

	// reset safty flag for parallel tasks 
	data_ptr->point_cloud_kit->can_parallel_edit = true;
}
///
void visual_processing::release_controller_pc_binding() {
	// update and render recorded with last mvp 
	data_ptr->point_cloud_in_hand->relative_model_matrix_controller_to_pc.identity();
	data_ptr->point_cloud_in_hand->last_model_matrix = data_ptr->point_cloud_in_hand->current_model_matrix;
	data_ptr->point_cloud_in_hand->use_current_matrix = false;
}
///
void visual_processing::on_rendering_settings_changed() {
	data_ptr->point_cloud_kit->on_rendering_settings_changed();
}
///
void visual_processing::download_points_from_gpu_to_memory() {
	data_ptr->point_cloud_kit->download_points_from_gpu_to_memory();
}
///
void visual_processing::reset_marking() {
	data_ptr->point_cloud_kit->reset_marking();
}
///
void visual_processing::compute_lods() {
	data_ptr->point_cloud_kit->compute_lods();
}
///
void visual_processing::load_sample_seeds_default() {
	/*std::string fn = cgv::gui::file_save_dialog("Open", "Region Growing Seeds:*");
	std::string fn = cgv::gui::file_open_dialog("Open", "Region Growing Seeds:*");*/
	std::string seed_file = data_ptr->point_cloud_kit->data_path + "/" + data_ptr->point_cloud_kit->file_name + ".seed";
	data_ptr->point_cloud_kit->recover_seed_for_regions(seed_file);
	std::cout << "load_sample_seeds_default: loaded!" << std::endl;
}
///
void visual_processing::load_sample_seeds_with_dialog() {
	std::string fn = cgv::gui::file_open_dialog("Open", "Region Growing Seeds:*.seed");
	data_ptr->point_cloud_kit->recover_seed_for_regions(fn);
	std::cout << "load_sample_seeds_with_dialog: loaded!" << std::endl;
}
///
void visual_processing::load_seeds_with_dialog_without_recover() {
	std::string fn = cgv::gui::file_open_dialog("Open", "Region Growing Seeds:*.seed");
	data_ptr->point_cloud_kit->load_seed_for_regions(fn);
	std::cout << "load_seeds_with_dialog_without_recover: loaded!" << std::endl;
}
///
void visual_processing::save_sample_seeds_default() {
	std::string seed_file = data_ptr->point_cloud_kit->data_path + "/" + data_ptr->point_cloud_kit->file_name + ".seed";
	data_ptr->point_cloud_kit->record_seed_for_regions(seed_file);
	std::cout << "save_sample_seeds_default: saved!" << std::endl;
}
/// this may overwrite! 
void visual_processing::save_sample_seeds_with_dialog() {
	std::string fn = cgv::gui::file_save_dialog("Open", "Region Growing Seeds:*.seed");
	data_ptr->point_cloud_kit->record_seed_for_regions(fn);
	std::cout << "save_sample_seeds_with_dialog: saved!" << std::endl;
}
///
void visual_processing::single_hit__prepare_region_grow_worker(bool overwrite_face_id) {
	// not used 
}
/// single thread, some pre-computing 
void visual_processing::single_hit__prepare_region_grow(bool overwrite_face_id) {
	std::cout << "preparing everything for region growing!" << std::endl;
	if (data_ptr->point_cloud_kit->pc.nearest_neighbour_indices.size() == 0)
		data_ptr->point_cloud_kit->extract_neighbours(); // will take some time for neighbour points extraction 
	if (data_ptr->point_cloud_kit->pc.curvature.size() == 0)
		data_ptr->point_cloud_kit->ep_compute_principal_curvature_and_colorize_unsigned(); // compute unsigned curvature 
	data_ptr->point_cloud_kit->prepare_grow(overwrite_face_id);
	std::cout << "prepared! " << std::endl;
}
/// 
void visual_processing::single_hit__regrow_accu_distance_based() {
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::ACCU_DISTANCE_BASED;
	data_ptr->point_cloud_kit->reset_queue_with_seeds(); // reset growing parameters, faces are marked as unmarked  
	stop_parallel_region_growing(); // stop if thread is not stopped 
	start_parallel_region_growing(); // start 
}
///
void visual_processing::single_hit__regrow_seed_distance_based() {
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::SEED_DISTANCE_BASED;
	data_ptr->point_cloud_kit->reset_queue_with_seeds(); // reset growing parameters, faces are marked as unmarked  
	stop_parallel_region_growing(); // stop if thread is not stopped 
	start_parallel_region_growing(); // start 
}
///
void visual_processing::single_hit__regrow_unsigned_mean_curvature_based() {
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::UNSIGNED_MEAN_CURVATURE_BASED;
	data_ptr->point_cloud_kit->reset_queue_with_seeds(); // reset growing parameters, faces are marked as unmarked, recover from seed_for_regions
	stop_parallel_region_growing(); // stop if thread is not stopped  
	start_parallel_region_growing(); // start 
}
///
void visual_processing::single_hit__regrow_distance_and_curvature_based() {
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
	data_ptr->point_cloud_kit->reset_queue_with_seeds(); // reset growing parameters, faces are marked as origin  
	stop_parallel_region_growing(); // stop if thread is not stopped 	s
	start_parallel_region_growing(); // start 
}
///
void visual_processing::single_hit__regrow_stop_at_high_curvature() {
	data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::STOP_ON_BOUNDARY;
	data_ptr->point_cloud_kit->reset_queue_with_seeds(); // reset growing parameters, faces are marked as unmarked  
	stop_parallel_region_growing(); // stop if thread is not stopped 	s
	start_parallel_region_growing(); // start 
}
///
void visual_processing::find_pointcloud()
{
	cgv::render::view* view_ptr = find_view_as_node();
	if (view_ptr) {
		const point_cloud_types::Box& sb = data_ptr->point_cloud_kit->pc.box();
		point_cloud_types::Box aabb(sb);
		view_ptr->set_focus(aabb.get_center());
		view_ptr->move(view_ptr->get_depth_of_focus() - 1.0);
	}
}
///
bool visual_processing::find_next_and_increase_curr_region() {
	if (selection_kit->curr_face_selecting_id == -1)
		selection_kit->curr_face_selecting_id = 1;
	else
		selection_kit->curr_face_selecting_id++;
	int pid_next = data_ptr->point_cloud_kit->find_next_seed_in_low_curvature_area();
	if (pid_next == -1)
		return false;
	if (selection_kit->curr_face_selecting_id > 24) // extract up to 24 regions 
		return false;
	data_ptr->point_cloud_kit->seed_for_regions[selection_kit->curr_face_selecting_id] = pid_next;
	data_ptr->point_cloud_kit->add_seed_to_queue(selection_kit->curr_face_selecting_id);
	return true;
}
/// delete seed, clear queue, and mark the points as raw if no enough number of points found 
/// x5 things should be restored 
void visual_processing::drop_unwanted_regions() {
	for (int i = 0; i < data_ptr->point_cloud_kit->pc.num_of_face_selections_rendered; i++) {
		if (data_ptr->point_cloud_kit->num_of_points_curr_region[i] < 10) { // too few points 
			if (data_ptr->point_cloud_kit->num_of_points_curr_region[i] < 1)
				continue;
			data_ptr->point_cloud_kit->seed_for_regions[i] = -1;
			std::priority_queue<point_priority_mapping, std::vector<point_priority_mapping>, LowSecComp>* q;
			// pop queue and restore attributes 
			q = &data_ptr->point_cloud_kit->queue_for_regions[i];
			while (!q->empty()) {
				point_priority_mapping curr_top = q->top();
				data_ptr->point_cloud_kit->pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
				data_ptr->point_cloud_kit->pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = false;
				q->pop();
			}
			// pop queue and restore attributes 
			q = &data_ptr->point_cloud_kit->suspend_queue_for_regions[i];
			while (!q->empty()) {
				point_priority_mapping curr_top = q->top();
				data_ptr->point_cloud_kit->pc.point_in_queue.at(std::get<ID>(curr_top)) = false;
				data_ptr->point_cloud_kit->pc.point_in_queue_which_group.at(std::get<ID>(curr_top)) = false;
				q->pop();
			}
			for (int pid = 0; pid < data_ptr->point_cloud_kit->pc.get_nr_points(); pid++) {
				if (data_ptr->point_cloud_kit->pc.face_id.at(pid) == i && !data_ptr->point_cloud_kit->pc.point_in_queue.at(pid)) {
					data_ptr->point_cloud_kit->pc.face_id.at(pid) = 0;
					data_ptr->point_cloud_kit->pc.point_visited.at(pid) = 0;
					data_ptr->point_cloud_kit->points_grown--; // x5 things should be restored 
					data_ptr->point_cloud_kit->num_of_points_curr_region[i]--;
				}
			}
		}
	}
}
/*
	algorithm:
		a. while still possible to find an acceptable seed:
			b. grow current region (with any growing method)
		c. drop unwanted regions (e.g. too few points) 

*/
///
void visual_processing::automatic_region_extraction() {
	selection_kit->curr_face_selecting_id = -1;
	while (find_next_and_increase_curr_region()) {
		data_ptr->point_cloud_kit->gm = data_ptr->point_cloud_kit->growing_mode::DISTANCE_AND_MEAN_CURVATURE_BASED_LOWERFIRST;
		data_ptr->point_cloud_kit->growing_latency = 0;
		data_ptr->point_cloud_kit->ignore_high_curvature_regions = true;
		data_ptr->point_cloud_kit->is_residual_grow = false;
		data_ptr->point_cloud_kit->is_synchronous_growth = false;
		parallel_region_growing(); // not parallel here
	}
	int points = 0;
	for (int i = 0; i < data_ptr->point_cloud_kit->pc.num_of_face_selections_rendered; i++) {
		points += data_ptr->point_cloud_kit->num_of_points_curr_region[i];
	}
	std::cout << "points_grown, visited = " << points << std::endl;
	std::cout << "points_grown = " << data_ptr->point_cloud_kit->points_grown << std::endl;
	drop_unwanted_regions(); // drop if num points too few 
	std::cout << "points_grown after dropping = " << data_ptr->point_cloud_kit->points_grown << std::endl;
	data_ptr->point_cloud_kit->resume_queue();
	data_ptr->point_cloud_kit->show_num_of_points_per_region();
	std::cout << "num of regions extracted = " << selection_kit->curr_face_selecting_id - 1 << std::endl;
}
/// 
void visual_processing::down_scale_model_one_step() {
	data_ptr->point_cloud_kit->model_scale = data_ptr->point_cloud_kit->model_scale - 0.1f;
	data_ptr->point_cloud_kit->scale_model();
}
///
void visual_processing::upscale_model_one_step() {
	data_ptr->point_cloud_kit->model_scale = data_ptr->point_cloud_kit->model_scale + 0.1f;
	data_ptr->point_cloud_kit->scale_model();
}
///
void visual_processing::undo_curr_region() {
	data_ptr->point_cloud_kit->undo_curr_region(selection_kit->curr_face_selecting_id);
}
///
void visual_processing::quiet_save() {
	// recorded when reading file 
	data_ptr->point_cloud_kit->pc.suggested_point_size = data_ptr->point_cloud_kit->surfel_style.point_size;
	std::string fn_without_extension =
		cgv::utils::file::get_file_name(cgv::utils::file::drop_extension(data_ptr->point_cloud_kit->file_dir)); 
	auto microsecondsUTC = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::system_clock::now().time_since_epoch()).count();
	std::string time_stamp_for_file_name = "_" + std::to_string(microsecondsUTC);
	data_ptr->point_cloud_kit->write_pc_to_file_with_given_dir(fn_without_extension + time_stamp_for_file_name + ".ypc");
	std::cout << "saved!" << std::endl;
}
///
void visual_processing::apply_transfrom_to_pc() {
	data_ptr->point_cloud_kit->apply_transfrom_to_pc();
}
/*gui */
///
void visual_processing::create_gui() {
	add_decorator("visual_computing", "heading", "level=2");
	
	//
	bool gui_io = true;
	bool gui_rg = false;
	bool gui_irg = true;
	bool gui_pc_cleaning = true;
	bool gui_pc_rendering_style = true;
	bool gui_Fitting = true;
	bool gui_point_Scale = true;
	bool gui_icp = true;
	bool gui_Connectivity = true;
	bool gui_Triangulation = true;

	bool gui_scanindex = true;
	bool gui_copy_points = true;
	bool gui_Tube = true;
	bool gui_addition = true;
	bool gui_Generation = true;
	bool gui_ControlLOD = true;
	bool gui_Merging = true;
	bool gui_Nml_computing = true;
	bool gui_Semantic = true;
	bool gui_Mesh = true;

	bool gui_Teleportation = true;
	bool gui_Roller_Coaster = true;
	bool gui_Draw = true;
	bool gui_mocap = true;
	bool gui_selection = true;
	bool gui_render_control = true;

	if (begin_tree_node("Render Control", gui_render_control, gui_render_control, "level=3")) {
		//
			/*connect_copy(add_button("rotate_right")->click,
				rebind(this, &visual_processing::rotate_right));
			connect_copy(add_button("rotate_left")->click,
				rebind(this, &visual_processing::rotate_left));
			add_member_control(this, "active_group", data_ptr->active_group, "value_slider", "min=0;max=10;log=false;ticks=true;");
			add_member_control(this, "paratone_2", data_ptr->paratone_2, "value_slider", "min=-1;max=1;log=false;ticks=true;");
			add_member_control(this, "paratone_3", data_ptr->paratone_3, "value_slider", "min=-1;max=1;log=false;ticks=true;");
			add_member_control(this, "paratone_4", data_ptr->paratone_4, "value_slider", "min=-1;max=1;log=false;ticks=true;");
			add_member_control(this, "paratone_5", data_ptr->paratone_5, "value_slider", "min=-1;max=1;log=false;ticks=true;");*/
		add_member_control(this, "render_skybox", render_skybox, "check");
		add_member_control(this, "render_env", render_env, "check");
		add_member_control(this, "put_points_to_table", put_points_to_table, "check");
		add_member_control(this, "camera_ready", camera_ready, "check");
		add_member_control(this, "render_handhold_gui", render_handhold_gui, "check");
		add_member_control(this, "render_parametric_surface", data_ptr->render_parametric_surface, "check");
		add_member_control(this, "render_control_points", data_ptr->render_control_points, "check");
		
		add_member_control(this, "render_with_topo_selction", data_ptr->point_cloud_kit->render_with_topo_selction, "check");
		add_member_control(this, "render_with_topo_selction_only", data_ptr->point_cloud_kit->render_with_topo_selctions_only, "check");
		add_member_control(this, "colorize_with_face_selection", data_ptr->point_cloud_kit->colorize_with_face_selection, "check");
		add_member_control(this, "colorize_with_face_selection_only", data_ptr->point_cloud_kit->colorize_with_face_selection_only, "check");

		add_member_control(this, "force_render_with_original_color", data_ptr->point_cloud_kit->force_render_with_original_color, "check");
		add_member_control(this, "highlight_unmarked_points", data_ptr->point_cloud_kit->highlight_unmarked_points, "check");
		add_member_control(this, "render_nmls", data_ptr->point_cloud_kit->show_nmls, "check");
		add_member_control(this, "colorize_with_scan_index", data_ptr->point_cloud_kit->colorize_with_scan_index, "check");
		add_member_control(this, "hmd_culling", data_ptr->point_cloud_kit->enable_headset_culling, "check");
		add_member_control(this, "compute_normal_after_read", data_ptr->point_cloud_kit->compute_normal_after_read, "check");
		add_member_control(this, "from_CC_txt", data_ptr->point_cloud_kit->pc.from_CC, "check");
		add_member_control(this, "parallel_reading", parallel_reading, "check");
		add_member_control(this, "auto_growing_prepare_thesis", auto_growing_prepare_thesis, "check");
		add_member_control(this, "overwrite_face_id", overwrite_face_id, "check");
		add_member_control(this, "instance_redraw", instance_redraw, "check");
		add_member_control(this, "show_neighbor_graph", data_ptr->point_cloud_kit->show_neighbor_graph, "check");
		add_member_control(this, "knn", data_ptr->point_cloud_kit->k,
			"value_slider", "min=1;max=50;log=false;ticks=true;");
		
		connect_copy(add_control("render_pc", render_pc, "check")->value_change, rebind(
			static_cast<drawable*>(this), &visual_processing::post_redraw));
		add_member_control(this, "surfel point size", data_ptr->point_cloud_kit->surfel_style.point_size,
			"value_slider", "min=0.01;max=5;log=false;ticks=false;");
		connect_copy(add_button("render_with_clod")->click, rebind(this,
			&visual_processing::switch_rendering_mode_clod_based));
		add_member_control(this, "use_octree_sampling", data_ptr->point_cloud_kit->use_octree_sampling, "check");
	}
	//
	if (begin_tree_node("IO", gui_io, true, "level=3")) {
		connect_copy(add_button("read_pc")->click, rebind(this, &visual_processing::start_reading_pc_parallel));
		connect_copy(add_button("[S,YPC]load_point_cloud_and_render_with_clod")->click, rebind(this, &visual_processing::single_hit__load_point_cloud_and_render_with_clod));
		connect_copy(add_button("[B]read_pc_append(obj raw scan)")->click, rebind(this, &visual_processing::read_pc_append));
		add_member_control(this, "Ignore Deleted Points", data_ptr->point_cloud_kit->pc.ignore_deleted_points, "check");
		connect_copy(add_button("save")->click, rebind(this, &visual_processing::start_writting_pc_parallel));
		connect_copy(add_button("quiet_save")->click, rebind(this, &visual_processing::quiet_save));
		connect_copy(add_button("print_pc_information")->click, rebind(this, &visual_processing::print_pc_information));
		connect_copy(add_button("clear_all_pcs")->click, rebind(this, &visual_processing::clear_all_pcs));
		connect_copy(add_button("[Clod,GPC]direct_buffer_loading")->click, rebind(this, &visual_processing::direct_buffer_loading));
		connect_copy(add_button("[Clod]direct_buffer_saving")->click, rebind(this, &visual_processing::direct_buffer_saving));
		add_member_control(this, "model_scale", data_ptr->point_cloud_kit->model_scale, 
			"value_slider", "min=0.001;max=10;log=false;ticks=true;");
		connect_copy(add_button("scale_model")->click, rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::scale_model));

		connect_copy(add_button("apply_transform")->click, rebind(this, &visual_processing::apply_transfrom_to_pc));
		//connect_copy(add_button("apply_transform")->click, rebind(data_ptr->point_cloud_kit->pc, &point_cloud::transform, data_ptr->point_cloud_kit->pc.last_additional_model_matrix));
	}
	//
	if (begin_tree_node("Interactive Region Growing", gui_irg, gui_irg, "level=3")) {
		//
		connect_copy(add_button("reset grow, delete seeds")->click, rebind(data_ptr->point_cloud_kit, 
			&point_cloud_interactable::prepare_grow, true)); 
		connect_copy(add_button("smooth_curvature_and_recolor")->click,
			rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::smooth_curvature_and_recolor));
		add_decorator("// parameters ", "heading", "level=3");
		add_member_control(this, "minimum_searching_neighbor_points", data_ptr->point_cloud_kit->minimum_searching_neighbor_points,
			"value_slider", "min=1;max=50;log=false;ticks=true;");
		add_member_control(this, "growing_latency", data_ptr->point_cloud_kit->growing_latency, // per point? 
			"value_slider", "min=1;max=2000;log=false;ticks=true;");
		add_member_control(this, "normal_threshold", data_ptr->point_cloud_kit->normal_threshold,
			"value_slider", "min=0;max=1;log=false;ticks=true;");
		add_member_control(this, "region_grow_check_normals",
			data_ptr->point_cloud_kit->region_grow_check_normals, "check");
		connect_copy(add_button("debug_region_growing_step_by_step_test")->click,
			rebind(this, &visual_processing::debug_region_growing_step_by_step_test));
		connect_copy(add_button("backward_grow_one_step")->click, rebind(data_ptr->point_cloud_kit,
			&point_cloud_interactable::backward_grow_one_step,selection_kit->curr_face_selecting_id));

		add_decorator("// loading seed ", "heading", "level=3");
		connect_copy(add_button("mark_sample_seed")->click, rebind(this, &visual_processing::mark_sample_seed));
		connect_copy(add_button("load_seeds_with_dialog_without_recover")->click, 
			rebind(this, &visual_processing::load_seeds_with_dialog_without_recover));
		connect_copy(add_button("mark_next_seed")->click, rebind(this, &visual_processing::mark_next_seed));
		connect_copy(add_button("load_sample_seeds_default")->click, rebind(this, &visual_processing::load_sample_seeds_default));
		connect_copy(add_button("save_sample_seeds_default")->click, rebind(this, &visual_processing::save_sample_seeds_default));
		connect_copy(add_button("load_sample_seeds_with_dialog")->click, rebind(this, &visual_processing::load_sample_seeds_with_dialog));
		connect_copy(add_button("save_sample_seeds_with_dialog")->click, rebind(this, &visual_processing::save_sample_seeds_with_dialog));
		connect_copy(add_button("find_next_and_increase_curr_region")->click,
			rebind(this, &visual_processing::find_next_and_increase_curr_region));
		
		add_decorator("// iterative interaction ", "heading", "level=3");
		add_member_control(this, "curr_region", selection_kit->curr_face_selecting_id,
			"value_slider", "min=1;max=50;log=false;ticks=true;");
		connect_copy(add_button("grow_with_dist_and_lowest_curvature")->click,
			rebind(this, &visual_processing::grow_with_dist_and_lowest_curvature)); 
		connect_copy(add_button("grow_with_dist_and_highest_curvature")->click,
				rebind(this, &visual_processing::grow_with_dist_and_highest_curvature));
		connect_copy(add_button("ep_start_grow_auto_curv_direction")->click, // calls the above functions under condition
			rebind(this, &visual_processing::ep_start_grow_auto_curv_direction));
		connect_copy(add_button("grow_with_highest_curvature_only")->click,
			rebind(this, &visual_processing::grow_with_highest_curvature_only));
		connect_copy(add_button("grow_with_accu_dist")->click,
			rebind(this, &visual_processing::grow_with_accu_dist)); 
		connect_copy(add_button("pause_continue_parallel_region_growing")->click,
			rebind(this, &visual_processing::pause_continue_parallel_region_growing));

		add_decorator("// undo functions ", "heading", "level=3");
		connect_copy(add_button("backward_growing")->click, rebind(this, &visual_processing::backward_growing));
		connect_copy(add_button("undo curr region")->click,
			rebind(this, &visual_processing::undo_curr_region));

		add_decorator("// sync grow ", "heading", "level=3");
		connect_copy(add_button("sync_grow_dist_curvature_based")->click,
			rebind(this, &visual_processing::sync_grow_dist_curvature_based));
		connect_copy(add_button("sync_grow_dist_based")->click,
			rebind(this, &visual_processing::sync_grow_dist_based));
		connect_copy(add_button("sync_grow_curv_based")->click,
			rebind(this, &visual_processing::sync_grow_curv_based));
		connect_copy(add_button("undo_sync_grow")->click,
			rebind(this, &visual_processing::undo_sync_grow));
	}
	// 
	if (begin_tree_node("Model Fitting (Connectivity )", gui_Fitting, gui_Fitting, "level=3")) {
		connect_copy(add_button("fitting_render_control_points_test")->click,
			rebind(this, &visual_processing::fitting_render_control_points_test));
		connect_copy(add_button("build_connectivity_graph_fitting_and_render_control_points")->click,
			rebind(this, &visual_processing::build_connectivity_graph_fitting_and_render_control_points));
	}
	//
	if (begin_tree_node("Region Growing", gui_rg, gui_rg, "level=3")) {
		//add_member_control(this, "check_the_queue_and_stop", data_ptr->point_cloud_kit->use_property_scale, "check");
		//add_member_control(this, "use_property_scale", data_ptr->point_cloud_kit->use_property_scale, "check");
		//connect_copy(add_button("clear_seed_for_regions")->click, rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::clear_seed_for_regions));
		//
		/*connect_copy(add_button("prepare_grow_ourmethod")->click,
			rebind(this, &visual_processing::prepare_grow_ourmethod));*/
		// prepare computing, extract neighbour graphs, commpute cuavature, compute knn 
		//connect_copy(add_button("[S,ONCE]prepare_region_grow")->click, rebind(this, &visual_processing::single_hit__prepare_region_grow));
		// reset 
		connect_copy(add_button("reset grow, delete seeds")->click, rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::prepare_grow, true)); // overwrite face ids 
		
		add_decorator("// automatic method ", "heading", "level=3");
		connect_copy(add_button("submit_face")->click, // all faces will be pushed to final queue
			rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::submit_face));
		// an other option is residual growing, lower speed, undo support 
		connect_copy(add_button("resume_queue")->click, // all faces will be pushed to final queue
			rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::resume_queue));
		/*connect_copy(add_button("final_grow_accu_dist")->click,
			rebind(this, &visual_processing::final_grow_accu_dist));*/
		//add_member_control(this, "is_residual_grow", data_ptr->point_cloud_kit->is_residual_grow, "check");
		connect_copy(add_button("residual_grow_curvature_based")->click,
			rebind(this, &visual_processing::residual_grow_curvature_based));
		connect_copy(add_button("residual_grow_dist_and_curvature_based")->click,
			rebind(this, &visual_processing::residual_grow_dist_and_curvature_based));
		connect_copy(add_button("[S]automatic_region_extraction")->click,
			rebind(this, &visual_processing::automatic_region_extraction));

		add_decorator("// seed selection ", "heading", "level=3");
		// seed selection 
		connect_copy(add_button("load_sample_seeds_default")->click, rebind(this, &visual_processing::load_sample_seeds_default));
		connect_copy(add_button("save_sample_seeds_default")->click, rebind(this, &visual_processing::save_sample_seeds_default));
		connect_copy(add_button("load_sample_seeds_with_dialog")->click, rebind(this, &visual_processing::load_sample_seeds_with_dialog));
		connect_copy(add_button("save_sample_seeds_with_dialog")->click, rebind(this, &visual_processing::save_sample_seeds_with_dialog));
		connect_copy(add_button("reset_queue_with_seeds")->click, rebind(data_ptr->point_cloud_kit, 
			&point_cloud_interactable::reset_queue_with_seeds));
		//
		connect_copy(add_button("mark_sample_seed")->click, rebind(this, &visual_processing::mark_sample_seed));
		connect_copy(add_button("mark_next_seed")->click, rebind(this, &visual_processing::mark_next_seed));
		// mark with controller possible 

		add_decorator("// growing variants", "heading", "level=3");

		// region growing variants 
		connect_copy(add_button("[S]grow_with_accu_distance")->click,
			rebind(this, &visual_processing::single_hit__regrow_accu_distance_based));
		connect_copy(add_button("[S]grow_with_seed_distance")->click,
			rebind(this, &visual_processing::single_hit__regrow_seed_distance_based));
		connect_copy(add_button("[S]grow_with_unsigned_mean_curvature")->click,
			rebind(this, &visual_processing::single_hit__regrow_unsigned_mean_curvature_based));
		connect_copy(add_button("[S]grow_with_distance_and_curvature")->click,
			rebind(this, &visual_processing::single_hit__regrow_distance_and_curvature_based));
		add_member_control(this, "check_the_queue_and_stop", data_ptr->point_cloud_kit->check_the_queue_and_stop, "check");
		add_member_control(this, "ignore_high_curvature_regions", data_ptr->point_cloud_kit->ignore_high_curvature_regions, "check");
		add_member_control(this, "minimum_searching_neighbor_points", data_ptr->point_cloud_kit->minimum_searching_neighbor_points,
			"value_slider", "min=1;max=50;log=false;ticks=true;");
		connect_copy(add_button("[S]grow_stop_on_boundaries")->click,
			rebind(this, &visual_processing::single_hit__regrow_stop_at_high_curvature));
		connect_copy(add_button("mark_sample_seed")->click, rebind(this, &visual_processing::mark_sample_seed));
		
		add_decorator("// debug the growing process", "heading", "level=3");
		add_member_control(this, "use_property_scale", data_ptr->point_cloud_kit->use_property_scale, "check");
		add_member_control(this, "knn", data_ptr->point_cloud_kit->k,
			"value_slider", "min=1;max=50;log=false;ticks=true;");
		add_member_control(this, "growing_latency", data_ptr->point_cloud_kit->growing_latency, // per point? 
			"value_slider", "min=1;max=500;log=false;ticks=true;");
		connect_copy(add_button("pause/continue")->click, rebind(this, &visual_processing::pause_continue_parallel_region_growing));
		connect_copy(add_button("start")->click, rebind(this, &visual_processing::start_parallel_region_growing));
		connect_copy(add_button("terminate")->click, rebind(this, &visual_processing::stop_parallel_region_growing));

		add_decorator("// curvature computing ", "heading", "level=3");
		connect_copy(add_button("smooth_curvature_and_recolor")->click,
			rebind(data_ptr->point_cloud_kit, &point_cloud_interactable::smooth_curvature_and_recolor));
		connect_copy(add_button("signed: compute_principal_curvature_and_colorize")->click,
			rebind(this, &visual_processing::ep_compute_principal_curvature_and_colorize_signed));
		connect_copy(add_button("unsigned: compute_principal_curvature_and_colorize")->click,
			rebind(this, &visual_processing::ep_compute_principal_curvature_and_colorize_unsigned));
		add_member_control(this, "coloring_threshold", data_ptr->point_cloud_kit->pc.curvinfo.coloring_threshold,
			"value_slider", "min=0.01;max=100;log=false;ticks=true;");
		connect_copy(add_button("ep_force_recolor")->click,
			rebind(this, &visual_processing::ep_force_recolor));
		connect_copy(add_button("compute_feature_points_and_colorize")->click,
			rebind(this, &visual_processing::compute_feature_points_and_colorize));
		/*connect_copy(add_button("convert_uint_to_int_face_selection_representation")->click,
			rebind(this, &visual_processing::convert_to_int_face_selection_representation));*/

	}
	//
	if (begin_tree_node("Effecient Point Cloud Rendering And Marking", gui_pc_cleaning, false, "level=3")) {
		connect_copy(add_button("[B,SingleFile]load and convert, poission lod")->click, rebind(this,
			&visual_processing::batch__load_and_convert_add_lod_single_file));
		connect_copy(add_button("[B]load and convert, poission lod")->click, rebind(this,
			&visual_processing::batch__load_and_convert_add_lod_info));
		connect_copy(add_button("compute_lods")->click, rebind(this,
			&visual_processing::compute_lods));
		connect_copy(add_button("download_points_from_gpu_to_memory")->click, rebind(this,
			&visual_processing::download_points_from_gpu_to_memory));
		add_member_control(this, "enable_marking", data_ptr->point_cloud_kit->cp_renderer.enable_marking, "check");
		connect_copy(add_button("reset_marking")->click, rebind(this, &visual_processing::reset_marking));
		add_member_control(this, "clod_scale", data_ptr->point_cloud_kit->clod_render_scale, "value_slider", "min=0;max=1;log=false;ticks=true;");
		add_member_control(this, "test_shader_selection", data_ptr->point_cloud_kit->test_shader_selection, "check");
		add_member_control(this, "color_based_on_lod", data_ptr->point_cloud_kit->color_based_on_lod, "check");
		connect_copy(add_button("on_rendering_settings_changed")->click, rebind(this,
			&visual_processing::on_rendering_settings_changed));
		connect_copy(add_button("find_pointcloud")->click, rebind(this, &visual_processing::find_pointcloud));
		
	}
	//
	if (begin_tree_node("Point Cloud Rendering Style", gui_pc_rendering_style, false, "level=3")) {
		/*add_member_control(this, "RENDERING_STRATEGY", data_ptr->point_cloud_kit->RENDERING_STRATEGY,
			"value_slider", "min=0;max=4;log=false;ticks=true;");*/
			//stich_rendering_mode_point_based 
		connect_copy(add_button("render_with_points")->click, rebind(this,
			&visual_processing::switch_rendering_mode_point_based));
		connect_copy(add_button("render_with_quads")->click, rebind(this,
			&visual_processing::switch_rendering_mode_quad_based));
		connect_copy(add_button("render_with_surfel")->click, rebind(this,
			&visual_processing::switch_rendering_mode_surfel_based));
		connect_copy(add_button("render_with_clod")->click, rebind(this,
			&visual_processing::switch_rendering_mode_clod_based));
	}
	//
	if (begin_tree_node("Point Scale", gui_point_Scale, false, "level=3")) {
		connect_copy(add_button("scale_points_to_disk")->click,
			rebind(this, &visual_processing::scale_points_to_desk));
		connect_copy(add_button("drop_other_info_points_only")->click,
			rebind(this, &visual_processing::drop_other_info_points_only)); // for cleaning point properties 
		//
	}
	//
	if (begin_tree_node("VR ICP", gui_icp , false, "level=3")) {
		//
		connect_copy(add_button("randomize_current_pc")->click, rebind(this, &visual_processing::randomize_current_pc));
		//
		connect_copy(add_button("update_scan_index_visibility_test")->click,
			rebind(this, &visual_processing::update_scan_index_visibility_test));
		connect_copy(add_button("point_visibility_vis_all_points")->click,
			rebind(this, &visual_processing::point_visibility_vis_all_points));
		connect_copy(add_button("set_src_and_target_scan_idx_as_test")->click,
			rebind(this, &visual_processing::set_src_and_target_scan_idx_as_test));
		connect_copy(add_button("extract_point_clouds_for_icp_test")->click,
			rebind(this, &visual_processing::extract_point_clouds_for_icp));
		//
		connect_copy(add_button("perform_icp_and_acquire_matrices")->click,
			rebind(this, &visual_processing::perform_icp_and_acquire_matrices));
		connect_copy(add_button("apply_register_matrices_for_the_original_point_cloud")->click,
			rebind(this, &visual_processing::apply_register_matrices_for_the_original_point_cloud));
		//
		add_member_control(this, "source_scan_index", data_ptr->point_cloud_kit->src_scan_idx,
			"value_slider", "min=0;max=10;log=false;ticks=true;");
		add_member_control(this, "target_scan_index", data_ptr->point_cloud_kit->target_scan_idx,
			"value_slider", "min=0;max=10;log=false;ticks=true;");
		//
		add_member_control(this, "icp_iterations", data_ptr->point_cloud_kit->icp_iterations,
			"value_slider", "min=1;max=20;log=false;ticks=true;");
		add_member_control(this, "icp_iterations", data_ptr->point_cloud_kit->icp_samples,
			"value_slider", "min=10;max=100;log=false;ticks=true;");
		connect_copy(add_button("do_icp_once")->click,
			rebind(this, &visual_processing::do_icp_once));
	}
	//
	if (begin_tree_node("Triangulation of The Points", gui_Triangulation, false, "level=3")) {
		connect_copy(add_button("triangulation_of_the_points")->click, rebind(this, &visual_processing::triangulation_of_the_points));
		connect_copy(add_button("export_to_an_obj_file")->click, rebind(this, &visual_processing::export_to_an_obj_file));
	}
	//  
	if (begin_tree_node("Connectivity Graph", gui_Connectivity, false, "level=3")) {
		connect_copy(add_button("boundary_extraction")->click, rebind(this, &visual_processing::boundary_extraction));
		connect_copy(add_button("topology_extraction")->click, rebind(this, &visual_processing::extract_connectivity_graph));
	}
	//
	if (begin_tree_node("Scan Index", gui_scanindex, false, "level=3")) {
		add_member_control(this, "scan_index", data_ptr->point_cloud_kit->pc.currentScanIdx_Recon, 
			"value_slider", "min=0;max=10;log=false;ticks=true;");
		add_member_control(this, "surfel point size", data_ptr->point_cloud_kit->surfel_style.point_size,
			"value_slider", "min=0.01;max=5;log=false;ticks=false;");
		add_member_control(this, "point size", data_ptr->point_cloud_kit->point_size,
			"value_slider", "min=0.01;max=5;log=false;ticks=false;");
		add_member_control(this, "percentual_halo_width", data_ptr->point_cloud_kit->percentual_halo_width,
			"value_slider", "min=0.05;max=5;log=false;ticks=false;");
		add_member_control(this, "colorize_with_scan_index", data_ptr->point_cloud_kit->colorize_with_scan_index, "check");
		add_member_control(this, "renderScan0", data_ptr->point_cloud_kit->renderScan0, "check");
		add_member_control(this, "renderScan1", data_ptr->point_cloud_kit->renderScan1, "check");
		add_member_control(this, "renderScan2", data_ptr->point_cloud_kit->renderScan2, "check");
		add_member_control(this, "renderScan3", data_ptr->point_cloud_kit->renderScan3, "check");
		add_member_control(this, "renderScan4", data_ptr->point_cloud_kit->renderScan4, "check");
		add_member_control(this, "renderScan5", data_ptr->point_cloud_kit->renderScan5, "check");
	}
	// point copy 
	if (begin_tree_node("Copy Points", gui_copy_points, false, "level=3")) {
		add_member_control(this, "render_handhold_pc", data_ptr->render_handhold_pc, "check");
		connect_copy(add_button("point_copy_btn_pressed")->click, rebind(this,
			&visual_processing::point_copy_btn_pressed));
		connect_copy(add_button("point_copy_btn_release")->click, rebind(this,
			&visual_processing::point_copy_btn_release));
		connect_copy(add_button("release_controller_pc_binding")->click, rebind(this,
			&visual_processing::release_controller_pc_binding));
		add_member_control(this, "use_current_matrix", data_ptr->point_cloud_in_hand->use_current_matrix, "check");
	}
	// tube 
	if (begin_tree_node("Tube Rendering", gui_Tube, false, "level=3")) {
		add_member_control(this, "render_an_animating_tube", data_ptr->render_an_animating_tube, "check");
		connect_copy(add_button("enlarge_tube_length")->click, rebind(this,
			&visual_processing::enlarge_tube_length));
		connect_copy(add_button("schrink_tube_length")->click, rebind(this,
			&visual_processing::schrink_tube_length));
	}
	// 
	if (begin_tree_node("point addition", gui_addition, false, "level=3")) {
		add_member_control(this, "render_the_quad", data_ptr->render_a_quad_on_righthand, "check");
		connect_copy(add_button("quad_addition_menu_btn_press")->click, rebind(this,
			&visual_processing::quad_addition_menu_btn_press));
		connect_copy(add_button("quad_addition_menu_btn_release")->click, rebind(this,
			&visual_processing::quad_addition_menu_btn_release));

		//
		connect_copy(add_button("step_back_selection")->click, rebind(this,
			&visual_processing::step_back_selection));
		connect_copy(add_button("step_forward_selection")->click, rebind(this,
			&visual_processing::step_forward_selection));

		//
		connect_copy(add_button("mark_all_active_points_as_tobedownsampled")->click, rebind(this,
			&visual_processing::mark_all_active_points_as_tobedownsampled));
		connect_copy(add_button("selective_downsampling_menu_btn_press")->click, rebind(this,
			&visual_processing::selective_downsampling_menu_btn_press));
		connect_copy(add_button("selective_downsampling_menu_btn_release")->click, rebind(this,
			&visual_processing::selective_downsampling_menu_btn_release));
	}
	//
	if (begin_tree_node("Point Cloud Generation", gui_Generation, false, "level=3")) {
		connect_copy(add_button("generate_pc_hemisphere")->click, 
			rebind(this, &visual_processing::generate_pc_hemisphere));
		connect_copy(add_button("generate_pc_cube")->click,
			rebind(this, &visual_processing::generate_pc_cube));
		connect_copy(add_button("generate_testing_plane")->click,
			rebind(this, &visual_processing::generate_testing_plane));
		connect_copy(add_button("generate_pc_init_sphere")->click,
			rebind(this, &visual_processing::generate_pc_init_sphere));
		connect_copy(add_button("generate_pc_random_sphere")->click,
			rebind(this, &visual_processing::generate_pc_random_sphere));
		connect_copy(add_button("generate_pc_unit_sylinder")->click,
			rebind(this, &visual_processing::generate_pc_unit_sylinder));
		connect_copy(add_button("generate_pc_unit_torus")->click,
			rebind(this, &visual_processing::generate_pc_unit_torus));
		//
		//
		connect_copy(add_button("send_updated_point_cloud_to_gpu")->click,
			rebind(this, &visual_processing::send_updated_point_cloud_to_gpu));
		// 
		connect_copy(add_button("force_nml_computing")->click, 
			rebind(this, &visual_processing::force_nml_computing));
		connect_copy(add_button("flip normals")->click,
			rebind(this, &visual_processing::toggle_normal_orientations));
		connect_copy(add_button("save")->click,
			rebind(this, &visual_processing::start_writting_pc_parallel));
	}
	//
	if (begin_tree_node("Point Cloud ControlLOD", gui_ControlLOD, false, "level=3")) {
		connect_copy(add_button("render_with_fullpc")->click, rebind(this, &visual_processing::render_with_fullpc));
		connect_copy(add_button("auto_downsampling")->click, rebind(this, &visual_processing::auto_downsampling));
		connect_copy(add_button("supersampling_with_bbox")->click, rebind(this, &visual_processing::supersampling_with_bbox));
		connect_copy(add_button("restore_supersampling")->click, rebind(this, &visual_processing::restore_supersampling));
		connect_copy(add_button("prepare_grow")->click, rebind(this, &visual_processing::prepare_grow));
	}
	//
	if (begin_tree_node("Point Cloud Merging Tool", gui_Merging, false, "level=3")) {
		connect_copy(add_button("read_pc_append")->click, rebind(this, &visual_processing::read_pc_append));
		add_member_control(this, "[0]step", step, "value_slider","min=1;max=1000;log=false;ticks=true;");
		add_member_control(this, "[1]num_of_points_wanted", num_of_points_wanted, "value_slider", "min=1;max=100000000;log=false;ticks=true;");
		add_member_control(this, "which_strategy", strategy, "value_slider", "min=0;max=1;log=false;ticks=true;");
		connect_copy(add_button("downsampling")->click, rebind(this, &visual_processing::downsampling));
		add_member_control(this, "write_reflectance", data_ptr->point_cloud_kit->pc.write_reflectance, "check");
		connect_copy(add_button("save")->click,
			rebind(this, &visual_processing::start_writting_pc_parallel));
	}
	//
	if (begin_tree_node("Point Cloud Nml Computing", gui_Nml_computing, false, "level=3")) {
		//connect_copy(add_button("add_to_file_list")->click, rebind(this, &visual_processing::add_to_file_list));
		//connect_copy(add_button("clean_file_list")->click, rebind(this, &visual_processing::clean_file_list));
		connect_copy(add_button("batch_compute_nmls_given_file_list")->click, rebind(this, &visual_processing::batch_compute_nmls_given_file_list));
		connect_copy(add_button("read_pc_queue")->click, rebind(this, &visual_processing::read_pc_queue));
		connect_copy(add_button("read_campose")->click, rebind(this, &visual_processing::read_campose));
		connect_copy(add_button("show_camposes")->click, rebind(this, &visual_processing::show_camposes));
		connect_copy(add_button("auto_conduct_nml_estimation_leica")->click, rebind(this, &visual_processing::auto_conduct_nml_estimation_leica));
		connect_copy(add_button("clean_all_pcs")->click, rebind(this, &visual_processing::clear_all_pcs));
		connect_copy(add_button("rotate_x")->click, rebind(this, &visual_processing::rotate_x));
		connect_copy(add_button("rotate_z")->click, rebind(this, &visual_processing::rotate_z));
		connect_copy(add_button("load_next_shot")->click, rebind(this, &visual_processing::load_next_shot));
		connect_copy(add_button("compute_nmls_intermediate_pc")->click, rebind(this, &visual_processing::compute_nmls_if_is_required));
		connect_copy(add_button("append_current_shot_to_stored_cloud")->click, rebind(this, &visual_processing::append_current_shot_to_stored_cloud));
		connect_copy(add_button("align_leica_scans_with_cgv")->click, rebind(this, &visual_processing::align_leica_scans_with_cgv));
		connect_copy(add_button("write_stored_pc_to_file")->click, rebind(this, &visual_processing::write_stored_pc_to_file));
		connect_copy(add_control("force_correct_num_pcs", force_correct_num_pcs, "check")->value_change, rebind(static_cast<drawable*>(this), &visual_processing::post_redraw));
		connect_copy(add_control("direct_write", direct_write, "check")->value_change, rebind(static_cast<drawable*>(this), &visual_processing::post_redraw));
	}
	//
	if (begin_tree_node("Point Cloud Semantic Tool", gui_Semantic, false, "level=3")) {
		connect_copy(add_button("read_pc")->click, rebind(this, &visual_processing::read_pc));
		connect_copy(add_button("read_pc_queue")->click, rebind(this, &visual_processing::read_pc_queue));
		//connect_copy(add_button("compute_feature_points")->click, rebind(this, &visual_processing::compute_feature_points));

	}
	//
	if (begin_tree_node("Mesh Tool (partial)", gui_Mesh, false, "level=3")) {
		//compute_coordinates_with_rot_correction
		connect_copy(add_button("compute_coordinates_with_rot_correction")->click, rebind(this, &visual_processing::compute_coordinates_with_rot_correction));
	}
	//
	if(teleportation_kit!=nullptr)
	if (begin_tree_node("Teleportation tool", gui_Teleportation, false, "level=3")) {
		add_member_control(this, "is_lifting", teleportation_kit->is_lifting, "check");
		add_member_control(this, "enable_gravity", teleportation_kit->enable_gravity, "check");
	}
	//
	if(roller_coaster_kit_1!=nullptr)
	if (begin_tree_node("Roller Coaster 1", gui_Roller_Coaster, false, "level=3")) {
		add_member_control(roller_coaster_kit_1, "para_x_0", roller_coaster_kit_1->para_x_0, "value_slider", "min=1;max=100;log=false;ticks=false;");
		add_member_control(roller_coaster_kit_1, "para_z_0", roller_coaster_kit_1->para_z_0, "value_slider", "min=1;max=100;log=false;ticks=false;");
		add_member_control(roller_coaster_kit_1, "para_x", roller_coaster_kit_1->para_x, "value_slider", "min=1;max=100;log=false;ticks=false;");
		add_member_control(roller_coaster_kit_1, "para_y", roller_coaster_kit_1->para_y, "value_slider", "min=1;max=100;log=false;ticks=false;");
		add_member_control(roller_coaster_kit_1, "para_z", roller_coaster_kit_1->para_z, "value_slider", "min=1;max=100;log=false;ticks=false;");
		add_member_control(roller_coaster_kit_1, "speed_factor", roller_coaster_kit_1->speed_factor, "value_slider", "min=1;max=30;log=false;ticks=false;");
		add_member_control(roller_coaster_kit_1, "resolution", roller_coaster_kit_1->resolution, "value_slider", "min=800;max=2000;log=false;ticks=false;");
	}
	//
	if (draw_kit != nullptr)
	if (begin_tree_node("Draw kit", gui_Draw, false, "level=3")) {
		connect_copy(add_button("write_trajectory")->click, rebind(this, &visual_processing::write_trajectory));
		connect_copy(add_button("read_trajectory")->click, rebind(this, &visual_processing::read_trajectory));
		connect_copy(add_button("clear_drawing")->click, rebind(this, &visual_processing::clear_drawing));
		add_member_control(this, "start_drawing", draw_kit->enable_drawing, "check");
		add_member_control(this, "render_enable_drawing", draw_kit->render_enable_drawing, "check");
	}
	//
	bool show_motioncap_kit = motioncap_kit != nullptr;
	if(show_motioncap_kit)
	if (begin_tree_node("Mocap kit", gui_mocap, false, "level=3")) {
		connect_copy(add_button("save_to_tj_file")->click, rebind(this, &visual_processing::save_to_tj_file));
		connect_copy(add_button("read_tj_file")->click, rebind(this, &visual_processing::read_tj_file));
		add_member_control(this, "start_rec", data_ptr->rec_pose, "check");
		//add_member_control(this, "replay", motioncap_kit->replay, "check");
		//
		add_member_control(this, "instanced_redraw", motioncap_kit->instanced_redraw, "check");
		connect_copy(add_button("start_replay_all")->click, rebind(this, &visual_processing::start_replay_all));
		connect_copy(add_button("stop_and_clear_mocap_data")->click, rebind(this, &visual_processing::stop_and_clear_mocap_data));
		add_member_control(this, "is_replay", data_ptr->is_replay, "check");

		//for (int i = 0; i < 44; i++) {
		//	data_ptr->tj_rendering_ignore.push_back(false);
		//}

		//
		/*for(int i = 0;i< data_ptr->tj_rendering_ignore.size();i++)
			add_member_control(this, "ignore#"+to_string(i), data_ptr->tj_rendering_ignore.at(i), "check");*/

		/*add_member_control(this, "ignore#0", data_ptr->tj_rendering_ignore.at(0), "check");
		add_member_control(this, "ignore#1", data_ptr->tj_rendering_ignore.at(1), "check");
		add_member_control(this, "ignore#2", data_ptr->tj_rendering_ignore.at(2), "check");
		add_member_control(this, "ignore#3", data_ptr->tj_rendering_ignore.at(3), "check");
		add_member_control(this, "ignore#4", data_ptr->tj_rendering_ignore.at(4), "check");*/

		//
		/*for (int i = 0; i < data_ptr->tj_rendering_ignore.size(); ++i) 
			add_member_control(this, "ignore", data_ptr->tj_rendering_ignore[i], "check");*/

		//add_member_control(this, "ignore#1", data_ptr->tj_rendering_ignore_0, "check");

		add_member_control(this, "frame_factor", data_ptr->frame_factor, "value_slider", "min=1;max=43;log=false;ticks=true;");
	}
	//
	if (begin_tree_node("Selection kit", gui_selection, false, "level=3")) {
		add_member_control(this, "pick_point_index", pick_point_index, "value_slider", "min=0;max=5;log=false;ticks=true;");
	}
}
///
#include <cgv/base/register.h>
cgv::base::object_registration<visual_processing> vr_pc_processing_reg("visual_processing");
