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
#include <cgv/gui/mouse_event.h>
#include <cgv/render/context.h>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>
#include "vr_kit_intersection.h"
#include "vis_kit_datastore.h"
#include "vis_visual_processing.h"

class vis_kit_selection :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	stereo_view_interactor* view_ptr = nullptr;
	cgv::render::context* ctx_ptr = nullptr;

	int pick_point_index = -1;
	cgv::render::sphere_render_style sphere_style;
	bool in_picking = false;

	bool picking_box_min = true;
	bool picking = true;
	bool record_controller_behavier_draw_circle = false;

	// point marking 
	sphere_render_style marking_style;

	// group picker 
	sphere_render_style palette_style;

	//
	sphere_render_style shading_effect_style;

	// animating 
	bool render_an_animating_tube_configured = false;
	std::vector<vec3> P;
	std::vector<float> R;
	std::vector<rgb> C;
	cgv::render::rounded_cone_render_style rounded_cone_style;
	vec3 timeline_left_end = vec3(-0.1, 0.05, -0.02);
	vec3 timeline_right_end = vec3(0.1, 0.05, -0.02);

	//
	vec3 updated_left_end = vec3(0);
	vec3 updated_right_end = vec3(0);

public:
	/*public accessble varibles */
	int current_selecting_idx = -1;
	/// initialize rotation angle
	vis_kit_selection()
	{
		connect(get_animation_trigger().shoot, this, &vis_kit_selection::timer_event); 
		/*sphere_style.map_color_to_material = CM_COLOR_AND_OPACITY;
		sphere_style.surface_color = rgb(0.8f, 0.4f, 0.4f);
		sphere_style.radius = 0.1;*/
		//sphere_style.radius = float(0.05*sqrt(B.get_extent().sqr_length() / vertex_count));

		marking_style.radius = 0.02;
		marking_style.material.set_transparency(0.4);

		palette_style.radius = 0.02;
		//palette_style.material.set_transparency(1);

		//
		shading_effect_style.material.set_transparency(0.4);
		
	}
	// call me 
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
	}
	void set_view_ptr(stereo_view_interactor* vp) {
		view_ptr = vp;
	}
	void set_context_str(cgv::render::context* cptr) {
		ctx_ptr = cptr;
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
		return "vis_kit_selection";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}

	bool on_pick(const cgv::gui::mouse_event & me)
	{
		if (!view_ptr)
			return false;
		if (!data_ptr)
			return false;
		if (ctx_ptr == nullptr)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location_ctx(*ctx_ptr, me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z) ||
			window_z > 0.999)
			return false;
		// check whether to pick a previously defined point
		pick_point_index = -1;
		double pick_dist = 0;
		double pick_dist_threshold = sphere_style.radius * sphere_style.radius_scale;
		for (int i = 0; i < (int)data_ptr->pick_points.size(); ++i) {
			double dist = (data_ptr->pick_points[i] - vec3(pick_point)).length();
			if (dist < pick_dist_threshold) {
				if (pick_point_index == -1 || dist < pick_dist) {
					pick_dist = dist;
					pick_point_index = i;
				}
			}
		}
		if (pick_point_index == -1) {
			pick_point_index = data_ptr->pick_points.size();
			data_ptr->pick_points.push_back(pick_point);
			data_ptr->pick_colors.push_back(rgb(0, 1, 0));
			return true;
		}
		return false;
		
	}
	
	bool on_drag(const cgv::gui::mouse_event& me)
	{
		if (!view_ptr)
			return false;
		if (!data_ptr)
			return false;
		if (pick_point_index == -1)
			return false;
		dvec3 pick_point;
		double window_z;
		if (!get_world_location(me.get_x(), me.get_y(), *view_ptr, pick_point, &window_z) ||
			window_z > 0.999) {
			data_ptr->pick_colors[pick_point_index] = rgb(0, 1, 0);
			pick_point_index = -1;
			post_redraw();
			return false;
		}
		data_ptr->pick_points[pick_point_index] = pick_point;
		post_redraw();
		return true;
	}

	/// overload to handle events, return true if event was processed
	/// call me
	bool handle(event& e)
	{
		/*if (e.get_kind() == EID_MOUSE) {
			auto& me = static_cast<cgv::gui::mouse_event&>(e);
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
					break;
				case MA_DRAG:
					if (in_picking) {
						on_drag(me);
						return true;
					}
					break;
			}
		}*/
		if (data_ptr == nullptr)
			return false;
		if (e.get_kind() == cgv::gui::EID_STICK) {
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			if (vrse.get_action() == cgv::gui::SA_MOVE) { // event 
				if (vrse.get_controller_index() == data_ptr->right_rgbd_controller_index) { // controller 
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nGroupPicker"))) { // selection 
						if (vrse.get_y() > 0) {
							marking_style.radius += 0.001f;
						}
						else {
							marking_style.radius -= 0.001f;
						}
					}
					// shading effects 
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) { // selection 
						if (vrse.get_y() > 0) {
							data_ptr->point_cloud_kit->controller_effect_range += 0.001f;
						}
						else {
							data_ptr->point_cloud_kit->controller_effect_range -= 0.001f;
						}
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nSeleciton"))) { // selection 
						if (vrse.get_y() > 0) {
							data_ptr->point_cloud_kit->controller_effect_range += 0.001f;
						}
						else {
							data_ptr->point_cloud_kit->controller_effect_range -= 0.001f;
						}
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCShading\nLinearMelting"))) { // selection 
						if (vrse.get_y() > 0) {
							data_ptr->point_cloud_kit->controller_effect_range += 0.001f;
						}
						else {
							data_ptr->point_cloud_kit->controller_effect_range -= 0.001f;
						}
					}
				}
			}
		}

		if (e.get_kind() == cgv::gui::EID_KEY) {
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			if (vrke.get_key() == vr::VR_MENU) { //
				if (vrke.get_action() == cgv::gui::KA_PRESS) { //
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { //
						if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nPrepare\nMarking"))) { //
							// this will overwrite face segmentations 
							data_ptr->point_cloud_kit->prepare_grow(false,
								&data_ptr->point_selection_colors, 
									data_ptr->point_cloud_kit->pc.max_num_of_selections);
						}
					}
				}
			}
			//if (vrke.get_key() == vr::VR_DPAD_DOWN) { //
			//	if (vrke.get_action() == cgv::gui::KA_PRESS) { //
			//		if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { //
			//			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nMarking"))) { //
			//				marking_style.radius -= 0.001;
			//			}
			//		}
			//	}
			//	return true;
			//}
			//if (vrke.get_key() == vr::VR_DPAD_UP) { //
			//	if (vrke.get_action() == cgv::gui::KA_PRESS) { //
			//		if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { //
			//			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nMarking"))) { //
			//				marking_style.radius += 0.001;
			//			}
			//		}
			//	}
			//	return true;
			//}
		}

		if (e.get_kind() == cgv::gui::EID_THROTTLE) {
			auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
			float v = te.get_value();
			bool d = (v == 1); // event 
			if (te.get_controller_index() == data_ptr->right_rgbd_controller_index) {
				if(data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nSuperSampling"))){ // trigger to compute/start 
					if (d) {
						record_controller_behavier_draw_circle = !record_controller_behavier_draw_circle;
						// if is going to end 
						if (record_controller_behavier_draw_circle == false) {
							data_ptr->supersampling_bounded_points_with_drawn_data();
							data_ptr->righthand_posi_list.clear();
							data_ptr->righthand_dir_list.clear();
						}
						else {
							std::cout << "recording enabled!" << std::endl;
						}
					}
				}
				//if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nMarking"))) { // hold throttle to mark
				//	if (v > 0 && current_selecting_idx!=-1) {
				//		data_ptr->point_cloud_kit->mark_points_with_conroller(
				//			data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
				//				marking_style.radius, true, current_selecting_idx);
				//	}
				//}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nGroupPicker"))) { // hold throttle to mark
					if (v > 0 && current_selecting_idx != -1) {
						if (data_ptr->point_cloud_kit->can_parallel_grow == true) { 
							data_ptr->point_cloud_kit->can_parallel_grow = false;
							data_ptr->point_cloud_kit->mark_points_with_conroller(
								data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
									marking_style.radius, 
										(cgv::type::uint8_type)current_selecting_idx);
							if (data_ptr->point_cloud_kit->add_to_seed) {
								// collect seeds
								data_ptr->point_cloud_kit->init_region_growing_by_collecting_group_and_seeds_vr(
									(cgv::type::uint8_type)current_selecting_idx);
								// then, grow on timer event, sleep optionally 
							}
							data_ptr->point_cloud_kit->can_parallel_grow = true;
						}
					}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nDelPoints\nTouchTo\nActivate"))) { // hold throttle to mark
					if (v > 0 && current_selecting_idx != -1) {
						if (data_ptr->point_cloud_kit->can_parallel_grow == true) {
							data_ptr->point_cloud_kit->can_parallel_grow = false;
							data_ptr->point_cloud_kit->mark_points_with_conroller(
								data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
									marking_style.radius,
										(cgv::type::uint8_type)current_selecting_idx);
							data_ptr->point_cloud_kit->can_parallel_grow = true;
						}
					}
				}
				if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nMarkAs\nOrig"))) { // hold throttle to mark
					if (v > 0 && current_selecting_idx != -1) {
						if (data_ptr->point_cloud_kit->can_parallel_grow == true) {
							data_ptr->point_cloud_kit->can_parallel_grow = false;
							data_ptr->point_cloud_kit->mark_points_with_conroller(
								data_ptr->cur_right_hand_posi + data_ptr->cur_off_right,
									marking_style.radius, 
										(cgv::type::uint8_type)current_selecting_idx);
							data_ptr->point_cloud_kit->can_parallel_grow = true;
						}
					}
				}
				// allow more btns to paint here 
			}
		}
		/// old version, old events. todo: rewrite this part with freq. test 
		switch (e.get_kind()) {
			case cgv::gui::EID_KEY:
			{
				cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
				int ci = vrke.get_controller_index();
				if (ci == data_ptr->right_rgbd_controller_index
					&& vrke.get_key() == vr::VR_DPAD_DOWN
					&& data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nSuperSampling")))
				{
					if (vrke.get_action() == cgv::gui::KA_PRESS)
					{
						////clear bbox 
						//data_ptr->supersampling_bbox.invalidate();
						//picking = false;
						//record_controller_behavier_draw_circle = !record_controller_behavier_draw_circle;
						//// if is going to end 
						//if (record_controller_behavier_draw_circle) {
						//	data_ptr->compute_bounded_points_with_drawn_data();
						//	data_ptr->righthand_posi_list.clear();
						//	data_ptr->righthand_dir_list.clear();
						//}
						//std::cout << "righthanddown pressed!" << std::endl;
					}
				}
				return true;
			}
			// THROTTLE to start the selection 
			//case cgv::gui::EID_THROTTLE:
			//{
			//	auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
			//	int ci = te.get_controller_index();
			//	float v = te.get_value(); 
			//	// e-c-s checking order 
			//	bool d = (v == 1); // event 
			//	if (ci == data_ptr->right_rgbd_controller_index)  // controller 
			//		
			//	}
			//	return true;
			//}
			case cgv::gui::EID_STICK:
			{
				cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
				int ci = vrse.get_controller_index();
				vec3 origin, direction;
				if (ci != -1)
					switch (vrse.get_action()) {
					case cgv::gui::SA_TOUCH:
						//std::cout << "righthand touch!" << std::endl;
						if (ci == data_ptr->right_rgbd_controller_index 
							&& data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nSuperSampling"))
								&& record_controller_behavier_draw_circle) {
							vrse.get_state().controller[ci].put_ray(&origin(0), &direction(0));
							data_ptr->righthand_posi_list.push_back(origin);
							data_ptr->righthand_dir_list.push_back(direction); 
						}
						// quick test 
						//if (ci == data_ptr->left_rgbd_controller_index) {
						//	record_controller_behavier_draw_circle = !record_controller_behavier_draw_circle;
						//	// if is going to end 
						//	if (record_controller_behavier_draw_circle) {
						//		data_ptr->compute_bounded_points_with_drawn_data();
						//		data_ptr->righthand_posi_list.clear();
						//		data_ptr->righthand_dir_list.clear();
						//	}
						//}
						break;
					case cgv::gui::SA_RELEASE:
						break;
					case cgv::gui::SA_PRESS:
						break;
					}
				return true;
			}
			case cgv::gui::EID_POSE:
			{
				cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
				// check for controller pose events
				int ci = vrpe.get_trackable_index();
				vec3 origin, direction;
				// right hand event 
				if (ci == data_ptr->right_rgbd_controller_index) {
					//if (picking) {
					//	//... bbox not so good for this task
					//}

					// too many points to compute 
					/*if (record_controller_behavier_draw_circle) {
						vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
						data_ptr->righthand_posi_list.push_back(origin);
						data_ptr->righthand_dir_list.push_back(direction);
					}*/
				}
				if (ci != -1) {
					// check and update current point group selection -> which color are we using 
					int smallestpalleteIdx = -1;
					bool hasOverlaping = false;
					float dist = std::numeric_limits<float>::max();
					for (int i = 0; i < data_ptr->palette_lefthand_object_positions.size();i++) {
						// just take the first one, righthand_object_positions has only one element
						float cur_dist = (data_ptr->palette_lefthand_object_positions[i] 
							- data_ptr->righthand_object_positions[0]).length();
						if (cur_dist < dist) {
							dist = cur_dist;
							smallestpalleteIdx = i;
						}
					}
					// update color with the one has minimal dist to the RC 
					// check the position on rigth hand, if it is close enough to palette balls, we change the current index 
					// the ball's size on right hand doesnt matter 
					if (dist < palette_style.radius) {
						hasOverlaping = true;
						data_ptr->palette_righthand_object_colors[0] = 
							data_ptr->palette_lefthand_object_colors[smallestpalleteIdx];
						// ok-todo: update picking group information 
						// current, we can only select regions, build on top of functional indices 
						current_selecting_idx = smallestpalleteIdx; //data_ptr->point_cloud_kit->pc.num_of_functional_selections 
					}

					data_ptr->point_cloud_kit->headset_direction = data_ptr->headset_direction;
					data_ptr->point_cloud_kit->headset_position = data_ptr->headset_object_positions[0];
					data_ptr->point_cloud_kit->right_controller_position = data_ptr->righthand_object_positions[0];
					data_ptr->point_cloud_kit->left_controller_position = data_ptr->lefthand_object_positions[0];
					// do not update range 
					//std::cout << "headset posi:" << data_ptr->point_cloud_kit->headset_position << std::endl;
				}

			}
			}
		return false;
	}
	float angle = 0;
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
		if (data_ptr->render_an_animating_tube) {
			angle+=data_ptr->speed;
			if (angle > 360) {
				angle = 0;
			}

			// a temp var. to be rotated 
			vec3 timeline_left_end_copy = data_ptr->tube_left_end + vec3(0,-0.05,0.02);
			vec3 timeline_right_end_copy = data_ptr->tube_right_end + vec3(0, -0.05, 0.02);

			// 
			quat combinedQ = quat();
			bool combinedComputation = false;

			// controller rotation first 
			if (combinedComputation) {
				combinedQ = combinedQ * data_ptr->cur_right_hand_rot_quat;
			}
			else {
				data_ptr->cur_right_hand_rot_quat.rotate(timeline_left_end_copy);
				data_ptr->cur_right_hand_rot_quat.rotate(timeline_right_end_copy);
			}

			// update positions, anim rotation 
			vec3 current_axis = 
				data_ptr->get_global_from_local_righthand(vec3(0, 0, 1)) - data_ptr->get_global_from_local_righthand(vec3(0, 0, 0));
			quat q = quat(current_axis,angle);

			if (combinedComputation) {
				combinedQ = combinedQ * q;
			}
			else {
				q.rotate(timeline_left_end_copy);
				q.rotate(timeline_right_end_copy);
			}

			/*combinedQ.rotate(timeline_left_end_copy);
			combinedQ.rotate(timeline_right_end_copy);*/

			// apply trasformation offset 
			vec3 transOffset = vec3(0,0.1,-0.2);

			// do not rotate them, just assign 
			updated_left_end = data_ptr->get_global_from_local_righthand(timeline_left_end_copy + transOffset);
			updated_right_end = data_ptr->get_global_from_local_righthand(timeline_right_end_copy + transOffset);
		}
	}
	/// setting the view transform yourself
	/// call me 
	void draw(context& ctx)
	{
		if (data_ptr == nullptr)
			return;
		for (int i = 0; i < data_ptr->righthand_posi_list.size(); i++) {
			//render_a_box_given_posi_and_size(ctx, data_ptr->righthand_posi_list.at(i),0.02);
			render_an_arrow_with_starting_point_and_ending(ctx, data_ptr->righthand_posi_list.at(i)
				, data_ptr->righthand_posi_list.at(i) + data_ptr->righthand_dir_list.at(i) * 0.2, rgb(0, 0.4, 0), 0.05);
		}
		// supersampling mode 
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nSuperSampling")))
		{
			if (record_controller_behavier_draw_circle) {
				vec3 startingdir = vec3(0, 0, -2);
				data_ptr->cur_right_hand_rot_quat.rotate(startingdir);
				vec3 endposi = data_ptr->cur_right_hand_posi + startingdir;

				render_a_handhold_arrow(ctx, rgb(0, 0.4, 0), 0.05f);
				render_line_cone_style(ctx, data_ptr->cur_right_hand_posi, endposi, rgb(1,0,0), 0.002f);
			}
			else
				render_a_handhold_box(ctx);
		}		
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nMarkAs\nOrig"))) {
			render_a_sphere_on_righthand(ctx);
			//data_ptr->righthand_object_colors[0] = rgb(1,0,0,1);
			data_ptr->righthand_object_colors[0] = data_ptr->point_selection_colors[1];
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PointCloud\nDelPoints\nTouchTo\nActivate"))) {
			render_a_sphere_on_righthand(ctx);
			data_ptr->righthand_object_colors[0] = data_ptr->point_selection_colors[2];
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("RegionGrowing\nGroupPicker"))) {
			render_palette_on_left_hand(ctx);
			render_palette_sphere_on_righthand(ctx);
		}

		// shading effects 
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nSeleciton"))) {
			render_a_sphere_on_righthand_shading_effect(ctx);
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCShading\nLinearMelting"))) {
			render_a_sphere_on_righthand_shading_effect(ctx);
		}

		// point cloud cleaning
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nClipping"))) {
			render_a_handhold_plane_for_clipping(ctx);
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nFake\nDel"))) {
			render_a_sphere_on_righthand_shading_effect(ctx);
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nSelective\nSubSampling"))) {
			render_a_sphere_on_righthand_shading_effect(ctx);
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nAddition\nQuad"))) {
			render_a_quad_on_righthand_vertical(ctx);
		}
		if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("PCCleaning\nCopyPoints"))) {
			render_a_sphere_on_righthand_shading_effect(ctx);
		}

		// animating 
		if(data_ptr->render_an_animating_tube)
			render_an_animating_tube_impl(ctx);

		// quick test 
		if(data_ptr->render_a_quad_on_righthand)
			render_a_quad_on_righthand_vertical(ctx);

	}
	/// call me 
	void finish_draw(context& ctx) {

	}
	///
	void render_a_handhold_plane_for_clipping(cgv::render::context& ctx) {
		vec3 ext = vec3(0, 2, 4);
		// surface renderer with texture 
		if (true) {
			// points for a label
			vec3 p1(0, 0.5 * ext.y(), 0.5 * ext.z());
			vec3 p2(0, -0.5 * ext.y(), 0.5 * ext.z());
			vec3 p3(0, 0.5 * ext.y(), -0.5 * ext.z());
			vec3 p4(0, -0.5 * ext.y(), -0.5 * ext.z());

			vec3 addi_offset = vec3(0, 0, -ext.z() / 2);

			p1 = p1 + addi_offset;
			p2 = p2 + addi_offset;
			p3 = p3 + addi_offset;
			p4 = p4 + addi_offset;

			/*quat tmp(vec3(0, 1, 0), var1);
			tmp.rotate(p1);
			tmp.rotate(p2);
			tmp.rotate(p3);
			tmp.rotate(p4);*/

			// rotate and translate according to the gui boxes
			data_ptr->cur_right_hand_rot_quat.rotate(p1);
			data_ptr->cur_right_hand_rot_quat.rotate(p2);
			data_ptr->cur_right_hand_rot_quat.rotate(p3);
			data_ptr->cur_right_hand_rot_quat.rotate(p4);

			p1 = p1 + data_ptr->cur_right_hand_posi;
			p2 = p2 + data_ptr->cur_right_hand_posi;
			p3 = p3 + data_ptr->cur_right_hand_posi;
			p4 = p4 + data_ptr->cur_right_hand_posi;

			cgv::render::shader_program& prog = ctx.ref_default_shader_program(false);
			int pi = prog.get_position_index();
			//int ti = prog.get_texcoord_index();
			std::vector<vec3> P;

			P.push_back(p1);
			P.push_back(p2);
			P.push_back(p3);
			P.push_back(p4);

			cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
			cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
			prog.enable(ctx);
			ctx.set_color(rgb(0.5));
			glDrawArrays(GL_TRIANGLE_STRIP, 0, (GLsizei)P.size());
			prog.disable(ctx);
			cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		}
	}
	///
	void render_an_animating_tube_impl(cgv::render::context& ctx){
		if (!render_an_animating_tube_configured) {
			P.push_back(timeline_left_end);
			P.push_back(timeline_right_end);
			R.push_back(0.002);
			R.push_back(0.002);
			C.push_back(0.6);
			C.push_back(0.6);
			render_an_animating_tube_configured = true;
		}
		// update positions, local to global at run time 
		if (!data_ptr->render_an_animating_tube) {
			P[0] = data_ptr->get_global_from_local_righthand(timeline_left_end);
			P[1] = data_ptr->get_global_from_local_righthand(timeline_right_end);
		}
		else {
			P[0] = updated_left_end;
			P[1] = updated_right_end;
		}
		//
		auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
		cr.set_render_style(rounded_cone_style);
		//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
		cr.set_position_array(ctx, P);
		cr.set_color_array(ctx, C);
		cr.set_radius_array(ctx, R);
		cr.render(ctx, 0, P.size());
	}
	///
	void render_palette_on_left_hand(cgv::render::context& ctx) {
		if (data_ptr->palette_lefthand_object_positions.size()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(palette_style);
			sr.set_position_array(ctx, data_ptr->palette_lefthand_object_positions);
			sr.set_color_array(ctx, data_ptr->palette_lefthand_object_colors);
			sr.render(ctx, 0, data_ptr->palette_lefthand_object_positions.size());
		}
	}
	///
	void render_palette_sphere_on_righthand(cgv::render::context& ctx) {
		if (data_ptr->righthand_object_positions.size()) {
			glDepthMask(GL_FALSE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_CULL_FACE);
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(marking_style);
			sr.set_position_array(ctx, data_ptr->righthand_object_positions);
			sr.set_color_array(ctx, data_ptr->palette_righthand_object_colors);
			sr.render(ctx, 0, data_ptr->righthand_object_positions.size());
			glDepthMask(GL_TRUE);
		}
	}
	
	/*shading effect*/ 
	void render_a_sphere_on_righthand_shading_effect(cgv::render::context& ctx) {
		if (data_ptr->righthand_object_positions.size()) {
			shading_effect_style.radius = data_ptr->point_cloud_kit->controller_effect_range;
			rgb oricol = data_ptr->righthand_object_colors[0];
			data_ptr->righthand_object_colors[0] = rgb(0.6,0.6,0.6);
			glDepthMask(GL_FALSE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_CULL_FACE);
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(shading_effect_style);
			sr.set_position_array(ctx, data_ptr->righthand_object_positions);
			sr.set_color_array(ctx, data_ptr->righthand_object_colors);
			sr.render(ctx, 0, data_ptr->righthand_object_positions.size());
			glDepthMask(GL_TRUE);
			data_ptr->righthand_object_colors[0] = oricol;
		}
	}
	///
	void render_a_point_cloud_on_right_hand_controller() {

	}
	
	/*editing tool */
	void render_a_quad_on_righthand_vertical(cgv::render::context& ctx) {
		//data_ptr->righthand_object_positions[0]

		vec3 quad_ext = data_ptr->quad_addition_ext;

		vec3 p1(0.5 * quad_ext.x(), 0.5 * quad_ext.y(), 0);
		vec3 p2(-0.5 * quad_ext.x(), 0.5 * quad_ext.y(), 0);
		vec3 p3(0.5 * quad_ext.x(), -0.5 * quad_ext.y(), 0);
		vec3 p4(-0.5 * quad_ext.x(), -0.5 * quad_ext.y(), 0);

		data_ptr->cur_right_hand_rot_quat.rotate(p1);
		data_ptr->cur_right_hand_rot_quat.rotate(p2);
		data_ptr->cur_right_hand_rot_quat.rotate(p3);
		data_ptr->cur_right_hand_rot_quat.rotate(p4);

		p1 = p1 + data_ptr->righthand_object_positions[0];
		p2 = p2 + data_ptr->righthand_object_positions[0];
		p3 = p3 + data_ptr->righthand_object_positions[0];
		p4 = p4 + data_ptr->righthand_object_positions[0];

		std::vector<vec3> P;
		std::vector<vec2> T;

		P.push_back(p1); //T.push_back(vec2(1.0f, 1.0f));
		P.push_back(p2); //T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p3); //T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p2); //T.push_back(vec2(0.0f, 1.0f));
		P.push_back(p3); //T.push_back(vec2(1.0f, 0.0f));
		P.push_back(p4); //T.push_back(vec2(0.0f, 0.0f));

		cgv::render::shader_program& prog = ctx.ref_default_shader_program();
		int pi = prog.get_position_index();
		//int ti = prog.get_texcoord_index();

		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
		cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
		/*cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ti, T);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ti);*/
		glDisable(GL_CULL_FACE);
		prog.enable(ctx);
		ctx.set_color(rgb(0,0.4,0));
		glDrawArrays(GL_TRIANGLES, 0, (GLsizei)P.size());
		prog.disable(ctx);
		glEnable(GL_CULL_FACE);
		cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
		//cgv::render::attribute_array_binding::disable_global_array(ctx, ti);
	}

	/**/
	///
	void render_a_sphere_on_righthand(cgv::render::context& ctx) {
		if (data_ptr->righthand_object_positions.size()) {
			glDepthMask(GL_FALSE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_CULL_FACE);
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(marking_style);
			sr.set_position_array(ctx, data_ptr->righthand_object_positions);
			sr.set_color_array(ctx, data_ptr->righthand_object_colors);
			sr.render(ctx, 0, data_ptr->righthand_object_positions.size());
			glDepthMask(GL_TRUE);
		}
	}
	///
	void render_line_cone_style(cgv::render::context& ctx, vec3 s,vec3 e, rgb c, float r = 0.002f) {
		cgv::render::rounded_cone_render_style rounded_cone_style;

		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgb> C;

		P.push_back(s);
		R.push_back(r);
		P.push_back(e);
		R.push_back(r + 0.001f);
		C.push_back(c);
		C.push_back(c);

		if (P.size() > 0) {
			auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
			cr.set_render_style(rounded_cone_style);
			//cr.set_eye_position(vr_view_ptr->get_eye_of_kit());
			cr.set_position_array(ctx, P);
			cr.set_color_array(ctx, C);
			cr.set_radius_array(ctx, R);
			if (!cr.render(ctx, 0, P.size())) {
				cgv::render::shader_program& prog = ctx.ref_default_shader_program();
				int pi = prog.get_position_index();
				int ci = prog.get_color_index();
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, pi, P);
				cgv::render::attribute_array_binding::enable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, C);
				cgv::render::attribute_array_binding::enable_global_array(ctx, ci);
				glLineWidth(3);
				prog.enable(ctx);
				glDrawArrays(GL_LINES, 0, (GLsizei)P.size());
				prog.disable(ctx);
				cgv::render::attribute_array_binding::disable_global_array(ctx, pi);
				cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
				glLineWidth(1);
			}
		}

	}
	///
	void render_a_box_given_posi_and_size(cgv::render::context& ctx,vec3 posi,float size) {
		auto& prog = ctx.ref_surface_shader_program();
		prog.set_uniform(ctx, "map_color_to_material", 3);
		prog.enable(ctx);
		ctx.set_color(rgb(0.4));
		ctx.tesselate_box(box3(posi - vec3(size / 2.0f), posi + vec3(size / 2.0f)), false, false);
		prog.disable(ctx);
	}
	///
	void render_an_arrow_with_starting_point_and_ending(cgv::render::context& ctx, vec3 s, vec3 e, rgb c, float r) {
		auto& prog = ctx.ref_surface_shader_program();
		prog.set_uniform(ctx, "map_color_to_material", 3);
		prog.enable(ctx);
		ctx.set_color(c);
		ctx.tesselate_arrow(s, e, r, 2.0, 0.5f);
		prog.disable(ctx);

	}
	///
	void render_a_handhold_box(cgv::render::context& ctx) {
		vec3 startingdir = vec3(0, 0, -0.1);
		data_ptr->cur_right_hand_rot_quat.rotate(startingdir);
		vec3 endposi = data_ptr->cur_right_hand_posi + startingdir;

		auto& prog = ctx.ref_surface_shader_program();
		prog.set_uniform(ctx, "map_color_to_material", 3);
		prog.enable(ctx);
		ctx.set_color(rgb(0.4));
		ctx.tesselate_box(box3(endposi - vec3(0.01), endposi + vec3(0.01)), false, false);
		prog.disable(ctx);
	}
	///
	void render_a_handhold_arrow(cgv::render::context& ctx, rgb c, float r, float l = 2.0) {
		if (data_ptr == nullptr)
			return;
		vec3 startingdir = vec3(0, 0, -0.2);
		data_ptr->cur_right_hand_rot_quat.rotate(startingdir);
		vec3 endposi = data_ptr->cur_right_hand_posi + startingdir;

		auto& prog = ctx.ref_surface_shader_program();
		prog.set_uniform(ctx, "map_color_to_material", 3);
		prog.enable(ctx);
		ctx.set_color(c);
		ctx.tesselate_arrow(data_ptr->cur_right_hand_posi, endposi, r, l, 0.5f);
		prog.disable(ctx);
	}
	/// will be moved to render_kit header -> this header will be used for many kits 
	/// todo: tesselate_box_cone_style
	void render_a_bbox(cgv::render::context& ctx, box3 b) {
		if (b.is_valid()) {
			auto& prog = ctx.ref_surface_shader_program();
			prog.set_uniform(ctx, "map_color_to_material", 3);
			prog.enable(ctx);
			ctx.set_color(rgb(0,0.4,0));
			ctx.tesselate_box(b, false, true);
			prog.disable(ctx);
		}
	}
	///
	void finish_frame(context& ctx) {
		/*if (!view_ptr)
			return;
		if (!data_ptr)
			return;

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		sphere_renderer& sr = ref_sphere_renderer(ctx);
		if (view_ptr)
			sr.set_y_view_angle(float(view_ptr->get_y_view_angle()));
		sr.set_render_style(sphere_style );

		glDepthMask(GL_FALSE);
		if (!data_ptr->pick_points.empty()) {
			sr.set_position_array(ctx, data_ptr->pick_points);
			sr.set_color_array(ctx, data_ptr->pick_colors);
			sr.validate_and_enable(ctx);
			glDrawArrays(GL_POINTS, 0, (GLsizei)data_ptr->pick_points.size());
			sr.disable(ctx);
		}
		if (pick_point_index != -1) {
			glDisable(GL_BLEND);
			glDisable(GL_DEPTH_TEST);
			vec3 p = data_ptr->pick_points[pick_point_index];
			if (view_ptr)
				p += 1.5f * sphere_style.radius * sphere_style.radius_scale * vec3(view_ptr->get_view_up_dir());
			std::stringstream ss;
			ss << "[" << p << "]";
			ss.flush();

			ctx.set_color(rgb(0.1f, 0.1f, 0.1f));
			ctx.set_cursor(p.to_vec(), ss.str(), (TextAlignment)TA_BOTTOM, 0, 0);
			ctx.output_stream() << ss.str();
			ctx.output_stream().flush();

			ctx.set_color(rgb(0.9f, 0.9f, 0.9f));
			ctx.set_cursor(p.to_vec(), ss.str(), (TextAlignment)TA_BOTTOM, 1, -1);
			ctx.output_stream() << ss.str();
			ctx.output_stream().flush();

			glEnable(GL_DEPTH_TEST);
			glEnable(GL_BLEND);
		}
		glDepthMask(GL_TRUE);
		glDisable(GL_BLEND);*/
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
