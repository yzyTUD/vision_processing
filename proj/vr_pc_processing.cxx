#include "vr_pc_processing.h"

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

#include "intersection.h"


void vr_pc_processing::init_cameras(vr::vr_kit* kit_ptr)
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

void vr_pc_processing::start_camera()
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

void vr_pc_processing::stop_camera()
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
void vr_pc_processing::compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
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
void vr_pc_processing::on_status_change(void* kit_handle, int ci, vr::VRStatus old_status, vr::VRStatus new_status)
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
void vr_pc_processing::on_device_change(void* kit_handle, bool attach)
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

/// construct boxes that represent a table of dimensions tw,td,th and leg width tW
void vr_pc_processing::construct_table(float tw, float td, float th, float tW) {
	// construct table
	rgb table_clr(0.3f, 0.2f, 0.0f);
	boxes.push_back(box3(
		vec3(-0.5f*tw, th - tW, -0.5f*td),
		vec3(0.5f*tw, th, 0.5f*td)));
	box_colors.push_back(table_clr);

	boxes.push_back(box3(vec3(-0.5f*tw + 2*tW, 0, -0.5f*td+2*tW), vec3(-0.5f*tw+tW, th - tW, -0.5f*td + tW)));
	boxes.push_back(box3(vec3(-0.5f*tw + 2*tW, 0, 0.5f*td-2*tW), vec3(-0.5f*tw + tW, th - tW, 0.5f*td - tW)));
	boxes.push_back(box3(vec3(0.5f*tw-2*tW, 0, -0.5f*td+tW), vec3(0.5f*tw - tW, th - tW, -0.5f*td +2* tW)));
	boxes.push_back(box3(vec3(0.5f*tw-2*tW, 0, 0.5f*td-2*tW), vec3(0.5f*tw - tW, th - tW, 0.5f*td - tW)));
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
	box_colors.push_back(table_clr);
}

/// construct boxes that represent a room of dimensions w,d,h and wall width W
void vr_pc_processing::construct_room(float w, float d, float h, float W, bool walls, bool ceiling) {
	// construct floor
	boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d), vec3(0.5f*w, 0, 0.5f*d)));
	box_colors.push_back(rgb(0.2f, 0.2f, 0.2f));

	if(walls) {
		// construct walls
		boxes.push_back(box3(vec3(-0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w, h, -0.5f*d)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));
		boxes.push_back(box3(vec3(-0.5f*w, -W, 0.5f*d), vec3(0.5f*w, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.8f, 0.5f, 0.5f));

		boxes.push_back(box3(vec3(0.5f*w, -W, -0.5f*d - W), vec3(0.5f*w + W, h, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.8f, 0.5f));
	}
	if(ceiling) {
		// construct ceiling
		boxes.push_back(box3(vec3(-0.5f*w - W, h, -0.5f*d - W), vec3(0.5f*w + W, h + W, 0.5f*d + W)));
		box_colors.push_back(rgb(0.5f, 0.5f, 0.8f));
	}
}

/// construct boxes for environment
void vr_pc_processing::construct_environment(float s, float ew, float ed, float w, float d, float h) {
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	unsigned n = unsigned(ew / s);
	unsigned m = unsigned(ed / s);
	float ox = 0.5f*float(n)*s;
	float oz = 0.5f*float(m)*s;
	for(unsigned i = 0; i < n; ++i) {
		float x = i * s - ox;
		for(unsigned j = 0; j < m; ++j) {
			float z = j * s - oz;
			if(fabsf(x) < 0.5f*w && fabsf(x + s) < 0.5f*w && fabsf(z) < 0.5f*d && fabsf(z + s) < 0.5f*d)
				continue;
			float h = 0.2f*(std::max(abs(x) - 0.5f*w, 0.0f) + std::max(abs(z) - 0.5f*d, 0.0f))*distribution(generator) + 0.1f;
			boxes.push_back(box3(vec3(x, 0.0f, z), vec3(x + s, h, z + s)));
			rgb color = cgv::media::color<float, cgv::media::HLS>(distribution(generator), 0.1f*distribution(generator) + 0.15f, 0.3f);
			box_colors.push_back(color);
			/*box_colors.push_back(
				rgb(0.3f*distribution(generator) + 0.3f,
					0.3f*distribution(generator) + 0.2f,
					0.2f*distribution(generator) + 0.1f));*/
		}
	}
}

/// construct boxes that can be moved around
void vr_pc_processing::construct_movable_boxes(float tw, float td, float th, float tW, size_t nr) {
	/*
	vec3 extent(0.75f, 0.5f, 0.05f);
	movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
	movable_box_colors.push_back(rgb(0, 0, 0));
	movable_box_translations.push_back(vec3(0, 1.2f, 0));
	movable_box_rotations.push_back(quat(1, 0, 0, 0));
	*/
	std::default_random_engine generator;
	std::uniform_real_distribution<float> distribution(0, 1);
	std::uniform_real_distribution<float> signed_distribution(-1, 1);
	for(size_t i = 0; i < nr; ++i) {
		float x = distribution(generator);
		float y = distribution(generator);
		vec3 extent(distribution(generator), distribution(generator), distribution(generator));
		extent += 0.01f;
		extent *= std::min(tw, td)*0.1f;

		vec3 center(-0.5f*tw + x * tw, th + tW, -0.5f*td + y * td);
		movable_boxes.push_back(box3(-0.5f*extent, 0.5f*extent));
		movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
		movable_box_translations.push_back(center);
		quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
		rot.normalize();
		movable_box_rotations.push_back(rot);
	}
}

/// construct a scene with a table
void vr_pc_processing::build_scene(float w, float d, float h, float W, float tw, float td, float th, float tW)
{
	construct_room(w, d, h, W, false, false);
	construct_table(tw, td, th, tW);
	construct_environment(0.3f, 3 * w, 3 * d, w, d, h);
	//construct_environment(0.4f, 0.5f, 1u, w, d, h);
	construct_movable_boxes(tw, td, th, tW, 50);
}

vr_pc_processing::vr_pc_processing() 
{
	frame_split = 0;
	extent_texcrd = vec2(0.5f, 0.5f);
	center_left  = vec2(0.5f,0.25f);
	center_right = vec2(0.5f,0.25f);
	seethrough_gamma = 0.33f;
	frame_width = frame_height = 0;
	background_distance = 2;
	background_extent = 2;
	undistorted = true;
	shared_texture = true;
	max_rectangle = false;
	nr_cameras = 0;
	camera_tex_id = -1;
	camera_aspect = 1;
	use_matrix = true;
	show_seethrough = false;
	set_name("vr_pc_processing");
	//build_scene(5, 7, 3, 0.2f, 0.8f, 0.8f, 0.72f, 0.03f);
	vr_view_ptr = 0;
	ray_length = 2;
	last_kit_handle = 0;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &vr_pc_processing::on_device_change);
	connect(cgv::gui::ref_vr_server().on_status_change, this, &vr_pc_processing::on_status_change);

	mesh_scale = 0.0005f;
	mesh_location = dvec3(0, 0.85f, 0);
	mesh_orientation = dquat(1, 0, 0, 0);

	srs.radius = 0.005f;

	label_outofdate = true;
	label_text = "Info Board";
	label_font_idx = 0;
	label_upright = true;
	label_face_type = cgv::media::font::FFA_BOLD;
	label_resolution = 256;
	label_size = 20.0f;
	label_color = rgb(1, 1, 1);

	cgv::media::font::enumerate_font_names(font_names);
	font_enum_decl = "enums='";
	for (unsigned i = 0; i < font_names.size(); ++i) {
		if (i>0)
			font_enum_decl += ";";
		std::string fn(font_names[i]);
		if (cgv::utils::to_lower(fn) == "calibri") {
			label_font_face = cgv::media::font::find_font(fn)->get_font_face(label_face_type);
			label_font_idx = i;
		}
		font_enum_decl += std::string(fn);
	}
	font_enum_decl += "'";
	state[0] = state[1] = state[2] = state[3] = IS_NONE;
}
	
void vr_pc_processing::stream_help(std::ostream& os) {
	os << "vr_pc_processing: no shortcuts defined" << std::endl;
}
	
void vr_pc_processing::on_set(void* member_ptr)
{
	if (member_ptr == &label_face_type || member_ptr == &label_font_idx) {
		label_font_face = cgv::media::font::find_font(font_names[label_font_idx])->get_font_face(label_face_type);
		label_outofdate = true;
	}
	if ((member_ptr >= &label_color && member_ptr < &label_color + 1) ||
		member_ptr == &label_size || member_ptr == &label_text) {
		label_outofdate = true;
	}

	vr::vr_kit* kit_ptr = vr::get_vr_kit(last_kit_handle);
	if (kit_ptr) {
		for (int ii = 0; ii < (int)left_inp_cfg.size(); ++ii)
			if (member_ptr >= &left_inp_cfg[ii] && member_ptr < &left_inp_cfg[ii] + 1)
				kit_ptr->set_controller_input_config(0, ii, left_inp_cfg[ii]);
	}
	update_member(member_ptr);
	post_redraw();
}
	
bool vr_pc_processing::handle(cgv::gui::event& e)
{
	b_interactable->handle(e);

	// check if vr event flag is not set and don't process events in this case
	if ((e.get_flags() & cgv::gui::EF_VR) == 0)
		return false;
	// check event id
	switch (e.get_kind()) {
	case cgv::gui::EID_KEY:
	{
		cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
		if (vrke.get_action() != cgv::gui::KA_RELEASE) {
			switch (vrke.get_key()) {
			case vr::VR_GRIP:
				std::cout << "grip button " << (vrke.get_controller_index() == 0 ? "left":"right") << " controller pressed" << std::endl;
				return true;
			case vr::VR_DPAD_RIGHT:
				std::cout << "touch pad of " << (vrke.get_controller_index() == 0 ? "left" : "right") << " controller pressed at right direction" << std::endl;
				return true;
			}
		}
		break;
	}
	case cgv::gui::EID_THROTTLE:
	{
		cgv::gui::vr_throttle_event& vrte = static_cast<cgv::gui::vr_throttle_event&>(e);
		std::cout << "throttle " << vrte.get_throttle_index() << " of controller " << vrte.get_controller_index()
			<< " adjusted from " << vrte.get_last_value() << " to " << vrte.get_value() << std::endl;
		return true;
	}
	case cgv::gui::EID_STICK:
	{
		cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
		switch (vrse.get_action()) {
		case cgv::gui::SA_TOUCH:
			if (state[vrse.get_controller_index()] == IS_OVER)
				state[vrse.get_controller_index()] = IS_GRAB;
			break;
		case cgv::gui::SA_RELEASE:
			if (state[vrse.get_controller_index()] == IS_GRAB)
				state[vrse.get_controller_index()] = IS_OVER;
			break;
		case cgv::gui::SA_PRESS:
		case cgv::gui::SA_UNPRESS:
			std::cout << "stick " << vrse.get_stick_index()
				<< " of controller " << vrse.get_controller_index()
				<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
				<< " at " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
			return true;
		case cgv::gui::SA_MOVE:
		case cgv::gui::SA_DRAG:
			return true;
			std::cout << "stick " << vrse.get_stick_index()
				<< " of controller " << vrse.get_controller_index()
				<< " " << cgv::gui::get_stick_action_string(vrse.get_action())
				<< " from " << vrse.get_last_x() << ", " << vrse.get_last_y()
				<< " to " << vrse.get_x() << ", " << vrse.get_y() << std::endl;
			return true;
		}
		return true;
	}
	case cgv::gui::EID_POSE:
		cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
		// check for controller pose events
		int ci = vrpe.get_trackable_index();
		if (ci != -1) {
			if (state[ci] == IS_GRAB) {
				// in grab mode apply relative transformation to grabbed boxes

				// get previous and current controller position
				vec3 last_pos = vrpe.get_last_position();
				vec3 pos = vrpe.get_position();
				// get rotation from previous to current orientation
				// this is the current orientation matrix times the
				// inverse (or transpose) of last orientation matrix:
				// vrpe.get_orientation()*transpose(vrpe.get_last_orientation())
				mat3 rotation = vrpe.get_rotation_matrix();
				// iterate intersection points of current controller
				for (size_t i = 0; i < intersection_points.size(); ++i) {
					if (intersection_controller_indices[i] != ci)
						continue;
					// extract box index
					unsigned bi = intersection_box_indices[i];
					// update translation with position change and rotation
					movable_box_translations[bi] = 
						rotation * (movable_box_translations[bi] - last_pos) + pos;
					// update orientation with rotation, note that quaternions
					// need to be multiplied in oposite order. In case of matrices
					// one would write box_orientation_matrix *= rotation
					movable_box_rotations[bi] = quat(rotation) * movable_box_rotations[bi];
					// update intersection points
					intersection_points[i] = rotation * (intersection_points[i] - last_pos) + pos;
				}
			}
			else {// not grab
				// clear intersections of current controller 
				size_t i = 0;
				while (i < intersection_points.size()) {
					if (intersection_controller_indices[i] == ci) {
						intersection_points.erase(intersection_points.begin() + i);
						intersection_colors.erase(intersection_colors.begin() + i);
						intersection_box_indices.erase(intersection_box_indices.begin() + i);
						intersection_controller_indices.erase(intersection_controller_indices.begin() + i);
					}
					else
						++i;
				}

				// compute intersections
				vec3 origin, direction;
				vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
				compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));
				label_outofdate = true;


				// update state based on whether we have found at least 
				// one intersection with controller ray
				if (intersection_points.size() == i)
					state[ci] = IS_NONE;
				else
					if (state[ci] == IS_NONE)
						state[ci] = IS_OVER;
			}
			post_redraw();
		}
		return true;
	}
	return false;
}

bool vr_pc_processing::init(cgv::render::context& ctx)
{
	if (!cgv::utils::has_option("NO_OPENVR"))
		ctx.set_gamma(1.0f);

	if (!seethrough.build_program(ctx, "seethrough.glpr"))
		cgv::gui::message("could not build seethrough program");
	
	cgv::media::mesh::simple_mesh<> M;
//#ifdef 1
	if (M.read("D:/data/surface/meshes/obj/Max-Planck_lowres.obj")) {
//#else
//	if (M.read("D:/data/surface/meshes/obj/Max-Planck_highres.obj")) {
//#endif
		MI.construct(ctx, M);
		MI.bind(ctx, ctx.ref_surface_shader_program(true), true);
	}

	cgv::gui::connect_vr_server(true);

	auto view_ptr = find_view_as_node();
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

	// config point_cloud_kit
	point_cloud_kit->surfel_style.point_size = 0.2f;
	point_cloud_kit->surfel_style.halo_color_strength = 0.0f;
	point_cloud_kit->surfel_style.percentual_halo_width = 25.0f;
	point_cloud_kit->surfel_style.blend_points = true;
	point_cloud_kit->surfel_style.blend_width_in_pixel = 1.0f;
	point_cloud_kit->show_neighbor_graph = false;
	point_cloud_kit->show_box = false;
	point_cloud_kit->do_auto_view = false;
	point_cloud_kit->pc.create_colors();

	one_shot_360pc->surfel_style.point_size = 0.2f;
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
	stored_cloud->pc.create_colors();

	point_cloud_kit->init(ctx);
	one_shot_360pc->init(ctx);
	stored_cloud->init(ctx);

	return true;
}

void vr_pc_processing::clear(cgv::render::context& ctx)
{
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
	point_cloud_kit->clear(ctx);
	one_shot_360pc->clear(ctx);
	stored_cloud->clear(ctx);
}

void vr_pc_processing::init_frame(cgv::render::context& ctx)
{
	b_interactable->init_frame(ctx);
	point_cloud_kit->init_frame(ctx);
	one_shot_360pc->init_frame(ctx);
	stored_cloud->init_frame(ctx);

	if (label_fbo.get_width() != label_resolution) {
		label_tex.destruct(ctx);
		label_fbo.destruct(ctx);
	}
	if (!label_fbo.is_created()) {
		label_tex.create(ctx, cgv::render::TT_2D, label_resolution, label_resolution);
		label_fbo.create(ctx, label_resolution, label_resolution);
		label_tex.set_min_filter(cgv::render::TF_LINEAR_MIPMAP_LINEAR);
		label_tex.set_mag_filter(cgv::render::TF_LINEAR);
		label_fbo.attach(ctx, label_tex);
		label_outofdate = true;
	}
	if (label_outofdate && label_fbo.is_complete(ctx)) {
		glPushAttrib(GL_COLOR_BUFFER_BIT);
		label_fbo.enable(ctx);
		label_fbo.push_viewport(ctx);
		ctx.push_pixel_coords();
			glClearColor(0.5f,0.5f,0.5f,1.0f);
			glClear(GL_COLOR_BUFFER_BIT);

			glColor4f(label_color[0], label_color[1], label_color[2], 1);
			ctx.set_cursor(20, (int)ceil(label_size) + 20);
			ctx.enable_font_face(label_font_face, label_size);
			ctx.output_stream() << label_text << "\n";
			ctx.output_stream().flush(); // make sure to flush the stream before change of font size or font face

			ctx.enable_font_face(label_font_face, 0.7f*label_size);
			for (size_t i = 0; i < intersection_points.size(); ++i) {
				ctx.output_stream()
					<< "box " << intersection_box_indices[i]
					<< " at (" << intersection_points[i]
					<< ") with controller " << intersection_controller_indices[i] << "\n";
			}
			ctx.output_stream().flush();

		ctx.pop_pixel_coords();
		label_fbo.pop_viewport(ctx);
		label_fbo.disable(ctx);
		glPopAttrib();
		label_outofdate = false;

		label_tex.generate_mipmaps(ctx);
	}
	if (vr_view_ptr && vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_eye() == 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
		vr::vr_kit* kit_ptr = vr_view_ptr->get_current_vr_kit();
		if (kit_ptr) {
			vr::vr_camera* camera_ptr = kit_ptr->get_camera();
			if (camera_ptr && camera_ptr->get_state() == vr::CS_STARTED) {
				uint32_t width = frame_width, height = frame_height, split = frame_split;
				if (shared_texture) {
					box2 tex_range;
					if (camera_ptr->get_gl_texture_id(camera_tex_id, width, height, undistorted, &tex_range.ref_min_pnt()(0))) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
					}
					else
						camera_tex_id = -1;
				}
				else {
					std::vector<uint8_t> frame_data;
					if (camera_ptr->get_frame(frame_data, width, height, undistorted, max_rectangle)) {
						camera_aspect = (float)width / height;
						split = camera_ptr->get_frame_split();
						switch (split) {
						case vr::CFS_VERTICAL:
							camera_aspect *= 2;
							break;
						case vr::CFS_HORIZONTAL:
							camera_aspect *= 0.5f;
							break;
						}
						cgv::data::data_format df(width, height, cgv::type::info::TI_UINT8, cgv::data::CF_RGBA);
						cgv::data::data_view dv(&df, frame_data.data());
						if (camera_tex.is_created()) {
							if (camera_tex.get_width() != width || camera_tex.get_height() != height)
								camera_tex.destruct(ctx);
							else
								camera_tex.replace(ctx, 0, 0, dv);
						}
						if (!camera_tex.is_created())
							camera_tex.create(ctx, dv);
					}
					else if (camera_ptr->has_error())
						cgv::gui::message(camera_ptr->get_last_error());
				}
				if (frame_width != width || frame_height != height) {
					frame_width = width;
					frame_height = height;

					center_left(0) = camera_centers[2](0) / frame_width;
					center_left(1) = camera_centers[2](1) / frame_height;
					center_right(0) = camera_centers[3](0) / frame_width;
					center_right(1) = camera_centers[3](1) / frame_height;

					update_member(&frame_width);
					update_member(&frame_height);
					update_member(&center_left(0));
					update_member(&center_left(1));
					update_member(&center_right(0));
					update_member(&center_right(1));
				}
				if (split != frame_split) {
					frame_split = split;
					update_member(&frame_split);
				}
			}
		}
	}
}

void vr_pc_processing::draw(cgv::render::context& ctx)
{
	b_interactable->draw(ctx);
	if(render_pc)
		point_cloud_kit->draw(ctx);
	//one_shot_360pc->draw(ctx);
	//stored_cloud->draw(ctx);

	if (vr_view_ptr) {
		if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
			if (vr_view_ptr->get_rendered_vr_kit() != 0 && vr_view_ptr->get_rendered_vr_kit() == vr_view_ptr->get_current_vr_kit()) {
				int eye = vr_view_ptr->get_rendered_eye();

				// compute billboard
				dvec3 vd = vr_view_ptr->get_view_dir_of_kit();
				dvec3 y = vr_view_ptr->get_view_up_dir_of_kit();
				dvec3 x = normalize(cross(vd, y));
				y = normalize(cross(x, vd));
				x *= camera_aspect * background_extent * background_distance;
				y *= background_extent * background_distance;
				vd *= background_distance;
				dvec3 eye_pos = vr_view_ptr->get_eye_of_kit(eye);
				std::vector<vec3> P;
				std::vector<vec2> T;
				P.push_back(eye_pos + vd - x - y);
				P.push_back(eye_pos + vd + x - y);
				P.push_back(eye_pos + vd - x + y);
				P.push_back(eye_pos + vd + x + y);
				double v_offset = 0.5 * (1 - eye);
				T.push_back(dvec2(0.0, 0.5 + v_offset));
				T.push_back(dvec2(1.0, 0.5 + v_offset));
				T.push_back(dvec2(0.0, v_offset));
				T.push_back(dvec2(1.0, v_offset));

				cgv::render::shader_program& prog = seethrough;
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_position_index(), P);
				cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_texcoord_index(), T);
				cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_position_index());
				cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_texcoord_index());

				GLint active_texture, texture_binding;
				if (shared_texture) {
					glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
					glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
					glActiveTexture(GL_TEXTURE0);
					glBindTexture(GL_TEXTURE_2D, camera_tex_id);
				}
				else
					camera_tex.enable(ctx, 0);
				prog.set_uniform(ctx, "texture", 0);
				prog.set_uniform(ctx, "seethrough_gamma", seethrough_gamma);
				prog.set_uniform(ctx, "use_matrix", use_matrix);

				// use of convenience function
				vr::configure_seethrough_shader_program(ctx, prog, frame_width, frame_height,
					vr_view_ptr->get_current_vr_kit(), *vr_view_ptr->get_current_vr_state(),
					0.01f, 2 * background_distance, eye, undistorted);

				/* equivalent detailed code relies on more knowledge on program parameters
				mat4 TM = vr::get_texture_transform(vr_view_ptr->get_current_vr_kit(), *vr_view_ptr->get_current_vr_state(), 0.01f, 2 * background_distance, eye, undistorted);
				prog.set_uniform(ctx, "texture_matrix", TM);

				prog.set_uniform(ctx, "extent_texcrd", extent_texcrd);
				prog.set_uniform(ctx, "frame_split", frame_split);
				prog.set_uniform(ctx, "center_left", center_left);
				prog.set_uniform(ctx, "center_right", center_right);
				prog.set_uniform(ctx, "eye", eye);
				*/
				prog.enable(ctx);
				ctx.set_color(rgba(1, 1, 1, 1));

				glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);


				prog.disable(ctx);

				if (shared_texture) {
					glActiveTexture(active_texture);
					glBindTexture(GL_TEXTURE_2D, texture_binding);
				}
				else
					camera_tex.disable(ctx);

				cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_position_index());
				cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_texcoord_index());
			}
		}
		if (vr_view_ptr) {
			std::vector<vec3> P;
			std::vector<float> R;
			std::vector<rgb> C;
			const vr::vr_kit_state* state_ptr = vr_view_ptr->get_current_vr_state();
			if (state_ptr) {
				for (int ci = 0; ci < 4; ++ci) if (state_ptr->controller[ci].status == vr::VRS_TRACKED) {
					vec3 ray_origin, ray_direction;
					state_ptr->controller[ci].put_ray(&ray_origin(0), &ray_direction(0));
					P.push_back(ray_origin);
					R.push_back(0.002f);
					P.push_back(ray_origin + ray_length * ray_direction);
					R.push_back(0.003f);
					rgb c(float(1 - ci), 0.5f * (int)state[ci], float(ci));
					C.push_back(c);
					C.push_back(c);
				}
			}
			if (P.size() > 0) {
				auto& cr = cgv::render::ref_rounded_cone_renderer(ctx);
				cr.set_render_style(cone_style);
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
	}
}

void vr_pc_processing::finish_draw(cgv::render::context& ctx)
{
	return;
	if ((!shared_texture && camera_tex.is_created()) || (shared_texture && camera_tex_id != -1)) {
		cgv::render::shader_program& prog = ctx.ref_default_shader_program(true);
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		GLint active_texture, texture_binding;
		if (shared_texture) {
			glGetIntegerv(GL_ACTIVE_TEXTURE, &active_texture);
			glGetIntegerv(GL_TEXTURE_BINDING_2D, &texture_binding);
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, camera_tex_id);
		}
		else
			camera_tex.enable(ctx, 0);

		prog.set_uniform(ctx, "texture", 0);
		ctx.push_modelview_matrix();
		ctx.mul_modelview_matrix(cgv::math::translate4<double>(0, 3, 0));
		prog.enable(ctx);
		ctx.set_color(rgba(1, 1, 1, 0.8f));
		ctx.tesselate_unit_square();
		prog.disable(ctx);
		if (shared_texture) {
			glActiveTexture(active_texture);
			glBindTexture(GL_TEXTURE_2D, texture_binding);
		}
		else
			camera_tex.disable(ctx);
		ctx.pop_modelview_matrix();
		glDisable(GL_BLEND);
	}
}

void vr_pc_processing::read_campose() {
	point_cloud_kit->read_pc_campose();
}

///  read the whole pc with out rendering 
void vr_pc_processing::read_pc() {
	point_cloud_kit->read_pc_with_dialog(false);
}

void vr_pc_processing::read_pc_append() {
	point_cloud_kit->read_pc_with_dialog(true);
}

void vr_pc_processing::write_read_pc_to_file() {
	point_cloud_kit->write_pc_to_file();
}

void  vr_pc_processing::align_leica_scans_with_cgv() {
	point_cloud_kit->align_leica_scans_with_cgv();
	stored_cloud->align_leica_scans_with_cgv();
}

void  vr_pc_processing::rotate_x() {
	point_cloud_kit->pc.rotate(quat(vec3(1, 0, 0), 5 * M_PI / 180));
	std::cout << "rotate 5 degree around x!" << std::endl;
}

void  vr_pc_processing::rotate_z() {
	point_cloud_kit->pc.rotate(quat(vec3(0, 0, 1), 5 * M_PI / 180));
	std::cout << "rotate 5 degree around z!" << std::endl;
}

bool vr_pc_processing::load_next_shot() {
	one_shot_360pc->pc.clear_all_for_get_next_shot();
	return one_shot_360pc->pc.get_next_shot(point_cloud_kit->pc);
}

void vr_pc_processing::compute_nmls_if_is_required() {
	one_shot_360pc->compute_normals();
}

void vr_pc_processing::append_current_shot_to_stored_cloud() {
	stored_cloud->append_frame(one_shot_360pc->pc, false);
	/// it is logical to have multiple clean functions 
	one_shot_360pc->pc.clear_all_for_get_next_shot();
}

///
void vr_pc_processing::write_stored_pc_to_file() {
	stored_cloud->write_pc_to_file();
}

void vr_pc_processing::print_cloud_info() {
	std::cout << "point_cloud_kit: " << point_cloud_kit->pc.get_nr_points() << std::endl;
	std::cout << "one_shot_360pc: " << one_shot_360pc->pc.get_nr_points() << std::endl;
	std::cout << "stored_cloud: " << stored_cloud->pc.get_nr_points() << std::endl;
}

void vr_pc_processing::auto_conduct_nml_estimation_leica() {
	int loop_num = point_cloud_kit->pc.num_of_shots;
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
	write_stored_pc_to_file();
}

void vr_pc_processing::clean_all_pcs() {
	point_cloud_kit->clear_all();
	one_shot_360pc->clear_all();
	stored_cloud->clear_all();
	std::cout << "all pcs cleared" << std::endl;
}

void vr_pc_processing::create_gui() {
	/*
	*	functionalities:
			pc merging 
			large scale pc nml computing 
			pc rendering in vr (not optimized)
	*/
	add_decorator("vr_pc_processing", "heading", "level=2");
	if (begin_tree_node("pc merging tool", new bool, false, "level=3")) {
		connect_copy(add_button("read_pc_append")->click, rebind(this, &vr_pc_processing::read_pc_append));
		connect_copy(add_button("write_read_pc_to_file")->click, rebind(this, &vr_pc_processing::write_read_pc_to_file));
	}
	
	if (begin_tree_node("large scale pc nml computing", new bool, false, "level=3")) {
		connect_copy(add_button("read_pc")->click, rebind(this, &vr_pc_processing::read_pc));
		connect_copy(add_button("read_campose")->click, rebind(this, &vr_pc_processing::read_campose));
		connect_copy(add_button("auto_conduct_nml_estimation_leica")->click, rebind(this, &vr_pc_processing::auto_conduct_nml_estimation_leica));
		connect_copy(add_button("clean_all_pcs")->click, rebind(this, &vr_pc_processing::clean_all_pcs));
		connect_copy(add_button("rotate_x")->click, rebind(this, &vr_pc_processing::rotate_x));
		connect_copy(add_button("rotate_z")->click, rebind(this, &vr_pc_processing::rotate_z));
		connect_copy(add_button("load_next_shot")->click, rebind(this, &vr_pc_processing::load_next_shot));
		connect_copy(add_button("compute_nmls_intermediate_pc")->click, rebind(this, &vr_pc_processing::compute_nmls_if_is_required));
		connect_copy(add_button("append_current_shot_to_stored_cloud")->click, rebind(this, &vr_pc_processing::append_current_shot_to_stored_cloud));
		connect_copy(add_button("align_leica_scans_with_cgv")->click, rebind(this, &vr_pc_processing::align_leica_scans_with_cgv));
		connect_copy(add_button("write_stored_pc_to_file")->click, rebind(this, &vr_pc_processing::write_stored_pc_to_file));
		connect_copy(add_control("render_pc", render_pc, "check")->value_change, rebind(static_cast<drawable*>(this), &vr_pc_processing::post_redraw));
		connect_copy(add_control("force_correct_num_pcs", force_correct_num_pcs, "check")->value_change, rebind(static_cast<drawable*>(this), &vr_pc_processing::post_redraw));
	}
}

#include <cgv/base/register.h>

cgv::base::object_registration<vr_pc_processing> vr_pc_processing_reg("vr_pc_processing");
