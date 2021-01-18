#include "visual_processing.h"

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
visual_processing::visual_processing() 
{
	set_name("visual_processing");
	vr_view_ptr = 0;
	ray_length = 2;
	connect(cgv::gui::ref_vr_server().on_device_change, this, &visual_processing::on_device_change);
	connect(cgv::gui::ref_vr_server().on_status_change, this, &visual_processing::on_status_change);

}
	
void visual_processing::stream_help(std::ostream& os) {
	os << "visual_processing: no shortcuts defined" << std::endl;
}
	
void visual_processing::on_set(void* member_ptr)
{
	update_member(member_ptr);
	post_redraw();
}
	
bool visual_processing::handle(cgv::gui::event& e)
{
	b_interactable->handle(e);
	return false;
}

bool visual_processing::init(cgv::render::context& ctx)
{
	skybox_kit->init(ctx);
	//image_renderer_kit->init(ctx);

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

void visual_processing::clear(cgv::render::context& ctx)
{
	cgv::render::ref_box_renderer(ctx, -1);
	cgv::render::ref_sphere_renderer(ctx, -1);
	cgv::render::ref_rounded_cone_renderer(ctx, -1);
	point_cloud_kit->clear(ctx);
	one_shot_360pc->clear(ctx);
	stored_cloud->clear(ctx);
}

void visual_processing::init_frame(cgv::render::context& ctx)
{
	b_interactable->init_frame(ctx);
	point_cloud_kit->init_frame(ctx);
	one_shot_360pc->init_frame(ctx);
	stored_cloud->init_frame(ctx);
	//image_renderer_kit->init_frame(ctx);

	if (have_new_mesh) {
		// auto-compute mesh normals if not available
		if (!M.has_normals())
			M.compute_vertex_normals();
		// [re-]compute mesh render info
		mesh_info.destruct(ctx);
		mesh_info.construct(ctx, M);
		// bind mesh attributes to standard surface shader program
		mesh_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
		mesh_info.bind_wireframe(ctx, ref_rounded_cone_renderer(ctx).ref_prog(), true);
		// ensure that materials are presented in gui
		//post_recreate_gui();
		have_new_mesh = false;

		// focus view on new mesh
		/*clipped_view* view_ptr = dynamic_cast<clipped_view*>(find_view_as_node());
		if (view_ptr) {
			box3 box = M.compute_box();
			view_ptr->set_scene_extent(box);
			view_ptr->set_focus(box.get_center());
			view_ptr->set_y_extent_at_focus(box.get_extent().length());
		}*/
	}
}

void visual_processing::draw(cgv::render::context& ctx)
{
	skybox_kit->draw(ctx);

	b_interactable->draw(ctx);

	if(render_pc)
		point_cloud_kit->draw(ctx);

	if (show_face) {
		// remember current culling setting
		GLboolean is_culling = glIsEnabled(GL_CULL_FACE);
		GLint cull_face;
		glGetIntegerv(GL_CULL_FACE_MODE, &cull_face);

		// ensure that opengl culling is identical to shader program based culling
		if (cull_mode > 0) {
			glEnable(GL_CULL_FACE);
			glCullFace(cull_mode == CM_BACKFACE ? GL_BACK : GL_FRONT);
		}
		else
			glDisable(GL_CULL_FACE);

		// choose a shader program and configure it based on current settings
		shader_program& prog = ctx.ref_surface_shader_program(true);
		prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
		prog.set_uniform(ctx, "map_color_to_material", (int)CM_COLOR_FRONT);
		//prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
		// set default surface color for color mapping which only affects 
		// rendering if mesh does not have per vertex colors and color_mapping is on
		prog.set_attribute(ctx, prog.get_color_index(), surface_color);

		// render the mesh from the vertex buffers with selected program
		mesh_info.draw_all(ctx, true, false);

		// recover opengl culling mode
		if (is_culling)
			glEnable(GL_CULL_FACE);
		else
			glDisable(GL_CULL_FACE);
		glCullFace(cull_face);
	}


	if (show_wireframe) {
		rounded_cone_renderer& cr = ref_rounded_cone_renderer(ctx);
		cr.set_render_style(cone_style);
		if (cr.enable(ctx)) {
			mesh_info.draw_wireframe(ctx);
			cr.disable(ctx);
		}
	}

	//if(render_img)
	//	image_renderer_kit->draw(ctx);

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

void visual_processing::finish_draw(cgv::render::context& ctx)
{
	if (show_face) {
		glDisable(GL_CULL_FACE);
		// choose a shader program and configure it based on current settings
		shader_program& prog = ctx.ref_surface_shader_program(true);
		mesh_info.draw_all(ctx, false, true);
	}
}

void visual_processing::read_campose() {
	point_cloud_kit->read_pc_campose(*get_context());
}

/* start pc reading and point_cloud_kit processing tool */
///  read the whole pc to point_cloud_kit
void visual_processing::read_pc() {
	point_cloud_kit->read_pc_with_dialog(false);
}

void visual_processing::read_pc_queue() {
	point_cloud_kit->read_pc_with_dialog_queue(false);
}

void visual_processing::read_pc_append() {
	point_cloud_kit->read_pc_with_dialog(true);
}
// perform actions to point_cloud_kit

/// 
void visual_processing::downsampling() {
	point_cloud_kit->downsampling(step, num_of_points_wanted, strategy);
}

void visual_processing::add_reflectance() {
	// = true;
}

//
///
void visual_processing::write_read_pc_to_file() {
	point_cloud_kit->write_pc_to_file();
}

void  visual_processing::align_leica_scans_with_cgv() {
	point_cloud_kit->align_leica_scans_with_cgv();
	stored_cloud->align_leica_scans_with_cgv();
}

void  visual_processing::rotate_x() {
	point_cloud_kit->pc.rotate(quat(vec3(1, 0, 0), 5 * M_PI / 180));
	std::cout << "rotate 5 degree around x!" << std::endl;
}

void  visual_processing::rotate_z() {
	point_cloud_kit->pc.rotate(quat(vec3(0, 0, 1), 5 * M_PI / 180));
	std::cout << "rotate 5 degree around z!" << std::endl;
}

/* end point_cloud_kit processing tool*/
/// load one shot from point_cloud_kit according to .campose file  
bool visual_processing::load_next_shot() {
	one_shot_360pc->pc.clear_all_for_get_next_shot();
	return one_shot_360pc->pc.get_next_shot(point_cloud_kit->pc);
}

void visual_processing::compute_nmls_if_is_required() {
	one_shot_360pc->compute_normals();
}

void visual_processing::append_current_shot_to_stored_cloud() {
	stored_cloud->append_frame(one_shot_360pc->pc, false);
	/// it is logical to have multiple clean functions 
	one_shot_360pc->pc.clear_all_for_get_next_shot();
}

///
void visual_processing::write_stored_pc_to_file() {
	stored_cloud->write_pc_to_file();
}

void visual_processing::write_stored_pc_to_file_direct() {
	stored_cloud->pc.write(cgv::base::ref_data_path_list()[0] + "/output.txt");
}

void visual_processing::print_cloud_info() {
	std::cout << "point_cloud_kit: " << point_cloud_kit->pc.get_nr_points() << std::endl;
	std::cout << "one_shot_360pc: " << one_shot_360pc->pc.get_nr_points() << std::endl;
	std::cout << "stored_cloud: " << stored_cloud->pc.get_nr_points() << std::endl;
}

void visual_processing::auto_conduct_nml_estimation_leica() {
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
	if (direct_write)
		write_stored_pc_to_file_direct();
	else
		write_stored_pc_to_file();
}

void visual_processing::clean_all_pcs() {
	point_cloud_kit->clear_all();
	one_shot_360pc->clear_all();
	stored_cloud->clear_all();
	std::cout << "all pcs cleared" << std::endl;
}

void visual_processing::read_mesh() {
	M.clear();
	std::string f = cgv::gui::file_open_dialog("Open", "Meshes:*");
	M.read(f);
	have_new_mesh = true;
	show_face = true;
	show_wireframe = true;

	cone_style.radius = 0.5f * float(0.05 * sqrt(M.compute_box().get_extent().sqr_length() / M.get_nr_positions()));
	cone_style.surface_color = rgb(0.6f, 0.5f, 0.4f);
	post_redraw();
}

void visual_processing::randomize_texcoordi() {
	M.randomize_texcoordi();
}

void visual_processing::write_mesh() {
	std::string f = cgv::gui::file_save_dialog("Save", "Meshes:*");
	M.write_with_materials(f);
}

void visual_processing::load_image_from_bin_files() {
	//std::string f = cgv::gui::file_open_dialog("Open", "Images:*");
	//image_renderer_kit->bind_image_to_camera_position(*get_context(),f,quat(),vec3(0,1,0));
	//render_img = true;
	//post_redraw();
}

void visual_processing::create_gui() {
	add_decorator("visual_processing", "heading", "level=2");
	if (begin_tree_node("Point Cloud Merging Tool", new bool, false, "level=3")) {
		connect_copy(add_button("read_pc_append")->click, rebind(this, &visual_processing::read_pc_append));
		add_member_control(this, "[0]step", step, "value_slider","min=1;max=1000;log=false;ticks=true;");
		add_member_control(this, "[1]num_of_points_wanted", num_of_points_wanted, "value_slider", "min=1;max=100000000;log=false;ticks=true;");
		add_member_control(this, "which_strategy", strategy, "value_slider", "min=0;max=1;log=false;ticks=true;");
		connect_copy(add_button("downsampling")->click, rebind(this, &visual_processing::downsampling));
		/// only for point_cloud_kit
		add_member_control(this, "write_reflectance", point_cloud_kit->pc.write_reflectance, "check");
		connect_copy(add_button("write_read_pc_to_file")->click, rebind(this, &visual_processing::write_read_pc_to_file));
	}
	
	if (begin_tree_node("Point Cloud Nml Computing", new bool, false, "level=3")) {
		connect_copy(add_button("read_pc")->click, rebind(this, &visual_processing::read_pc));
		connect_copy(add_button("read_pc_queue")->click, rebind(this, &visual_processing::read_pc_queue));
		connect_copy(add_button("read_campose")->click, rebind(this, &visual_processing::read_campose));
		connect_copy(add_button("auto_conduct_nml_estimation_leica")->click, rebind(this, &visual_processing::auto_conduct_nml_estimation_leica));
		connect_copy(add_button("clean_all_pcs")->click, rebind(this, &visual_processing::clean_all_pcs));
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

	if (begin_tree_node("Meshing Tools", new bool, false, "level=3")) {
		connect_copy(add_button("read_mesh")->click, rebind(this, &visual_processing::read_mesh));
		add_member_control(this, "cull mode", cull_mode, "dropdown", "enums='none,back,front'");
		add_member_control(this, "show_face", show_face, "check");
		add_member_control(this, "show_wireframe", show_wireframe, "check");
		add_member_control(this, "", surface_color, "", "w=42");
		//randomize_texcoordi
		connect_copy(add_button("randomize_texcoordi")->click, rebind(this, &visual_processing::randomize_texcoordi));
		connect_copy(add_button("write_mesh")->click, rebind(this, &visual_processing::write_mesh));
	}

	if (begin_tree_node("Image Processing", render_img, false, "level=3")) {
		//connect_copy(add_button("load_image")->click, rebind(this, &visual_processing::load_image_from_bin_files));
		//add_member_control(this, "apply_aspect", image_renderer_kit->apply_aspect, "check");
		//add_member_control(this, "render_frame", image_renderer_kit->render_frame, "check");
	}

	connect_copy(add_control("render_pc", render_pc, "check")->value_change, rebind(static_cast<drawable*>(this), &visual_processing::post_redraw));
}

#include <cgv/base/register.h>

cgv::base::object_registration<visual_processing> vr_pc_processing_reg("visual_processing");