#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv_gl/gl/gl.h>
#include <cgv/gui/trigger.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/provider.h>
#include <cgv/math/ftransform.h>
#include <random>
#include <cgv_gl/sphere_renderer.h>
#include <cgv/gui/trigger.h>

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>
#include <vr_kit_intersection.h>

class vr_kit_teleportation :
	public base,    // base class of all to be registered classes
	public event_handler, // necessary to receive events
	public drawable, // registers for drawing with opengl
	public cgv::signal::tacker
{
private:
	vr_view_interactor* vr_view_ptr;

	// hand tracking
	int left_rgbd_controller_index = 0;
	int right_rgbd_controller_index = 1;
	vec3 cur_left_hand_posi;
	vec3 cur_left_hand_dir;
	mat3 cur_left_hand_rot;
	quat cur_left_hand_rot_quat;
	mat3 cur_left_hand_rot_mat;
	vec3 cur_right_hand_posi;
	quat cur_right_hand_rot_quat;
	mat3 cur_right_hand_rot_as_mat;
	vec3 global_offset;
	bool has_ctrl_posi = false;
	bool hand_touching = false;
	vec3 touching_origin = vec3(0);
	vec3 touching_stopped = vec3(0);
	vec3 tracking_origin_when_touching = vec3(0);
	bool accept_event = true;

	// handhold sphere rendering
	std::vector<vec3> points;
	std::vector<rgba> point_colors;
	vec3 sphere_scale = vec3(0.01, 0.01, 0.01);

	// ray rendering 
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};// state of current interaction with boxes for all controllers
	InteractionState state[4];
	float ray_length = 2;
	cgv::render::rounded_cone_render_style rounded_cone_style;


	// timer event
	double start_t = 0;
	bool the_first_enter = true;

public:
	// interaction modes, x5 impl.
	enum interaction_mode {
		None = 0,
		TELEPORT,
		DIRECTIONAL,
		CLIMB = 3 // decrate case, a little complicated when update tracking origin at each pose event 
	}mode;
	int max_idx_num = 3;
	bool is_lifting = false;
	bool enable_gravity = false;

	//
	cgv::render::sphere_render_style srs;
	vec3 offset_in_ori_pose;
	vec3 plane_nml_ori_dir = vec3(1, 0, 0);

	vr_kit_teleportation() {
		srs.radius = sphere_scale.x();
		srs.map_color_to_material = cgv::render::CM_COLOR_AND_OPACITY;
		srs.material.set_brdf_type(cgv::media::illum::BT_PHONG);
		mode = interaction_mode::None;
		connect(get_animation_trigger().shoot, this, &vr_kit_teleportation::timer_event);
		configure_the_handhold_sphere(vec3(0,0,-0.2),0.03);
	}
	void timer_event(double t, double dt) {
		if (vr_view_ptr && is_lifting) {
			vec3 delta = vec3(0,5*dt,0);
			vr_view_ptr->set_tracking_origin(vr_view_ptr->get_tracking_origin() + delta);
		}
		if (vr_view_ptr && enable_gravity && vr_view_ptr->get_tracking_origin().y()>0) {
			if (the_first_enter) {
				start_t = t;
				the_first_enter = false;
			}
			vec3 delta = vec3(0, -9.8 * (t-start_t) * dt, 0);
			vr_view_ptr->set_tracking_origin(vr_view_ptr->get_tracking_origin() + delta);
		}
		if (!enable_gravity) {
			the_first_enter = true;
		}
	}
	// compute intersection points of controller ray with movable boxes
	bool init(cgv::render::context& ctx) {
		return true;
	}
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "vr_kit_teleportation";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}
	/// self reflection allows to change values in the config file
	bool self_reflect(reflection_handler& rh)
	{
		return true;
	}
	void set_left_hand_index(int left_idx) {
		left_rgbd_controller_index = left_idx;
	}
	vec3 compute_ray_plane_intersection_point(const vec3& origin, const vec3& direction)
	{
		float t_result;
		vec3  p_result = vec3(0);
		vec3  n_result;
		box3 floor = box3(vec3(-100, -1, -100), vec3(100, 0, 100));
		if (cgv::media::ray_axis_aligned_box_intersection(
			origin, direction,
			floor,
			t_result, p_result, n_result, 0.000001f)) {
			return p_result;
		}

		return p_result;
	}
	/// overload to handle events, return true if event was processed
	void set_mode(interaction_mode m){
		mode = m;
	}
	void set_vr_view_ptr(vr_view_interactor* p) {
		vr_view_ptr = p;
	}
	bool handle(event& e)
	{
		// check if vr event flag is not set and don't process events in this case
		/*if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;*/
			// check event id
		// ensure the view ptr for teleportation 
		if (!vr_view_ptr)
			return false;
		switch (e.get_kind()) {
		case cgv::gui::EID_KEY:
		{
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			int ci = vrke.get_controller_index();

			// right hand 
			if (ci == right_rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
					vr_view_ptr->set_tracking_rotation(vr_view_ptr->get_tracking_rotation() + 10);
			}
			if (ci == right_rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS)
					vr_view_ptr->set_tracking_rotation(vr_view_ptr->get_tracking_rotation() - 10);
			}

			// spatial interactive 
			// left hand key event for mode switching and lifting/gravity  
			if (ci == left_rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_LEFT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS) {
					int cur_mode = static_cast<int>(mode);
					cur_mode--;
					if (cur_mode < 0)
						mode = static_cast<interaction_mode>(0);
					else
						mode = static_cast<interaction_mode>(cur_mode);
				}

			}
			if (ci == left_rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_RIGHT)
			{
				if (vrke.get_action() == cgv::gui::KA_PRESS) {
					int cur_mode = static_cast<int>(mode);
					cur_mode++;
					if (cur_mode > max_idx_num)
						mode = static_cast<interaction_mode>(max_idx_num);
					else
						mode = static_cast<interaction_mode>(cur_mode);
				}
			}
			if (ci == left_rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_UP)
			{
				//if (vrke.get_action() == cgv::gui::KA_PRESS) 
					is_lifting = !is_lifting;
			}
			if (ci == left_rgbd_controller_index && vrke.get_key() == vr::VR_DPAD_DOWN)
			{
				//if (vrke.get_action() == cgv::gui::KA_PRESS)
					enable_gravity = !enable_gravity;
			}
			return true;
		}
		case cgv::gui::EID_THROTTLE:
		{
			return true;
		}
		case cgv::gui::EID_STICK:
		{		
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			int ci = vrse.get_controller_index();
			vec3 origin, direction;
			if(ci != -1)
			switch (vrse.get_action()) {
				case cgv::gui::SA_TOUCH:
					if (mode == interaction_mode::TELEPORT && ci == right_rgbd_controller_index) {
						vrse.get_state().controller[ci].put_ray(&origin(0), &direction(0));
						vec3 posi = compute_ray_plane_intersection_point(origin, direction);
						vr_view_ptr->set_tracking_origin(vec3(posi.x(), vr_view_ptr->get_tracking_origin().y(), posi.z()));
					}
					if (mode == interaction_mode::DIRECTIONAL && ci == right_rgbd_controller_index) {
						vrse.get_state().controller[ci].put_ray(&origin(0), &direction(0));
						direction.normalize();
						vr_view_ptr->set_tracking_origin(vr_view_ptr->get_tracking_origin() + direction);
					}
					if (mode == interaction_mode::CLIMB) {
						if (hand_touching) {
							vrse.get_state().controller[ci].put_ray(&origin(0), &direction(0));
							touching_origin = origin;
							tracking_origin_when_touching = vr_view_ptr->get_tracking_origin();
							hand_touching = !hand_touching;
						}
						else {
							vrse.get_state().controller[ci].put_ray(&origin(0), &direction(0));
							touching_stopped = origin;
							vec3 from_origin_to_current = touching_stopped - touching_origin;
							vr_view_ptr->set_tracking_origin(tracking_origin_when_touching - from_origin_to_current);
							hand_touching = !hand_touching;
						}
					}
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
			// left hand event 
			if (ci == left_rgbd_controller_index) {
				// update positions 
				cur_left_hand_posi = vrpe.get_position();
				cur_left_hand_rot = vrpe.get_orientation();
				cur_left_hand_rot_quat = vrpe.get_quaternion();
			}
			if (ci == right_rgbd_controller_index) {
				cur_right_hand_posi = vrpe.get_position();
				cur_right_hand_rot_quat = vrpe.get_quaternion();
				vrpe.get_quaternion().put_matrix(cur_right_hand_rot_as_mat);
				has_ctrl_posi = true;
				//if (mode == interaction_mode::CLIMB && hand_touching) {
				//	if (accept_event) {
				//		vec3 from_origin_to_current = vrpe.get_position() - touching_origin;
				//		std::cout << "from_origin_to_current: " << from_origin_to_current << std::endl;
				//		if(from_origin_to_current.x()>0 || from_origin_to_current.y()>0 || from_origin_to_current.z()>0)
				//			vr_view_ptr->set_tracking_origin(tracking_origin_when_touching - from_origin_to_current);
				//	}
				//	accept_event = !accept_event;
				//}
			}
			if (ci != -1) {
				// update globle offset 
				global_offset = offset_in_ori_pose;
				cur_right_hand_rot_quat.rotate(global_offset);
				global_offset += cur_right_hand_posi;
				if (points.size())
					points.at(0) = global_offset;
				// update globle offset 
				/*global_offset = offset_in_ori_pose;
				cur_left_hand_rot_quat.rotate(global_offset);
				global_offset += cur_left_hand_posi;
				if (points.size()>1)
					points.at(1) = global_offset;*/
			}
			return true;
		}
		}
		return false;
	}
	///
	void init_frame(cgv::render::context& ctx) {
	}

	void finish_draw(context& ctx) {
		if (mode == interaction_mode::None)
		{

		}
		if (mode == interaction_mode::TELEPORT) //do nothing for now
		{
			render_lines_for_controllers(ctx);
		}
		if (mode == interaction_mode::DIRECTIONAL)
		{
			render_a_handhold_arrow(ctx);
		}
		if (mode == interaction_mode::CLIMB)
		{
			render_a_handhold_sphere_if_configured(ctx);
		}
	}

	void configure_the_handhold_sphere(vec3 _offset_in_ori_pose, float radii) {
		offset_in_ori_pose = _offset_in_ori_pose;
		sphere_scale = vec3(radii, radii, radii);
		srs.radius = radii;
		points.push_back(vec3(0));
		point_colors.push_back(rgba(0.4f, 0.4f, 0.4f, 0.5f));
		//points.push_back(vec3(0));
		//point_colors.push_back(rgba(0.4f, 0.4f, 0.4f, 0.5f));
	}

	void render_a_handhold_sphere_if_configured(cgv::render::context& ctx) {
		// sphere renderer with materials 
		if (points.size()) {
			glDepthMask(GL_FALSE);
			glEnable(GL_BLEND);
			glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
			glDisable(GL_CULL_FACE);
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(srs);
			sr.set_position_array(ctx, points);
			sr.set_color_array(ctx, point_colors);
			sr.render(ctx, 0, points.size());
			glDepthMask(GL_TRUE);
		}
	}

	void render_a_handhold_arrow(cgv::render::context& ctx) {
		if (points.size()) {
			auto& prog = ctx.ref_surface_shader_program();
			prog.set_uniform(ctx, "map_color_to_material", 3);
			prog.enable(ctx);
			ctx.set_color(rgb(0.4));
			ctx.tesselate_arrow(cur_right_hand_posi, points.at(0),0.1f, 2.0f, 0.5f);
			prog.disable(ctx);
		}
	}

	void render_a_handhold_box(cgv::render::context& ctx) {
		if (points.size()) {
			auto& prog = ctx.ref_surface_shader_program();
			prog.set_uniform(ctx, "map_color_to_material", 3);
			prog.enable(ctx);
			ctx.set_color(rgb(0.4));
			ctx.tesselate_box(box3(points.at(0) - vec3(0.01), points.at(0) + vec3(0.01)),false,false);
			prog.disable(ctx);
		}
	}

	void render_lines_for_controllers(cgv::render::context& ctx) {
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
	}

};
