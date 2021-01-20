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

// these are the vr specific headers
#include <vr/vr_driver.h>
#include <cg_vr/vr_server.h>
#include <vr_view_interactor.h>
#include <vr_render_helpers.h>
#include <cg_vr/vr_events.h>

class vr_kit_gui_rendering :
	public base,    // base class of all to be registered classes
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	/*
		hand tracking
	*/
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

	/*
		sphere renderer
	*/
	std::vector<vec3> points;
	std::vector<rgba> point_colors;

	/*
		custom renderer 
	*/
	vec3 sphere_scale = vec3(0.01,0.01,0.01);
	cgv::render::shader_program cube_prog;

	/*
		texture stuffs 
	*/
	float texture_frequency = 50;
	float texture_frequency_aspect = 1;
	cgv::render::texture* t_ptr;
	cgv::render::textured_material tex_mat;


public:
	cgv::render::sphere_render_style srs;
	vec3 offset_in_ori_pose;
	vec3 plane_nml_ori_dir = vec3(1,0,0);
	/// initialize rotation angle
	vr_kit_gui_rendering() {
		srs.radius = sphere_scale.x();
		srs.map_color_to_material = cgv::render::CM_COLOR_AND_OPACITY;
		srs.material.set_brdf_type(cgv::media::illum::BT_PHONG);
		//srs.material = tex_mat; // never do this, no texture support in the sphere shader 
		//srs.halo_color = rgba(0,0,1,0.2);
		//srs.halo_color_strength
		// srs. transparent
	}
	// compute intersection points of controller ray with movable boxes
	bool init(cgv::render::context& ctx) {
		bool succ = cube_prog.build_program(ctx, "color_cube.glpr", true);

		char* cgv_data = getenv("CGV_DATA");
		std::string data_dir = std::string(cgv_data);

		int di = tex_mat.add_image_file(data_dir + "//images//1.jpg");
		tex_mat.set_diffuse_index(di);
		if (tex_mat.ensure_textures(ctx))
			t_ptr = tex_mat.get_texture(di);
		else
			return false;

		/*int n = 1024;
		cgv::data::data_format df(n, n, cgv::type::info::TI_FLT32, cgv::data::CF_L);
		cgv::data::data_view dv(&df);
		int i, j;
		float* ptr = (float*)dv.get_ptr<unsigned char>();
		for (i = 0; i < n; ++i)
			for (j = 0; j < n; ++j)
				ptr[i * n + j] = (float)(((i / 8) & 1) ^ ((j / 8) & 1));
		t_ptr->create(ctx, dv);*/

		tex_mat.set_transparency(0.3);
		srs.material = tex_mat;
		return true;
	}
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "boxgui_interactable";
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
	/// overload to handle events, return true if event was processed
	bool handle(event& e)
	{
		// check if vr event flag is not set and don't process events in this case
		/*if ((e.get_flags() & cgv::gui::EF_VR) == 0)
			return false;*/
		// check event id
		switch (e.get_kind()) {
			case cgv::gui::EID_KEY:
			{
				return true;
			}
			case cgv::gui::EID_THROTTLE:
			{
				return true;
			}
			case cgv::gui::EID_STICK:
			{
				return true;
			}
			case cgv::gui::EID_POSE: 
			{
				cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
				// check for controller pose events
				int ci = vrpe.get_trackable_index();
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
				}
				if (ci != -1) {
					global_offset = offset_in_ori_pose;
					cur_right_hand_rot_quat.rotate(global_offset);
					global_offset += cur_right_hand_posi;
					if (points.size())
						points.at(0) = global_offset;
				}
				return true;
			}
		}
		return false;
	}
	///
	void init_frame(cgv::render::context& ctx) {
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		
	}

	void configure_the_handhold_sphere(vec3 _offset_in_ori_pose) {
		offset_in_ori_pose = _offset_in_ori_pose;
		points.push_back(vec3(0));
		point_colors.push_back(rgba(0.0f, 0.0f, 1.0f, 0.9f));
	}

	void configure_the_handhold_sphere(vec3 _offset_in_ori_pose, float radii) {
		offset_in_ori_pose = _offset_in_ori_pose;
		sphere_scale = vec3(radii, radii, radii);
		srs.radius = radii;
		points.push_back(vec3(0));
		point_colors.push_back(rgba(0.4f, 0.4f, 0.4f, 0.5f));
	}
	void get_sphere_properties(float& transparency, float& radii) {
	
	}

	bool get_select_tool_info(vec3& posi, float& radii) {
		if (!points.size())
			return false;
		posi = global_offset;
		radii = srs.radius;
		return true;
	}

	void get_clipping_plane_info(vec3& cur_plane_normal,vec3& a_point_on_the_plane) {
		a_point_on_the_plane = global_offset;
		cur_plane_normal = plane_nml_ori_dir;
		cur_right_hand_rot_quat.rotate(cur_plane_normal);
	}

	void render_a_handhold_plane_for_clipping(cgv::render::context& ctx) {
		vec3 ext = vec3(0,2,4);
		// surface renderer with texture 
		if (has_ctrl_posi) {
			// points for a label
			vec3 p1(0,0.5 * ext.y(), 0.5 * ext.z());
			vec3 p2(0,-0.5 * ext.y(), 0.5 * ext.z());
			vec3 p3(0,0.5 * ext.y(), -0.5 * ext.z());
			vec3 p4(0,-0.5 * ext.y(), -0.5 * ext.z());

			vec3 addi_offset = vec3(0,0,-ext.z() / 2);

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
			cur_right_hand_rot_quat.rotate(p1);
			cur_right_hand_rot_quat.rotate(p2);
			cur_right_hand_rot_quat.rotate(p3);
			cur_right_hand_rot_quat.rotate(p4);

			p1 = p1 + cur_right_hand_posi;
			p2 = p2 + cur_right_hand_posi;
			p3 = p3 + cur_right_hand_posi;
			p4 = p4 + cur_right_hand_posi;

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
		
		// surface renderer with texture 
		//if (points.size()) {
		//	glDepthMask(GL_FALSE);
		//	glEnable(GL_BLEND);
		//	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		//	glDisable(GL_CULL_FACE);

		//	/*
		//		with custom shader 
		//	*/ 
		//	/*cube_prog.enable(ctx);
		//	mat4 mvp, R;
		//	quat rot(cur_left_hand_rot);
		//	rot.put_homogeneous_matrix(R);
		//	mvp = cgv::math::scale4<double>(0.01 * obj_scale_factor, 0.01 * obj_scale_factor, 0.01 * obj_scale_factor);
		//	mvp = R * mvp;
		//	mvp = cgv::math::translate4<double>(points.at(0)) * mvp;
		//	cube_prog.set_uniform(ctx, "mvp", mvp);*/

		//	/*
		//		with default surface shader. todo: position update ineffecient 
		//	*/
		//	cgv::render::shader_program& prog = ctx.ref_surface_shader_program(true);
		//	prog.enable(ctx);
		//	mat4 R; quat rot(cur_left_hand_rot); rot.put_homogeneous_matrix(R);
		//	ctx.push_modelview_matrix();
		//	//ctx.tesselate_unit_sphere(25, false, false);
		//	ctx.mul_modelview_matrix(cgv::math::translate4<double>(points.at(0))
		//		* R
		//		* cgv::math::scale4<double>(sphere_scale)
		//	);
		//	//ctx.enable_material(tex_mat);
		//		switch (2) {
		//		case 0:
		//			ctx.tesselate_unit_cube(false, false);
		//			break;
		//		case 1:
		//			ctx.tesselate_unit_sphere(25, false, false);
		//			break;
		//		case 2:
		//			ctx.tesselate_unit_prism(false, false);
		//			break;
		//		case 3:
		//			ctx.tesselate_unit_icosahedron(false, false);
		//			break;
		//		}
		//	//ctx.disable_material(tex_mat);
		//	//cube_prog.disable(ctx);

		//	prog.disable(ctx);
		//	ctx.pop_modelview_matrix();

		//	glDepthMask(GL_TRUE);
		//}
	}
	
};
