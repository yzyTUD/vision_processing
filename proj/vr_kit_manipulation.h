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
#include "vis_kit_datastore.h"

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

class vr_kit_manipulation :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
private:
	vis_kit_data_store_shared* data_ptr = nullptr;
	vr_view_interactor* vr_view_ptr;

	// different interaction states for the controllers
	enum InteractionState {
		IS_NONE,
		IS_OVER,
		IS_GRAB
	};
	// state of current interaction with boxes for all controllers
	InteractionState state[4];
	float ray_length = 2;
	cgv::render::rounded_cone_render_style rounded_cone_style;

	bool test_manipulate = false;
public:
	/// initialize rotation angle
	vr_kit_manipulation()
	{
		connect(get_animation_trigger().shoot, this, &vr_kit_manipulation::timer_event);
		state[0] = state[1] = state[2] = state[3] = IS_NONE;
	}
	/// call this 
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
		// after we have data ptr 
		gen_random_movable_boxes();
	}
	void set_vr_view_ptr(vr_view_interactor* p) {
		vr_view_ptr = p;
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
		return "vr_kit_manipulation";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}
	/// compute intersection points of controller ray with movable boxes
	void compute_intersections(const vec3& origin, const vec3& direction, int ci, const rgb& color)
	{
		if (data_ptr==nullptr)
			return;
		for (size_t i = 0; i < data_ptr->movable_boxes.size(); ++i) {
			vec3 origin_box_i = origin - data_ptr->movable_box_translations[i];
			data_ptr->movable_box_rotations[i].inverse_rotate(origin_box_i);
			vec3 direction_box_i = direction;
			data_ptr->movable_box_rotations[i].inverse_rotate(direction_box_i);
			float t_result;
			vec3  p_result;
			vec3  n_result;
			if (cgv::media::ray_axis_aligned_box_intersection(
				origin_box_i, direction_box_i,
				data_ptr->movable_boxes[i],
				t_result, p_result, n_result, 0.000001f)) {

				// transform result back to world coordinates
				data_ptr->movable_box_rotations[i].rotate(p_result);
				p_result += data_ptr->movable_box_translations[i];
				data_ptr->movable_box_rotations[i].rotate(n_result);

				// store intersection information
				data_ptr->intersection_points.push_back(p_result);
				data_ptr->intersection_colors.push_back(color);
				data_ptr->intersection_box_indices.push_back((int)i);
				data_ptr->intersection_controller_indices.push_back(ci);
			}
		}

		for (size_t i = 0; i < data_ptr->iba->boxarr.size(); ++i) {
			vec3 origin_box_i = origin - data_ptr->iba->posiarr[i];
			data_ptr->iba->oriarr[i].inverse_rotate(origin_box_i);
			vec3 direction_box_i = direction;
			data_ptr->iba->oriarr[i].inverse_rotate(direction_box_i);
			float t_result;
			vec3  p_result;
			vec3  n_result;
			if (cgv::media::ray_axis_aligned_box_intersection(
				origin_box_i, direction_box_i,
				data_ptr->iba->boxarr[i],
				t_result, p_result, n_result, 0.000001f)) {

				// transform result back to world coordinates
				data_ptr->iba->oriarr[i].rotate(p_result);
				p_result += data_ptr->iba->posiarr[i];
				data_ptr->iba->oriarr[i].rotate(n_result);

				// store intersection information
				data_ptr->ipimg.push_back(p_result);
				data_ptr->icimg.push_back(color);
				data_ptr->ibidximg.push_back((int)i);
				data_ptr->icidximg.push_back(ci);
			}
		}
	}
	void init_trackable_data_store() {
		// saparated impl.
	}
	/// overload to handle events, return true if event was processed
	bool handle(event& e)
	{
		if (e.get_kind() == cgv::gui::EID_STICK) {
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
			}
		}
		if (e.get_kind() == cgv::gui::EID_POSE) {
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			if (ci != -1) {
				if (data_ptr==nullptr)
					return false;
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
					for (size_t i = 0; i < data_ptr->intersection_points.size(); ++i) {
						if (data_ptr->intersection_controller_indices[i] != ci)
							continue;
						// extract box index
						unsigned bi = data_ptr->intersection_box_indices[i];
						// update translation with position change and rotation
						data_ptr->movable_box_translations[bi] =
							rotation * (data_ptr->movable_box_translations[bi] - last_pos) + pos;
						// update orientation with rotation, note that quaternions
						// need to be multiplied in oposite order. In case of matrices
						// one would write box_orientation_matrix *= rotation
						data_ptr->movable_box_rotations[bi] = quat(rotation) * data_ptr->movable_box_rotations[bi];
						// update intersection points
						data_ptr->intersection_points[i] = rotation * (data_ptr->intersection_points[i] - last_pos) + pos;

						// update trackable_box_list at the same time , setter 
						// upload_to_trackable_list
						auto& t = data_ptr->trackable_box_list.at(bi);
						t.set_position_orientation_write(data_ptr->movable_box_translations[bi], data_ptr->movable_box_rotations[bi]);
					}

					for (size_t i = 0; i < data_ptr->ipimg.size(); ++i) {
						if (data_ptr->icidximg[i] != ci)
							continue;
						// extract box index
						unsigned bi = data_ptr->ibidximg[i];
						// update translation with position change and rotation
						data_ptr->iba->posiarr[bi] =
							rotation * (data_ptr->iba->posiarr[bi] - last_pos) + pos;
						// update orientation with rotation, note that quaternions
						// need to be multiplied in oposite order. In case of matrices
						// one would write box_orientation_matrix *= rotation
						data_ptr->iba->oriarr[bi] = quat(rotation) * data_ptr->iba->oriarr[bi];
						// update intersection points
						data_ptr->ipimg[i] = rotation * (data_ptr->ipimg[i] - last_pos) + pos;

						// update trackable_box_list at the same time , setter 
						// upload_to_trackable_list
						// upload the posi/ ori of the images 
						data_ptr->iba->update_posi_ori_img_given_idx(data_ptr->iba->posiarr[bi], data_ptr->iba->oriarr[bi],bi);
						auto& t = data_ptr->trackable_imagebox_list.at(bi);
						t.set_position_orientation_write(data_ptr->iba->posiarr[bi], data_ptr->iba->oriarr[bi]);
					}
				}
				else {// not grab
					// clear intersections of current controller 
					size_t i = 0;
					while (i < data_ptr->intersection_points.size()) {
						if (data_ptr->intersection_controller_indices[i] == ci) {
							data_ptr->intersection_points.erase(data_ptr->intersection_points.begin() + i);
							data_ptr->intersection_colors.erase(data_ptr->intersection_colors.begin() + i);
							data_ptr->intersection_box_indices.erase(data_ptr->intersection_box_indices.begin() + i);
							data_ptr->intersection_controller_indices.erase(data_ptr->intersection_controller_indices.begin() + i);
						}
						else
							++i;
					}
					int ii = 0;
					while (ii < data_ptr->ipimg.size()) {
						if (data_ptr->icidximg[ii] == ci) {
							data_ptr->ipimg.erase(data_ptr->ipimg.begin() + ii);
							data_ptr->icimg.erase(data_ptr->icimg.begin() + ii);
							data_ptr->ibidximg.erase(data_ptr->ibidximg.begin() + ii);
							data_ptr->icidximg.erase(data_ptr->icidximg.begin() + ii);
						}
						else
							++ii;
					}

					// compute intersections
					vec3 origin, direction;
					vrpe.get_state().controller[ci].put_ray(&origin(0), &direction(0));
					compute_intersections(origin, direction, ci, ci == 0 ? rgb(1, 0, 0) : rgb(0, 0, 1));

					// update state based on whether we have found at least 
					// one intersection with controller ray
					/*if (data_ptr->intersection_points.size() == i)
						state[ci] = IS_NONE;
					else
						if (state[ci] == IS_NONE)
							state[ci] = IS_OVER;

					if (data_ptr->ipimg.size() == ii)
						state[ci] = IS_NONE;
					else
						if (state[ci] == IS_NONE)
							state[ci] = IS_OVER;*/

					state[ci] = IS_OVER;
				}
				post_redraw();
			}
			return true;
		}
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{
		if(data_ptr)
		if (data_ptr->rec_pose && test_manipulate) {
			if (data_ptr) {
				// scene won't be changed at the same time, dont do that 
				/*for (auto& tra:data_ptr->trackable_box_list) {
					tra.set_ori_direct_manipulation(quat(sin(t), sin(t), sin(t), sin(t)));
				}*/
				for (int i = 0; i < data_ptr->movable_box_rotations.size(); i++) {
					data_ptr->movable_box_rotations.at(i) = quat(sin(t), sin(t), sin(t), sin(t));
				}
				data_ptr->test_upload_to_trackable_list_ori();
			}
		}
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		if (data_ptr==nullptr)
			return;
		//if (!data_ptr->is_replay) 
		render_lines_for_controllers(ctx);
		render_random_boxes(ctx);

	}
	void gen_random_movable_boxes() {
		if (data_ptr==nullptr)
			return;
		float tw = 0.8f;
		float td = 0.8f;
		float th = 0.72f;
		float tW = 0.03f;
		int nr = 50;
		std::default_random_engine generator;
		std::uniform_real_distribution<float> distribution(0, 1);
		std::uniform_real_distribution<float> signed_distribution(-1, 1);
		for (size_t i = 0; i < nr; ++i) {
			float x = distribution(generator);
			float y = distribution(generator);
			vec3 extent(distribution(generator), distribution(generator), distribution(generator));
			extent += 0.01f;
			extent *= std::min(tw, td) * 0.1f;

			vec3 center(-0.5f * tw + x * tw, th + tW, -0.5f * td + y * td);
			data_ptr->movable_boxes.push_back(box3(-0.5f * extent, 0.5f * extent));
			data_ptr->movable_box_colors.push_back(rgb(distribution(generator), distribution(generator), distribution(generator)));
			data_ptr->movable_box_translations.push_back(center);
			quat rot(signed_distribution(generator), signed_distribution(generator), signed_distribution(generator), signed_distribution(generator));
			rot.normalize();
			data_ptr->movable_box_rotations.push_back(rot);

			// init trackable_box_list 
			// upload_to_trackable_list
			data_ptr->trackable_box_list.push_back(*(new trackable_box("box_"+std::to_string(i), box3(-0.5f * extent, 0.5f * extent))));
			data_ptr->trackable_box_list.back().set_position_orientation_write(
				data_ptr->movable_box_translations[i], data_ptr->movable_box_rotations[i]);
			data_ptr->trackable_box_list.back().set_color(data_ptr->movable_box_colors[i]);
		}
	}
	/// call this 
	void render_random_boxes(context& ctx) {
		if (data_ptr) {
			if (data_ptr->movable_boxes.size()) {
				cgv::render::box_renderer& renderer = cgv::render::ref_box_renderer(ctx);
				renderer.set_render_style(data_ptr->movable_style);
				renderer.set_box_array(ctx, data_ptr->movable_boxes);
				renderer.set_color_array(ctx, data_ptr->movable_box_colors);
				renderer.set_translation_array(ctx, data_ptr->movable_box_translations);
				renderer.set_rotation_array(ctx, data_ptr->movable_box_rotations);
				renderer.render(ctx, 0, data_ptr->movable_boxes.size());
			}
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
	/// overload the create gui method
	void create_gui()
	{
	}
};
