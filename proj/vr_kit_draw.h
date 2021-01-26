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
#include <fstream>
#include <cgv/defines/quote.h>

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
#include <vr_kit_intersection.h>

class vr_kit_draw :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
public:
	//@name tube graph for 3d drawing, trajectory visualization 
	//@{	
	/// vertex data structure with position, radius and color attributes
	struct vertex {
		vec3  position;
		float radius;
		rgba  color;
	};
	/// edge data structure with indices of two vertices that it connects together
	struct edge {
		uint32_t origin_vi;
		uint32_t target_vi;
	};
protected:
	/// graph vertices
	std::vector<vertex> vertices;
	/// graph edges
	std::vector<edge> edges;
	// render style for rendering vertices as spheres
	cgv::render::sphere_render_style srs;
	/// render style for rendering edges as rounded cones
	cgv::render::rounded_cone_render_style rcrs;
public:
	/// add a new vertex
	uint32_t add_vertex(const vertex& v) { uint32_t vi = uint32_t(vertices.size()); vertices.push_back(v); ++nr_vertices; return vi; }
	/// add a new edge
	uint32_t add_edge(const edge& e) { uint32_t ei = uint32_t(edges.size()); edges.push_back(e); ++nr_edges; return ei; }
	/// return number of vertices
	uint32_t get_nr_vertices() const { return (uint32_t)vertices.size(); }
	/// writable access to vertex
	vertex& ref_vertex(uint32_t vi) { return vertices[vi]; }
	/// readonly access to vertex
	const vertex& get_vertex(uint32_t vi) const { return vertices[vi]; }
	/// return number of edges
	uint32_t get_nr_edges() const { return (uint32_t)edges.size(); }
	/// writable access to edge
	edge& ref_edge(uint32_t ei) { return edges[ei]; }
	/// readonly access to edge
	const edge& get_edge(uint32_t ei) const { return edges[ei]; }
	//@}

	//@name scene management
	//@{
	/// path to be scanned for drawing files
	std::string draw_file_path;
	/// vector of drawing file names
	std::vector<std::string> draw_file_names;
	/// index of current scene
	int current_drawing_idx;
	/// number of vertices in current scene to be shown in UI
	uint32_t nr_vertices;
	/// number of edges in current scene to be shown in UI
	uint32_t nr_edges;
	/// label index to show statistics
	uint32_t li_stats;
	/// labels to show help on controllers
	uint32_t li_help[2];
	/// 
	bool use_data_dir = true;
	/// call this 
	bool enable_drawing = false;
	///
	bool render_enable_drawing = true;
	/// set this
	vr_view_interactor* vr_view_ptr;
	///
	vr::vr_kit_state* state_ptr;
	///
	void set_vr_view_ptr(vr_view_interactor* p) {
		vr_view_ptr = p;
		state_ptr = (vr::vr_kit_state*)p->get_current_vr_state();
	}
	/// clear the current drawing
	void clear_drawing()
	{
		vertices.clear();
		edges.clear();
		nr_vertices = 0;
		nr_edges = 0;
		update_member(&nr_vertices);
		update_member(&nr_edges);
	}
	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return rh.reflect_member("draw_file_path", draw_file_path);
	}
	/// generate a new drawing file name
	std::string get_new_draw_file_name() const
	{
		int i = (int)draw_file_names.size() - 1;
		std::string file_name;
		do {
			++i;
			file_name = "trajectory_";
			std::string s = cgv::utils::to_string(i);
			while (s.length() < 5)
				s = std::string("0") + s;
			file_name += s + ".tj";
		} while (cgv::utils::file::exists(draw_file_path + "/" + file_name));
		return file_name;
	}
	/// read drawing from file
	bool read_trajectory()
	{
		std::string f = cgv::gui::file_open_dialog("Open", "trajectory files:*");
		std::ifstream is(f);
		if (is.fail())
			return false;
		clear_drawing();

		vertex V;
		edge E;
		while (!is.eof()) {
			char buffer[2048];
			is.getline(&buffer[0], 2048);
			std::string line(buffer);
			if (line.empty())
				continue;
			std::stringstream ss(line, std::ios_base::in);
			char c;
			ss.get(c);
			switch (toupper(c)) {
			case 'V':
				ss >> V.position >> V.radius >> V.color;
				if (!ss.fail())
					vertices.push_back(V);
				break;
			case 'E':
				ss >> E.origin_vi >> E.target_vi;
				if (!ss.fail())
					edges.push_back(E);
				break;
			}
		}

		nr_vertices = (uint32_t)vertices.size();
		nr_edges = (uint32_t)edges.size();
		update_member(&nr_vertices);
		update_member(&nr_edges);
		post_redraw();
		return !is.fail();
	}
	/// write_trajectory to file
	bool write_trajectory()
	{
		std::string f;
		if(use_data_dir)
			f = cgv::base::ref_data_path_list()[0] + "/mocap_trajectory/" + get_new_draw_file_name();
		else
			f = cgv::gui::file_save_dialog("Open", "trajectory files:*");
		std::ofstream os(f);
		if (os.fail())
			return false;
		for (size_t vi = 0; vi < vertices.size(); ++vi)
			os << "v " << vertices[vi].position << " " << vertices[vi].radius << " " << vertices[vi].color << std::endl;
		for (size_t ei = 0; ei < edges.size(); ++ei)
			os << "e " << edges[ei].origin_vi << " " << edges[ei].target_vi << std::endl;
		return true;
	}
	//@}
protected:
	/// different drawing modes
	enum DrawMode {
		DM_POINT,    // only draw points
		DM_LINE,     // draw points connected with lines
		DM_COLORIZE  // change color of points
	};
	DrawMode draw_mode[2];
	/// per controller radius used to draw with touch pad center
	float draw_radius[2];
	/// per controller a draw color
	rgb   draw_color[2];
	/// members used for color adjustment
	bool in_color_selection[2];
	vec3 color_selection_ref[2];
	rgb last_color[2];
	/// distance of drawing point from controller origin
	float draw_distance;
	/// threshold for new vertex creation in measured in meters
	float creation_threshold;
	/// parameters to map trigger to radius
	float min_trigger;
	float min_radius;
	float max_radius;
	/// distance of point p to line through l0 and l1 
	static float distance(const vec3& p, const vec3& l0, const vec3& l1)
	{
		vec3 dl = l1 - l0;
		vec3 dp = p - l0;
		float lambda = dot(dp, dl) / dot(dl, dl);
		if (lambda > 0.0f && lambda < 1.0f)
			return length(dp + lambda * dl);
		return std::min(length(dp), length(p - l1));
	}
private:
	// per controller cache of previously drawn vertices, may overwrite later varibles 
	int32_t prev[2];
	int32_t prev_prev[2];
	int32_t prev_prev_prev[2];
	// per controller whether we are drawing 
	bool   drawing[2];
	// per controller last used radius
	float  last_radius[2];

	bool in_radius_adjustment[2];
	float initial_radius[2];
	float initial_y[2];

	/// transform point with pose to lab coordinate system 
	vec3 compute_lab_draw_position(const float* pose, const vec3& p)
	{
		return mat34(3, 4, pose) * vec4(p, 1.0f);
	}
	/// transform default draw point with pose to lab coordinate system 
	vec3 compute_lab_draw_position(const float* pose)
	{
		return compute_lab_draw_position(pose, vec3(0.0f, 0.0f, -draw_distance));
	}
	/// check newly tracked position and add new vertex if necessary
	void consider_vertex(int ci, const vec3& p, double time, float radius)
	{
		//if (!scene_ptr)
		//	return;
		// manage radius
		if (radius == -1.0f)
			radius = last_radius[ci];
		else
			last_radius[ci] = radius;

		// when we start drawing, just add new vertex
		if (prev[ci] == -1) {
			prev[ci] = add_vertex({ p, radius, draw_color[ci] });
			// std::cout << " starting" << std::endl;
		}
		else {
			// otherwise check if we can update prev vertex
			auto& v_prev = ref_vertex(prev[ci]);
			float dist = length(v_prev.position - p);
			// first check if new ball encloses previous or previous encloses new ball
			if (dist + v_prev.radius < radius ||
				dist + radius < v_prev.radius) {
				v_prev.position = p;
				v_prev.radius = radius;
				// std::cout << " inout" << std::endl;
			}
			else {
				// otherwise compute prediction
				bool no_update = true;
				vec3  p_pred = v_prev.position;
				float r_pred = v_prev.radius;
				if (prev_prev[ci] != -1) {
					const auto& v_prev_prev = get_vertex(prev_prev[ci]);
					// check for direction reversal
					vec3 d_pred = v_prev.position - v_prev_prev.position;
					vec3 d = p - v_prev.position;
					if (dot(d_pred, d) >= 0.0f) {
						no_update = false;
						p_pred = v_prev_prev.position;
						r_pred = v_prev_prev.radius;
						if (prev_prev_prev[ci] != -1) {
							const auto& v_prev_prev_prev = get_vertex(prev_prev_prev[ci]);
							vec3 d_pred = v_prev_prev.position - v_prev_prev_prev.position;
							float l_pred_sqr = dot(d_pred, d_pred);
							if (l_pred_sqr > 1e-8f) {
								vec3 d = p - v_prev_prev.position;
								float lambda = dot(d_pred, d) / l_pred_sqr;
								if (lambda < 0)
									lambda = 0;
								p_pred = v_prev_prev.position + lambda * d_pred;
								r_pred = v_prev_prev.radius + lambda * (v_prev_prev.radius - v_prev_prev_prev.radius);
							}
						}
					}
				}
				// and check whether this is not good enough
				if (length(p - p_pred) > creation_threshold ||
					abs(radius - r_pred) > creation_threshold) {

					prev_prev_prev[ci] = prev_prev[ci];
					prev_prev[ci] = prev[ci];
					prev[ci] = add_vertex({ p, radius, draw_color[ci] });
					// std::cout << " new" << std::endl;

					if (draw_mode[ci] == DM_LINE)
						add_edge({ uint32_t(prev_prev[ci]), uint32_t(prev[ci]) });

				}
				else {
					if (!no_update) {
						v_prev.position = p;
						v_prev.radius = radius;
					}
				}
			}
		}
		post_redraw();
	}
	/// simplest approach to colorize scene vertices
	void colorize_vertex(int ci, const vec3& p, float radius)
	{
		//if (!scene_ptr)
		//	return;
		for (uint32_t vi = 0; vi < get_nr_vertices(); ++vi) {
			if ((get_vertex(vi).position - p).length() < radius)
				ref_vertex(vi).color = draw_color[ci];
		}
	}
	/// helper function called when we start drawing
	void start_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		drawing[ci] = true;
		if (draw_mode[ci] != DM_COLORIZE) {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose);
			consider_vertex(ci, p, time, radius);
		}
		else {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose, vec3(0, 0, -50 * draw_radius[ci]));
			colorize_vertex(ci, p, 10 * draw_radius[ci]);
		}
	}
	/// helper function called when we continue drawing
	void continue_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		if (draw_mode[ci] != DM_COLORIZE) {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose);
			consider_vertex(ci, p, time, radius);
		}
		else {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose, vec3(0, 0, -50 * draw_radius[ci]));
			colorize_vertex(ci, p, 10 * draw_radius[ci]);
		}
	}
	/// helper function called when we stop drawing
	void stop_drawing(int ci, const vr::vr_kit_state& state, double time, float radius = -1.0f)
	{
		if (draw_mode[ci] != DM_COLORIZE) {
			vec3 p = compute_lab_draw_position(state.controller[ci].pose);
			consider_vertex(ci, p, time, radius);
			prev[ci] = prev_prev[ci] = prev_prev_prev[ci] = -1;
		}
		drawing[ci] = false;
	}
public:
	/// initialize rotation angle
	vr_kit_draw()
	{
		current_drawing_idx = 0;
		nr_vertices = 0;
		nr_edges = 0;
		draw_file_path = QUOTE_SYMBOL_VALUE(INPUT_DIR) "../data";

		draw_mode[0] = draw_mode[1] = DM_LINE;
		in_color_selection[0] = in_color_selection[1] = false;
		in_radius_adjustment[0] = in_radius_adjustment[1] = false;
		draw_radius[0] = draw_radius[1] = 0.01f;
		draw_color[0] = rgb(1.0f, 0.3f, 0.7f);
		draw_color[1] = rgb(0.7f, 0.3f, 1.0f);
		draw_distance = 0.1f;
		creation_threshold = 0.002f;
		min_trigger = 0.03f;
		min_radius = 0.001f;
		max_radius = 0.03f;
		li_help[0] = li_help[1] = -1;

		drawing[0] = drawing[1] = false;
		prev[0] = prev_prev[0] = prev_prev_prev[0] = prev[1] = prev_prev[1] = prev_prev_prev[1] = -1;
		connect(get_animation_trigger().shoot, this, &vr_kit_draw::timer_event);
	}
	/// call this
	bool init(cgv::render::context& ctx)
	{
		cgv::render::ref_sphere_renderer(ctx, 1);
		cgv::render::ref_rounded_cone_renderer(ctx, 1);
		return true;
	}
	/// 
	void on_set(void* member_ptr)
	{
		update_member(member_ptr);
		post_redraw();
	}
	/// self reflection allows to change values in the config file
	/*bool self_reflect(reflection_handler& rh)
	{
		return true;
	}*/
	/// return the type name of the class derived from base
	std::string get_type_name() const
	{
		return "vr_kit_draw";
	}
	/// show statistic information
	void stream_stats(std::ostream& os)
	{
	}
	/// show help information
	void stream_help(std::ostream& os)
	{
	}
	/// overload to handle events, return true if event was processed
	/// call this
	bool handle(event& e)
	{
		if (e.get_kind() == cgv::gui::EID_POSE)
		{
			cgv::gui::vr_pose_event& vrpe = static_cast<cgv::gui::vr_pose_event&>(e);
			// check for controller pose events
			int ci = vrpe.get_trackable_index();
			if (ci != -1) {
				if (enable_drawing) {
					if (drawing[ci]) {
						float v = max_radius;
						last_radius[ci] = min_radius + (max_radius - min_radius) * (v - min_trigger) / (1.0f - min_trigger);
						//last_radius[ci] = min_radius;
						continue_drawing(ci, vrpe.get_state(), vrpe.get_time(), last_radius[ci]);
					}
					else
						start_drawing(ci, vrpe.get_state(), vrpe.get_time(), min_radius);
				}
				else if (drawing[ci])
					stop_drawing(ci, vrpe.get_state(), vrpe.get_time(), min_radius);
			}
		}
		//if (e.get_kind() == cgv::gui::EID_THROTTLE) {
		//	auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
		//	int ci = te.get_controller_index();
		//	float v = te.get_value();
		//	bool d = v >= min_trigger;
		//	if (d) {
		//		if (drawing[ci]) {
		//			last_radius[ci] = min_radius + (max_radius - min_radius) * (v - min_trigger) / (1.0f - min_trigger);
		//			continue_drawing(ci, te.get_state(), te.get_time(), last_radius[ci]);
		//		}
		//		else
		//			start_drawing(ci, te.get_state(), te.get_time(), min_radius);
		//	}
		//	else if (drawing[ci])
		//		stop_drawing(ci, te.get_state(), te.get_time(), min_radius);
		//}
		return false;
	}
	///
	void record_this_obj(std::string name, vec3 posi, quat ori) {

	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double t, double dt)
	{

	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		
	}
	///
	/// call this
	void render_trajectory(context& ctx) {
		if (!(vr_view_ptr && render_enable_drawing))
			return;
		state_ptr = (vr::vr_kit_state*)vr_view_ptr->get_current_vr_state();
		if (!state_ptr)
			return;
		auto& sr = cgv::render::ref_sphere_renderer(ctx);
		sr.set_render_style(srs);
		auto& rcr = cgv::render::ref_rounded_cone_renderer(ctx);
		rcr.set_render_style(rcrs);
		// draw vertex edge graph
		if (!vertices.empty()) {
			sr.set_position_array(ctx, &vertices.front().position, vertices.size(), sizeof(vertex));
			sr.set_radius_array(ctx, &vertices.front().radius, vertices.size(), sizeof(vertex));
			sr.set_color_array(ctx, &vertices.front().color, vertices.size(), sizeof(vertex));
			sr.render(ctx, 0, (GLsizei)vertices.size());
		}
		if (!edges.empty()) {
			rcr.set_position_array(ctx, &vertices.front().position, vertices.size(), sizeof(vertex));
			rcr.set_radius_array(ctx, &vertices.front().radius, vertices.size(), sizeof(vertex));
			rcr.set_color_array(ctx, &vertices.front().color, vertices.size(), sizeof(vertex));
			rcr.set_indices(ctx, &edges.front().origin_vi, 2 * edges.size());
			rcr.render(ctx, 0, (GLsizei)(2 * edges.size()));
		}
		// draw spheres that represent the pen
		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgb> C;
		for (int ci = 0; ci < 2; ++ci)
			if (draw_mode[ci] != DM_COLORIZE && state_ptr->controller[ci].status == vr::VRS_TRACKED) {
				P.push_back(compute_lab_draw_position(state_ptr->controller[ci].pose));
				R.push_back(drawing[ci] ? last_radius[ci] : draw_radius[ci]);
				C.push_back(draw_color[ci]);
			}
		if (!P.empty()) {
			sr.set_position_array(ctx, P);
			sr.set_radius_array(ctx, R);
			sr.set_color_array(ctx, C);
			sr.render(ctx, 0, (GLsizei)P.size());
		}
	}
	/// overload the create gui method
	void create_gui()
	{
	}
};
