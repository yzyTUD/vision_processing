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

using namespace cgv::base;
using namespace cgv::reflect;
using namespace cgv::gui;
using namespace cgv::signal;
using namespace cgv::render;

class vis_kit_meshes :
	public base,    // base class of all to be registered classes
	public provider, // is derived from tacker, which is not necessary as base anymore
	public event_handler, // necessary to receive events
	public drawable // registers for drawing with opengl
{
public:
	typedef cgv::media::mesh::simple_mesh<float> mesh_type;
	cgv::render::rounded_cone_render_style cone_style;
	sphere_render_style sphere_style;
	cgv::render::mesh_render_info mesh_info;
	mesh_type M;
	rgb surface_color = rgb(0.4);
	CullingMode cull_mode;
	bool have_new_mesh = false;
	bool show_vertices = false;
	bool show_face = false;
	bool show_wireframe = false;

public:
	/// initialize rotation angle
	vis_kit_meshes()
	{
		//connect(get_animation_trigger().shoot, this, &vis_kit_meshes::timer_event);
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
		return "vis_kit_meshes";
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
	bool handle(event& e)
	{
		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	bool init(context& ctx)
	{
		ref_sphere_renderer(ctx, 1);
		ref_rounded_cone_renderer(ctx, 1);
		return true;
	}
	void destruct(context& ctx)
	{
		ref_rounded_cone_renderer(ctx, -1);
		ref_sphere_renderer(ctx, -1);
	}
	void init_frame(context& ctx) {
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
			post_recreate_gui();
			have_new_mesh = false;

			sphere_style.radius = float(0.05 * sqrt(M.compute_box().get_extent().sqr_length() / M.get_nr_positions()));
			sphere_style.surface_color = rgb(0.8f, 0.3f, 0.3f);

			cone_style.radius = 0.5f * sphere_style.radius;
			cone_style.surface_color = rgb(0.6f, 0.5f, 0.4f);
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
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		if (show_vertices) {
			sphere_renderer& sr = ref_sphere_renderer(ctx);
			sr.set_render_style(sphere_style);
			sr.set_position_array(ctx, M.get_positions());
			if (M.has_colors())
				sr.set_color_array(ctx, *reinterpret_cast<const std::vector<rgb>*>(M.get_color_data_vector_ptr()));
			sr.render(ctx, 0, M.get_nr_positions());
		}
		if (show_wireframe) {
			rounded_cone_renderer& cr = ref_rounded_cone_renderer(ctx);
			cr.set_render_style(cone_style);
			if (cr.enable(ctx)) {
				mesh_info.draw_wireframe(ctx);
				cr.disable(ctx);
			}
		}
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
	}
	void finish_draw(context& ctx) {
		if (show_face) {
			glDisable(GL_CULL_FACE);
			// choose a shader program and configure it based on current settings
			shader_program& prog = ctx.ref_surface_shader_program(true);
			mesh_info.draw_all(ctx, false, true);
		}
	}
	/////////////////////////////////////////////////////////////////////
	void load_mesh() {
		M.clear();
		std::string f = cgv::gui::file_open_dialog("Open", "Meshes:*");
		M.read(f);
		have_new_mesh = true;
		show_face = true;
		show_wireframe = true;
		post_redraw();
	}
	void randomize_coordinates() {
		M.randomize_texcoordi();
	}
	void generate_demo_surface() {
		// vec3(1,1,0),vec3(1,-1,0),vec3(-1,1,0),vec3(-1,-1,-1)
		std::vector<vec3> Pnts;
		Pnts.push_back(vec3(1, 1, 0));
		Pnts.push_back(vec3(1, -1, 0));
		Pnts.push_back(vec3(-1, 1, 0));
		Pnts.push_back(vec3(-1, -1, -1));

		M.clear();

		M.start_face();
		for (auto p : Pnts) {
			int vi = M.new_position(p);
			M.new_corner(vi);
		}

		M.compute_vertex_normals();

		have_new_mesh = true;		
		post_redraw();
	}
	void generate_dini_surface()
	{
		int n, m;
		float a, b;
		float lb, ub;

		a = 1;
		b = 0.2f;
		lb = 0.01f;
		ub = 2.0f;
		n = m = 20;

		M.clear();
		// allocate per vertex colors of type rgb with float components
		M.ensure_colors(cgv::media::CT_RGB, (n + 1) * m);

		for (int i = 0; i <= n; ++i) {
			float y = (float)i / n;
			float v = (ub - lb) * y + lb;
			for (int j = 0; j < m; ++j) {
				float x = (float)j / m;
				float u = float(4.0f * M_PI) * x;
				// add new position to the mesh (function returns position index, which is i*m+j in our case)
				int vi = M.new_position(vec3(a * cos(u) * sin(v), a * sin(u) * sin(v), a * (cos(v) + log(tan(0.5f * v))) + b * u));
				// set color
				M.set_color(vi, rgb(x, y, 0.5f));
				// add quad connecting current vertex with previous ones
				if (i > 0) {
					int vi = ((i - 1) * m + j);
					int delta_j = -1;
					if (j == 0)
						delta_j = m - 1;
					M.start_face();
					M.new_corner(vi);
					M.new_corner(vi + m);
					M.new_corner(vi + m + delta_j);
					M.new_corner(vi + delta_j);
				}
			}
		}
		// compute surface normals at mesh vertices from quads
		M.compute_vertex_normals();

		have_new_mesh = true;
		post_redraw();
	}
	///////////////////////////////////////////////////////////////////////
	void write_to_obj() {
		std::string f = cgv::gui::file_save_dialog("Save", "Meshes:*");
		M.write_with_materials(f);
	}
	void create_gui()
	{
		if (begin_tree_node("Meshing IO", show_face, false, "level=3")) {
			connect_copy(add_button("read_mesh")->click, rebind(this, &vis_kit_meshes::load_mesh));
			connect_copy(add_button("write_mesh")->click, rebind(this, &vis_kit_meshes::write_to_obj));
		}
		if (begin_tree_node("Meshing Processing", show_vertices, false, "level=3")) {
			connect_copy(add_button("generate_demo_surface")->click, cgv::signal::rebind(this, &vis_kit_meshes::generate_demo_surface));
			connect_copy(add_button("generate_dini_surface")->click, cgv::signal::rebind(this, &vis_kit_meshes::generate_dini_surface));
			connect_copy(add_button("randomize_texcoordi")->click, rebind(this, &vis_kit_meshes::randomize_coordinates));
		}
		if (begin_tree_node("Meshing Rendering", cull_mode, false, "level=3")) {
			add_member_control(this, "cull mode", cull_mode, "dropdown", "enums='none,back,front'");
			add_member_control(this, "show_vertices", show_vertices, "check");
			add_member_control(this, "show_face", show_face, "check");
			add_member_control(this, "show_wireframe", show_wireframe, "check");
			add_member_control(this, "", surface_color, "", "w=42");
		}

	}
};
