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

#include "halfedgemesh.h"
#include "aabb_tree.h"
#include "mesh_utils.h"
#include "triangle.h"
#include <ray_intersection.h>
#include "vis_kit_datastore.h"

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
	vis_kit_data_store_shared* data_ptr = nullptr;

	//
	typedef cgv::media::mesh::simple_mesh<float> mesh_type;
	cgv::render::rounded_cone_render_style cone_style;
	sphere_render_style sphere_style, sphere_hidden_style;
	rgb surface_color = rgb(0.4);
	CullingMode cull_mode;
	ColorMapping color_mapping;
	IlluminationMode illumination_mode;
	bool use_texture = false;

	///
	cgv::render::mesh_render_info mesh_info;
	cgv::render::shader_program mesh_prog;
	mesh_type M;
	box3 B;
	bool have_new_mesh = false;

	cgv::render::mesh_render_info MI_smoothing;
	cgv::render::shader_program mesh_prog_smoothing;
	mesh_type smoothingMesh;
	box3 B_smoothing;
	bool have_new_smoothingMesh = false;

	/// 
	bool show_vertices = false;
	bool show_face = false;
	bool show_wireframe = false;
	bool show_bounding_box = false;

	/// HEMesh support 
	HE_Mesh* he;
	AabbTree<triangle> aabb_tree;
	std::vector<box3> boxes;
	cgv::render::box_render_style movable_style;	
	// transformation
	mat3 mesh_rotation_matrix;	
	vec3 mesh_translation_vector;

	float rx = 0;
	float ry = 0;
	float rz = 0;
	float sy = 0.5;
	float oy = 0.0;
	float h = 0;

	// picking support 
	std::vector<vec3> picking_points; 
	std::vector<rgb> picking_points_color; 
	sphere_render_style sphere_style_picking;
	float picking_points_radius = 0.05;

	// face creation 
	bool is_creating_new_face = false;

public:
	/// initialize rotation angle
	vis_kit_meshes()
	{
		//connect(get_animation_trigger().shoot, this, &vis_kit_meshes::timer_event);
		sphere_style_picking.radius = picking_points_radius;
	}
	/// call me 
	void set_data_ptr(vis_kit_data_store_shared* d_ptr) {
		data_ptr = d_ptr;
	}
	/// 
	void on_set(void* member_ptr) { update_member(member_ptr); post_redraw(); }
	/// self reflection allows to change values in the config file
	bool self_reflect(reflection_handler& rh){return true;}
	/// return the type name of the class derived from base
	std::string get_type_name() const { return "vis_kit_meshes"; }
	/// show statistic information
	void stream_stats(std::ostream& os) {}
	/// show help information
	void stream_help(std::ostream& os) {}
	/// overload to handle events, return true if event was processed
		// demostration: M.compute_ray_mesh_intersections(vec3(0,1,0));
		// scs: 
	// return true for accept 
	bool check_picking_point_exists(vec3 new_p) {
		for (auto p : picking_points) {
			float dist = (new_p - p).length();
			if (dist < picking_points_radius)
				return false;
		}
		return true;
	}

	int check_any_sphere_intersection_current_righthand() {
		vec3 ori = data_ptr->cur_right_hand_posi;
		vec3 dir = normalize(data_ptr->cur_off_right);

		for (int i = 0; i < picking_points.size(); i++) {
			vec3 L = picking_points.at(i) - ori;
			float tc = dot(L, dir);
			vec3 tc_vec = dir * tc;
			if (tc >= 0.0) {
				float d = (tc_vec - L).length();
				if (d <= picking_points_radius)
					return i;
			}
		}
		return -1;
	}

	bool handle(event& e)
	{
		if (data_ptr == nullptr)
			return false;
		if (e.get_kind() == cgv::gui::EID_KEY) {
			cgv::gui::vr_key_event& vrke = static_cast<cgv::gui::vr_key_event&>(e);
			//if (vrke.get_key() == vr::VR_DPAD_UP) // 
			//{
			//	if (vrke.get_action() == cgv::gui::KA_PRESS) {
			//		if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { // 
			//			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nAdjestPointSize"))) { // 
			//				picking_points_radius += 0.001f;
			//				sphere_style_picking.radius = picking_points_radius;
			//			}
			//		}
			//	}
			//}
			//if (vrke.get_key() == vr::VR_DPAD_DOWN) // 
			//{
			//	if (vrke.get_action() == cgv::gui::KA_PRESS) {
			//		if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { // 
			//			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nAdjestPointSize"))) { // 
			//				picking_points_radius -= 0.001f;
			//				sphere_style_picking.radius = picking_points_radius;
			//			}
			//		}
			//	}
			//}
			if (vrke.get_key() == vr::VR_MENU) {
				if (vrke.get_action() == cgv::gui::KA_PRESS) { // 
					if (vrke.get_controller_index() == data_ptr->right_rgbd_controller_index) { // 
						if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nFaceCreation"))) //
						{
							if (!is_creating_new_face) {
								M.start_face();
								is_creating_new_face = true;
							}
							else { // stop creating new faces, prepare for rendering 
								is_creating_new_face = false;

								//// add picking points to simple mesh: too much conputation here 
								//for (auto p:picking_points) {
								//	M.new_position(p);
								//}

								// clear picking point color for visual feedback 
								for (auto& pc : picking_points_color)
									pc = rgb(0, 0, 1);

								// render the mesh 
								if (M.get_nr_positions() > 0) {
									M.compute_vertex_normals();
									M.compute_face_normals();

									have_new_mesh = true;

									// rendering 
									show_vertices = false; // we have points already 
									show_face = true;
									show_wireframe = true;
									cull_mode = CullingMode::CM_OFF;

									post_redraw();
								}
							}
						}
					
						if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nSaveMesh"))) {
							// todo: add timestemp as file name 
							M.write(data_ptr->data_dir + "/generated_mesh_vr_meshing_tool_"+ data_ptr->get_timestemp_for_filenames() +".obj");
						}
					}
				}
			}

			return true;
		}
		if (e.get_kind() == cgv::gui::EID_STICK) {
			cgv::gui::vr_stick_event& vrse = static_cast<cgv::gui::vr_stick_event&>(e);
			if (vrse.get_action() == cgv::gui::SA_TOUCH) { // event 
				if (vrse.get_controller_index() == data_ptr->right_rgbd_controller_index) { // controller 
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nPickingPoints"))) { // selection
						// check exists 
						vec3 cur_picking_point = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right;
						if (check_picking_point_exists(cur_picking_point)) {
							picking_points.push_back(cur_picking_point);
							M.new_position(cur_picking_point);
							picking_points_color.push_back(rgb(0, 0, 1));
						}
					}
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nFaceCreation"))) { // selection
						int intersec_sphere_idx = check_any_sphere_intersection_current_righthand();
						std::cout << "intersect idx: " << intersec_sphere_idx << std::endl;
						if (intersec_sphere_idx != -1) {
							M.new_corner(intersec_sphere_idx);
							// visual feedback 
							picking_points_color.at(intersec_sphere_idx) = rgb(1, 0, 0);
						}
					}
				}
				return true;
			}

			if (vrse.get_action() == cgv::gui::SA_MOVE) { // event 
				if (vrse.get_controller_index() == data_ptr->right_rgbd_controller_index) { // controller 
					if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nAdjestPointSize"))) { // selection
						if (vrse.get_y() > 0) {
							picking_points_radius += 0.001f;
							sphere_style_picking.radius = picking_points_radius;
						}
						else {
							picking_points_radius -= 0.001f;
							sphere_style_picking.radius = picking_points_radius;
						}
					}
				}
			}

			if (vrse.get_action() == cgv::gui::SA_RELEASE) {
				if (vrse.get_controller_index() == data_ptr->right_rgbd_controller_index) { // controller -> selection 

				}
				return true;
			}
		}

		//if (e.get_kind() == cgv::gui::EID_THROTTLE) {
		//	auto& te = static_cast<cgv::gui::vr_throttle_event&>(e);
		//	bool d = (te.get_value() == 1); 
		//	
		//	return true;
		//}

		return false;
	}
	/// declare timer_event method to connect the shoot signal of the trigger
	void timer_event(double, double dt)
	{
	}
	///
	bool init(context& ctx)
	{
		illumination_mode = IlluminationMode::IM_ONE_SIDED;
		color_mapping = CM_COLOR;
		if (!mesh_prog.build_program(ctx, "mesh.glpr", true))
			abort();
		if (!mesh_prog_smoothing.build_program(ctx, "mesh.glpr", true))
			abort();
		ref_sphere_renderer(ctx, 1);
		ref_rounded_cone_renderer(ctx, 1);
		return true;
	}
	///
	void destruct(context& ctx)
	{
		ref_rounded_cone_renderer(ctx, -1);
		ref_sphere_renderer(ctx, -1);
	}
	///
	void init_frame(context& ctx) {
		if (have_new_mesh) {
			// auto-compute mesh normals if not available
			if (!M.has_normals())
				M.compute_vertex_normals();
			// [re-]compute mesh render info
			mesh_info.destruct(ctx);
			mesh_info.construct(ctx, M);
			// bind mesh attributes to standard surface shader program
			mesh_info.bind(ctx, mesh_prog, true);
			mesh_info.bind_wireframe(ctx, ref_rounded_cone_renderer(ctx).ref_prog(), true);
			// ensure that materials are presented in gui
			post_recreate_gui();
			have_new_mesh = false;

			sphere_style.radius = float(0.05 * sqrt(M.compute_box().get_extent().sqr_length() / M.get_nr_positions()));
			sphere_style.surface_color = rgb(0.8f, 0.3f, 0.3f);

			cone_style.radius = 0.5f * sphere_style.radius;
			cone_style.surface_color = rgb(0.6f, 0.5f, 0.4f);

			sphere_hidden_style.radius = sphere_style.radius;
			// focus view on new mesh
			/*clipped_view* view_ptr = dynamic_cast<clipped_view*>(find_view_as_node());
			if (view_ptr) {
				box3 box = M.compute_box();
				view_ptr->set_scene_extent(box);
				view_ptr->set_focus(box.get_center());
				view_ptr->set_y_extent_at_focus(box.get_extent().length());
			}*/
		}
		if (have_new_smoothingMesh) {
			if (!smoothingMesh.get_positions().empty()) {
				if (!smoothingMesh.has_normals())
					smoothingMesh.compute_vertex_normals();
				MI_smoothing.destruct(ctx);
				MI_smoothing.construct(ctx, smoothingMesh);
				MI_smoothing.bind(ctx, mesh_prog_smoothing, true);
				MI_smoothing.bind_wireframe(ctx, cgv::render::ref_rounded_cone_renderer(ctx).ref_prog(), true);
			}
			have_new_smoothingMesh = false;
		}
	}
	/// setting the view transform yourself
	void draw(context& ctx)
	{
		if (mesh_info.is_constructed()) {
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
				shader_program& prog = mesh_prog;
				prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
				prog.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
				//
				prog.set_uniform(ctx, "use_texture", use_texture);
				//prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
				// set default surface color for color mapping which only affects 
				// rendering if mesh does not have per vertex colors and color_mapping is on
				prog.set_attribute(ctx, prog.get_color_index(), surface_color);

				// render the mesh from the vertex buffers with selected program
				mesh_info.draw_all(ctx, false, true);

				// recover opengl culling mode
				if (is_culling)
					glEnable(GL_CULL_FACE);
				else
					glDisable(GL_CULL_FACE);
				glCullFace(cull_face);
			}
			// draw bounding box
			if (show_bounding_box) {
				cgv::render::box_wire_renderer& box_render = cgv::render::ref_box_wire_renderer(ctx);
				box_render.init(ctx);
				visit_tree(aabb_tree.Root());
				box_render.set_box_array(ctx, boxes);
				//box_render.render(ctx, 0, boxes.size());
				if (box_render.validate_and_enable(ctx)) {
					glDrawArrays(GL_POINTS, 0, (GLsizei)boxes.size());
					box_render.disable(ctx);
				}
			}
		}
		// render smoothing mesh
		if (MI_smoothing.is_constructed())
		{
			// choose a shader program and configure it based on current settings
			cgv::render::shader_program& prog_sm = mesh_prog_smoothing;

			//prog.set_uniform(ctx, "culling_mode", (int)cull_mode);
			prog_sm.set_uniform(ctx, "map_color_to_material", (int)color_mapping);
			//prog.set_uniform(ctx, "illumination_mode", (int)illumination_mode);
			// set default surface color for color mapping which only affects 
			// rendering if mesh does not have per Vector colors and color_mapping is on
			if (prog_sm.get_color_index() != -1)
				prog_sm.set_attribute(ctx, prog_sm.get_color_index(), rgb(0.0f, 0.0f, 0.0f));
			MI_smoothing.draw_all(ctx, false, true);
		}

		// interactive meshing: picking points 
		render_picking_points(ctx);

		if (data_ptr != nullptr) {
			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nAdjestPointSize"))) { // selection 
				render_a_sphere_on_righthand(ctx);
			}
			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nPickingPoints"))) { // selection 
				render_a_sphere_on_righthand(ctx);
			}
			if (data_ptr->check_roulette_selection(data_ptr->get_id_with_name("Meshing\nFaceCreation"))) { // selection 
				if(is_creating_new_face) // as visual feedback 
					render_line_on_righthand(ctx);
			}
		}

	}

	void render_picking_points(cgv::render::context& ctx) {
		if (picking_points.size() > 0) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(sphere_style_picking);
			sr.set_position_array(ctx, picking_points);
			sr.set_color_array(ctx, picking_points_color);
			sr.render(ctx, 0, picking_points.size());
		}
	}

	void render_line_on_righthand(cgv::render::context& ctx) {
		cgv::render::rounded_cone_render_style rounded_cone_style;

		std::vector<vec3> P;
		std::vector<float> R;
		std::vector<rgb> C;

		vec3 s = data_ptr->cur_right_hand_posi;
		vec3 e = data_ptr->cur_right_hand_posi + data_ptr->cur_off_right * 10;
		float r = 0.002f;
		rgb c = rgb(1, 0, 0);

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
	
	// toimprove: effec.
	void render_a_sphere_on_righthand(cgv::render::context& ctx) {
		std::vector<vec3> points;
		std::vector<rgb> point_colors;
		points.push_back(data_ptr->cur_right_hand_posi + data_ptr->cur_off_right);
		point_colors.push_back(rgb(0, 0, 1));
		if (points.size()) {
			auto& sr = cgv::render::ref_sphere_renderer(ctx);
			sr.set_render_style(sphere_style_picking);
			sr.set_position_array(ctx, points);
			sr.set_color_array(ctx, point_colors);
			sr.render(ctx, 0, points.size());
		}
	}
	///
	void finish_draw(context& ctx) {
		//if (show_face) {
		//	glDisable(GL_CULL_FACE);
		//	// choose a shader program and configure it based on current settings
		//	shader_program& prog = ctx.ref_surface_shader_program(true);
		//	mesh_info.draw_all(ctx, false, true);
		//}
	}
	////////////////////////////////////////////////////////////////////////
	/// simple mesh related operations 
	void load_mesh() {
		M.clear();
		std::string f = cgv::gui::file_open_dialog("Open", "Meshes:*");
		M.read(f);
		have_new_mesh = true;
		show_face = true;
		show_wireframe = true;
		//compute_coordinates();
		post_redraw();
	}

	void apply_transform_for_test() {
		/*quat rz = quat(vec3(0, 0, 1), 25 * M_PI / 180);
		quat rx = quat(vec3(1, 0, 0), -90 * M_PI / 180);
		quat r_align = rx * rz;*/

		mat3 r;
		r.identity();
		//r_align.put_matrix(r);
		M.transform(r,vec3(0,0.6,0));
		have_new_mesh = true;
		post_redraw();
	}
	///
	void randomize_coordinates() {
		M.randomize_texcoordi();
	}	
	///
	void compute_coordinates() {
		M.compute_texcoordi(rx,ry,rz,vec3(0,sy,oy));
		have_new_mesh = true;
		post_redraw();
	}
	/// not used 
	void compute_coordinates_with_rot_correction(quat alignq, vec3 alignt) {
		/*M.compute_texcoordi(rx, ry, h, alignq, alignt);
		have_new_mesh = true;
		post_redraw();*/
	}
	///
	void generate_demo_surface() {
		M.clear();                    
		std::vector<vec3> Pnts;
		int vi = 0;
		Pnts.push_back(vec3(1, 1, 0));
		Pnts.push_back(vec3(1, -1, 0));
		Pnts.push_back(vec3(-1, -1,0 ));
		Pnts.push_back(vec3(-1, 1, 0));
		
		M.new_position(Pnts[0]);
		M.new_position(Pnts[1]);
		M.new_position(Pnts[2]);
		M.new_position(Pnts[3]);

		M.start_face();
		M.new_corner(0);
		M.new_corner(1);
		M.new_corner(2);
		M.new_corner(3);

		M.compute_vertex_normals();
		M.compute_face_normals();

		have_new_mesh = true;		
		post_redraw();
	}
	///
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

	///////////////////// HEMesh helper functions ////////
	///push back the leaf node
	void visit_tree(AabbTree<triangle>::AabbNode* a)
	{
		if (a->is_leaf() == true)
		{
			boxes.push_back(a->get_box());
		}

		if (a->is_leaf() == false)
		{
			visit_tree(a->left_child());
			visit_tree(a->right_child());
		}
	}
	///
	HE_Mesh* generate_from_simple_mesh(mesh_type M) {
		auto newMesh = new HE_Mesh();

		if (M.get_positions().empty()) return nullptr; // mesh is empty, no conversion neccessary

		/// define index type
		typedef cgv::type::uint32_type idx_type;
		/// define index triple type
		typedef cgv::math::fvec<idx_type, 3> vec3i;

		// first (re)compute the normals to make sure they are calculated
		M.compute_vertex_normals();

		auto originalPositions = M.get_positions();
		std::vector<unsigned int> triangleBuffer;
		std::vector<idx_type> vectorIndices;
		std::vector<vec3i> uniqueTriples;

		M.merge_indices(vectorIndices, uniqueTriples, false, false);
		M.extract_triangle_element_buffer(vectorIndices, triangleBuffer);

		for (auto i = 0; i < triangleBuffer.size(); i += 3) {
			unsigned int vectorAIndex = uniqueTriples.at(triangleBuffer.at(i))[0];
			unsigned int vectorBIndex = uniqueTriples.at(triangleBuffer.at(i + 1))[0];
			unsigned int vectorCIndex = uniqueTriples.at(triangleBuffer.at(i + 2))[0];

			// adding the 3 vectors
			auto vectorA = newMesh->AddVector(vectorAIndex, originalPositions.at(vectorAIndex));
			auto vectorB = newMesh->AddVector(vectorBIndex, originalPositions.at(vectorBIndex));
			auto vectorC = newMesh->AddVector(vectorCIndex, originalPositions.at(vectorCIndex));

			auto face = newMesh->AddFace();

			// generating 3 half edges per triangle
			auto halfEdgeC = newMesh->AddHalfEdge(vectorC, vectorA, face);
			auto halfEdgeB = newMesh->AddHalfEdge(vectorB, vectorC, face, halfEdgeC);
			auto halfEdgeA = newMesh->AddHalfEdge(vectorA, vectorB, face, halfEdgeB);

			// closing the loop
			halfEdgeC->next = halfEdgeA;
		}

		// construct boundaries
		for (auto edge_it : *newMesh->GetHalfEdges()) {
			if (edge_it->twin == nullptr)
				newMesh->AddBoundary(edge_it);
		}
		uniqueTriples.clear();
		triangleBuffer.clear();
		vectorIndices.clear();

		return newMesh;
	}
	///
	void read_mesh_and_construct_HEMesh() {
		mesh_type tmp;
		std::string f = cgv::gui::file_open_dialog("Open", "Meshes:*");
		tmp.read(f);
		if (tmp.get_nr_positions() > 0) {
			M.clear();
			if(he)
				he->~HE_Mesh();
			M = tmp;
			B = M.compute_box();
			int Vector_count = M.get_nr_positions();
			he = generate_from_simple_mesh(M);
			build_aabbtree_from_triangles(he, aabb_tree);
			float volume = mesh_utils::volume(he);
			float surface = mesh_utils::surface(he);
		}
		have_new_mesh = true;
		show_face = true;
		show_wireframe = true;
		show_bounding_box = true;
		post_redraw();
	}
	///
	void build_simple_mesh_from_HE() {
		M.clear();
		//build map from orginal index to new index of position for quicly searching
		//key: originalindex, value: new index 
		std::map<int, idx_type> indexmap;
		std::map<int, idx_type>::iterator it;
		//add vertices to new simple mesh
		for (auto v : *he->GetVertices()) {
			vec3 pos = v->position;
			//pos = local_to_global(pos);
			idx_type pos_idx = M.new_position(pos);
			indexmap.insert(std::make_pair(v->originalIndex, pos_idx));
		}

		//add faces to new simple mesh
		for (auto f : *he->GetFaces()) {
			std::vector<HE_Vertex*> vertices = he->GetVerticesForFace(f);
			//build a normal, later will be calculated again
			idx_type normal_idx = M.new_normal(vec3(1.0, 0.0, 0.0));
			//build a new face
			M.start_face();
			//through originalindex find the new index of position
			it = indexmap.find(vertices[0]->originalIndex);
			if (it != indexmap.end()) {
				M.new_corner(it->second, normal_idx);
			}
			it = indexmap.find(vertices[1]->originalIndex);
			if (it != indexmap.end()) {
				M.new_corner(it->second, normal_idx);
			}
			it = indexmap.find(vertices[2]->originalIndex);
			if (it != indexmap.end()) {
				M.new_corner(it->second, normal_idx);
			}
		}
		M.compute_vertex_normals();

		B = M.compute_box();
		//write the new simple mesh to a object
		//M.write("new.obj");
	}
	/// add rotation to matrix via 3x3 rotation matrix
	vec3 global_to_local(vec3 pos) {
		pos = transpose(mesh_rotation_matrix) * (pos - mesh_translation_vector);
		return pos;
	}
	/// returns pos in the local coordinate system
	vec3 local_to_global(vec3 pos) {
		pos = (mesh_rotation_matrix * pos) + mesh_translation_vector;
		return pos;
	}
	/// render path with lines, not finished 
	void drawpath(cgv::render::context& ctx, std::vector<vec3> path_list) {
		auto& prog = ctx.ref_default_shader_program();
		int ci = prog.get_color_index();
		rgb a = (1, 0, 0);
		std::vector<rgb> color_list;
		color_list.push_back(a);
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, prog.get_position_index(), path_list);
		cgv::render::attribute_array_binding::enable_global_array(ctx, prog.get_position_index());
		cgv::render::attribute_array_binding::set_global_attribute_array(ctx, ci, color_list);
		cgv::render::attribute_array_binding::enable_global_array(ctx, ci);

		glLineWidth(3);
		prog.enable(ctx);
		glDrawArrays(GL_LINES, 0, (GLsizei)path_list.size());
		prog.disable(ctx);
		cgv::render::attribute_array_binding::disable_global_array(ctx, prog.get_position_index());
		cgv::render::attribute_array_binding::disable_global_array(ctx, ci);
	}
	/// add a new face to the second simple_mesh "smoothingMesh" which is used to diplay the selected faces for smoothing
	void add_face_to_smoothingMesh(HE_Face* f) {
		std::cout << "add new face to smoothing mesh" << std::endl;
		// add new faces to simple mesh#
		std::vector<HE_Vertex*> Fvertices = he->GetVerticesForFace(f);
		//create a new face
		smoothingMesh.start_face();
		for (int j = 0; j < Fvertices.size(); j++) {
			//vec4 point = transformation_matrix * vec4(Fvertices[j]->position, 1.0);
			//vr_mesh_view::local_to_global
			vec3 point = Fvertices[j]->position;
			idx_type pos_idx = smoothingMesh.new_position(vec3(point.x(), point.y(), point.z()));
			idx_type normal_idx = smoothingMesh.new_normal(vec3(0.0f, -1.0f, 0.0f));
			// tell the mesh to save a new corner (vertex) with the position and normal given as indices
			smoothingMesh.new_corner(pos_idx, normal_idx);
		}
		//compute the normals again
		smoothingMesh.compute_vertex_normals();
		B_smoothing = smoothingMesh.compute_box();
		have_new_smoothingMesh = true;
		post_redraw();
	}
	
	void pick_face(vec3 p) {
		M.pick_face(p);
		have_new_mesh = true;
		post_redraw();
	}

	///////////////////// HEMesh usage ////////////////
	void test_ray_highlight_triangle() {
		ray_intersection::ray tes_ray = ray_intersection::ray(vec3(0), vec3(0,1,0));
		float t = 0.0;
		bool tt = ray_intersection::rayTreeIntersect(tes_ray, aabb_tree, t);
		if (tt) {
			HE_Face* face = ray_intersection::getIntersectedFace(tes_ray, he);
			std::vector<HE_Vertex*> vertices_of_face = he->GetVerticesForFace(face);
			add_face_to_smoothingMesh(face);
		}
	}
	
	void coordi_correction() {
		M.coordi_correction_leica();
		have_new_mesh = true;
		post_redraw();
	}
	/// smoothing of the whole mesh
	void apply_smoothing() {
		if (M.get_positions().empty()) return;
		// vector with new positions of vertices
		std::vector<vec3> newPositions;
		// calculation of the new positions of the vertices via averaging over the neighbor vertices
		for (HE_Vertex* v : *he->GetVertices()) {
			int number = 0;
			vec3 newpos = vec3(0, 0, 0);
			for (HE_Vertex* neighbor : he->GetNeighborVertices(v)) {
				number++;
				newpos += neighbor->position;
			}
			newpos /= number;
			newPositions.push_back(newpos);
		}

		// updating the positions of the vertices in the halfedge DS and simple_mesh
		int i = 0;
		for (HE_Vertex* v : *he->GetVertices()) {
			v->position = newPositions[i];
			// vec3 new_posi = ... to support mesh transformation 
			M.position(v->originalIndex) = vec3(newPositions[i].x(), newPositions[i].y(), newPositions[i].z());
			++i;
		}
		M.compute_vertex_normals();
		B = M.compute_box();
		have_new_mesh = true;
		//maybe here transform from local to global
		//
		post_redraw();
		//rebuild aabb
		build_aabbtree_from_triangles(he, aabb_tree);
	}
	/// additional 2 faces will be added, 1->3 faces, not finished 
	void tesselete_by_clicking() {
		vec3 new_origin, new_dir;
		// create ray
		ray_intersection::ray tes_ray = ray_intersection::ray(new_origin, new_dir);
		float t = 0.0;
		//check if the ray intersects with the aabb tree 
		if (ray_intersection::rayTreeIntersect(tes_ray, aabb_tree, t)) {

			vec3 tes_inter_point = ray_intersection::getIntersectionPoint(tes_ray, t);
			HE_Face* tes_face = ray_intersection::getIntersectedFace(tes_ray, he);
			//get three vertices in the tessellated face
			auto tes_point = he->GetVerticesForFace(tes_face);
			//add the intersected point to the simple mesh
			idx_type new_point_index = M.new_position(tes_inter_point);
			if (he->deleteFace(tes_face)) {
				std::cout << "Tessellation: Deleted face in half edge data structure." << std::endl;
				// add three faces to the original half edge mesh
				for (int i = 0; i < 3; i++) {
					unsigned int vectorAIndex = tes_point[i]->originalIndex;
					unsigned int vectorBIndex = tes_point[(i + 1) % 3]->originalIndex;
					unsigned int vectorCIndex = new_point_index;

					// adding the 3 vectors
					auto vectorA = he->AddVector(vectorAIndex, tes_point[i]->position);
					auto vectorB = he->AddVector(vectorBIndex, tes_point[(i + 1) % 3]->position);
					auto vectorC = he->AddVector(vectorCIndex, tes_inter_point);

					auto face = he->AddFace();

					// generating 3 half edges per triangle
					auto halfEdgeC = he->AddHalfEdge(vectorC, vectorA, face);
					auto halfEdgeB = he->AddHalfEdge(vectorB, vectorC, face, halfEdgeC);
					auto halfEdgeA = he->AddHalfEdge(vectorA, vectorB, face, halfEdgeB);

					// closing the loop
					halfEdgeC->next = halfEdgeA;
				}
				//build a new simple mesh from half edge data structure 
				//build_simple_mesh_from_HE();

				have_new_mesh = true;
				post_redraw();
				build_aabbtree_from_triangles(he, aabb_tree);
			}
			else {
				std::cout << "Tessellation: Face Deletion is not successful." << std::endl;
			}
		}
		else {
			std::cout << "Tessellation: No intersection." << std::endl;
		}
	}
	///
	void vertex_deletion() {
		vec3 new_origin;
		vec3 point_on_ray;
		vec3 new_point_on_ray;
		vec3 new_dir = new_point_on_ray - new_origin;
		HE_Vertex* intersectedVertex;

		ray_intersection::ray ray = ray_intersection::ray(new_origin, new_dir);
		float t = 0;

		//auto start = std::chrono::high_resolution_clock::now();
		//bool boxIntersection = ray_intersection::rayTreeIntersect(ray, aabb_tree, t);
		//auto stop = std::chrono::high_resolution_clock::now();
		//auto duration = std::chrono::duration<double>(stop - start);
		std::cout << "Duration using aabb_tree/bounding box ray intersection: " << std::endl;
		vec3 intersectionPoint = ray_intersection::getIntersectionPoint(ray, t);

		//std::cout << "boxIntersection: " << boxIntersection << std::endl;
		//std::cout << "Box Intersection t: " << t << std::endl;
		std::cout << "Intersection point: " << intersectionPoint << std::endl;

		HE_Face* f;
		bool ff = ray_intersection::getIntersectedFace_with_t(ray, he, t, f);
		std::vector<HE_Vertex*> vertices_of_face;
		if (ff) {
			vertices_of_face = he->GetVerticesForFace(f);
		}
		else {
			std::cout << "no intersecting face, no vertex deletion" << std::endl;
			return;
		}

		//std::vector<HE_Vertex*> vertices_of_mesh = *he->GetVertices();
		bool vertexIntersection = ray_intersection::vertexIntersection(intersectionPoint, vertices_of_face, intersectedVertex);

		if (vertexIntersection && intersectedVertex != nullptr) {
			std::cout << "Picked vertex: " << intersectedVertex << " with position: " << intersectedVertex->position << std::endl;
			std::vector<HE_Vertex*> neighbor_vertices = he->GetNeighborVertices(intersectedVertex);
			std::vector<HE_Face*> neighbor_faces = he->GetAdjacentFaces(intersectedVertex);
			//neighbor_vertices.push_back(neighbor_vertices[0]);

			//Neighbor vertices
			/*
			int i = 0;
			for (auto n : neighbor_vertices) {
				std::cout << "neighbor_vertices " << i << " data: " << n << std::endl;
				i++;
			}*/

			std::cout << "Number of halfEdges before deletion: " << (*he->GetHalfEdges()).size() << std::endl;
			//std::cout << "Number of halfEdges before deletion: " << (*he->GetHalfEdges()).size() << std::endl;
			//std::cout << "Number of faces before deletion: " << (*he->GetFaces()).size() << std::endl;
			for (int i = 0; i < neighbor_faces.size(); i++) {
				std::cout << i << std::endl;
				he->deleteFace(neighbor_faces[i]);
			}
			//std::cout << "Number of faces after deletion: " << (*he->GetFaces()).size() << std::endl;
			std::cout << "Number of halfEdges after deletion: " << (*he->GetHalfEdges()).size() << std::endl;

			//std::cout << "Number of vertices before deletion: " << (*he->GetVertices()).size() << std::endl;
			he->deleteVector(intersectedVertex);
			//std::cout << "Number of vertices after deletion: " << (*he->GetVertices()).size() << std::endl;

			//std::cout << "Number of halfEdges before addition: " << (*he->GetHalfEdges()).size() << std::endl;

			//This for loop creates suitable triangle faces and adds all the missing halfedges 
			for (int i = 0; i < neighbor_vertices.size() - 2; i++) {
				auto face = he->AddFace();

				//Adding halfedges
				auto newHalfEdge = he->AddHalfEdge(neighbor_vertices[0], neighbor_vertices[i + 2], face);
				auto newHalfEdge2 = he->AddHalfEdge(neighbor_vertices[i + 1], neighbor_vertices[0], face, newHalfEdge);
				auto newHalfEdge3 = he->AddHalfEdge(neighbor_vertices[i + 2], neighbor_vertices[i + 1], face, newHalfEdge2);
				newHalfEdge->next = newHalfEdge3;
			}
			//std::cout << "Number of halfEdges after addition: " << (*he->GetHalfEdges()).size() << std::endl;
			std::cout << "Number of faces after addition: " << (*he->GetFaces()).size() << std::endl;
			//Building the simple mesh takes a lot of time, around 4-5 seconds
			//build_simple_mesh_from_HE();
			have_new_mesh = true;
			post_redraw();
			build_aabbtree_from_triangles(he, aabb_tree);

			std::cout << "Vertex deleted!" << std::endl;
		}
		else
			std::cout << "No vertex intersection, vertex deletion couldn't be operated." << std::endl;
	}
	///
	void vertex_manipulate(HE_Vertex* vertex, vec3 pos, vec3 last_pos) {

		if (he->changeVertexPos(vertex, vertex->position + pos - last_pos)) {
			M.position(vertex->originalIndex) = M.position(vertex->originalIndex) + local_to_global(pos) - local_to_global(last_pos);
			//std::cout << "Vertex is manipulated." << std::endl;
			post_redraw();
		}
		else
			std::cout << "Vertex position couldn't be manipulated." << std::endl;
	}

	void compute_volume(){}

	void compute_surfaces(){}

	///////////////////////////////////////////////////////////////////////
	///
	void write_to_obj() {
		std::string f = cgv::gui::file_save_dialog("Save", "Meshes:*");
		M.write_with_materials(f);
	}
	///
	void create_gui()
	{
		if (begin_tree_node("Meshing IO", show_face, true, "level=3")) {
			connect_copy(add_button("read_mesh")->click, rebind(this, &vis_kit_meshes::load_mesh));
			//apply_transform_for_test
			connect_copy(add_button("apply_transform_for_test")->click, rebind(this, &vis_kit_meshes::apply_transform_for_test));
			connect_copy(add_button("read_mesh_HE")->click, rebind(this, &vis_kit_meshes::read_mesh_and_construct_HEMesh));
			connect_copy(add_button("write_mesh")->click, rebind(this, &vis_kit_meshes::write_to_obj));
		}
		if (begin_tree_node("Meshing Processing", show_vertices, true, "level=3")) {
			connect_copy(add_button("generate_demo_surface")->click, cgv::signal::rebind(this, &vis_kit_meshes::generate_demo_surface));
			connect_copy(add_button("generate_dini_surface")->click, cgv::signal::rebind(this, &vis_kit_meshes::generate_dini_surface));
			connect_copy(add_button("randomize_texcoordi")->click, rebind(this, &vis_kit_meshes::randomize_coordinates));
			connect_copy(add_button("compute_coordinates")->click, rebind(this, &vis_kit_meshes::compute_coordinates));
			add_member_control(this, "rx", rx, "value_slider", "min=-60;max=60;log=false;ticks=false;");
			add_member_control(this, "ry", ry, "value_slider", "min=-60;max=60;log=false;ticks=false;");
			add_member_control(this, "rz", rz, "value_slider", "min=-60;max=60;log=false;ticks=false;");
			add_member_control(this, "sy", sy, "value_slider", "min=-1;max=1;log=false;ticks=false;");
			add_member_control(this, "oy", oy, "value_slider", "min=-1;max=1;log=false;ticks=false;");
			
			connect_copy(add_button("apply_smoothing")->click, rebind(this, &vis_kit_meshes::apply_smoothing));
			//
			connect_copy(add_button("coordi_correction")->click, rebind(this, &vis_kit_meshes::coordi_correction));
			connect_copy(add_button("test_ray_highlight_triangle")->click, rebind(this, &vis_kit_meshes::test_ray_highlight_triangle));
		}
		if (begin_tree_node("Meshing Rendering", cull_mode, true, "level=3")) {
			add_member_control(this, "cull mode", cull_mode, "dropdown", "enums='none,back,front'");
			//use_texture
			add_member_control(this, "use_texture", use_texture, "check");
			add_member_control(this, "show_vertices", show_vertices, "check");
			add_member_control(this, "show_face", show_face, "check");
			add_member_control(this, "show_wireframe", show_wireframe, "check");
			add_member_control(this, "show_bounding_box", show_bounding_box, "check");
			add_member_control(this, "", surface_color, "", "w=42");
			if (begin_tree_node("color_mapping", color_mapping)) {
				align("\a");
				add_gui("color mapping", color_mapping, "bit_field_control",
					"enums='COLOR_FRONT=1,COLOR_BACK=2,OPACITY_FRONT=4,OPACITY_BACK=8'");
				align("\b");
				end_tree_node(color_mapping);
			}
			add_member_control(this, "illumination", illumination_mode, "dropdown", "enums='none,one sided,two sided'");
		}
	}
};
