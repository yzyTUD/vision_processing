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
#include <libs/cgv_gl/gl/mesh_render_info.h>
#include <libs/cgv_gl/gl/mesh_drawable.h>
#include <mesh_utils.h>
#include <libs/cgv_gl/gl/gl_tools.h>

class trackable : public cgv::render::render_types {
protected:
	std::string name;
	vec3 posi;
	quat orie;

	vec3 read_posi;
	quat read_quat;
	mat4 read_mat;
	rgb color;
	std::string time_stemp;
public:
	bool replay;
	trackable(std::string n) {
		name = n;
		replay = false;
	}
	std::string get_name() {
		return name;
	}
	void get_position_orientation(vec3& p,quat& q) {
		p = posi;
		q = orie;
	}
	void get_position_orientation_read(vec3& p, quat& q) {
		p = read_posi;
		q = read_quat;
	}
	void set_position_orientation_write(vec3 p, quat q) {
		posi = p;
		orie = q;
	}
	void set_ori_direct_manipulation(quat o) {
		orie = o;
	}
	void set_position_orientation_read(vec3 p, quat q) {
		read_posi = p;
		read_quat = q;
		read_mat = q.get_homogeneous_matrix();
	}
	void set_color(rgb c) {
		color = c;
	}
	rgb get_color() {
		return color;
	}
	void draw(context& ctx){}
};

class trackable_photo : public trackable {
public:
	box3 b;
	cgv::render::texture tex;
	trackable_photo(std::string n, box3 ori_b) :trackable(n) { b = ori_b; }
	void set_box(box3 read_b) {
		b = read_b;
	}
};

class trackable_box : public trackable {
public:
	box3 b;
	trackable_box(std::string n, box3 ori_b) :trackable(n) { b = ori_b; }
	void set_box(box3 read_b) {
		b = read_b;
	}
	box3 get_box() {
		return b;
	}

};
class trackable_mesh : public trackable {
public:
	cgv::render::mesh_render_info mesh_info;
	mesh_type mesh;
	std::string mesh_dir;

	bool have_new_mesh;
	trackable_mesh(std::string n,std::string m_dir):trackable(n) {
		mesh_dir = m_dir;
		mesh.read(mesh_dir);
		have_new_mesh = true;
	}
	void draw(context& ctx) {
		if (have_new_mesh) {
			//
			if (!mesh.has_normals())
				mesh.compute_vertex_normals();
			mesh_info.destruct(ctx);
			mesh_info.construct(ctx, mesh);
			mesh_info.bind(ctx, ctx.ref_surface_shader_program(true), true);
			have_new_mesh = false;
		}
		if (replay)
		if (mesh_info.is_constructed()) {
			glDisable(GL_CULL_FACE);
			shader_program& prog = ctx.ref_surface_shader_program(true);
			prog.set_uniform(ctx, "map_color_to_material", (int)cgv::render::ColorMapping::CM_COLOR);
			prog.set_attribute(ctx, prog.get_color_index(), rgb(0.4));
			//
			ctx.push_modelview_matrix();
			ctx.mul_modelview_matrix(translate4(read_posi) * read_mat);
			mesh_info.draw_all(ctx, false, true);
			ctx.pop_modelview_matrix();

			glEnable(GL_CULL_FACE);
		}
	}
};