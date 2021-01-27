
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
	mat4 read_mat;
public:
	trackable(std::string n) {
		name = n;
	}
	void get_position_orientation(vec3& p,quat& q) {
		p = posi;
		q = orie;
	}
	void set_position_orientation(vec3 p, quat q) {
		posi = p;
		orie = q;
	}
};

class trackable_photo : public trackable {
public:
	box3 b;
	cgv::render::texture tex;
	trackable_photo(std::string n) :trackable(n) {}
};

class trackable_box : public trackable {
public:
	box3 b;
	trackable_box(std::string n) :trackable(n) {}
};

class trackable_mesh : public trackable {
public:
	cgv::render::mesh_render_info MI_l_hand;
	mesh_type M_l_hand;
	std::string mesh_dir;
	trackable_mesh(std::string n,std::string m_dir):trackable(n) {
		mesh_dir = m_dir;
	}
};