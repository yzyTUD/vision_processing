#include "gl_surface_reconstructor_drawable.h"

bool gl_surface_reconstructor_drawable::handle(cgv::gui::event& e)
{
    return false;
}

void gl_surface_reconstructor_drawable::create_gui()
{
}

gl_surface_reconstructor_drawable::gl_surface_reconstructor_drawable()
{
}

gl_surface_reconstructor_drawable::gl_surface_reconstructor_drawable(const std::string& file_name)
{
}

bool gl_surface_reconstructor_drawable::self_reflect(cgv::reflect::reflection_handler& rh)
{
    return false;
}

void gl_surface_reconstructor_drawable::on_set(void* member_ptr)
{
}

bool gl_surface_reconstructor_drawable::read(const std::string& file_name)
{
    return false;
}

const point_cloud* gl_surface_reconstructor_drawable::get_point_cloud() const
{
    return nullptr;
}

void gl_surface_reconstructor_drawable::set_focus_point(int vi, unsigned int j, bool flip_nml)
{
}

void gl_surface_reconstructor_drawable::set_view()
{
}

void gl_surface_reconstructor_drawable::on_set_receiver()
{
}

void gl_surface_reconstructor_drawable::set_point_cloud(point_cloud* _pc)
{
}

const char* gl_surface_reconstructor_drawable::get_name() const
{
    return nullptr;
}

void gl_surface_reconstructor_drawable::render_box(const Box& box)
{
}

void gl_surface_reconstructor_drawable::draw_vertex_color(unsigned int vi) const
{
}

void gl_surface_reconstructor_drawable::draw_edge_color(unsigned int vi, unsigned int j, bool is_symm, bool is_start) const
{
}

void gl_surface_reconstructor_drawable::draw_debug_vertex(unsigned int vi)
{
}

void gl_surface_reconstructor_drawable::draw_normal(unsigned int vi, unsigned int vj, unsigned int vk) const
{
}

void gl_surface_reconstructor_drawable::draw_boxes()
{
}

void gl_surface_reconstructor_drawable::draw_points()
{
}

void gl_surface_reconstructor_drawable::draw_normals()
{
}

void gl_surface_reconstructor_drawable::draw_graph()
{
}

void gl_surface_reconstructor_drawable::draw_face_corners()
{
}

void gl_surface_reconstructor_drawable::draw_point_labels()
{
}

void gl_surface_reconstructor_drawable::draw_debug()
{
}

void gl_surface_reconstructor_drawable::draw_consistent()
{
}

void gl_surface_reconstructor_drawable::draw_region_growing()
{
}

void gl_surface_reconstructor_drawable::init_frame(cgv::render::context& ctx)
{
}

void gl_surface_reconstructor_drawable::draw(cgv::render::context& ctx)
{
}

void gl_surface_reconstructor_drawable::stream_stats(std::ostream& os)
{
}

void gl_surface_reconstructor_drawable::stream_help(std::ostream& os)
{
}
