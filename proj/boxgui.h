#pragma once

#include <cgv/base/node.h>
#include <cgv/signal/rebind.h>
#include <cgv/base/register.h>
#include <cgv/gui/event_handler.h>
#include <cgv/math/ftransform.h>
#include <cgv/utils/scan.h>
#include <cgv/utils/options.h>
#include <cgv/gui/provider.h>
#include <cgv/render/drawable.h>
#include <cgv/render/shader_program.h>
#include <cgv/render/frame_buffer.h>
#include <cgv/render/attribute_array_binding.h>
#include <cgv_gl/box_renderer.h>
#include <cgv_gl/sphere_renderer.h>
#include <libs\cgv_gl\gl\gl_tools.h>
#include <cgv/signal/signal.h>

///@ingroup VR
///@{

/**@file
   example plugin for vr usage
*/

// these are the vr specific headers

class boxgui_label_texture : public cgv::render::render_types {
public:
	std::string label_text;
	int label_font_idx;
	bool label_upright;
	float label_size;
	rgb label_color;
	bool label_outofdate; // whether label texture is out of date
	unsigned label_resolution; // resolution of label texture
	cgv::render::texture label_tex; // texture used for offline rendering of label
	cgv::render::frame_buffer label_fbo; // fbo used for offline rendering of label

	boxgui_label_texture(std::string label, float font_size) {
		label_text = label;
		label_font_idx = 0;
		label_upright = false;
		label_size = font_size; // 150
		label_color = rgb(1, 1, 1);
		label_outofdate = true; // whether label texture is out of date
		label_resolution = 256; // resolution of label texture
		// 
		// this two will be initialized during init_frame() func.: label_tex; label_fbo;
	}
};

class boxgui_button :
	public cgv::render::render_types, 
	public cgv::signal::tacker // still not work after adding this 
{
public:
	vec3 ext;
	vec3 local_trans;
	quat local_rot;
	vec3 trans;
	quat rot;
	rgb color;
	bool use_label;
	bool use_icon;
	//std::string labelstr;
	boxgui_label_texture* labeltex;
	unsigned int icontex;
	std::string iconpath;
	bool has_intersec;
	bool is_static;
	float radio = 19.0f / 4.0f;
	int group;
	void* f;
	// may not work due to the loss of tracker information 
	cgv::signal::signal<int> button_clicked; 
	boxgui_button(vec3 e, vec3 t, quat r, rgb col, int which_group) {
		ext = e;
		trans = local_trans = t;
		rot = local_rot = r;
		color = col;
		use_label = false;
		use_icon = false;
		//labelstr = "";
		labeltex = nullptr;
		icontex = -1;
		iconpath = "";
		has_intersec = false;
		is_static = false;
		group = which_group;
	}
	void set_label(std::string str, float font_size) {
		//labelstr = str;
		labeltex = new boxgui_label_texture(str, font_size);
		use_label = true;
	}
	void front_append_info_to_label(std::string str) {
		//labelstr = str + "\n" + labelstr;
		if (labeltex)
			labeltex->label_text = str + "\n" + labeltex->label_text;
	}
	void set_icon(std::string path) {
		iconpath = path;
		double aspect_ptr;
		bool has_alpha_ptr;
		double aspect;
		icontex = cgv::render::gl::read_image_to_texture(iconpath, false, &aspect, &has_alpha_ptr);
		use_icon = true;
	}
	void set_static() {
		is_static = true;
	}
	std::string get_label_text() {
		return labeltex->label_text;
	}
};
