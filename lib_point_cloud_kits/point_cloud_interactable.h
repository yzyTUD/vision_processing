#pragma once

#include <cgv/base/group.h>
#include <cgv/gui/provider.h>
#include <cgv/gui/event_handler.h>
#include <cgv/gui/trigger.h>
#include <cgv/base/register.h>
#include "gl_point_cloud_drawable.h"
#include "ann_tree.h"
#include "neighbor_graph.h"
#include "normal_estimator.h"
#include "ICP.h"
#include "vr_kit_image_renderer.h"
#include <queue>

#include "lib_begin.h"

enum PointCloudChangeEvent
{
	PCC_NEW_POINT_CLOUD        = 0x0001,
	PCC_POINTS                 = 0x0002,
	PCC_POINTS_RESIZE          = 0x0003,
	PCC_POINTS_MASK            = 0x0003,
							   
	PCC_COMPONENTS_CREATE      = 0x0004,
	PCC_COMPONENTS_RESIZE      = 0x0008,
	PCC_COMPONENTS_DESTRUCT    = 0x000C,
	PCC_COMPONENTS_MASK        = 0x000C,
							   
	PCC_NORMALS_CREATE         = 0x0010,
	PCC_NORMALS                = 0x0020,
	PCC_NORMALS_DESTRUCT       = 0x0030,
	PCC_NORMALS_MASK           = 0x0030,
							   
	PCC_COLORS_CREATE          = 0x0040,
	PCC_COLORS                 = 0x0080,
	PCC_COLORS_DESTRUCT        = 0x00C0,
	PCC_COLORS_MASK            = 0x00C0,
							   
	PCC_TEXCOORDS_CREATE       = 0x0100,
	PCC_TEXCOORDS              = 0x0200,
	PCC_TEXCOORDS_DESTRUCT     = 0x0300,
	PCC_TEXCOORDS_MASK         = 0x0300,
							   
	PCC_PIXCOORDS_CREATE       = 0x0400,
	PCC_PIXCOORDS              = 0x0800,
	PCC_PIXCOORDS_DESTRUCT     = 0x0C00,
	PCC_PIXCOORDS_MASK         = 0x0C00,

	PCC_NEIGHBORGRAPH_CREATE   = 0x1000,
	PCC_NEIGHBORGRAPH          = 0x2000,
	PCC_NEIGHBORGRAPH_DESTRUCT = 0x3000,
	PCC_NEIGHBORGRAPH_MASK     = 0x3000,

	PCC_WEIGHTS                = 0x4000

};

class pointHistoryEntry {
public:
	int point_index;
	int from_face_id;
	int to_face_id;
	int timestamp;
	//
};

// the second float value indicates the priority, it can be distance, curvature...
// dequeue lowest float value first by default 
const int ID = 0, DIST = 1, CURVATURE = 2;
typedef std::tuple<int, float, float> point_priority_mapping;
// Structure of the operator
// overloading for comparison
struct LowSecComp {
	constexpr bool operator()(
		point_priority_mapping const& a,
		point_priority_mapping const& b)
		const noexcept
	{
		return std::get<DIST>(a) > std::get<DIST>(b); // choose b // impl compare here 
	}
};

// a wrapper priority queue and use a hash set to keep track of the queue.
//class non_redundant_priority_queue {
//public:
//	non_redundant_priority_queue() {}
//
//	void push(int pid, float accu_dist, float curr_curvature) {
//		if (!contains(pid)) {
//			pq_.push(std::make_tuple(pid, accu_dist, curr_curvature));
//			set_.emplace(pid);
//		}
//	}
//	void pop() {
//		if (!empty()) {
//			point_priority_mapping top = pq_.top();
//			set_.erase(std::get<ID>(top)); // only the first value of the pair is recorded in a set 
//			pq_.pop(); 
//		}
//	}
//	point_priority_mapping top() { return pq_.top(); }
//	point_priority_mapping front() { return pq_.top(); }
//	bool contains(int item) { return set_.find(item) != set_.end(); }
//	bool empty() const { return set_.empty(); }
//	void clear() { 
//		std::priority_queue<point_priority_mapping, std::vector<point_priority_mapping>, LowSecComp> empty_pq_; 
//		pq_ = empty_pq_; 
//		set_.clear(); 
//	}
//
//private:
//	std::priority_queue<point_priority_mapping, std::vector<point_priority_mapping>, LowSecComp> pq_;
//	std::set<int> set_;
//};

/** the point cloud view adds a gui to the gl_point_cloud_drawable_base and adds
    some basic processing like normal computation as well as some debug rendering
	of the neighbor graph*/
class CGV_API point_cloud_interactable :
	public cgv::base::group,             // integration into the cgv::base::base hierarchy necessary for proper registration of instances, group type supports adding children 
	public gl_point_cloud_drawable,      // manages a point cloud and provides implementation of the cgv::render::drawable interface to render a point cloud
	public cgv::gui::event_handler,      // allows to handle key, mouse and vr events (under development) through the handle(event) method
	public cgv::gui::provider,           // allows to provide a gui through the create_gui() method
	public cgv::base::argument_handler   // allows to handle program arguments through the handle_args(args) method
{
public:

	/*point cloud IO */
	/// path of last opened or saved file
	std::string data_path;
	///
	std::string file_dir;
	/// file name without path nor extension of current file
	std::string file_name;
	/// helper member for updating the file name without overwriting the current one. This is needed for concatenation of file names when appending them
	std::string new_file_name;
	/// directory_name is used to support reading all point clouds from a directory by setting directory_path in the config file
	std::string directory_name;
	/// print_pc_information
	void print_pc_information();
	/// update data_path and file_name members from full file name
	void update_file_name(const std::string& ffn, bool append = false);
	/// save current point cloud to file with name fn
	bool save(const std::string& fn);
	///
	void write_pc_to_file();
	///
	void write_pc_to_file_with_given_dir(const std::string f);
	/// open a new point cloud
	bool open(const std::string& fn);
	/// open a new point cloud by reading all point cloud files in given directory
	bool open_directory(const std::string& dn);
	/// open and append a new point cloud by reading file with name fn
	bool open_and_append(const std::string& fn);
	/// open or append depending on event that triggered this function
	bool open_or_append(cgv::gui::event& e, const std::string& file_name);
	///
	bool reading_from_raw_scan = false;
	///
	bool read_pc_with_dialog(bool append);
	/// reading options 
	bool read_cgvcad_with_dialog();
	/// reading options 
	bool read_pc_with_dialog_queue(bool append);
	/// reading options 
	bool read_pc_subsampled_with_dialog();
	/// store a copy of the original point cloud 
	void store_original_pc();
	/// flag that tells whether reading a point cloud will append it to current or replace current
	bool do_append;
	/// whether to automatically set the view after reading new points
	bool do_auto_view;
	/// whether to automatically compute normals after point reading 
	bool compute_normal_after_read = false;

	/*reading camera positions: .campose file  */
	///
	std::vector<vr_kit_image_renderer> image_renderer_list;
	///
	bool read_pc_campose(cgv::render::context& ctx, quat initialcamq);
	///
	bool check_valid_pc_and_campose();
	///
	void apply_further_transformation(int which, quat q, vec3 t);
	///
	void align_leica_scans_with_cgv();
	/// control varibles 
	bool render_camposes = false;

	/*point generation */
	/// generate points
	bool generate_pc_hemisphere();
	/// generate points
	void generate_pc_unit_torus();
	/// generate points
	void generate_pc_random_sphere();
	/// generate points
	void generate_pc_init_sphere();
	/// generate points
	void generate_pc_unit_cylinder();
	/// generate points
	void generate_testing_plane();
	/// generate a cube 
	bool generate_pc_cube();

	/*vr point addition */
	/// generate points with in the quad we are holding 
	void spawn_points_in_the_handhold_quad(quat controllerq, vec3 controllert, vec3 quadextent);
	/// put point cloud to table 
	void auto_scale_after_read_points();
	/// hwo many points are expected to be added
	int num_of_points_to_be_added = 100;

	/* support for point copying */
	///
	point_cloud to_be_copied_pointcloud;
	///
	void mark_all_points_and_push_to_tmp_pointcloud_test(Pnt p, float r, int ignore_id);
	///
	void mark_points_and_push_to_tmp_pointcloud(Pnt p, float r, int ignore_id);

	/*point scale (M)*/
	///
	void scale_points_to_desk();
	
	/*point sampling (M) */
	/// down sample a point cloud 
	void downsampling(int step, int num_of_points_wanted, int which_strategy);
	/// down sample a point cloud automatically
	void auto_downsampling();
	/// 
	void supersampling_within_clips(std::vector<Pnt> positions, std::vector<Dir> dirs);
	///
	void supersampling_with_bbox(box3 range_as_box);
	///
	void restore_supersampling();
	///
	void render_with_fullpc();
	/// selective subsampling within given point type, with a global adjustable ratio 
	void selective_subsampling_cpu();
	/// selective_subsampling_radio
	float selective_subsampling_radio = 0.5;

	/* direct marking, correction (M) */
	/// make sure vector size: face_id, point_visited
	void prepare_marking();
	/// mark with controllers 
	void mark_face_id_with_controller(Pnt p, float r, int objctive);
	/// mark with controllers 
	void mark_topo_id_with_controller(Pnt p, float r, int objctive);
	/// marking test 
	void marking_test_mark_all_points_as_given_group(int objective);
	/// quick point deletion: with a clipping plane 
	void mark_points_with_clipping_plane(Pnt p, Nml plane_normal, int objctive);

	/*histore support, undo support (M)*/
	/// history recording 
	std::stack<std::vector<pointHistoryEntry>> point_marking_history;
	/// history recording 
	int history_indexer = 0;
	/// support step back 
	void new_history_recording();
	///
	void reset_last_marking_non_processed_part(int which_is_marked_and_not_used);
	///
	void reset_last_marked_points();
	/// 
	void step_back_last_selection();
	/// 
	void step_forward_selection();

	/*VR ICP*/
	/// a quick test 
	void update_scan_index_visibility_test(); // typically, we update them directly with controller 
	/// just a test, set src = 1, target = 0
	void set_src_and_target_scan_idx_as_test();
	/// extract point clouds, all regions 
	void extract_point_clouds_for_icp();
	/// inner function, extract marked regions to point clouds, which will be used for ICP 
	void extract_point_clouds_for_icp_marked_only();
	/// inner usage 
	void perform_icp_and_acquire_matrices();
	/// entry point
	void perform_icp_given_four_pair_points();
	/// apply matrices to the pc varible and move the point cloud, access points 
	void apply_register_matrices_for_the_original_point_cloud();
	/// entry point 
	void perform_icp_manual_clicking();
	/// step back 
	void drop_last_registration();
	/// only render src point cloud and disable others 
	void render_select_src_cloud_only();
	/// only render target point cloud and disable others 
	void render_select_target_cloud_only();
	/// render both and disable others
	void render_both_src_target_clouds();
	/// deprecated 
	void highlight_last_pc();
	/// deprecated
	void reset_highlighted_last_pc();
	/// perform ICP algo. with given source and target clouds subsampled, deprecated
	void register_with_subsampled_pcs(point_cloud& _pc);
	/// perform ICP algo. with given source and target clouds  
	void register_cur_and_last_pc_if_present();

	/* Master Thesis Related */
	/*Feature Computation*/	
	/// signed version 
	void compute_principal_curvature_signed();
	///
	void colorize_with_computed_curvature_signed();
	///
	void ep_compute_principal_curvature_and_colorize_signed();
	/// unsigned version 
	void compute_principal_curvature_unsigned();
	///
	void colorize_with_computed_curvature_unsigned();
	///
	void ep_compute_principal_curvature_and_colorize_unsigned();
	/// used in common 
	void fill_curvature_structure();
	/// helper function: recolor for vis 
	void ep_force_recolor();
	/// helper function
	void print_curvature_computing_info();
	/// helper function: compute a threshold automatically
	void auto_cluster_kmeans();

	const float gui2real_scale = 1e-6;
	const float real2gui_scale = 1e6;

	/* Point Classification based on Interactive Region Growing - */
	/// seed representation 
	std::vector<std::priority_queue<point_priority_mapping, 
		std::vector<point_priority_mapping>, LowSecComp>> queue_for_regions; 
	///
	std::vector<std::priority_queue<point_priority_mapping,
		std::vector<point_priority_mapping>, LowSecComp>> suspend_queue_for_regions;
	/// 
	std::vector<int> seed_for_regions;
	/// 
	std::vector<int> loaded_seeds_for_regions;
	///
	float max_accu_dist = 0;
	///
	float max_dist = 0;
	///
	int points_grown = 0;
	///
	int bkp_points_grown = 0;
	///
	std::vector<int> knn;
	/// reset seeds, and face_id and topo_id
	void prepare_grow(bool read_from_file);
	/// 
	void extract_neighbours();
	void rerender_seeds();
	/// clear prev seed 
	void clear_previous_queue(int prev_pid, int which_group);
	/// add seed to queue given group, others are not changed 
	void add_seed_to_queue(int which_group);
	/// collect marked points to queue, add seeds for the region growing 
	void init_region_growing_by_collecting_group_and_seeds_vr(int curr_face_selecting_id);
	/// grow one step, check_nml is not used 
	bool grow_one_step_bfs(bool final_grow, int which_group);
	///
	void region_growing();
	///
	void show_num_of_points_per_region();
	///
	void submit_face();
	///
	void resume_queue();
	///	
	void record_current_state_before_sync_grow();
	///
	void undo_sync_grow();
	/// opposite to submit face, we may want to undo current grow 
	void undo_curr_region(int curr_region);
	///
	void scale_model();
	///
	void grow_curr_region(int curr_region);
	///
	void sync_grow();
	///
	void grow_one_region(int gi);
	/// deprecated, not good to keep an other thread running 
	void do_region_growing_timer_event(double t, double dt);
	/// reset, not used 
	void reset_queue_with_seeds();
	/// reset, not used 
	void reset_region_growing_seeds();
	/// save to file 
	void record_seed_for_regions(std::string fn);
	///
	void clear_seed_for_regions();
	/// load only 
	void load_seed_for_regions(std::string fn);
	/// read back from file with the following file format: size of the vector, content 
	void recover_seed_for_regions(std::string fn);
	/// automatic seed finding 
	int find_next_seed_in_low_curvature_area();
	///  not used 
	void init_region_growing_by_setting_group_and_seeds(int growing_group, std::queue<int> picked_id_list);
	/// check if all points growed, deprecated
	bool all_points_growed();
	/// ensure that all points are growed, deprecated 
	void grow_one_step_bfs_ensure();
	/// not used 
	void mark_points_inside_selection_tool(Pnt p, float r, bool confirmed, int objctive);
	/// build tree ds for acc picking of the target and source pc
	bool assign_pc_and_ensure_source_tree_ds(point_cloud& _pc);
	/// deprecated, too complex 
	void ensure_point_selection();
	/// deprecated
	void ensure_point_selection_pc();
	/// deprecated
	void subsampling_source(Pnt& posi, float& radii, bool confirmed);
	/// deprecated
	void collect_to_subsampled_pcs();
	/// deprecated
	void fill_subsampled_pcs_with_cur_pc_as_a_test();
	/// deprecated
	void subsampling_target(Pnt& posi, float& radii, bool confirmed);
	/// not used 
	bool marked = false;
	/// if allow auto region growing, deprecated 
	bool do_region_growing_directly = false;
	/// if check nmls when growing, not used  
	bool region_grow_check_normals = true;
	///
	float normal_threshold = 0.6;
	/// not used 
	bool can_sleep = false;
	/// if the points marked by controller will be add to growing queue as seed 
	bool add_to_seed = true;
	/// critical section start/ stop
	bool can_parallel_edit = true;
	/// pause the growing, exit the thread (can not and not good to keep it run )
	bool pause_growing = false;
	/// how many points will be growed before sleep, not used, use 100 just 
	int steps_per_event_as_speed = 200;
	/// latency after 100 points growed 
	int growing_latency = 0; // ms
	///
	std::chrono::duration<double> Elapsed_knn;
	///
	enum growing_mode {
		// already implemented
		ACCU_DISTANCE_BASED,
		SEED_DISTANCE_BASED,
		UNSIGNED_MEAN_CURVATURE_BASED,
		DISTANCE_AND_MEAN_CURVATURE_BASED,
		STOP_ON_BOUNDARY,

		// todo 
		NORMAL_BASED
	}gm;
	///
	/*std::string mode_defs = "enums='ACCU_DISTANCE_BASED;SEED_DISTANCE_BASED;
		NORMAL_BASED;CURVATURE_BASED;UNSIGNED_MEAN_CURVATURE_BASED_0'";*/
	std::vector<std::thread*> growing_thread_pool;
	/// loop all points to check is too slow 
	bool check_the_queue_and_stop = false;
	/// ignore high curvature points, sometime searching radius is too large 
	bool ignore_high_curvature_regions = true;
	/// 
	std::vector<int> num_of_knn_used_for_each_group;
	///
	std::vector<int> num_of_points_curr_region;
	///
	bool decrease_searching_radius_on_high_curvature = true;
	///
	//bool final_grow = false;
	///
	bool is_residual_grow = false;
	///
	bool is_synchronous_growth = false;
	///
	bool use_property_scale = false;
	///
	int minimum_searching_neighbor_points = 10;
	///
	float model_scale = 1;
	///
	float accu_model_scale = 1;
	///
	float last_model_scale = 1;
	///
	mat4 model_translation;
	/// 
	mat4 inv_model_translation;
	///
	/*backup grow parameters */
	std::vector<int> bkp_face_id; // from point_selection, write to .scan file  
	/// mark if current point already visited 
	std::vector<bool> bkp_point_visited; // point_visited
	/// mark if current point in queue 
	std::vector<bool> bkp_point_in_queue;
	/// binded with the above one, indicates which group it belongs to 
	std::vector<int> bkp_point_in_queue_which_group;
	/// 
	//additionally, queue should be restored, but it is easy 


	/* Fine-Grained Point Classification */
	/// thw quality of the boundaries depends on region growing steps, how good faces are marked 
	void point_classification();
	///  quality depends on the prev. steps 
	void face_extraction();
	/// do some region growing and extract to global ds 
	void corner_extraction();
	/// do some region growing and extract to global ds 
	void edge_extraction();
	/// entry point, batch operation 
	void extract_all();

	/* Topological Information Extraction*/
	/// currently done in point_cloud

	/* Model Fitting */
	/// set points and indices for a demo surface 
	void fitting_render_control_points_test();
	///
	void build_connectivity_graph_fitting_and_render_control_points() {
		point_classification();
		extract_all();
		pc.make_explicit();
		pc.fit_all();
	}

	/*point cleaning paper */
	//void direct_buffer_loading();

public:
	int src_scan_idx = 1;
	int target_scan_idx = 0;
	int icp_iterations = 1;
	int icp_samples = 50;
	///
	point_cloud pc_src;
	point_cloud pc_target;
	point_cloud_types::Mat rmat;
	point_cloud_types::Dir tvec;
	point_cloud S, Q;
	std::vector<vec3> feature_points_src; // visualization 
	std::vector<rgb> feature_point_colors_src;
	std::vector<vec3> feature_points_target;
	std::vector<rgb> feature_point_colors_target;
	cgv::render::sphere_render_style srs_icp_feature_points;
	//
	std::vector<vec3> icp_clicking_points_src; // visualization 
	std::vector<rgb> icp_clicking_point_colors_src;
	std::vector<vec3> icp_clicking_points_target;
	std::vector<rgb> icp_clicking_point_colors_target;
	//
	point_cloud crs_srs_pc; // tmp source and target pcs 
	point_cloud crs_tgt_pc;
	point_cloud oripc;
	point_cloud pc_last;
	point_cloud pc_to_be_append;
	point_cloud tmppc;
	bool is_highlighting = false;
	cgv::pointcloud::ICP icp;
	point_cloud pc_last_subsampled;
	point_cloud pc_to_be_append_subsampled;
	point_cloud_types::Mat rotation;
	point_cloud_types::Dir translation;
	cgv::pointcloud::ICP::Sampling_Type icp_filter_type = cgv::pointcloud::ICP::RANDOM_SAMPLING;


	/**@name rendering speedup techniques */
	//@{
	/// allows to restrict rendering to a subrange of all points
	std::size_t show_point_start, show_point_count;
	/// allows to subsample point rendering during interaction in order to speed up interaction
	unsigned interact_point_step;
	/// delay in seconds waited before interaction subsampling is turned off again
	double interact_delay;
	/// trigger used to check for end of interaction subsampling
	cgv::gui::trigger interact_trigger;
	/// callback attached to interact_trigger
	void interact_callback(double t, double dt);
	/// current state of interaction subsampling
	enum InteractionState {
		IS_INTERMEDIATE_FRAME,
		IS_WAIT_INTERACTION_TO_STOP,
		IS_DRAW_FULL_FRAME,
		IS_FULL_FRAME
	} interact_state;
	//@}

	/**@name ann tree, neighbor graph and picking*/
	//@{
	/// whether ann tree needs rebuild
	bool tree_ds_out_of_date;
	/// the ann tree is used for nearest neighbor queries
	ann_tree* tree_ds;
	///
	ann_tree* pc_src_ds;
	///
	ann_tree* pc_target_ds;
	//
	ann_tree* tree_ds_target_pc_last_frame;
	//
	ann_tree* tree_ds_source_pc;
	/// ensure that ann tree is built and current
	void ensure_tree_ds();
	///
	void ensure_neighbor_graph();
	/// k parameter for building neighbor graph
	unsigned k;
	/// whether to symmetric neighbor graph after build
	bool do_symmetrize;
	/// knn-neighbor graph built with tree_ds
	neighbor_graph ng;
	/// build the neighbor graph
	void build_neighbor_graph();
	/// build the neighbor graph
	void build_neighbor_graph_componentwise();
	/// normal estimation member
	normal_estimator ne;
	/// whether to use ann_tree to acceleration picking
	bool accelerate_picking;
	/// return the point closest to ray through given mouse position
	bool get_picked_point(int x, int y, unsigned& index);
	//@}

	/**@name normal computation and orientation*/
	//@{
	bool reorient_normals;
	//
	void compute_normals();
	void recompute_normals();
	void toggle_normal_orientations();
	void orient_normals();
	void orient_normals_to_view_point();
	void orient_normals_to_view_point_vr(vec3 cam_posi);
	//@}

	/**@name computing feature points */
	std::vector<vec3> feature_points;
	void compute_feature_points_and_colorize();
	cgv::render::box_render_style fea_style;
	std::vector<box3> fea_box;
	std::vector<rgb> fea_box_color;
	std::vector<vec3> fea_box_trans;
	std::vector<quat> fea_box_rot;

	/// pointer to instance that defines the view
	cgv::render::view* view_ptr;
	/// call this before using the view ptr for the first time
	bool ensure_view_pointer();

	// IO related vars 
	int num_of_pcs = 0;
	std::vector<int> frame_pointers;
	std::vector<Pnt> frame_cam_posi;

public:
	/// construct viewer with default configuration
	point_cloud_interactable();
	///
	void pc_changed();
	///
	void append_frame(point_cloud& _pc, bool registered);
	/// adjust view direction and extent at focus to bounding box of point cloud
	void auto_set_view();
	///
	point_cloud* ref_pc() { return &pc; }

	//**@name self reflection of class */
	//@{
	/// return type name of point_cloud_interactable
	std::string get_type_name() const;
	/// describe members
	bool self_reflect(cgv::reflect::reflection_handler& srh);
	/// stream out textual statistical information shown with F8
	void stream_stats(std::ostream&);
	/// stream out textual help information shown with F1
	void stream_help(std::ostream& os);
	//@}

	/**@name rendering */
	//@{
	/// whether to show neighbor graph
	bool show_neighbor_graph;
	/// function to activate edge color 
	void draw_edge_color(unsigned int vi, unsigned int j, bool is_symm, bool is_start) const;
	/// graph rendering function
	void draw_graph(cgv::render::context& ctx);
public:
	/// initialization on creation
	bool init(cgv::render::context& ctx);
	/// per frame initialization
	void init_frame(cgv::render::context& ctx);
	/// main rendering method
	void draw(cgv::render::context& ctx);
	//@}

	/**@name user interface */
	//@{
	/// helper function to ensure that limites of slides for point range selection are correct with respect to number of vertices in current point cloud
	void configure_subsample_controls();
	/// process command line arguments
	void handle_args(std::vector<std::string>& args);
	/// process key and mouse events
	bool handle(cgv::gui::event& e);
	/// used to update all dependent variables in case of changes to the point cloud, managememt function 
	virtual void on_point_cloud_change_callback(PointCloudChangeEvent pcc_event);
	/// used to update all dependent variables in case of changes to member variables
	void on_set(void* member_ptr);
	/// user interface creation
	void create_gui();
	///
	void clear_all();
	//@}
};

//typedef cgv::data::ref_ptr<point_cloud_interactable, true> point_cloud_viewer_ptr;

#include <cgv/config/lib_end.h>