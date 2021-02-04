#pragma once

#include <vector>
#include "point_cloud.h"
#include "ann_tree.h"
#include <random>
#include <ctime>
#include <cgv/math/svd.h> 

#include <cgv/math/det.h> 
#include <cgv/math/fvec.h>
#include "neighbor_graph.h" 
#include "normal_estimator.h"
#include <cgv/math/normal_estimation.h>
#include <cgv\render\context.h>

#include "lib_begin.h"
#include <memory>



namespace cgv {

	namespace pointcloud {

		class CGV_API ICP : public point_cloud_types {
		public:
			enum Sampling_Type {
				PRECISE_SAMPLING = 0,
				RANDOM_SAMPLING = 1,
				FULL_MATCHING = 2, // feature based 
				NORMAL_SPACE_SAMPLING = 3
			} S_type;

			point_cloud* sourceCloud;
			point_cloud* targetCloud;
			int maxIterations;
			int numRandomSamples;
			float eps;
			point_cloud* crspd_source;
			point_cloud* crspd_target;

			neighbor_graph ng_t;
			point_cloud tmp_pc;

			ICP();
			~ICP();

			void build_ann_tree();
			void clear();
			void set_source_cloud(point_cloud& inputCloud);
			void set_target_cloud(point_cloud& inputCloud);
			void set_iterations(int Iter);
			void set_num_random(int NR);
			void set_eps(float e);

			void aquire_crspds_simple(point_cloud& S, point_cloud& Q, int least_num_once);
			void aquire_crspds_bilateral(point_cloud& S, point_cloud& Q, int least_num_once);
			void aquire_crspds_full_matching(point_cloud& S, point_cloud& Q, int least_num_once);
			void aquire_crspds_normal_space(point_cloud& S, point_cloud& Q);

			bool correspondences_filter_bilateral(std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, Pnt& source_p, Pnt& target_p);
			bool correspondences_filter_full_matching(std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, point_cloud& S, point_cloud& Q, int least_num_once);
			bool correspondences_feature_matching(normal_estimator& ne_s, normal_estimator& ne_t, std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, Pnt& source_p, Pnt& target_p);
			bool correspondences_filter_nml(std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, Pnt& source_p, Pnt& target_p);

			void reg_icp(Mat& rotation_m, Dir& translation_v, point_cloud& S, point_cloud& Q, cgv::pointcloud::ICP::Sampling_Type icp_filter_type, cgv::render::context* ctx);
			void get_center_point(const point_cloud& input, Pnt& mid_point);
			float error(Pnt& ps, Pnt& pd, Mat& r, Dir& t);
			void get_crspd(Mat& rotation_m, Dir& translation_v, point_cloud& pc1, point_cloud& pc2);
			void print_rotation(float* rotationMatrix);
			void print_translation(float* translation);


			void feature_precomputing(normal_estimator& ne_s, normal_estimator& ne_t);
			float dis_pts(const Pnt& source_p, const Pnt& target_p);
		private:
			std::shared_ptr<ann_tree> tree;
			normal_estimator ne_t;
			bool QS_not_changed = false;
		};
	}
}
#include <cgv/config/lib_end.h>
