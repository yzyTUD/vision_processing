#include "point_cloud.h"
#include <fstream>
#include "ICP.h"
#include <queue>


namespace cgv {
	namespace pointcloud {
		ICP::ICP() :ne_t(tmp_pc, ng_t) {
			sourceCloud = nullptr;
			targetCloud = nullptr;
			tree = nullptr;
			crspd_source = nullptr;
			crspd_target = nullptr;
			this->maxIterations = 400;
			this->numRandomSamples = 400;
			this->eps = 1e-8;
		}

		ICP::~ICP() {

		}

		void ICP::build_ann_tree()
		{
			if (!targetCloud) {
				std::cerr << "ICP::build_ann_tree: target cloud missing, can't build ann tree\n";
				return;
			}

			tree = std::make_shared<ann_tree>();
			tree->build(*targetCloud);
		}

		void ICP::clear()
		{
			tree = nullptr;
		}

		void ICP::set_source_cloud(point_cloud& inputCloud) {
			sourceCloud = &inputCloud;
		}

		void ICP::set_target_cloud(point_cloud& inputCloud) {
			targetCloud = &inputCloud;
		}

		void ICP::set_iterations(int Iter) {
			this->maxIterations = Iter;
		}

		void ICP::set_num_random(int NR) {
			this->numRandomSamples = NR;
		}

		void ICP::set_eps(float e) {
			this->eps = e;
		}

		// aquire crspds, the precise way 
		void ICP::aquire_crspds_simple(point_cloud& S, point_cloud& Q, int num) {
			/*std::srand(std::time(0));
			for (int i = 0; i < numRandomSamples; i++) {
				int cur_rand_idx = std::rand() % sourceCloud->get_nr_points();
				S.add_point(sourceCloud->pnt(cur_rand_idx));
				Q.add_point(targetCloud->pnt(cur_rand_idx));
			}*/
		}

		//
		void ICP::aquire_crspds_bilateral(point_cloud& S, point_cloud& Q, int num) {

			// the tree for source cloud has to be built each iteration
			// the cloud should be transformed to updated position  
			std::shared_ptr<ann_tree> tree_inv = std::make_shared<ann_tree>();
			tree_inv->build(*sourceCloud);

			if (num == 0) {
				for (int i = 0; i < numRandomSamples; i++) //sourceCloud->get_nr_points()
				{
					Pnt temp_s(0.0);
					Pnt temp_q(0.0);
					if (correspondences_filter_bilateral(tree, tree_inv, temp_s, temp_q))
					{
						S.add_point(temp_s);
						Q.add_point(temp_q);
					}
				}
			}
			else {
				while (S.get_nr_points() < num)
				{
					Pnt temp_s(0.0);
					Pnt temp_q(0.0);
					if (correspondences_filter_bilateral(tree, tree_inv, temp_s, temp_q))
					{
						// check for dulplicate 
						bool reject = false;
						for (int i = 0; i < S.get_nr_points(); i++) {
							if (S.pnt(i) == temp_s) {
								reject = true;
							}
						}
						if (!reject) {
							S.add_point(temp_s);
							Q.add_point(temp_q);
						}
					}
				}
			}
		}

		void ICP::aquire_crspds_full_matching(point_cloud& S, point_cloud& Q, int num) {
			// the tree for source cloud has to be built each iteration
			// the cloud should be transformed to updated position  
			std::shared_ptr<ann_tree> tree_inv = std::make_shared<ann_tree>();
			tree_inv->build(*sourceCloud);

			correspondences_filter_full_matching(tree, tree_inv, S, Q, num);

			// heuristic based, random based 
				//while(S.get_nr_points()<num) 
				//{
				//	Pnt temp_s(0.0);
				//	Pnt temp_q(0.0);
				//	if (correspondences_filter_full_matching(tree, tree_inv, temp_s, temp_q))
				//	{
				//		// check for dulplicate 
				//		bool reject = false;
				//		for (int i = 0; i < S.get_nr_points(); i++) {
				//			if (S.pnt(i) == temp_s) {
				//				reject = true;
				//			}
				//		}
				//		if (!reject) {
				//			S.add_point(temp_s);
				//			Q.add_point(temp_q);
				//			std::cout << "1 feature point computed!" << std::endl;
				//		}
				//	}
				//}
		}

		void ICP::aquire_crspds_normal_space(point_cloud& S, point_cloud& Q)
		{

		}
		bool ICP::correspondences_filter_nml(std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, Pnt& source_p, Pnt& target_p)
		{
			// uniform sampling in normal space 
			return false;
		}

		///output the rotation matrix and translation vector
		void ICP::reg_icp(Mat& rotation_mat, Dir& translation_vec, point_cloud& S, point_cloud& Q, cgv::pointcloud::ICP::Sampling_Type icp_filter_type, cgv::render::context* ctx) {
			if (!tree) {
				/// create the ann tree
				build_ann_tree();
				//std::cerr << "ICP::reg_icp: called reg_icp before initialization!\n";
			}
			if (!(sourceCloud && targetCloud)) {
				std::cerr << "ICP::reg_icp: source or target cloud not set!\n";
				return;
			}
			std::cout << "current num of points, source = " << sourceCloud->get_nr_points() << std::endl;
			std::cout << "current num of points, target = " << targetCloud->get_nr_points() << std::endl;

			float cost = 1.0;
			// define min as Infinity
			float min = std::numeric_limits<float>::infinity();
			// initializes fA to matrix filled with zeros
			Mat fA(0.0f);
			cgv::math::mat<float> U, V;
			cgv::math::diag_mat<float> Sigma;
			U.zeros();
			V.zeros();
			Sigma.zeros();
			//point_cloud S, Q;// subset of the pcs, used for samples 

			// make sure that sample points no more than nr of points in source cloud 
			if (numRandomSamples > sourceCloud->get_nr_points())
				numRandomSamples = sourceCloud->get_nr_points();

			// init the centers  
			S.clear();
			Q.clear();
			/*S.resize(numRandomSamples);
			Q.resize(numRandomSamples);*/
			Pnt source_center; source_center.zeros();
			Pnt target_center; target_center.zeros();

			// pre compute local feature: 
			cgv::utils::statistics he_stats;
			int m = 60;

			// check if pre-computation needed 
			if (!sourceCloud->has_features && icp_filter_type == FULL_MATCHING) {
				neighbor_graph ng_t;
				ng_t.clear();
				// build_neighbor_graph
				ng_t.build(targetCloud->get_nr_points(), m, *tree, &he_stats);
				ne_t.set_ng(ng_t);
				ne_t.set_pc(*targetCloud);


				/* compute local feature
				 knn-neighbor graph built with tree_ds*/
				std::shared_ptr<ann_tree> tree_inv = std::make_shared<ann_tree>();
				tree_inv->build(*sourceCloud);
				neighbor_graph ng_s;
				ng_s.clear();
				// build_neighbor_graph
				ng_s.build(sourceCloud->get_nr_points(), m, *tree_inv, &he_stats);
				normal_estimator ne_s(*sourceCloud, ng_s);

				feature_precomputing(ne_s, ne_t);
			}

			//aquire_crspds_reject_nmls(S, Q, 4);
			//aquire_crspds_bilateral(S,Q,4);

			// used to jump out of the local minima
			int times_smaller_err_found = 0;

			// main iteration of the ICP algorithm 
			int num_of_iters = 0;
			for (int iter = 0; iter < maxIterations && abs(cost) > eps; iter++)
			{
				if (iter % 50 == 0) {
					min = std::numeric_limits<float>::infinity();
					//std::cout << "reset min!" << std::endl;
				}

				std::srand((unsigned)std::time(0) + iter);

				if (!QS_not_changed) {
					S.clear();
					Q.clear();

					// sampling, just pass the tree through
					// this won't help too much, similar to the origenal one 
					switch (icp_filter_type) {
					case PRECISE_SAMPLING:
						aquire_crspds_simple(S, Q, 0);
						break;
					case RANDOM_SAMPLING:
						aquire_crspds_bilateral(S, Q, 0);
						break;
					case FULL_MATCHING:
						aquire_crspds_full_matching(S, Q, numRandomSamples);
						break;
					case NORMAL_SPACE_SAMPLING:

						break;
					}

				}
				/*std::cout << "current iteration = " << iter << std::endl;
				std::cout << "current err = " << cost << std::endl;*/
				std::cout << "num of real samples used = " <<
					S.get_nr_points() << std::endl;

				// QS not changed 
				// compute QS in pre-compute step


				// 
				if (S.get_nr_points() < 3)
					continue;

				// re-compute center of the sampled points  
				get_center_point(S, source_center);
				get_center_point(Q, target_center);

				// compute cov matrix 
				fA.zeros();
				for (int i = 0; i < S.get_nr_points(); i++) {
					fA += Mat(Q.pnt(i) - target_center, S.pnt(i) - source_center);
				}

				///cast fA to A
				cgv::math::mat<float> A(3, 3, &fA(0, 0));
				cgv::math::svd(A, U, Sigma, V);
				Mat fU(3, 3, &U(0, 0)), fV(3, 3, &V(0, 0));
				///get new R and t
				rotation_mat = fU * cgv::math::transpose(fV);
				cgv::math::mat<float> R(3, 3, &rotation_mat(0, 0));
				if (cgv::math::det(R) < 0) {
					// multiply the 1,1...-1 diag matrix 
					Mat fS;
					fS.zeros();
					fS(0, 0) = 1;
					fS(1, 1) = 1;
					fS(2, 2) = -1;
					rotation_mat = fU * fS * cgv::math::transpose(fV);
				}
				translation_vec = target_center - rotation_mat * source_center;
				///calculate error function E(R,t)
				cost = 0.0f;
				for (int i = 0; i < S.get_nr_points(); i++) {
					///the new rotation matrix: rotation_mat
					cost += error(Q.pnt(i), S.pnt(i), rotation_mat, translation_vec);
				}
				cost /= S.get_nr_points();
				/// judge if cost is decreasing, 
				/// and is larger than eps. If so, update the R and t, 
				/// otherwise stop and output R and t
				if (min >= abs(cost)) {
					// perform multiple times can somehow avoids local minima!
					// a long iter won't help, there should be some strategies to 
					// jump out of this!

					std::cout << "lower err found!" << std::endl;
					std::cout << "current err = " << cost << std::endl;

					// re-compute the transform of the source cloud 
					// update the source cloud for tree computation 
					sourceCloud->rotate(cgv::math::quaternion<float>(rotation_mat));
					sourceCloud->translate(translation_vec);

					// update S 
					for (int i = 0; i < S.get_nr_points(); i++) {
						S.pnt(i) = rotation_mat * S.pnt(i) + translation_vec;
					}

					//ctx->post_redraw();

					min = abs(cost);
				}
				num_of_iters++;
			}
			std::cout << "--------------" << std::endl;
			std::cout << "iter used: " << num_of_iters << std::endl;
			std::cout << "cost = " << min << std::endl;
			std::cout << "coverged = " << (num_of_iters < maxIterations) << std::endl;
		}

		void ICP::get_center_point(const point_cloud& input, Pnt& center_point) {
			center_point.zeros();
			for (unsigned int i = 0; i < input.get_nr_points(); i++)
				center_point += input.pnt(i);
			center_point /= (float)input.get_nr_points();
		}

		float ICP::error(Pnt& ps, Pnt& pd, Mat& r, Dir& t)
		{
			//Pnt res;
			//res = r * pd;
			//float err = pow(ps.x() - res.x() - t[0], 2.0) + pow(ps.y() - res.y() - t[1], 2.0) + pow(ps.z() - res.z() - t[2], 2.0);
			//return err;
			Pnt tmp = ps - r * pd - t;
			return dot(tmp, tmp);
		}

		void ICP::get_crspd(Mat& rotation_mat, Dir& translation_vec, point_cloud& pc1, point_cloud& pc2)
		{
			//Pnt source_center;
			//Pnt target_center;
			//source_center.zeros();
			//target_center.zeros();
			///// create the ann tree
			//ann_tree* tree = new ann_tree();
			//tree->build(*targetCloud);
			//size_t num_source_points = sourceCloud->get_nr_points();

			//get_center_point(*targetCloud, target_center);
			//get_center_point(*sourceCloud, source_center);
			//Pnt p;

			//float cost = 1.0;
			/////define min as Infinity
			//float min = DBL_MAX;
			//std::srand(std::time(0));

			//Mat fA(0.0f);             // this initializes fA to matrix filled with zeros

			//cgv::math::mat<float> U, V;
			//cgv::math::diag_mat<float> Sigma;
			//U.zeros();
			//V.zeros();
			//Sigma.zeros();
			//for (int iter = 0; iter < maxIterations && abs(cost) > eps; iter++)
			//{
			//	cost = 0.0;
			//	point_cloud Q, S;
			//	S.clear();
			//	Q.clear();
			//	source_center = rotation_mat * source_center + translation_vec;
			//	fA.zeros();
			//	for (int i = 0; i < numRandomSamples; i++) //sourceCloud->get_nr_points()
			//	{
			//		Pnt temp_s(0.0);
			//		Pnt temp_q(0.0);
			//		if (correspondences_filter(temp_s, temp_q))
			//		{
			//			S.add_point(temp_s);
			//			Q.add_point(temp_q);
			//			fA += Mat(temp_q - target_center, rotation_mat * temp_s + translation_vec - source_center);
			//		}
			//	}
			//	///cast fA to A
			//	cgv::math::mat<float> A(3, 3, &fA(0, 0));
			//	cgv::math::svd(A, U, Sigma, V);
			//	Mat fU(3, 3, &U(0, 0)), fV(3, 3, &V(0, 0));
			//	///get new R and t
			//	Mat rotation_update_mat = fU * cgv::math::transpose(fV);
			//	Dir translation_update_vec = target_center - rotation_update_mat * source_center;
			//	///calculate error function E(R,t)
			//	for (int i = 0; i < S.get_nr_points(); i++) {
			//		///transform Pi to R*Pi + t
			//		S.pnt(i) = rotation_mat * S.pnt(i) + translation_vec;
			//		///the new rotation matrix: rotation_update_mat
			//		//float tempcost = error(Q.pnt(i), S.pnt(i), rotation_update_mat, translation_update_vec);
			//		cost += error(Q.pnt(i), S.pnt(i), rotation_update_mat, translation_update_vec);
			//	}
			//	cost /= sourceCloud->get_nr_points();
			//	///judge if cost is decreasing, and is larger than eps. If so, update the R and t, otherwise stop and output R and t
			//	if (min >= abs(cost)) {
			//		///update the R and t
			//		rotation_mat = rotation_update_mat * rotation_mat;
			//		translation_vec = rotation_update_mat * translation_vec + translation_update_vec;
			//		min = abs(cost);
			//	}
			//	pc1.clear();
			//	pc2.clear();
			//	for (int i = 0; i < S.get_nr_points(); i++)
			//		pc1.add_point(S.pnt(i));
			//	for (int i = 0; i < Q.get_nr_points(); i++)
			//		pc2.add_point(Q.pnt(i));
			//}
			//std::cout << "rotate_mat: " << rotation_mat << std::endl;
			//std::cout << "translation_vec: " << translation_vec << std::endl;
			//delete tree;
		}
		///print rotation matrix
		void ICP::print_rotation(float* rotation) {
			std::cout << "rotation" << std::endl;
			for (int i = 0; i < 9; i = i + 3) {
				std::cout << rotation[i] << " " << rotation[i + 1] << " " << rotation[i + 2] << std::endl;
			}
		}
		///print translation vector
		void ICP::print_translation(float* translation) {
			std::cout << "translation" << std::endl;
			std::cout << translation[0] << " " << translation[1] << " " << translation[2] << std::endl;
		}

		bool ICP::correspondences_filter_bilateral(std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, Pnt& source_p, Pnt& target_p)
		{
			float dist, dist_inv = 0.0;
			int randSample = std::rand() % sourceCloud->get_nr_points();
			source_p = sourceCloud->pnt(randSample);
			target_p = targetCloud->pnt(tree->find_closest(source_p));

			// check and reject some samples 
			dist = dis_pts(source_p, target_p);
			Pnt source_p_inv = sourceCloud->pnt(tree_inv->find_closest(target_p));
			dist_inv = dis_pts(source_p_inv, target_p);
			if (dist > 1.5 * dist_inv || dist < 0.667 * dist_inv)
			{
				return false;
			}
			return true;
		}

		void ICP::feature_precomputing(normal_estimator& ne_s, normal_estimator& ne_t) {
			sourceCloud->create_features();
			for (int i = 0; i < sourceCloud->get_nr_points(); i++) {
				float d_source[3] = { 0,0,0 };
				{
					std::vector<Crd> weights;
					std::vector<Pnt> points;
					ne_s.compute_weights(i, weights, &points);
					Nml new_nml;
					cgv::math::estimate_normal_wls((unsigned)points.size(), points[0], &weights[0], new_nml, &d_source[0]);
				}
				Pnt d_s = Pnt(d_source[0], d_source[1], d_source[2]);
				//d_s.normalize();
				sourceCloud->fea(i) = d_s;
				//Dir(d_s[0], d_s[1], d_s[2]);
			}

			targetCloud->create_features();
			for (int i = 0; i < targetCloud->get_nr_points(); i++) {
				float d_target[3] = { 0,0,0 };
				{
					std::vector<Crd> weights;
					std::vector<Pnt> points;
					ne_t.compute_weights(i, weights, &points);
					Nml new_nml;
					cgv::math::estimate_normal_wls((unsigned)points.size(), points[0], &weights[0], new_nml, &d_target[0]);
				}
				Pnt d_t = Pnt(d_target[0], d_target[1], d_target[2]);
				//d_t.normalize();
				targetCloud->fea(i) = d_t;
				//Dir(d_t[0], d_t[1], d_t[2]);
			}


		}

		bool ICP::correspondences_feature_matching(normal_estimator& ne_s, normal_estimator& ne_t, std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, Pnt& source_p, Pnt& target_p) {
			float max_source_val = 0;
			int max_source_idx = -1;
			for (int i = 0; i < sourceCloud->get_nr_points(); i++) {
				float d_source[3] = { 0,0,0 };
				{
					std::vector<Crd> weights;
					std::vector<Pnt> points;
					ne_s.compute_weights(i, weights, &points);
					Nml new_nml;
					cgv::math::estimate_normal_wls((unsigned)points.size(), points[0], &weights[0], new_nml, &d_source[0]);
				}
				Pnt d_s = Pnt(d_source[0], d_source[1], d_source[2]);
				if (d_s.length() > max_source_val) {
					max_source_val = d_s.length();
					max_source_idx = i;
				}
			}
			source_p = sourceCloud->pnt(max_source_idx);

			return true;
		}

		struct WeightComparer
		{
			bool operator()(const std::pair<std::pair<int, int>, float>& p1, const std::pair<std::pair<int, int>, float>& p2)
			{
				return p1.second > p2.second;
			}
		};

		bool ICP::correspondences_filter_full_matching(std::shared_ptr<ann_tree> tree, std::shared_ptr<ann_tree> tree_inv, point_cloud& S, point_cloud& Q, int num)
		{
			// O(n * m), full iter, global minimal selected, the best result for identity meshes 
			// ori below: can quit earlier
			// clustering
			// no better than random sampling in case of diff models
			int found_points = 0;
			std::priority_queue<std::pair<std::pair<int, int>, float>, std::vector<std::pair<std::pair<int, int>, float>>, WeightComparer> pq;
			for (int s_idx = 0; s_idx < sourceCloud->get_nr_points(); s_idx++) {
				float d_source[3];
				d_source[0] = sourceCloud->fea(s_idx).x();
				d_source[1] = sourceCloud->fea(s_idx).y();
				d_source[2] = sourceCloud->fea(s_idx).z();

				// find feature points  
				float minimal_similarity = 1; // or max number 
				int minimal_idx = -1;
				Pnt d_s = Pnt(d_source[0], d_source[1], d_source[2]);
				float d_target[3];
				for (int j = 0; j < targetCloud->get_nr_points(); j++) {

					d_target[0] = targetCloud->fea(j).x();
					d_target[1] = targetCloud->fea(j).y();
					d_target[2] = targetCloud->fea(j).z();

					Pnt d_t = Pnt(d_target[0], d_target[1], d_target[2]);

					//d_s.normalize();
					//d_t.normalize();

					// L2 metric 
					float eigen_value_similarity = (d_s - d_t).length();
					if (eigen_value_similarity < minimal_similarity) {
						minimal_similarity = eigen_value_similarity;
						minimal_idx = j;
					}
				}
				pq.push(std::make_pair(std::make_pair(s_idx, minimal_idx), minimal_similarity));
			}

			for (int i = 0; i < num; ++i)
			{
				auto cur_pair = pq.top(); pq.pop();
				S.add_point(sourceCloud->pnt(cur_pair.first.first));
				Q.add_point(targetCloud->pnt(cur_pair.first.second));
				std::cout << "accept minimal_similarity: " << cur_pair.second << std::endl;
			}

			return true;


			//std::cout << "minimal_similarity: " << minimal_similarity << std::endl;
			//if (minimal_idx == -1 || minimal_similarity > 1e-12) {
			//	// not in the other cloud 
			//	//std::cout << "minimal_similarity too large!, drop " << std::endl;
			//	//return false;
			//	continue;
			//}
			//else {
			//	std::cout << "accept minimal_similarity: " << minimal_similarity << std::endl;
			//	S.add_point(sourceCloud->pnt(s_idx));
			//	Q.add_point(targetCloud->pnt(minimal_idx));
			//	found_points++;
			//	if (found_points == num) {
			//		return true;
			//	}
			//}

		// taken as heuristic, error!, full matching can only work as exact match
		// for the same object! 
		// 1. mesh feature computes correctly
		// 2. for further use (checking)

		/*if (!sourceCloud->has_normals()||!targetCloud->has_normals()) {
			return false;
		}*/

		// compute dist and nml infos 
		//float dist, dist_inv = 0.0;
		/*int rand_sample_idx = std::rand() % sourceCloud->get_nr_points();
		source_p = sourceCloud->pnt(rand_sample_idx);*/
		/*
		int closest_inv_idx = tree->find_closest(source_p);
		target_p = targetCloud->pnt(closest_inv_idx);*/

		// we have rand_sample_idx in source and closest_inv_idx in target now

		//
		/*if (sourceCloud->has_normals() && targetCloud->has_normals()) {
			Dir source_p_nml = sourceCloud->nml(rand_sample_idx);
			Dir target_p_nml = targetCloud->nml(closest_inv_idx);
		}*/


		/*float d_source[3] = { 0,0,0 };
		{
			std::vector<Crd> weights;
			std::vector<Pnt> points;
			ne_s.compute_weights(rand_sample_idx, weights, &points);
			Nml new_nml;
			cgv::math::estimate_normal_wls((unsigned)points.size(), points[0], &weights[0], new_nml, &d_source[0]);
		}*/


		//float d_target[3] = { 1,1,1 };
		//{
		//	std::vector<Crd> weights;
		//	std::vector<Pnt> points;
		//	ne_t.compute_weights(closest_inv_idx, weights, &points);
		//	Nml new_nml; 
		//	cgv::math::estimate_normal_wls((unsigned)points.size(), points[0], &weights[0], new_nml, &d_target[0]);
		//}

		//Pnt d_s = Pnt(d_source[0], d_source[1], d_source[2]);
		//Pnt d_t = Pnt(d_target[0], d_target[1], d_target[2]);

		//d_s.normalize();
		//d_t.normalize();

		//// L2 metric 
		//float eigen_value_similarity = (d_s - d_t).length();

		//std::cout << "eigen_value_similarity  " << eigen_value_similarity << std::endl;

		// compute the dist_inv just for checking 
		//dist = dis_pts(source_p, target_p);
		//int closest_inv_inv_idx = tree_inv->find_closest(target_p);
		//Pnt source_p_inv = sourceCloud->pnt(closest_inv_inv_idx);
		//dist_inv = dis_pts(source_p_inv, target_p);

		//// check distance, reject when necessary 
		//if (dist > 1.5 * dist_inv 
		//	|| dist < 0.667 * dist_inv )
		//{
		//	return false;
		//}

		}

		float ICP::dis_pts(const Pnt& source_p, const Pnt& target_p)
		{
			float dist = 0.0;
			dist = sqrt(pow((source_p.x() - target_p.x()), 2) + pow((source_p.y() - target_p.y()), 2) + pow((source_p.z() - target_p.z()), 2));
			return dist;
		}
	}
}