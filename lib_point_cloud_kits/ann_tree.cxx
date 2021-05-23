#include "ann_tree.h"

#define ANN_USE_FLOAT
#include <ANN/ANN.h>

struct ann_struct
{
	ANNpointArray pa;
	ANNpointSet* ps;
	ann_struct() : pa(0), ps(0) {}
};

ann_tree::ann_tree()
{
	ann_impl = 0;
	k = 30;
	pc = 0;
}

ann_tree::~ann_tree()
{
	clear();
}

void ann_tree::clear()
{
	if (!ann_impl)
		return;
	ann_struct*& ann = (ann_struct*&)ann_impl;
	delete [] ann->pa;
	delete ann->ps;
	delete ann;
	ann = 0;
	pc = 0;
}

bool ann_tree::is_empty() const
{
	return pc == 0;
}

void ann_tree::build(const point_cloud& _pc)
{
	std::cout << "building tree structure..." << std::endl;
	//
	clear();
	// store pointer to points in point cloud
	pc = &_pc;

	// construct ann data structure container
	if (!ann_impl)
		ann_impl = new ann_struct();
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);

	// build point array
	Cnt n = (Cnt)pc->get_nr_points();

	ann->pa = new ANNpoint[n];
	Idx i;
	for (i=0; i<(Idx)n; ++i)
		ann->pa[i] = const_cast<ANNpoint>(&pc->pnt(i)[0]);

	// construct search tree
	ann->ps = new ANNkd_tree(ann->pa,n,3);
}

/// build from given components
void ann_tree::build(const point_cloud& _pc, const std::vector<Idx>& component_indices)
{
	//
	clear();
	// store pointer to points in point cloud
	pc = &_pc;

	// construct ann data structure container
	if (!ann_impl)
		ann_impl = new ann_struct();
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);

	// build point array
	Cnt n = 0;
	for (Idx ci : component_indices)
		n += Cnt(pc->component_point_range(ci).nr_points);
	ann->pa = new ANNpoint[n];
	Idx i = 0;
	for (Idx ci : component_indices) {
		Idx pi_end = Idx(pc->component_point_range(ci).index_of_first_point + pc->component_point_range(ci).nr_points);
		for (Idx pi = Idx(pc->component_point_range(ci).index_of_first_point); pi < pi_end; ++pi) {
			ann->pa[i] = const_cast<ANNpoint>(&pc->pnt(pi)[0]);
			++i;
		}
	}
	// construct search tree
	ann->ps = new ANNkd_tree(ann->pa, n, 3);
}

void ann_tree::extract_neighbors(Idx i, Idx k, std::vector<Idx>& N) const
{
	static std::vector<float> dists;
	static std::vector<Idx> tmp;
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);
	if (!ann) {
		std::cerr << "no ann_tree built" << std::endl;
		return;
	}
	N.resize(k);
	tmp.resize(k+1);
	dists.resize(k+1);
	ann->ps->annkSearch(const_cast<ANNpoint>(&pc->pnt(i)[0]), k+1, (ANNidxArray)&tmp[0], &dists[0]);
	std::copy(tmp.begin()+1,tmp.end(),N.begin());
}

ann_tree::Idx ann_tree::find_closest(const Pnt& p) const
{
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);
	if (!ann) {
		std::cerr << "no ann_tree built" << std::endl;
		return -1;
	}
	float dist;
	unsigned int result;
	ann->ps->annkSearch(const_cast<ANNpoint>(&p[0]), 1, (ANNidxArray)&result, &dist);
	return result;
}

void ann_tree::find_closest_and_its_dist(const Pnt& p, float& dist, int& result)
{
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);
	if (!ann) {
		std::cerr << "no ann_tree built" << std::endl;
		return;
	}
	ann->ps->annkSearch(const_cast<ANNpoint>(&p[0]), 1, (ANNidxArray)&result, &dist);
}

void ann_tree::find_closest_points(const Pnt& p, Idx k, std::vector<int>* knn) const
{
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);
	if (!ann) {
		std::cerr << "no ann_tree built" << std::endl;
		return;
	}
	knn->resize(k);
	ANNdistArray dist_array = new ANNdist[k];
	ANNidxArray index_array = new ANNidx[k];
	ann->ps->annkSearch(const_cast<ANNpoint>(&p[0]), k, index_array, dist_array);
	for (Idx i = 0; i < k; ++i)
		knn->at(i) = index_array[i];
	delete[] dist_array;
	delete[] index_array;
}

void ann_tree::find_neighbor_point_given_radius(const Pnt& p, int maximum_num_points, float radius, std::vector<int>* knn) {
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);
	if (!ann) {
		std::cerr << "no ann_tree built" << std::endl;
		return;
	}
	ANNidxArray index_array = new ANNidx[maximum_num_points];
	ANNdistArray dist_array = new ANNdist[maximum_num_points];
	ann->ps->annkFRSearch(const_cast<ANNpoint>(&p[0]), radius, maximum_num_points, index_array, dist_array);
	for (Idx i = 0; i < maximum_num_points; ++i) {
		if (dist_array[i] > radius)
			break;
		knn->push_back(index_array[i]);
	}
	delete[] dist_array;
	delete[] index_array;
}

void ann_tree::find_closest_points(const Pnt& p, Idx k, std::vector<int>& knn, std::vector<float>& dist) const
{
	ann_struct* ann = static_cast<ann_struct*>(ann_impl);
	if (!ann) {
		std::cerr << "no ann_tree built" << std::endl;
		return;
	}
	knn.resize(k);
	dist.resize(k);
	ANNdistArray dist_array = new ANNdist[k];
	ANNidxArray index_array = new ANNidx[k];
	ann->ps->annkSearch(const_cast<ANNpoint>(&p[0]), k, index_array, dist_array);
	for (Idx i = 0; i < k; ++i)
		knn[i] = index_array[i];
	for (Idx i = 0; i < k; ++i)
		dist[i] = dist_array[i];
	delete[] dist_array;
	delete[] index_array;
}