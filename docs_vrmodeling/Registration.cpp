// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Registration.h"

Eigen::Affine3f CalculateRigidRegistration(std::vector<correspondence>& correspondences)
{
	//transform to compute
	Eigen::Affine3f T = Eigen::Affine3f(Eigen::Matrix3f::Identity());

	if(correspondences.size() < 3)
		return T;	
	
	//<snippet task="5.2.2">
	//<student>
	///* Task 5.2.2 */
	//</student>
	//<solution>
	//Calculate centroids
	Eigen::Vector3f mean1, mean2;
	mean1.setZero(); mean2.setZero();
	for (auto& c : correspondences)
	{
		mean1 += c.first;
		mean2 += c.second;
	}
	mean1 *= 1.0f / correspondences.size();
	mean2 *= 1.0f / correspondences.size();

	//Calculate covariance matrix
	Eigen::Matrix3f cov;
	cov.setZero();

	for (auto& c : correspondences)
	{
		Eigen::Vector3f x = c.first - mean1;
		Eigen::Vector3f y = c.second - mean2;

		cov += x * y.transpose();
	}

	Eigen::JacobiSVD<Eigen::Matrix3f> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3f R = svd.matrixU() * svd.matrixV().transpose();
	if (R.determinant() < 0)
	{
		Eigen::Vector3f adapt;
		adapt << 1, 1, -1;
		R = svd.matrixU() * adapt.asDiagonal() * svd.matrixV().transpose();
	}

	Eigen::Vector3f t = mean1 - R * mean2;

	T = Eigen::Translation3f(t) * Eigen::Affine3f(R);
	//</solution>
	//</snippet>

	return T;
}
