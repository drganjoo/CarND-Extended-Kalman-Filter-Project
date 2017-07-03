#include <iostream>
#include "tools.h"
#include <cassert>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() == 0 || estimations.size() != ground_truth.size()) {
		assert(false); // how come estimates and ground truth don't have equal sizes?
		return rmse;
	}

	for (size_t i = 0; i < estimations.size(); ++i) {
		const VectorXd &e = estimations[i];
		const VectorXd &g = ground_truth[i];

		VectorXd diff = e - g;
		diff = diff.array() * diff.array();

		rmse += diff;
	}

	//calculate the mean
	rmse /= estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

// Formula mostly adopted from the discussion at:
// https://discussions.udacity.com/t/ukf-getting-stuck-on-second-dataset/240080/25
// Slight change is to use angle_radian + M_PI for fmod in both cases as that makes more sense
// that first we move the angle_radian from -Pi to Pi to 0 to 2* Pi, mod the value to get it in between
// 0 to 2Pi and then we take it back to a -Pi to Pi range by subtracting Pi from it

double Tools::NormalizeAngleRad(double angle_radian)
{
	double angle_norm;

	if (angle_radian > M_PI)
		angle_norm = fmod(angle_radian + M_PI, 2 * M_PI) - M_PI;
	else if (angle_radian < -M_PI)
		angle_norm = fmod(angle_radian + M_PI, 2 * M_PI) + M_PI;
	else 
		angle_norm = angle_radian;

	return angle_norm;
}
