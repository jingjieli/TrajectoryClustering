#include "trajectory.h"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"

#ifndef EXTRACTFEATURE_H
#define EXTRACTFEATURE_H

std::vector<std::vector<double>> createGaussianFilter(int size);
traj_elem_t resizeTrajectory(traj_elem_t& currTraj, int targetSize, int interpolation);
traj_elem_t resampleTrajectory(traj_elem_t& currTraj, int targetSize);
traj_elem_t gaussianFilter(traj_elem_t& currTraj, std::vector<std::vector<double>>& gKernel);

#endif