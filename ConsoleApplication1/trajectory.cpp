#include <iostream>
#include "trajectory.h"
#include "point.h"

void printTrajectory(traj_elem_t &traj) {
	std::cout << "=======================" << std::endl;
	std::cout << "Current trajectory is: " << std::endl;
	std::cout << "trajId: " << traj.trajId << std::endl;
	std::cout << "Number of points in trajectory: " << traj.numOfPoints << std::endl;
	for (size_t i = 0; i < traj.points.size(); i++) {
		printPoint(traj.points[i]);
	}
}