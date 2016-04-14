#include <vector>
#include "point.h"

#ifndef TRAJECTORY_H
#define TRAJECTORY_H

struct traj_elem_t {
	int trajId;
	int numOfPoints;
	int clusterId;
	std::vector<point_t> points;
};

void printTrajectory(traj_elem_t &traj);

#endif 