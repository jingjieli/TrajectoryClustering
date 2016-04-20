#include <vector>
#include "trajectory.h"

#ifndef InputFileHandler_H
#define InputFileHandler_H

std::vector<traj_elem_t> readTrajDataFromFile(std::string filename, int minTrajDataSize, int maxTrajDataSize);

#endif 