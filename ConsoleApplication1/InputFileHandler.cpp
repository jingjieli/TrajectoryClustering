#include "InputFileHandler.h"
#include "StringSplit.h"

#include <vector>
#include <iostream>
#include <fstream>
#include <string> 

// read trajectpry data from a file 
std::vector<traj_elem_t> readTrajDataFromFile(std::string filename, int minTrajDataSize, int maxTrajDataSize) {

	std::vector<traj_elem_t> trajs;
	std::ifstream trajDataFile;
	trajDataFile.open(filename);
	if (trajDataFile.is_open()) {

		std::string curr_line;
		std::vector<std::string> fileLine;

		int trajId = 0;

		while (getline(trajDataFile, curr_line)) {

			fileLine = split(curr_line, ' ');

			int currNumOfPoints = std::stoi(fileLine[1]);

			if (currNumOfPoints >= minTrajDataSize && currNumOfPoints <= maxTrajDataSize) {

				//int currTrajId = std::stoi(fileLine[0]);
				//int currNumOfPoints = std::stoi(fileLine[1]);
				int currTrajId = trajId;
				int clusterId = std::stoi(fileLine[2]);
				std::vector<point_t> pointsInCurrTraj;
				for (std::size_t i = 3; i < fileLine.size() - 1; i = i + 2) {
					point_t curr_point = {
						currTrajId,
						std::stod(fileLine[i]),
						std::stod(fileLine[i + 1]),
						0.0,
						0.0,
						std::stod(fileLine[3]),
						std::stod(fileLine[4]),
						std::stod(fileLine[fileLine.size() - 2]),
						std::stod(fileLine[fileLine.size() - 1]),
					};
					pointsInCurrTraj.push_back(curr_point);
				}
				traj_elem_t curr_traj = {
					currTrajId,
					currNumOfPoints,
					clusterId,
					pointsInCurrTraj,
				};

				trajs.push_back(curr_traj);

				trajId++;
			}

		}

		//std::cout << trajs[1].points[1].start_x << " " << trajs[1].points[1].end_y << std::endl;
		//std::cout << trajs[1].points[2].start_x << " " << trajs[1].points[2].end_y << std::endl;

		trajDataFile.close();

	}
	else {
		std::cout << "Failed to read" << std::endl;
	}

	std::cout << "Load data succeed, there are " << trajs.size() << " trajectories being used." << std::endl;

	return trajs;
}
