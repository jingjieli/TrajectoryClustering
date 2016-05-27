#include <vector>
#include <map>
#include "trajectory.h"
#include "point.h"
#include "opencv2\imgproc\imgproc.hpp"
#include "opencv2\highgui\highgui.hpp"

#ifndef CLUSTERING_H
#define CLUSTERING_H

//#define TRAJS_DIST_THRESHOLD 25.0
#define MAX_NEIGHBORS_SIZE 80

typedef std::vector<double> Column;
typedef std::vector<Column> Matrix;
typedef std::map<int, std::vector<std::pair<int, float>>> NeighborPointsMap;

enum ConvolutionType {
	CONVOLUTION_FULL,
	CONVOLUTION_SAME,
	CONVOLUTION_VALID
};

// sort pairs in into ascending order
struct sort_pair
{
	bool operator()(const std::pair<int, float> &left, const std::pair<int, float> &right) {
		return left.second < right.second;
	}
};

void pair_insert(std::vector<std::pair<int, float>>& targetVector, std::pair<int, float> targetPair);

cv::Mat getGaussianKernel2D(int rows, int cols, double sigma);

std::vector<point_t> createPointsCollection(std::vector<traj_elem_t>& trajs); // create a collection of points from input trajectories

std::vector<point_t> createPointsCollection(std::vector<traj_elem_t>& trajs, double &minX, double &maxX, double &minY, double &maxY);

NeighborPointsMap createNeighborsMap(std::vector<point_t>& pointsCollection,
	std::vector<point_t>& newPointsCollection, double currRadius);  // find all neighbors within radius for all the points in collection

NeighborPointsMap fastNeighborsMap(std::vector<point_t>& pointsCollection, std::vector<point_t>& newPointsCollection,
	NeighborPointsMap& originalMap, double currRadius); // call after first iteration when all neighbors have been identified

std::vector<point_t> meanShiftClustering(std::vector<traj_elem_t>& trajs, double currRadius); // mean shift clustering 

std::vector<traj_elem_t>& updateTrajs(std::vector<traj_elem_t>& trajs, std::vector<point_t>& pointsCollection); // update points info in each traj with estimated coordinates

Matrix buildDensityMatrix(std::vector<traj_elem_t>& trajs, double currRadius, double &minX, double &maxX, double &minY, double &maxY); 

std::vector<point_t> fastAMKSClustering(std::vector<traj_elem_t>& trajs, NeighborPointsMap& neighborsMap, double currRadius); // AMKS Clustering

int trajsPartition(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, double trajThreshold, std::vector<std::vector<float>>& distMatrix);

bool isInSameCluster(traj_elem_t firstTraj, traj_elem_t secondTraj, double distThreshold, float& trajsDist); // return true if two trajs are in the same cluster (use Dynamic Time Warping)

bool isAbnormal(std::vector<traj_elem_t> centerTrajs, traj_elem_t testTraj);

cv::Mat conv2d(const cv::Mat& src, const cv::Mat& kernel, ConvolutionType type); // 2D convolution

std::vector<double> conv1d(std::vector<double> src, std::vector<double> gKernel);

#endif 