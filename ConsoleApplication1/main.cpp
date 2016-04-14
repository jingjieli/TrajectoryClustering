#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include <ctime>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include "ClusteringComponent.h"

#define ITERATIONS 6
#define MAX_RADIUS 40.0
#define MIN_RADIUS 15.0
#define POINTS_PER_TRAJ 30
#define MIN_NUM_OF_POINTS_IN_TRAJ 50

typedef std::vector<point_t> PointsCollection;

int main() {

	std::vector<cv::Scalar> colors = { cv::Scalar(240, 120, 100), cv::Scalar(170, 80, 110), cv::Scalar(120, 30, 60), cv::Scalar(140, 140, 200),
		cv::Scalar(140, 100, 140), cv::Scalar(240, 160, 80), cv::Scalar(170, 200, 120), cv::Scalar(150, 200, 60),
		cv::Scalar(200, 170, 140), cv::Scalar(30, 60, 140), cv::Scalar(140, 140, 60), cv::Scalar(90, 60, 240),
		cv::Scalar(120, 120, 30), cv::Scalar(60, 200, 220), cv::Scalar(170, 230, 80), cv::Scalar(240, 170, 130),
		cv::Scalar(150, 180, 180), cv::Scalar(60, 240, 30), cv::Scalar(240, 130, 110), cv::Scalar(140, 60, 170),
		cv::Scalar(160, 160, 180), cv::Scalar(90, 200, 120), cv::Scalar(160, 160, 80), cv::Scalar(90, 170, 60) };

	std::string srcFileName = "Test.txt";
	std::string srcImageName = "Test.png";

	std::vector<traj_elem_t> origTrajs, origTrajsResample, msTrajs, amksTrajs, centerTrajs;
	NeighborPointsMap prevNeighborMap, currNeighborsMap;
	PointsCollection origPointsCollection, prevMSPointsCollection, msPointsCollection, prevAMKSCollection;

	std::vector<int> labels, centerLabels;

	// instantiate a new ClusteirngComponent object
	ClusteringComponent cComponent(srcFileName, srcImageName, MAX_RADIUS, MIN_RADIUS, POINTS_PER_TRAJ, ITERATIONS);

	// get input data
	origTrajs = cComponent.readTrajDataFromSrc(cComponent.textFileName, MIN_NUM_OF_POINTS_IN_TRAJ);

	// extract feature
	int interpolationType = cv::INTER_AREA;
	origTrajsResample = cComponent.featureExtraction(origTrajs, cComponent.targetSize, interpolationType);
	msTrajs = cComponent.featureExtraction(origTrajs, cComponent.targetSize, interpolationType);
	amksTrajs = cComponent.featureExtraction(origTrajs, cComponent.targetSize, interpolationType);

	cComponent.compDrawTrajectories(origTrajsResample, cComponent.imageName, "Resampled Trajectories");

	// create points collection for original trajs
	origPointsCollection = cComponent.createPointsCollectionFromTrajs(origTrajsResample);

	// enter AMKSClustering iterations
	cComponent.runAMKSClustering(origPointsCollection, prevAMKSCollection, prevAMKSCollection, prevNeighborMap, msTrajs, amksTrajs, cComponent.iterations);

	// identify number of clusters
	int numOfClusters = cComponent.findNumberOfClusters(amksTrajs, labels);

	// compute clustering centers
	std::vector<double> clusterPercentage;
	clusterPercentage = cComponent.findClusteringCenters(origTrajsResample, centerTrajs, labels, centerLabels, colors, numOfClusters);

	//cComponent.compDrawTrajectories(centerTrajs, centerLabels, colors, cComponent.imageName, "Clustering Centers");
	cComponent.compDrawTrajectoriesWithPercentage(centerTrajs, centerLabels, colors, cComponent.imageName, "Clustering Centers", clusterPercentage);
	//cComponent.compDrawCenters(centerTrajs, centerLabels, colors, cComponent.imageName, "Group Directions");

	std::vector<traj_elem_t> centerTrajsResample = cComponent.featureExtraction(centerTrajs, 50, cv::INTER_CUBIC);

	cComponent.compDrawCentersWithCurve(centerTrajsResample, centerLabels, colors, cComponent.imageName, "Center Points");
	cComponent.compDrawTrajectoriesWithPercentage(centerTrajsResample, centerLabels, colors, cComponent.imageName, "Smoothed Centers", clusterPercentage);

	cv::waitKey(0);

	return 0;
}