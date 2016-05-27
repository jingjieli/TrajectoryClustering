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

#define ITERATIONS 7
#define MAX_RADIUS 25.0
#define MIN_RADIUS 10.0
//#define POINTS_PER_TRAJ 20
//#define MIN_NUM_OF_POINTS_IN_TRAJ 20
#define SMOOTH_GAUSSIAN_SIZE 21

typedef std::vector<point_t> PointsCollection;

int main_1(int argc, char *argv[]) {

	std::vector<cv::Scalar> colors = { cv::Scalar(240, 120, 100), cv::Scalar(170, 80, 110), cv::Scalar(120, 30, 60), cv::Scalar(140, 140, 200),
		cv::Scalar(140, 100, 140), cv::Scalar(240, 160, 80), cv::Scalar(170, 200, 120), cv::Scalar(150, 200, 60),
		cv::Scalar(200, 170, 140), cv::Scalar(30, 60, 140), cv::Scalar(140, 140, 60), cv::Scalar(90, 60, 240),
		cv::Scalar(120, 120, 30), cv::Scalar(60, 200, 220), cv::Scalar(170, 230, 80), cv::Scalar(240, 170, 130),
		cv::Scalar(150, 180, 180), cv::Scalar(60, 240, 30), cv::Scalar(240, 130, 110), cv::Scalar(140, 60, 170),
		cv::Scalar(160, 160, 180), cv::Scalar(90, 200, 120), cv::Scalar(160, 160, 80), cv::Scalar(90, 170, 60) };

	std::string srcFileName = argv[1];
	std::string srcImageName = argv[2];
	int minPointsPerTraj = std::stoi(argv[3]); // enter min traj length
	int maxPointsPerTraj = std::stoi(argv[4]); // enter max traj length
	int resampleSize = std::stoi(argv[5]); // resample traj size
	int featureExtractionOption = std::stoi(argv[6]); // 0: none; 1: resize; others: resize & smooth
	double trajsDistThreshold = std::stod(argv[7]); // set distance threshold between 2 trajs

	/*std::string srcFileName = "DataLin_split_1.txt";
	std::string srcImageName = "DataLin.png";*/

	std::vector<traj_elem_t> origTrajs, origTrajsResample, msTrajs, amksTrajs, centerTrajs;
	NeighborPointsMap prevNeighborMap, currNeighborsMap;
	PointsCollection origPointsCollection, prevMSCollection, prevAMKSCollection;

	std::vector<int> labels, centerLabels;

	// instantiate a new ClusteirngComponent object
	//ClusteringComponent cComponent(srcFileName, srcImageName, MAX_RADIUS, MIN_RADIUS, POINTS_PER_TRAJ, ITERATIONS);
	ClusteringComponent cComponent(srcFileName, srcImageName, MAX_RADIUS, MIN_RADIUS, resampleSize, ITERATIONS);

	// get input data
	//origTrajs = cComponent.readTrajDataFromSrc(cComponent.textFileName, MIN_NUM_OF_POINTS_IN_TRAJ);
	origTrajs = cComponent.readTrajDataFromSrc(cComponent.textFileName, minPointsPerTraj, maxPointsPerTraj);
	int numOfTrajs = origTrajs.size();

	// extract feature with options
	int interpolationType = cv::INTER_AREA;
	if (featureExtractionOption == 0) {
		// no options selected
		origTrajsResample = origTrajs;
		msTrajs = origTrajs;
		amksTrajs = origTrajs;
	}
	else if (featureExtractionOption == 1) {
		// traj resize
		origTrajsResample = cComponent.featureExtraction(origTrajs, cComponent.targetSize, interpolationType);
		msTrajs = cComponent.featureExtraction(origTrajs, cComponent.targetSize, interpolationType);
		amksTrajs = cComponent.featureExtraction(origTrajs, cComponent.targetSize, interpolationType);
	}
	else {
		// traj resize and smooth
		int kernelSize = 5;
		origTrajsResample = cComponent.featureExtractionWithSmooth(origTrajs, cComponent.targetSize, interpolationType, kernelSize);
		msTrajs = cComponent.featureExtractionWithSmooth(origTrajs, cComponent.targetSize, interpolationType, kernelSize);
		amksTrajs = cComponent.featureExtractionWithSmooth(origTrajs, cComponent.targetSize, interpolationType, kernelSize);
	}

	cComponent.compDrawTrajectories(origTrajsResample, cComponent.imageName, "Resampled Trajectories");

	// create points collection for original trajs
	origPointsCollection = cComponent.createPointsCollectionFromTrajs(origTrajsResample);

	// enter AMKSClustering iterations
	cComponent.runAMKSClustering(origPointsCollection, prevMSCollection, prevAMKSCollection, prevNeighborMap, msTrajs, amksTrajs, cComponent.iterations);
	//cComponent.runAMKSClustering(origPointsCollection, prevAMKSCollection, prevAMKSCollection, prevNeighborMap, msTrajs, amksTrajs, cComponent.iterations);

	// identify number of clusters
	std::vector<std::vector<float>> trajsDistMatrix(numOfTrajs, std::vector<float>(numOfTrajs));
	int numOfClusters = cComponent.findNumberOfClusters(amksTrajs, labels, trajsDistThreshold, trajsDistMatrix);

	// compute clustering centers
	std::vector<double> clusterPercentage;
	//clusterPercentage = cComponent.findClusteringCenters(origTrajsResample, centerTrajs, labels, centerLabels, colors, numOfClusters);
	clusterPercentage = cComponent.findClusteringMedroids(origTrajsResample, centerTrajs, labels, centerLabels, colors, numOfClusters, trajsDistMatrix);
	std::cout << centerTrajs.size() << std::endl;
	cComponent.compDrawTrajectories(centerTrajs, centerLabels, colors, "Test.png", "Medroids");

	//cComponent.compDrawTrajectories(centerTrajs, centerLabels, colors, cComponent.imageName, "Clustering Centers");
	cComponent.compDrawTrajectoriesWithPercentage(centerTrajs, centerLabels, colors, cComponent.imageName, "Clustering Centers", clusterPercentage);
	//cComponent.compDrawCenters(centerTrajs, centerLabels, colors, cComponent.imageName, "Group Directions");

	std::vector<traj_elem_t> centerTrajsResample = cComponent.featureExtractionWithSmooth(centerTrajs, 50, cv::INTER_CUBIC, SMOOTH_GAUSSIAN_SIZE);

	//cComponent.compDrawCentersWithCurve(centerTrajsResample, centerLabels, colors, cComponent.imageName, "Center Points");
	cComponent.compDrawTrajectoriesWithPercentage(centerTrajsResample, centerLabels, colors, cComponent.imageName, "Smoothed Centers", clusterPercentage);

	cv::waitKey(0);

	return 0;
}