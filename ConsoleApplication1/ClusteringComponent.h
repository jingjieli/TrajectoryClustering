#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdio>
#include <ctime>
#include <math.h>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include "SVGCurveLib.h"

#include "point.h"
#include "trajectory.h"
#include "InputFileHandler.h"
#include "StringSplit.h"
#include "ExtractFeature.h"
#include "Clustering.h"

#define M_PI 3.14159265358979323846
#ifndef CLUSTERINGCOMPONENT_H
#define CLUSTERINGCOMPONENT_H

typedef std::vector<point_t> PointsCollection;

class ClusteringComponent {
//private:
//	static std::string textFileName;
//	static std::string imageName;
//	static double startRadius;
//	static double endRadius;
//	static double currRadius;
//	static int targetSize;
//	static int iterations;
public:
	std::string textFileName; // input txt file to read trajectories data
	std::string imageName; // input background image 
	double startRadius; // search radius at the beginning
	double endRadius; // search radius at the end 
	double currRadius; // current working radius
	int targetSize; // resample traj size
	int iterations; // total iteration of clustering
	ClusteringComponent(std::string srcFileName, std::string srcImageName, double maxRadius, double minRadius, int resampleSize, int iterations);
	//~ClusteringComponent();
	/*static std::string getTextFileName(void);
	static std::string getImageName(void);
	static double getStartRadius(void);
	static double getEndRadius(void);
	static double getCurrentRadius(void);
	static int getTargetTrajSize(void);
	static int getIterations(void);*/
	std::vector<traj_elem_t> featureExtraction(std::vector<traj_elem_t>& originalTrajs, int targetSize, int interpolation); // resample input trajectories to target size
	std::vector<traj_elem_t> featureExtractionWithSmooth(std::vector<traj_elem_t>& originalTrajs, int targetSize, int interpolation, int kernelSize);
	std::vector<traj_elem_t> resampleTrajectories(std::vector<traj_elem_t>& originalTrajs, int targetSize);
	std::vector<traj_elem_t> readTrajDataFromSrc(std::string filename, int filterSize); // handle input file
	std::vector<point_t> createPointsCollectionFromTrajs(std::vector<traj_elem_t>& trajs); // create a collection of points from all trajectories
	std::string to_string_with_precision(double input_value, int n);

	void compDrawTrajectories(std::vector<traj_elem_t>& trajs, std::string filename, std::string windowName);
	void compDrawTrajectories(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName);
	void compDrawTrajectoriesWithPercentage(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName, std::vector<double> percentages);
	void compDrawCenters(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName);
	void compDrawCluster(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, int clusterId, std::string filename, std::string windowName);
	void compDrawCentersWithCurve(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName);

	traj_elem_t computeClusterCenter(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, int clusterId); // calculate representative traj for a cluster
	traj_elem_t computeClusterCenterWithFlippedTraj(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, int clusterId); 

	void runAMKSClustering(PointsCollection& origPoints, PointsCollection& prevMSPoints, PointsCollection& prevAMKSPoints, NeighborPointsMap& prevMap, 
		std::vector<traj_elem_t>& msTrajs, std::vector<traj_elem_t>& amksTrajs, int iterations); // adaptive multi kernel clustering

	int findNumberOfClusters(std::vector<traj_elem_t>& trajs, std::vector<int>& labels); // return the number of clusters being identified
	std::vector<double> findClusteringCenters(std::vector<traj_elem_t>& origTrajs, std::vector<traj_elem_t>& centerTrajs,
		std::vector<int>& labels, std::vector<int>& centerLabels, std::vector<cv::Scalar> colors, int numberOfClusters); // find all centers for all clusters 
};

#endif 