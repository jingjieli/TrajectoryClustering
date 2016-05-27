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

#include "point.h"
#include "trajectory.h"
#include "InputFileHandler.h"
#include "StringSplit.h"
#include "ExtractFeature.h"
#include "Clustering.h"
#include "ClusteringComponent.h"

#define ITERATIONS 6
#define MAX_RADIUS 40.0
#define MIN_RADIUS 10.0
#define POINTS_PER_TRAJ 50

typedef std::vector<point_t> PointsCollection;

void drawTrajectories(std::vector<traj_elem_t>& trajs, std::string filename, std::string windowName);
void drawTrajectories(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName);
void drawCenters(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName);
void drawCluster(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, int clusterId, std::string filename, std::string windowName);
traj_elem_t computeClusterCenter(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, int clusterId);

int main() {

	std::string textFileName = "sciotracks.txt";
	std::string imageName = "Test.png";

	std::vector<cv::Scalar> colors = { cv::Scalar(240, 120, 100), cv::Scalar(170, 80, 110), cv::Scalar(120, 30, 60), cv::Scalar(140, 140, 200),
		cv::Scalar(140, 100, 140), cv::Scalar(240, 160, 80), cv::Scalar(170, 200, 120), cv::Scalar(150, 200, 60), 
		cv::Scalar(200, 170, 140), cv::Scalar(30, 60, 140), cv::Scalar(140, 140, 60), cv::Scalar(90, 60, 240),
		cv::Scalar(120, 120, 30), cv::Scalar(60, 200, 220), cv::Scalar(170, 230, 80), cv::Scalar(240, 170, 130),
		cv::Scalar(150, 180, 180), cv::Scalar(60, 240, 30), cv::Scalar(240, 130, 110), cv::Scalar(140, 60, 170),
		cv::Scalar(160, 160, 180), cv::Scalar(90, 200, 120), cv::Scalar(160, 160, 80), cv::Scalar(90, 170, 60) };

	/*double currRadius = MAX_RADIUS;
	int targetSize = POINTS_PER_TRAJ;*/

	std::vector<traj_elem_t> trajs = readTrajDataFromFile(textFileName, 2, 20);

	drawTrajectories(trajs, imageName, "Original");

	std::vector<traj_elem_t> repTrajs = readTrajDataFromFile("sciotracks_traclus.txt", 2, 20);

	drawTrajectories(repTrajs, imageName, "Center Trajs");

	//std::vector<traj_elem_t> msTrajs, amksTrajs, origTrajs;

	//NeighborPointsMap currNeighborsMap;

	//// resample trajectories
	//for (size_t i = 0; i < trajs.size(); i++) {
	//	msTrajs.push_back(resizeTrajectory(trajs[i], targetSize));
	//	amksTrajs.push_back(resizeTrajectory(trajs[i], targetSize));
	//}

	//// original trajs after resampling
	//origTrajs = amksTrajs;
	//drawTrajectories(origTrajs, imageName, "Resampled Trajectories");

	//PointsCollection origPointsCollection, msPointsCollection;
	////std::vector<PointsCollection> msCollections, amksCollections;
	////std::vector<NeighborPointsMap> mapsCollection;

	//PointsCollection prevMSPointsCollection, prevAMKSCollection;
	//NeighborPointsMap prevNeighborsMap;

	//origPointsCollection = createPointsCollection(msTrajs);
	//for (int iter = 1; iter <= ITERATIONS; iter++) {

	//	std::clock_t startTime = std::clock();

	//	std::cout << "=================" << std::endl;
	//	std::cout << "Enter iteration: " << iter << std::endl;

	//	PointsCollection newPointsCollection;
	//	NeighborPointsMap newNeighborsMap;

	//	if (iter == 1) {
	//		std::cout << "Call createNeighborsMap..." << std::endl;
	//		std::clock_t createMapStartTime = std::clock();
	//		newNeighborsMap = createNeighborsMap(origPointsCollection, newPointsCollection, currRadius);
	//		double createMapDuration = (std::clock() - createMapStartTime) / (double)CLOCKS_PER_SEC;
	//		std::cout << "createNeighborsMap takes " << createMapDuration << " seconds." << std::endl;
	//	}
	//	else {
	//		std::clock_t fastMapStartTime = std::clock();
	//		std::cout << "Call fastNeighborsMap..." << std::endl;
	//		newNeighborsMap = fastNeighborsMap(prevMSPointsCollection, newPointsCollection, prevNeighborsMap, currRadius);
	//		double fastMapDuration = (std::clock() - fastMapStartTime) / (double)CLOCKS_PER_SEC;
	//		std::cout << "fastNeighborsMap takes " << fastMapDuration << " seconds." << std::endl;
	//	}
	//	prevMSPointsCollection.clear();
	//	prevMSPointsCollection = newPointsCollection;
	//	prevNeighborsMap.clear();
	//	prevNeighborsMap = newNeighborsMap;
	//	msTrajs = updateTrajs(msTrajs, newPointsCollection);

	//	std::cout << "Call fastAMSKClustering..." << std::endl;
	//	std::clock_t fastAMKSStartTime = std::clock();
	//	PointsCollection amksPointsCollection = fastAMKSClustering(amksTrajs, newNeighborsMap, currRadius);
	//	double fastAMKSDuration = (std::clock() - fastAMKSStartTime) / (double)CLOCKS_PER_SEC;
	//	std::cout << "fastAMKSClustering takes " << fastAMKSDuration << " seconds." << std::endl;
	//	prevAMKSCollection.clear();
	//	prevAMKSCollection = amksPointsCollection;
	//	amksTrajs = updateTrajs(amksTrajs, amksPointsCollection);

	//	//drawTrajectories(amksTrajs, imageName, "Iter"+std::to_string(iter));

	//	currRadius = currRadius - (MAX_RADIUS - MIN_RADIUS) / (ITERATIONS - 1);

	//	std::cout << "Leave iteration: " << iter << std::endl;

	//	double iterDuration = (std::clock() - startTime) / (double)CLOCKS_PER_SEC;
	//	std::cout << "Iteration " << iter << " takes " << iterDuration << " seconds." << std::endl;
	//}

	//// find number of clusters from shrunk trajectories
	//std::vector<int> labels;
	//std::clock_t partitionStartTime = std::clock();
	//std::cout << "......" << std::endl;
	//int numberOfClusters = cv::partition(amksTrajs, labels, isInSameCluster);
	//double partitionDuration = (std::clock() - partitionStartTime) / (double)CLOCKS_PER_SEC;
	//std::cout << "Identified number of clusters " << numberOfClusters << " takes " << partitionDuration << " seconds." << std::endl;

	//// compute center for each cluster
	//std::vector<traj_elem_t> centerTrajs;
	//std::vector<int> centerLabels;
	//for (int id = 0; id < numberOfClusters; id++) {
	//	drawCluster(origTrajs, labels, colors, id, imageName, "Cluster " + std::to_string(id));
	//	traj_elem_t currCenter = computeClusterCenter(origTrajs, labels, id);
	//	centerTrajs.push_back(currCenter);
	//	centerLabels.push_back(id);
	//}
	//drawTrajectories(centerTrajs, centerLabels, colors, imageName, "Clustering Centers");
	//drawCenters(centerTrajs, centerLabels, colors, imageName, "Centers With Direction");

	////// find abnormal trajs
	////std::vector<traj_elem_t> abnormalTrajs;
	////for (size_t i = 0; i < origTrajs.size(); i++) {
	////	traj_elem_t currTraj = origTrajs[i];
	////	if (isAbnormal(centerTrajs, currTraj)) {
	////		abnormalTrajs.push_back(currTraj);
	////	}
	////}
	////std::cout << "Total " << abnormalTrajs.size() << " abnormal trajectories found." << std::endl;
	////drawTrajectories(abnormalTrajs, imageName, "Abnormal Trajectories");

	cv::waitKey(0);

	return 0;
}

void drawTrajectories(std::vector<traj_elem_t>& trajs, std::string filename, std::string windowName) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		if (trajs[i].points.size() >= 2) {
			int clusterId = trajs[i].clusterId;
			for (size_t j = 0; j < trajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate),
					cv::Point(trajs[i].points[j + 1].x_coordinate, trajs[i].points[j + 1].y_coordinate), cv::Scalar(110, 220, 0), 2, 8);
			}
		}

		int numOfPoints = trajs[i].points.size();

		if (numOfPoints >= 3) {

			float x = trajs[i].points.back().x_coordinate;
			float y = trajs[i].points.back().y_coordinate;

			// draw arrow at end point
			int arrowMagnitude = 18;
			double angle = std::atan2(trajs[i].points.back().y_coordinate - trajs[i].points[numOfPoints - 2].y_coordinate,
				trajs[i].points.back().x_coordinate - trajs[i].points[numOfPoints - 2].x_coordinate);

			double first_x = trajs[i].points.back().x_coordinate - arrowMagnitude * cos(angle + M_PI / 6.0);
			double first_y = trajs[i].points.back().y_coordinate - arrowMagnitude * sin(angle + M_PI / 6.0);

			cv::line(image, cv::Point2d(first_x, first_y), cv::Point2d(trajs[i].points.back().x_coordinate, trajs[i].points.back().y_coordinate), cv::Scalar(110, 220, 0), 2, CV_AA);

			double second_x = trajs[i].points.back().x_coordinate - arrowMagnitude * cos(angle - M_PI / 6.0);
			double second_y = trajs[i].points.back().y_coordinate - arrowMagnitude * sin(angle - M_PI / 6.0);

			cv::line(image, cv::Point2d(second_x, second_y), cv::Point2d(trajs[i].points.back().x_coordinate, trajs[i].points.back().y_coordinate), cv::Scalar(110, 220, 0), 2, CV_AA);
		}
	}

	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
	//cv::waitKey(0);
}

void drawTrajectories(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		if (trajs[i].points.size() >= 2) {
			int clusterId = trajs[i].clusterId;
			for (size_t j = 0; j < trajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate),
					cv::Point(trajs[i].points[j + 1].x_coordinate, trajs[i].points[j + 1].y_coordinate), colors[labels[i]], 1, 8);
			}
		}
	}
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
	//cv::waitKey(0);
}

void drawCenters(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		if (trajs[i].points.size() >= 2) {
			int clusterId = trajs[i].clusterId;
			/*for (size_t j = 0; j < trajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate),
					cv::Point(trajs[i].points[j + 1].x_coordinate, trajs[i].points[j + 1].y_coordinate), colors[labels[i]], 2, 8);
			}*/
			cv::arrowedLine(image, cv::Point(trajs[i].points.front().x_coordinate, trajs[i].points.front().y_coordinate),
				cv::Point(trajs[i].points.back().x_coordinate, trajs[i].points.back().y_coordinate), colors[labels[i]], 2, 8);
		}
	}
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
}

void drawCluster(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, int clusterId, std::string filename, std::string windowName) {
	std::vector<traj_elem_t> targetTrajs;
	for (size_t i = 0; i < labels.size(); i++) {
		if (labels[i] == clusterId) {
			targetTrajs.push_back(trajs[i]);
		}
	}

	double percentage = (double)targetTrajs.size() / (double)trajs.size() * 100.0;

	std::cout << "----------------------------------" << std::endl;
	std::cout << "Number of trajectories in cluster " << clusterId << ": " << targetTrajs.size() << " (" << std::setprecision(3) << percentage << "%)" << std::endl;

	if (targetTrajs.size() > 5) {

		cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
		if (image.empty()) {
			std::cout << "Fail to load image..." << std::endl;
		}
		for (size_t i = 0; i < targetTrajs.size(); i++) {
			for (size_t j = 0; j < targetTrajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point(targetTrajs[i].points[j].x_coordinate, targetTrajs[i].points[j].y_coordinate),
					cv::Point(targetTrajs[i].points[j + 1].x_coordinate, targetTrajs[i].points[j + 1].y_coordinate), colors[clusterId], 1, 8);
			}
		}
		cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
		cv::imshow(windowName, image);
		cv::waitKey(0);
	}
}

traj_elem_t computeClusterCenter(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, int clusterId) {
	traj_elem_t centerTraj;
	centerTraj.trajId = clusterId;
	centerTraj.clusterId = clusterId;

	// find all trajs in same cluster
	std::vector<traj_elem_t> targetTrajs;
	for (size_t i = 0; i < labels.size(); i++) {
		if (labels[i] == clusterId) {
			targetTrajs.push_back(trajs[i]);
		}
	}

	if (targetTrajs.size() > 5) {

		for (int j = 0; j < targetTrajs[0].numOfPoints; j++) {
			double xCoorSum = 0.0, yCoorSum = 0.0, speedXSum = 0.0, speedYSum = 0.0, startXSum = 0.0, startYSum = 0.0, endXSum = 0.0, endYSum = 0.0;
			for (size_t k = 0; k < targetTrajs.size(); k++) {
				point_t currPoint = targetTrajs[k].points[j];
				xCoorSum = xCoorSum + currPoint.x_coordinate;
				yCoorSum = yCoorSum + currPoint.y_coordinate;
				speedXSum = speedXSum + currPoint.speed_x;
				speedYSum = speedYSum + currPoint.speed_y;
				startXSum = startXSum + currPoint.start_x;
				startYSum = startYSum + currPoint.start_y;
				endXSum = endXSum + currPoint.end_x;
				endYSum = endYSum + currPoint.end_y;
			}
			point_t newPoint = {
				clusterId,
				xCoorSum / targetTrajs.size(),
				yCoorSum / targetTrajs.size(),
				speedXSum / targetTrajs.size(),
				speedYSum / targetTrajs.size(),
				startXSum / targetTrajs.size(),
				startYSum / targetTrajs.size(),
				endXSum / targetTrajs.size(),
				endYSum / targetTrajs.size(),
			};
			centerTraj.points.push_back(newPoint);
		}
	}
	else {
		centerTraj.points = {};
	}
	centerTraj.numOfPoints = centerTraj.points.size();

	return centerTraj;
}