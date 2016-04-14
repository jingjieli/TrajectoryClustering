#define _USE_MATH_DEFINES

#include "ExtractFeature.h"
#include "Clustering.h"
#include <cmath>
#include <vector>
#include <iomanip>
#include <iostream>

std::vector<std::vector<double>> createGaussianFilter(int size) {

	double sigma = size / 3.0;
	double sum = 0.0;
	std::vector<std::vector<double>> gKernel;
	int col = 1;
	int row = 2 * size + 1;
	gKernel.resize(row, std::vector<double>(col, 0)); 

	for (int x = -(row - 1) / 2; x <= (row - 1) / 2; x++) {
		int y = 0;
		double r = sqrt(x * x + y * y);
		gKernel[x + (row - 1)/2][y] = (exp(-(r*r) / (2.0*sigma*sigma))) / (M_PI*2.0*sigma*sigma);
		sum = sum + gKernel[x + (row - 1)/2][y];
	}

	for (int i = 0; i < row; i++) {
		gKernel[i][0] /= sum;
		std::cout << gKernel[i][0] << std::endl;
	}

	return gKernel;
}

traj_elem_t resizeTrajectory(traj_elem_t& currTraj, int targetSize, int interpolation) {
	traj_elem_t newTraj;
	newTraj.trajId = currTraj.trajId;
	newTraj.numOfPoints = targetSize;
	newTraj.clusterId = currTraj.clusterId;
	cv::vector<double> xCoordinates, yCoordinates;
	for (size_t i = 0; i < currTraj.points.size(); i++) {
		xCoordinates.push_back(currTraj.points[i].x_coordinate);
		yCoordinates.push_back(currTraj.points[i].y_coordinate);
	}

	// convert std::vector to cv::Mat
	cv::Mat xMat(xCoordinates, true);
	cv::Mat yMat(yCoordinates, true);

	// std::cout << xMat.size() << std::endl;
	// std::cout << yMat.size() << std::endl;
	// std::cout << xMat << std::endl;

	cv::Mat xResultMat, yResultMat, xFilteredMat, yFilteredMat;

	// resize to target size
	cv::resize(xMat, xResultMat, cv::Size(1, targetSize), 0, 0, interpolation);
	cv::resize(yMat, yResultMat, cv::Size(1, targetSize), 0, 0, interpolation);

	/*std::cout << xResultMat.size() << std::endl;
	std::cout << yResultMat.size() << std::endl;*/

	/*cv::Mat gKernel = cv::getGaussianKernel(7, 0, CV_64F);

	xFilteredMat = conv2(xResultMat, gKernel, CONVOLUTION_VALID);
	yFilteredMat = conv2(yResultMat, gKernel, CONVOLUTION_VALID);*/

	// std::cout << xResultMat << std::endl;
	// std::cout << xResultMat.size() << std::endl;
	// std::cout << yResultMat.size() << std::endl;

	// create a new traj
	for (int j = 0; j < targetSize; j++) {
		point_t newPoint = {
			currTraj.trajId,
			xResultMat.at<double>(j, 0),
			yResultMat.at<double>(j, 0),
			0.0,
			0.0,
			xResultMat.at<double>(0, 0),
			yResultMat.at<double>(0, 0),
			xResultMat.at<double>(targetSize-1, 0),
			yResultMat.at<double>(targetSize-1, 0),
		};
		newTraj.points.push_back(newPoint);
	}
	//std::cout << newTraj.points.size() << std::endl;
	return newTraj;
}

traj_elem_t resizeTrajectoryAndSmooth(traj_elem_t& currTraj, int targetSize, int interpolation, int kernelSize) {
	traj_elem_t newTraj;
	newTraj.trajId = currTraj.trajId;
	newTraj.numOfPoints = targetSize;
	newTraj.clusterId = currTraj.clusterId;
	cv::vector<double> xCoordinates, yCoordinates;
	for (size_t i = 0; i < currTraj.points.size(); i++) {
		xCoordinates.push_back(currTraj.points[i].x_coordinate);
		yCoordinates.push_back(currTraj.points[i].y_coordinate);
	}

	// convert std::vector to cv::Mat
	cv::Mat xMat(xCoordinates, true);
	cv::Mat yMat(yCoordinates, true);

	// std::cout << xMat.size() << std::endl;
	// std::cout << yMat.size() << std::endl;
	// std::cout << xMat << std::endl;

	cv::Mat xResultMat, yResultMat, xFilteredMat, yFilteredMat;

	// resize to target size
	cv::resize(xMat, xResultMat, cv::Size(1, targetSize), 0, 0, interpolation);
	cv::resize(yMat, yResultMat, cv::Size(1, targetSize), 0, 0, interpolation);

	/*std::cout << xResultMat.size() << std::endl;
	std::cout << yResultMat.size() << std::endl;*/

	cv::Mat gKernel = cv::getGaussianKernel(kernelSize, kernelSize / 6.0, CV_64F);

	xFilteredMat = conv2(xResultMat, gKernel, CONVOLUTION_SAME);
	yFilteredMat = conv2(yResultMat, gKernel, CONVOLUTION_SAME);

	// std::cout << xResultMat << std::endl;
	// std::cout << xResultMat.size() << std::endl;
	// std::cout << yResultMat.size() << std::endl;

	// create a new traj
	for (int j = 0; j < targetSize; j++) {
		point_t newPoint = {
			currTraj.trajId,
			xFilteredMat.at<double>(j, 0),
			yFilteredMat.at<double>(j, 0),
			0.0,
			0.0,
			xFilteredMat.at<double>(0, 0),
			yFilteredMat.at<double>(0, 0),
			xFilteredMat.at<double>(targetSize - 1, 0),
			yFilteredMat.at<double>(targetSize - 1, 0),
		};
		newTraj.points.push_back(newPoint);
	}
	//std::cout << newTraj.points.size() << std::endl;
	return newTraj;
}

traj_elem_t resampleTrajectory(traj_elem_t& currTraj, int targetSize) {
	traj_elem_t newTraj;
	newTraj.trajId = currTraj.trajId;
	newTraj.numOfPoints = targetSize;
	newTraj.clusterId = currTraj.clusterId;
	cv::vector<double> xCoordinates, yCoordinates, xResample, yResample;
	for (size_t i = 0; i < currTraj.points.size(); i++) {
		xCoordinates.push_back(currTraj.points[i].x_coordinate);
		yCoordinates.push_back(currTraj.points[i].y_coordinate);
	}

	for (int i = 0; i < targetSize - 1; i++) {
		int index = i * xCoordinates.size() / (targetSize - 1);
		int p = i * xCoordinates.size() % (targetSize - 1);

		xResample.push_back(((p * xCoordinates[index + 1]) + (xCoordinates.size() - p) * xCoordinates[index]) / (targetSize - 1));
		yResample.push_back(((p * yCoordinates[index + 1]) + (yCoordinates.size() - p) * yCoordinates[index]) / (targetSize - 1));
	}

	xResample.push_back(xCoordinates.back());
	yResample.push_back(yCoordinates.back());

	// create a new traj
	for (int j = 0; j < targetSize; j++) {
		point_t newPoint = {
			currTraj.trajId,
			xResample[j],
			yResample[j],
			0.0,
			0.0,
			xResample.front(),
			yResample.front(),
			xResample.back(),
			yResample.back(),
		};
		newTraj.points.push_back(newPoint);
	}
	//std::cout << newTraj.points.size() << std::endl;
	return newTraj;
}

/*
traj_elem_t gaussianFilter(traj_elem_t currTraj, std::vector<std::vector<double>> gKernel) {
	traj_elem_t filteredTraj;
	std::vector<double> xCoordinates, yCoordinates, convResult;
	for (size_t i = 0; i < currTraj.points.size(); i++) {
		point_t curr_point = currTraj.points[i];
		xCoordinates.push_back(curr_point.x_coordinate);
		yCoordinates.push_back(curr_point.y_coordinate);
	}

	cv::filter2D(xCoordinates, convResult, -1, gKernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
	std::cout << convResult.size() << std::endl;

	int convLength = currTraj.numOfPoints + gKernel.size() - 1;
	for (int i = 0; i < convLength; i++) {
		int iCopy = i;
		double temp_x = 0.0;
		double temp_y = 0.0;
		for (size_t j = 0; j < gKernel.size(); j++) {
			if (iCopy >= 0 && iCopy < currTraj.numOfPoints) {
				temp_x = temp_x + (xCoordinates[iCopy] * gKernel[j][0]);
				temp_y = temp_y + (yCoordinates[iCopy] * gKernel[j][0]);
			}
			iCopy = iCopy - 1;
			// convResult[i] 
		}
	}

	return filteredTraj;
}
*/
