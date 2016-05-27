#define _USE_MATH_DEFINES
#include "Clustering.h"
#include "point.h"
#include <iostream>
#include <map>
#include <limits>
#include <algorithm>
#include <math.h>
#include <cmath>
#include <ctime>

void pair_insert(std::vector<std::pair<int, float>>& targetVector, std::pair<int, float> targetPair) {
	std::vector<std::pair<int, float>>::iterator it = std::lower_bound(targetVector.begin(), targetVector.end(), targetPair, sort_pair());
	targetVector.insert(it, targetPair);
}

std::vector<double> conv1d(std::vector<double> src, std::vector<double> gKernel) {
	//int i, j, k;
	std::vector<double> result, paddedSrc;

	int halfSize = gKernel.size() / 2;

	// init paddedSrc (size = len(src) + len(gKernel) - 1)
	for (int i = 0; i < halfSize; i++) {
		paddedSrc.push_back(src[halfSize - i]); // symmetric padding with elems center around first point
		//paddedSrc.push_back(src.front());
	}

	for (int i = 0; i < src.size(); i++) {
		paddedSrc.push_back(src[i]); // fill with elems in src
	}

	for (int i = 0; i < halfSize; i++) {
		int last = src.size() - 1;
		paddedSrc.push_back(src[last - i - 1]); // symmetric padding with elems center around last point
		//paddedSrc.push_back(src.back());
	}

	// set start index into paddedSrc
	int srcIndex = halfSize;
	for (int i = srcIndex; i < paddedSrc.size() - halfSize; i++) {
		double sum = 0.0;
		int j, gIndex;
		for (j = i - halfSize, gIndex = 0; j <= i + halfSize; j++, gIndex++) {
			sum = sum + paddedSrc[j] * gKernel[gIndex];
		}
		result.push_back(sum);
	}

	return result;
}

cv::Mat conv2d(const cv::Mat& src, const cv::Mat& kernel, ConvolutionType type) {
	cv::Mat dest;
	cv::Mat source = src;
	if (type == CONVOLUTION_FULL) {
		source = cv::Mat();
		const int additionalRows = kernel.rows - 1;
		const int additionalCols = kernel.cols - 1;
		cv::copyMakeBorder(src, source, (additionalRows + 1) / 2, additionalRows / 2, (additionalCols + 1) / 2, 
			additionalCols / 2, cv::BORDER_CONSTANT, cv::Scalar(0));
	}

	cv::Point anchor(kernel.cols - kernel.cols/2 - 1, kernel.rows - kernel.rows/2 - 1);
	cv::Mat flippedKernel;
	cv::flip(kernel, flippedKernel, -1);
	cv::filter2D(source, dest, src.depth(), flippedKernel, anchor, 0, cv::BORDER_CONSTANT);

	if (type == CONVOLUTION_VALID) {
		dest = dest.colRange((kernel.cols - 1) / 2, dest.cols - kernel.cols / 2);
		dest = dest.rowRange((kernel.rows - 1) / 2, dest.rows - kernel.rows / 2);
	}

	return dest;
}

cv::Mat getGaussianKernel2D(int rows, int cols, double sigma) {

	cv::Mat gKernel(rows, cols, CV_64F);

	double halfSize = (double)gKernel.rows / 2.0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			double x = (double)j - halfSize;
			double y = (double)i - halfSize;
			gKernel.at<double>(j, i) = (1.0 / (M_PI*pow(sigma, 4))) * (1 - (x*x + y*y) / (sigma*sigma))* (pow(2.718281828, -(x*x + y*y) / 2 * sigma*sigma));
		}
	}

	return gKernel;
}

Matrix buildDensityMatrix(std::vector<traj_elem_t>& trajs, double currRadius, double &minX, double &maxX, double &minY, double &maxY) {

	int floorX = (int)floor(minX);
	int ceilX = (int)ceil(maxX);
	int floorY = (int)floor(minY);
	int ceilY = (int)ceil(maxY);

	int rowSize = ceilY - floorY + 1;
	int colSize = ceilX - floorX + 1;

	Matrix densityMatrix(rowSize, Column(colSize, 0.0));

	for (size_t i = 0; i < trajs.size(); i++) {
		for (size_t j = 0; j < trajs[i].points.size(); j++) {
			point_t currPoint = trajs[i].points[j];
			int rowIndex = std::max(floorY, (int)floor(currPoint.y_coordinate));
			int colIndex = std::max(floorX, (int)floor(currPoint.x_coordinate));
			//densityMatrix[rowIndex - floorY + 1][colIndex - floorX + 1] = densityMatrix[rowIndex - floorY + 1][colIndex - floorX + 1] + 1.0; 
			densityMatrix[rowIndex - floorY][colIndex - floorX] = densityMatrix[rowIndex - floorY][colIndex - floorX] + 1.0;
		}
	}

	// update each cell in matrix to logarithm	
	for (int i = 0; i < rowSize; i++) {
		for (int j = 0; j < colSize; j++) {
			densityMatrix[i][j] = log(densityMatrix[i][j] + 1.0);
		}
	}
	
	// convolution with Gaussian filter ...
	cv::Mat densityCVMat(rowSize, colSize, CV_64F);
	for (int i = 0; i < rowSize; i++) {
		for (int j = 0; j < colSize; j++) {
			densityCVMat.at<double>(i, j) = densityMatrix[i][j];
		}
	}

	//cv::Mat gKernel = cv::getGaussianKernel((int)ceil(currRadius) * 2 + 1, currRadius / 3.0, CV_64F); // 1D Gaussian (size * 1)
	cv::Mat gKernel = getGaussianKernel2D((int)ceil(currRadius) * 2 + 1, (int)ceil(currRadius) * 2 + 1, currRadius / 3.0);
	//cv::Mat gKernelTrans = gKernel.t();
	cv::Mat resultCVMat = conv2d(densityCVMat, gKernel, CONVOLUTION_SAME);
	//cv::GaussianBlur(densityCVMat, resultCVMat, cv::Size((int)ceil(currRadius) * 2 + 1, (int)ceil(currRadius) * 2 + 1), currRadius / 3.0, currRadius / 3.0);
	for (int i = 0; i < rowSize; i++) {
		for (int j = 0; j < colSize; j++) {
			densityMatrix[i][j] = resultCVMat.at<double>(i, j);
		}
	}

	return densityMatrix;
}

// create points collection, find min/max x/y
std::vector<point_t> createPointsCollection(std::vector<traj_elem_t>& trajs) {
	std::vector<point_t> pointsCollection;
	for (size_t i = 0; i < trajs.size(); i++) {
		for (size_t j = 0; j < trajs[i].points.size(); j++) {
			pointsCollection.push_back(trajs[i].points[j]);
		}
	}
	return pointsCollection;
}

std::vector<point_t> createPointsCollection(std::vector<traj_elem_t>& trajs, double &minX, double &maxX, double &minY, double &maxY) {
	std::vector<point_t> pointsCollection;
	for (size_t i = 0; i < trajs.size(); i++) {
		for (size_t j = 0; j < trajs[i].points.size(); j++) {
			point_t currPoint = trajs[i].points[j];
			pointsCollection.push_back(trajs[i].points[j]);
			if (minX > currPoint.x_coordinate) {
				minX = currPoint.x_coordinate;
			} 
			if (maxX < currPoint.x_coordinate) {
				maxX = currPoint.x_coordinate;
			}
			if (minY > currPoint.y_coordinate) {
				minY = currPoint.y_coordinate;
			}
			if (maxY < currPoint.y_coordinate) {
				maxY = currPoint.y_coordinate;
			}
		}
	}
	//std::cout << maxX << " " << minX << " " << maxY << " " << minY << std::endl;
	return pointsCollection;
}

std::vector<traj_elem_t>& updateTrajs(std::vector<traj_elem_t>& trajs, std::vector<point_t>& pointsCollection) {

	int prevTrajId = -1;
	traj_elem_t newTraj; // make a new trajectory with updated point info

	for (size_t i = 0; i < pointsCollection.size(); i++) {
		int currTrajId = pointsCollection[i].trajId;
		if (prevTrajId != currTrajId) {
			// add completed trajectory to trajs
			if (prevTrajId != -1) {
				trajs[prevTrajId] = newTraj;
				newTraj.points.clear();
			}
			prevTrajId = currTrajId; // store newly found trajId
			newTraj.trajId = currTrajId;
			newTraj.numOfPoints = trajs[currTrajId].numOfPoints;
			newTraj.clusterId = trajs[currTrajId].clusterId;
		}
		newTraj.points.push_back(pointsCollection[i]);
	}

	for (size_t j = 0; j < trajs.size(); j++) {
		traj_elem_t currTraj = trajs[j];
		for (size_t k = 0; k < trajs[j].points.size(); k++) {
			point_t currPoint = trajs[j].points[k];
			if (k != 0) {
				point_t prevPoint = trajs[j].points[k - 1];
				trajs[j].points[k].speed_x = currPoint.x_coordinate - prevPoint.x_coordinate;
				trajs[j].points[k].speed_y = currPoint.y_coordinate - prevPoint.y_coordinate;
			}
			trajs[j].points[k].start_x = currTraj.points.front().x_coordinate;
			trajs[j].points[k].start_y = currTraj.points.front().y_coordinate;
			trajs[j].points[k].end_x = currTraj.points.back().x_coordinate;
			trajs[j].points[k].end_y = currTraj.points.back().y_coordinate;
		}
	}

	return trajs;
}

NeighborPointsMap createNeighborsMap(std::vector<point_t>& pointsCollection,
	std::vector<point_t>& newPointsCollection, double currRadius) {

	// calculate pairwise distance between every two points in collection 
	// store result in a matrix
	//int numOfPoints = pointsCollection.size();
	//std::cout << "Create distMatrix ..." << std::endl;
	//Matrix distMatrix(numOfPoints, Column(numOfPoints, 0.0));
	//std::cout << distMatrix.size() << std::endl;
	/*for (int i = 0; i < numOfPoints; i++) {
		for (int j = 0; j < numOfPoints; j++) {
			if (j > i) {
				distMatrix[i][j] = sqrt(pow(pointsCollection[i].x_coordinate - pointsCollection[j].x_coordinate, 2.0) +
					pow(pointsCollection[i].y_coordinate - pointsCollection[j].y_coordinate, 2.0));
			}
			else if (j < i) {
				distMatrix[i][j] = distMatrix[j][i];
			}
		}
	}*/

	newPointsCollection.resize(pointsCollection.size());
	NeighborPointsMap neighborsMap;
	
#pragma omp parallel for schedule(static,1) num_threads(4)
	for (int j = 0; j < pointsCollection.size(); j++) {
		point_t currPoint = pointsCollection[j];
		std::vector<std::pair<int, float>> currNeighbors;
		for (int k = 0; k < pointsCollection.size(); k++) {
			if (j != k) {
				point_t currTestPoint = pointsCollection[k];

				// calculate Euclidean distance only if currTestPoint is roughly within range
				if (currTestPoint.x_coordinate >= currPoint.x_coordinate - currRadius &&
					currTestPoint.x_coordinate <= currPoint.x_coordinate + currRadius &&
					currTestPoint.y_coordinate >= currPoint.y_coordinate - currRadius &&
					currTestPoint.y_coordinate <= currPoint.y_coordinate + currRadius) {

					float currDistance = (float)sqrt(pow((currPoint.x_coordinate - currTestPoint.x_coordinate), 2.0) +
						pow((currPoint.y_coordinate - currTestPoint.y_coordinate), 2.0));
					if (currDistance <= currRadius) {
						std::pair<int, float> currPair(k, currDistance);
						currNeighbors.push_back(currPair);
						//pair_insert(currNeighbors, currPair);
					}
				}
			}

			if (currNeighbors.size() >= 250) {
				break;
			}
		}
		/*for (int k = 0; k < numOfPoints; k++) {
			if (j != k && distMatrix[j][k] <= currRadius) {
				std::pair<int, double> currPair(k, distMatrix[j][k]);
				currNeighbors.push_back(currPair);
			}
		}*/

		// constrain neighbor size 
		if (currNeighbors.size() > MAX_NEIGHBORS_SIZE) {
			std::vector<std::pair<int, float>> resizeCurrNeighbors;
			std::sort(currNeighbors.begin(), currNeighbors.end(), sort_pair()); // sort into ascending order
			for (int i = 0; i < MAX_NEIGHBORS_SIZE; i++) {
				resizeCurrNeighbors.push_back(currNeighbors[i]);
			}
			//neighborsMap.insert(std::pair<int, std::vector<std::pair<int, double>>>(j, resizeCurrNeighbors));
			currNeighbors = resizeCurrNeighbors; // update currNeighbors
		}
		
		// critical section for inserting element into neighborsMap
#pragma omp critical 
{
		neighborsMap.insert(std::pair<int, std::vector<std::pair<int, float>>>(j, currNeighbors));
}

		if (currNeighbors.size() >= 10) {

			// create a weighting function for currPoint after getting all of its neighbors
			std::vector<double> weights;
			double weightsSum = 0.0;
			for (size_t l = 0; l < currNeighbors.size(); l++) {
				double currDist = currNeighbors[l].second;
				double currWeight = exp(-pow(currDist, 2.0) / pow(currRadius, 2.0));
				weights.push_back(currWeight);
				weightsSum = weightsSum + currWeight;
			}

			for (size_t m = 0; m < weights.size(); m++) {
				weights[m] = weights[m] / weightsSum;
			}

			// use weights to update currPoint coordinates
			double xSum = 0.0;
			double ySum = 0.0;
			for (size_t n = 0; n < currNeighbors.size(); n++) {
				int currNeighborIndex = currNeighbors[n].first;
				point_t currNeighborPoint = pointsCollection[currNeighborIndex];
				xSum = xSum + currNeighborPoint.x_coordinate * weights[n];
				ySum = ySum + currNeighborPoint.y_coordinate * weights[n];
			}
			point_t newPoint = {
				currPoint.trajId,
				(float)xSum,
				(float)ySum,
				currPoint.speed_x,
				currPoint.speed_y,
				currPoint.start_x,
				currPoint.start_y,
				currPoint.end_x,
				currPoint.end_y,
			};
			newPointsCollection[j] = newPoint;
			//newPointsCollection.push_back(newPoint);
		}
		else {
			point_t newPoint = currPoint;
			newPointsCollection[j] = newPoint;
			//newPointsCollection.push_back(newPoint);
		}

		/*if (j % 1000 == 0) {
			std::cout << ">>>>>> Finish processing " << j << " points." << std::endl;
		}*/
	}
	return neighborsMap;
}

NeighborPointsMap fastNeighborsMap(std::vector<point_t>& pointsCollection, std::vector<point_t>& newPointsCollection,
	NeighborPointsMap& originalMap, double currRadius) {

	newPointsCollection.resize(pointsCollection.size());
	NeighborPointsMap resultMap;

#pragma omp parallel for schedule(static,1) num_threads(4)

	for (int j = 0; j < pointsCollection.size(); j++) {
		point_t currPoint = pointsCollection[j];
		std::vector<std::pair<int, float>> currNeighbors;
		// only iterate over the neighbors previously found
		for (size_t k = 0; k < originalMap.find(j)->second.size(); k++) {
			int currNeighborIndex = originalMap.find(j)->second[k].first;
			point_t currTestPoint = pointsCollection[currNeighborIndex];

			// calculate Euclidean distance only if currTestPoint is roughly within range
			if (currTestPoint.x_coordinate >= currPoint.x_coordinate - currRadius &&
				currTestPoint.x_coordinate <= currPoint.x_coordinate + currRadius &&
				currTestPoint.y_coordinate >= currPoint.y_coordinate - currRadius &&
				currTestPoint.y_coordinate <= currPoint.y_coordinate + currRadius) {

				float currDistance = (float)sqrt(pow((currPoint.x_coordinate - currTestPoint.x_coordinate), 2.0) +
					pow((currPoint.y_coordinate - currTestPoint.y_coordinate), 2.0));
				if (currDistance <= currRadius) {
					std::pair<int, float> currPair(currNeighborIndex, currDistance);
					currNeighbors.push_back(currPair);
				}
			}
		}

		// critical section for inserting element into resultMap
#pragma omp critical 
{
		resultMap.insert(std::pair<int, std::vector<std::pair<int, float>>>(j, currNeighbors));
}

		if (currNeighbors.size() >= 5) {

			// create a weighting function for currPoint after getting all of its neighbors
			std::vector<double> weights;
			double weightsSum = 0.0;
			for (size_t l = 0; l < currNeighbors.size(); l++) {
				double currDist = currNeighbors[l].second;
				double currWeight = exp(-pow(currDist, 2.0) / pow(currRadius, 2.0));
				weights.push_back(currWeight);
				weightsSum = weightsSum + currWeight;
			}

			for (size_t m = 0; m < weights.size(); m++) {
				weights[m] = weights[m] / weightsSum;
			}

			// use weights to update currPoint coordinates
			double xSum = 0.0;
			double ySum = 0.0;
			for (size_t n = 0; n < currNeighbors.size(); n++) {
				int currNeighborIndex = currNeighbors[n].first;
				point_t currNeighborPoint = pointsCollection[currNeighborIndex];
				xSum = xSum + currNeighborPoint.x_coordinate * weights[n];
				ySum = ySum + currNeighborPoint.y_coordinate * weights[n];
			}
			point_t newPoint = {
				currPoint.trajId,
				(float)xSum,
				(float)ySum,
				currPoint.speed_x,
				currPoint.speed_y,
				currPoint.start_x,
				currPoint.start_y,
				currPoint.end_x,
				currPoint.end_y,
			};
			newPointsCollection[j] = newPoint;
			//newPointsCollection.push_back(newPoint);
		}
		else {
			point_t newPoint = currPoint;
			newPointsCollection[j] = newPoint;
			//newPointsCollection.push_back(newPoint);
		}
	}

	return resultMap;
}


//std::vector<point_t> meanShiftClustering(std::vector<traj_elem_t>& trajs, double currRadius) {
//	std::vector<point_t> newPointsCollection;
//	double maxX = std::numeric_limits<double>::min();
//	double maxY = std::numeric_limits<double>::min();
//	double minX = std::numeric_limits<double>::max();
//	double minY = std::numeric_limits<double>::max();
//	std::vector<point_t> pointsCollection = createPointsCollection(trajs, minX, maxX, minY, maxY);
//	NeighborPointsMap neighborsMap = createNeighborsMap(pointsCollection, newPointsCollection, currRadius);
//	// trajs before update
//	//printPoint(trajs[3].points[5]);
//	trajs = updateTrajs(trajs, newPointsCollection);
//	// trajs after update
//	//printPoint(trajs[3].points[5]);
//
//	std::vector<point_t> anotherNewPointsCollection;
//	NeighborPointsMap updatedMap = fastNeighborsMap(newPointsCollection, anotherNewPointsCollection, neighborsMap, currRadius - 5.0);
//	trajs = updateTrajs(trajs, anotherNewPointsCollection);
//
//	return anotherNewPointsCollection;
//}

std::vector<point_t> fastAMKSClustering(std::vector<traj_elem_t>& trajs, NeighborPointsMap& neighborsMap, double currRadius) {

	std::vector<point_t> newPointsCollection;
	double maxX = std::numeric_limits<double>::min();
	double maxY = std::numeric_limits<double>::min();
	double minX = std::numeric_limits<double>::max();
	double minY = std::numeric_limits<double>::max();
	std::vector<point_t> pointsCollection = createPointsCollection(trajs, minX, maxX, minY, maxY);
	Matrix densityMatrix = buildDensityMatrix(trajs, currRadius, minX, maxX, minY, maxY);

	newPointsCollection.resize(pointsCollection.size());

	int floorX = (int)floor(minX);
	int floorY = (int)floor(minY);

#pragma omp parallel for schedule(static,1) num_threads(4)

	for (int i = 0; i < pointsCollection.size(); i++) {
		point_t currPoint = pointsCollection[i];
		std::vector<int> neighborsX, neighborsY;
		std::vector<double> geoDist, aimDist, speedDist, densityDist;

		std::vector<std::pair<int, float>> currNeighbors = neighborsMap.find(i)->second; // find all neighbors of a given point

		if (currNeighbors.size() >= 5) {

			// iterate over all neighbors and calculate distances
			for (size_t j = 0; j < currNeighbors.size(); j++) {
				point_t currNeighborPoint = pointsCollection[currNeighbors[j].first];

				geoDist.push_back(currNeighbors[j].second); 

				aimDist.push_back(sqrt(pow(currPoint.start_x - currNeighborPoint.start_x, 2.0) +
					pow(currPoint.start_y - currNeighborPoint.start_y, 2.0) + pow(currPoint.end_x - currNeighborPoint.end_x, 2.0) +
					pow(currPoint.end_y - currNeighborPoint.end_y, 2.0)));

				speedDist.push_back(sqrt(pow(currPoint.speed_x - currNeighborPoint.speed_x, 2.0) +
					pow(currPoint.speed_y - currNeighborPoint.speed_y, 2.0)));

				if ((int)floor(currNeighborPoint.x_coordinate) < floorX) {
					neighborsX.push_back(floorX);
				}
				else {
					neighborsX.push_back((int)floor(currNeighborPoint.x_coordinate));
				}

				if ((int)floor(currNeighborPoint.y_coordinate) < floorY) {
					neighborsY.push_back(floorY);
				}
				else {
					neighborsY.push_back((int)floor(currNeighborPoint.y_coordinate));
				}
			}

			for (size_t k = 0; k < neighborsY.size(); k++) {
				//densityDist.push_back(densityMatrix[neighborsY[k] - floorY + 1][neighborsX[k] - floorX + 1]);
				densityDist.push_back(densityMatrix[neighborsY[k] - floorY][neighborsX[k] - floorX]);
			}

			// create weights with distances for each neighbor
			std::vector<double> weights;
			double weightSum = 0.0;
			for (size_t m = 0; m < currNeighbors.size(); m++) {

				double currWeight = densityDist[m] * exp(-pow(speedDist[m], 2.0) / pow(currRadius, 2.0)) *
					exp(-pow(geoDist[m], 2.0) / pow(currRadius, 2.0)) * exp(-aimDist[m] / currRadius);

				weights.push_back(currWeight);
				weightSum = weightSum + currWeight;
			}
			for (size_t n = 0; n < weights.size(); n++) {
				weights[n] = weights[n] / weightSum;
			}

			// calculate new coordinates for current point with weights
			double xSum = 0.0;
			double ySum = 0.0;
			double xSpeed = 0.0;
			double ySpeed = 0.0;
			for (size_t n = 0; n < currNeighbors.size(); n++) {
				int currNeighborIndex = currNeighbors[n].first;
				point_t currNeighborPoint = pointsCollection[currNeighborIndex];
				xSum = xSum + currNeighborPoint.x_coordinate * weights[n];
				ySum = ySum + currNeighborPoint.y_coordinate * weights[n];
				xSpeed = xSpeed + currNeighborPoint.speed_x * weights[n];
				ySpeed = ySpeed + currNeighborPoint.speed_y * weights[n];
			}

			point_t newPoint = {
				currPoint.trajId,
				(float)xSum,
				(float)ySum,
				(float)xSpeed,
				(float)ySpeed,
				currPoint.start_x,
				currPoint.start_y,
				currPoint.end_x,
				currPoint.end_y,
			};
			newPointsCollection[i] = newPoint;
			//newPointsCollection.push_back(newPoint);
		}
		else {
			point_t newPoint = currPoint;
			newPointsCollection[i] = newPoint;
			//newPointsCollection.push_back(newPoint);
		}
	}

	return newPointsCollection;
}

// reference: cv::partition
int trajsPartition(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, double distThreshold, std::vector<std::vector<float>>& distMatrix) {

	int i, j, N = trajs.size();
	const traj_elem_t* traj = &trajs[0];

	std::vector<std::vector<bool>> matrix(N, std::vector<bool>(N)); // i-j: in same cluster?

	const int PARENT = 0;
	const int RANK = 1;

	std::vector<int> _nodes(N * 2);
	int(*nodes)[2] = (int(*)[2])&_nodes[0];

	// First O(N) pass: create N single-vertex trees
	for (i = 0; i < N; i++) {
		nodes[i][PARENT] = -1;
		nodes[i][RANK] = 0;
	}

	// The main O(N^2) pass: merge connected components
	for (i = 0; i < N; i++) {

		//std::clock_t oneTrajStart = std::clock();

		int root = i;

		// find root
		while (nodes[root][PARENT] >= 0)
		{
			root = nodes[root][PARENT];
		}

#pragma omp parallel for schedule(static,1) num_threads(4)

		for (j = 0; j < N; j++) {

			bool inSame;
			float trajsDist = 0.0;

			if (j > i) {
				inSame = isInSameCluster(trajs[i], trajs[j], distThreshold, trajsDist);
			}
			else if (j < i) {
				inSame = matrix[j][i];
				trajsDist = distMatrix[j][i];
			}
			else {
				inSame = true;
				trajsDist = 0.0;
			}

#pragma omp critical 
			{
				matrix[i][j] = inSame;
				distMatrix[i][j] = trajsDist;
			}

			//if (i == j || !isInSameCluster(trajs[i], trajs[j], distThreshold))
			if (i == j || !matrix[i][j])
				continue;
			int root2 = j;

			while (nodes[root2][PARENT] >= 0)
			{
				root2 = nodes[root2][PARENT];
			}

			if (root2 != root) {
				// unite both trees
				int rank = nodes[root][RANK], rank2 = nodes[root2][RANK];

#pragma omp critical 
				{
					if (rank > rank2) {
						nodes[root2][PARENT] = root;
					}
					else {
						nodes[root][PARENT] = root2;
						nodes[root2][RANK] += rank == rank2;
						root = root2;
					}

					//CV_Assert(nodes[root][PARENT] < 0);

					int k = j, parent;

					// compares the path from node to root

					while ((parent = nodes[k][PARENT]) >= 0) {
						nodes[k][PARENT] = root;
						k = parent;
					}


					// compares the path from node to root
					k = i;

					while ((parent = nodes[k][PARENT]) >= 0) {
						nodes[k][PARENT] = root;
						k = parent;
					}
				}
			}
		}

		//double oneTrajDuration = (std::clock() - oneTrajStart) / (double)CLOCKS_PER_SEC;
		//std::cout << "Process Traj " << i << " takes " << oneTrajDuration << " seconds." << std::endl;

	}

	// Final O(N) pass: enumerate classes
	labels.resize(N);
	int nclasses = 0;

	for (i = 0; i < N; i++) {
		int root = i;
		while (nodes[root][PARENT] >= 0) {
			root = nodes[root][PARENT];
		}

		// reuse rank as class label
		if (nodes[root][RANK] >= 0) {
			nodes[root][RANK] = ~nclasses++;
		}
		labels[i] = ~nodes[root][RANK];
	}

	return nclasses;
}

bool isInSameCluster(traj_elem_t firstTraj, traj_elem_t secondTraj, double distThreshold, float& trajsDist) {
	int rowSize = firstTraj.points.size() + 1;
	int colSize = secondTraj.points.size() + 1;
	Matrix dtw(rowSize, Column(colSize, 0.0));

	// init dtw matrix
	for (int i = 1; i < rowSize; i++) {
		dtw[i][0] = std::numeric_limits<double>::max();
	}
	for (int j = 1; j < colSize; j++) {
		dtw[0][j] = std::numeric_limits<double>::max();
	}

	for (int i = 1; i < rowSize; i++) {
		// consider only points nearby
		for (int j = i - 3; j <= i + 3; j++) {
			if (j >= 1 && j < colSize) {
				float cost = pointDifference(firstTraj.points[i - 1], secondTraj.points[j - 1]);
				dtw[i][j] = cost + std::min(dtw[i - 1][j], std::min(dtw[i][j - 1], dtw[i - 1][j - 1]));
			}
		}
		/*for (int j = 1; j < colSize; j++) {
			double cost = pointDifference(firstTraj.points[i - 1], secondTraj.points[j - 1]);
			dtw[i][j] = cost + std::min(dtw[i-1][j], std::min(dtw[i][j-1], dtw[i-1][j-1]));
		}*/
	}

	trajsDist = dtw[rowSize - 1][colSize - 1] / (rowSize - 1);
	double dtwDist = (double)dtw[rowSize - 1][colSize - 1] / (rowSize - 1);
	if (dtwDist < distThreshold) {
		return true;
	}

	return false;

	//double sum = 0.0;
	//for (size_t i = 0; i < firstTraj.points.size(); i++) {
	//	point_t currPoint = firstTraj.points[i];
	//	double dist = std::numeric_limits<double>::max();
	//	for (size_t j = 0; j < secondTraj.points.size(); j++) {
	//		point_t testPoint = secondTraj.points[j];
	//		//double testDist = sqrt(pow(currPoint.x_coordinate - testPoint.x_coordinate, 2.0) +
	//			//pow(currPoint.y_coordinate - testPoint.y_coordinate, 2.0));
	//		double testDist = pointDifference(currPoint, testPoint);
	//		if (testDist < dist) {
	//			dist = testDist;
	//		}
	//	}
	//	sum = sum + dist;
	//}

	//double avg_sum = exp(sum / 30.0);

	//if (avg_sum < 140.0) {
	//	return true;
	//}

	//return false;
}

//bool isAbnormal(std::vector<traj_elem_t> centerTrajs, traj_elem_t testTraj) {
//	for (size_t i = 0; i < centerTrajs.size(); i++) {
//		if (centerTrajs[i].numOfPoints != 0 && isInSameCluster(centerTrajs[i], testTraj)) {
//			return false;
//		}
//	}
//	return true;
//}