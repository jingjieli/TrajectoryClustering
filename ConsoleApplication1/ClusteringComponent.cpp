#include "ClusteringComponent.h"

ClusteringComponent::ClusteringComponent(std::string srcFileName, std::string srcImageName, double maxRadius, double minRadius, int resampleSize, int iters) {
	textFileName = srcFileName;
	imageName = srcImageName;
	startRadius = maxRadius;
	endRadius = minRadius;
	currRadius = maxRadius;
	targetSize = resampleSize;
	iterations = iters;
}

//ClusteringComponent::~ClusteringComponent() {
//	std::cout << "Deconstructor method is called." << std::endl;
//}

//std::string ClusteringComponent::getTextFileName(void) {
//	return textFileName;
//}
//
//std::string ClusteringComponent::getImageName(void) {
//	return imageName;
//}
//
//double ClusteringComponent::getStartRadius(void) {
//	return startRadius;
//}
//
//double ClusteringComponent::getEndRadius(void) {
//	return endRadius;
//}
//
//double ClusteringComponent::getCurrentRadius(void) {
//	return currRadius;
//}
//
//int ClusteringComponent::getTargetTrajSize(void) { 
//	return targetSize;
//}
//
//int ClusteringComponent::getIterations(void) {
//	return iterations;
//}

std::vector<traj_elem_t> ClusteringComponent::featureExtraction(std::vector<traj_elem_t>& originalTrajs, int targetSize, int interpolation) {
	std::vector<traj_elem_t> newTrajs;
	for (size_t i = 0; i < originalTrajs.size(); i++) {
		newTrajs.push_back(resizeTrajectory(originalTrajs[i], targetSize, interpolation));
	}
	return newTrajs;
}

std::vector<traj_elem_t> ClusteringComponent::featureExtractionWithSmooth(std::vector<traj_elem_t>& originalTrajs, int targetSize, int interpolation, int kernelSize) {
	std::vector<traj_elem_t> newTrajs;
	for (size_t i = 0; i < originalTrajs.size(); i++) {
		newTrajs.push_back(resizeTrajectoryAndSmooth(originalTrajs[i], targetSize, interpolation, kernelSize));
	}
	return newTrajs;
}

std::vector<traj_elem_t> ClusteringComponent::readTrajDataFromSrc(std::string filename, int minTrajDataSize, int maxTrajDataSize) {
	std::vector<traj_elem_t> srcTrajs;
	srcTrajs = readTrajDataFromFile(filename, minTrajDataSize, maxTrajDataSize);
	return srcTrajs;
}

std::vector<point_t> ClusteringComponent::createPointsCollectionFromTrajs(std::vector<traj_elem_t>& trajs) {
	std::vector<point_t> newPointsCollection;
	newPointsCollection = createPointsCollection(trajs);
	return newPointsCollection;
}

void ClusteringComponent::compDrawTrajectories(std::vector<traj_elem_t>& trajs, std::string filename, std::string windowName) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		if (trajs[i].points.size() >= 2) {
			int clusterId = trajs[i].clusterId;
			for (size_t j = 0; j < trajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point2d(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate),
					cv::Point2d(trajs[i].points[j + 1].x_coordinate, trajs[i].points[j + 1].y_coordinate), cv::Scalar(110, 220, 0), 2, 8);
			}
		}
	}
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
	//cv::waitKey(0);
}

void ClusteringComponent::compDrawTrajectories(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		if (trajs[i].points.size() >= 2) {
			int clusterId = trajs[i].clusterId;
			for (size_t j = 0; j < trajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point2d(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate),
					cv::Point2d(trajs[i].points[j + 1].x_coordinate, trajs[i].points[j + 1].y_coordinate), colors[labels[i]], 3, CV_AA);
			}
		}
	}
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
	//cv::waitKey(0);
}

void ClusteringComponent::compDrawTrajectoriesWithPercentage(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, 
	std::vector<cv::Scalar>& colors, std::string filename, std::string windowName, std::vector<double> percentages) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		if (trajs[i].points.size() >= 2 && percentages[i] >= 1.0) {
		//if (trajs[i].points.size() >= 2) {
			std::cout << "Traj " << i << " " << percentages[i] << std::endl;
			int numOfPoints = trajs[i].points.size();
			//int clusterId = trajs[i].clusterId;
			for (size_t j = 0; j < trajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point2d(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate),
					cv::Point2d(trajs[i].points[j + 1].x_coordinate, trajs[i].points[j + 1].y_coordinate), colors[labels[i]], 3, CV_AA);
			}

			float x = trajs[i].points.back().x_coordinate;
			float y = trajs[i].points.back().y_coordinate;

			// draw arrow at end point
			int arrowMagnitude = 18;
			double angle = std::atan2(trajs[i].points.back().y_coordinate - trajs[i].points[numOfPoints - 2].y_coordinate,
				trajs[i].points.back().x_coordinate - trajs[i].points[numOfPoints - 2].x_coordinate);

			double first_x = trajs[i].points.back().x_coordinate - arrowMagnitude * cos(angle + M_PI / 6.0);
			double first_y = trajs[i].points.back().y_coordinate - arrowMagnitude * sin(angle + M_PI / 6.0);

			cv::line(image, cv::Point2d(first_x, first_y), cv::Point2d(trajs[i].points.back().x_coordinate, trajs[i].points.back().y_coordinate), colors[labels[i]], 3, CV_AA);

			double second_x = trajs[i].points.back().x_coordinate - arrowMagnitude * cos(angle - M_PI / 6.0);
			double second_y = trajs[i].points.back().y_coordinate - arrowMagnitude * sin(angle - M_PI / 6.0);

			cv::line(image, cv::Point2d(second_x, second_y), cv::Point2d(trajs[i].points.back().x_coordinate, trajs[i].points.back().y_coordinate), colors[labels[i]], 3, CV_AA);

			// cv::putText code here...
			if (x > image.size[0] - 20.0) {
				x = trajs[i].points.front().x_coordinate;
				y = trajs[i].points.front().y_coordinate;
			}

			cv::rectangle(
				image,
				cv::Point2f(x+12, y-18),
				cv::Point2f(x+19*4, y-2),
				cv::Scalar(255, 255, 255), CV_FILLED
				);

			cv::putText(image, to_string_with_precision(percentages[i], 3)+"%", cv::Point2d(x+12, y), 1.3, 1.3, cv::Scalar(0, 0, 0), 1, 8);
		}
	}
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
	//cv::waitKey(0);
}

void ClusteringComponent::compDrawCenters(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName) {
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
			cv::arrowedLine(image, cv::Point2d(trajs[i].points.front().x_coordinate, trajs[i].points.front().y_coordinate),
				cv::Point2d(trajs[i].points.back().x_coordinate, trajs[i].points.back().y_coordinate), colors[labels[i]], 2, 8);
		}
	}
	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
}

void ClusteringComponent::compDrawCluster(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, int clusterId, std::string filename, std::string windowName) {
	std::vector<traj_elem_t> targetTrajs;
	for (size_t i = 0; i < labels.size(); i++) {
		if (labels[i] == clusterId) {
			targetTrajs.push_back(trajs[i]);
		}
	}

	double percentage = (double)targetTrajs.size() / (double)trajs.size() * 100.0;

	std::cout << "----------------------------------" << std::endl;
	std::cout << "Number of trajectories in cluster " << clusterId << ": " << targetTrajs.size() << " (" << std::setprecision(3) << percentage << "%)" << std::endl;

	if (targetTrajs.size() >= 3) {

		cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
		if (image.empty()) {
			std::cout << "Fail to load image..." << std::endl;
		}
		for (size_t i = 0; i < targetTrajs.size(); i++) {
			for (size_t j = 0; j < targetTrajs[i].points.size() - 1; j++) {
				cv::line(image, cv::Point2d(targetTrajs[i].points[j].x_coordinate, targetTrajs[i].points[j].y_coordinate),
					cv::Point2d(targetTrajs[i].points[j + 1].x_coordinate, targetTrajs[i].points[j + 1].y_coordinate), colors[clusterId], 1, 8);
			}
		}
		cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
		cv::imshow(windowName, image);
		cv::waitKey(0);
	}
}

void ClusteringComponent::compDrawCentersWithCurve(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, std::vector<cv::Scalar>& colors, std::string filename, std::string windowName) {
	cv::Mat image = cv::imread(filename, CV_LOAD_IMAGE_UNCHANGED);
	if (image.empty()) {
		std::cout << "Fail to load image..." << std::endl;
	}

	for (size_t i = 0; i < trajs.size(); i++) {
		std::vector<cv::Point> inputCurve;
		if (trajs[i].points.size() >= 2) {
			int clusterId = trajs[i].clusterId;

			// estimate current traj
			for (size_t j = 0; j < trajs[i].points.size(); j++) {
				cv::Point2d newPoint = cv::Point2d(trajs[i].points[j].x_coordinate, trajs[i].points[j].y_coordinate);
				inputCurve.push_back(newPoint);
			}

			const cv::Point *pts = (const cv::Point*) cv::Mat(inputCurve).data;

			int numOfPoints = cv::Mat(inputCurve).rows;

			// draw current traj with approx.
			for (int j = 0; j < inputCurve.size(); j++) {
				cv::line(image, inputCurve[j], inputCurve[j], colors[labels[i]], 5, CV_AA);
			}

			// draw with svg-curve-lib
			/*SVGCurveLib::PointGeneric<> startPoint(inputCurve.front().x, inputCurve.front().y);
			SVGCurveLib::PointGeneric<> endPoint(inputCurve.back().x, inputCurve.back().y);
			int midIndex = inputCurve.size() / 2;
			SVGCurveLib::PointGeneric<> controlPoint(inputCurve[midIndex].x, inputCurve[midIndex].y);
			std::cout << SVGCurveLib::PointOnQuadraticBezierCurve(startPoint, endPoint, controlPoint, 0.5) << std::endl;*/

			//cv::polylines(image, &pts, &numOfPoints, 1, false, colors[labels[i]], 3, 8, 0);
		}
	}

	cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
	cv::imshow(windowName, image);
}

std::string ClusteringComponent::to_string_with_precision(double input_value, int n) 
{
	std::ostringstream out;
	out << std::setprecision(n) << input_value;
	return out.str();
}

traj_elem_t ClusteringComponent::findClusterMedroid(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, 
	int clusterId, std::vector<std::vector<float>>& distMatrix) {
	
	traj_elem_t medroidTraj;
	medroidTraj.trajId = clusterId;
	medroidTraj.clusterId = clusterId;

	// find all trajs in smae cluster
	std::vector<traj_elem_t> targetTrajs;
	for (size_t i = 0; i < labels.size(); i++) {
		if (labels[i] == clusterId) {
			targetTrajs.push_back(trajs[i]);
		}
	}

	// find medroid only for cluster with enough trajs 
	if (targetTrajs.size() >= 10) {

		float currMinSum = std::numeric_limits<float>::max(); // min dist currently known 
		int currMinIndex = 0; // the index of element to targetTrajs with currMinSum

		for (int i = 0; i < targetTrajs.size(); i++) {
			
			int currTrajIndex = targetTrajs[i].trajId;
			float distSum = 0.0;

			for (int j = 0; j < targetTrajs.size(); j++) {
				int testTrajIndex = targetTrajs[j].trajId;
				distSum = distSum + distMatrix[currTrajIndex][testTrajIndex];
			}

			distSum = distSum / (targetTrajs.size() - 1);

			if (distSum < currMinSum) {
				currMinSum = distSum;
				currMinIndex = i;
			}
		}

		// the traj with minimum distSum in targetTrajs
		traj_elem_t foundTraj = targetTrajs[currMinIndex];
		std::cout << "Traj found with index " << foundTraj.trajId << " with sum: " << currMinSum << std::endl;
		medroidTraj.points = foundTraj.points;
		medroidTraj.trajId = foundTraj.trajId;
	}
	else {
		medroidTraj.points = {};
	}
	medroidTraj.numOfPoints = medroidTraj.points.size();

	return medroidTraj;
}

traj_elem_t ClusteringComponent::computeClusterCenter(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, int clusterId) {
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

	if (targetTrajs.size() >= 10) {
		// compute cluster center only if there're enough tracks 
		for (int j = 0; j < targetTrajs[0].numOfPoints; j++) {
			float xCoorSum = 0.0, yCoorSum = 0.0, speedXSum = 0.0, speedYSum = 0.0, startXSum = 0.0, startYSum = 0.0, endXSum = 0.0, endYSum = 0.0;
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

traj_elem_t ClusteringComponent::computeClusterCenterWithFlippedTraj(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, int clusterId) {
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

	//// flipping routine
	//// flip trajs of different direction 
	//for (size_t i = 0; i < targetTrajs.size(); i++) {
	//	traj_elem_t currTraj = targetTrajs[i];
	//	// flip a traj starts from left to right
	//	if (currTraj.points.front().x_coordinate < currTraj.points.back().x_coordinate) {
	//		traj_elem_t newTraj;
	//		newTraj.trajId = currTraj.trajId;
	//		newTraj.clusterId = currTraj.clusterId;
	//		newTraj.numOfPoints = currTraj.numOfPoints;
	//		for (size_t j = currTraj.points.size() - 1; j >= 0; j--) {
	//			newTraj.points.push_back(currTraj.points[j]);
	//		}
	//		targetTrajs[i] = newTraj;
	//	}
	//}

	// compute cluster centers only if there're enough trajs in the group
	if (targetTrajs.size() >= 3) {

		// flipping routine
		// flip trajs of different direction 
		for (size_t i = 0; i < targetTrajs.size(); i++) {
			traj_elem_t currTraj = targetTrajs[i];
			// flip a traj starts from left to right
			if (currTraj.points.front().x_coordinate < currTraj.points.back().x_coordinate) {
				traj_elem_t newTraj;
				newTraj.trajId = currTraj.trajId;
				newTraj.clusterId = currTraj.clusterId;
				newTraj.numOfPoints = currTraj.numOfPoints;
				for (int j = currTraj.points.size() - 1; j >= 0; j--) {
					newTraj.points.push_back(currTraj.points[j]);
				}
				targetTrajs[i] = newTraj;
			}
		}

		for (int j = 0; j < targetTrajs[0].numOfPoints; j++) {
			float xCoorSum = 0.0, yCoorSum = 0.0, speedXSum = 0.0, speedYSum = 0.0, startXSum = 0.0, startYSum = 0.0, endXSum = 0.0, endYSum = 0.0;
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

void ClusteringComponent::runAMKSClustering(PointsCollection& origPoints, PointsCollection& prevMSPoints, PointsCollection& prevAMKSPoints, NeighborPointsMap& prevMap,
	std::vector<traj_elem_t>& msTrajs, std::vector<traj_elem_t>& amksTrajs, int iterations) {
		for (int iter = 1; iter <= iterations; iter++) {
	
			std::clock_t startTime = std::clock();
	
			std::cout << "=================" << std::endl;
			std::cout << "Enter iteration: " << iter << std::endl;
	
			PointsCollection newPointsCollection;
			newPointsCollection.resize(origPoints.size());
			NeighborPointsMap newNeighborsMap;
	
			if (iter == 1) {
				std::cout << "Call createNeighborsMap..." << std::endl;
				std::clock_t createMapStartTime = std::clock();
				newNeighborsMap = createNeighborsMap(origPoints, newPointsCollection, currRadius);
				double createMapDuration = (std::clock() - createMapStartTime) / (double)CLOCKS_PER_SEC;
				std::cout << "createNeighborsMap takes " << createMapDuration << " seconds." << std::endl;
			}
			else {
				std::clock_t fastMapStartTime = std::clock();
				std::cout << "Call fastNeighborsMap..." << std::endl;
				newNeighborsMap = fastNeighborsMap(prevMSPoints, newPointsCollection, prevMap, currRadius);
				double fastMapDuration = (std::clock() - fastMapStartTime) / (double)CLOCKS_PER_SEC;
				std::cout << "fastNeighborsMap takes " << fastMapDuration << " seconds." << std::endl;
			}
			
			//prevMSPoints.clear();
			prevMSPoints = newPointsCollection;
			//prevMap.clear();
			prevMap = newNeighborsMap;
			//msTrajs = updateTrajs(msTrajs, newPointsCollection);
	
			std::cout << "Call fastAMSKClustering..." << std::endl;
			std::clock_t fastAMKSStartTime = std::clock();
			PointsCollection amksPointsCollection = fastAMKSClustering(amksTrajs, newNeighborsMap, currRadius);
			double fastAMKSDuration = (std::clock() - fastAMKSStartTime) / (double)CLOCKS_PER_SEC;
			std::cout << "fastAMKSClustering takes " << fastAMKSDuration << " seconds." << std::endl;
			//prevAMKSPoints.clear();
			prevAMKSPoints = amksPointsCollection;
			amksTrajs = updateTrajs(amksTrajs, amksPointsCollection);
		
			// draw trajectories after certain iteration
			/*if (iter % 2 == 0) {
				compDrawTrajectories(amksTrajs, imageName, "Iter" + std::to_string(iter));
			}
			else if (iter == iterations) {
				compDrawTrajectories(amksTrajs, imageName, "Iter" + std::to_string(iter));
			}*/
	
			currRadius = currRadius - (startRadius - endRadius) / (iterations - 1);
	
			std::cout << "Leave iteration: " << iter << std::endl;
	
			double iterDuration = (std::clock() - startTime) / (double)CLOCKS_PER_SEC;
			std::cout << "Iteration " << iter << " takes " << iterDuration << " seconds." << std::endl;
		}
}

int ClusteringComponent::findNumberOfClusters(std::vector<traj_elem_t>& trajs, std::vector<int>& labels, 
	double distThreshold, std::vector<std::vector<float>>& trajsDistMatrix) {

	std::clock_t partitionStartTime = std::clock();
	std::cout << "......" << std::endl;
	//int numberOfClusters = cv::partition(trajs, labels, isInSameCluster);
	int numberOfClusters = trajsPartition(trajs, labels, distThreshold, trajsDistMatrix);
	double partitionDuration = (std::clock() - partitionStartTime) / (double)CLOCKS_PER_SEC;
	std::cout << "Identified number of clusters " << numberOfClusters << " takes " << partitionDuration << " seconds." << std::endl;
	return numberOfClusters;
}

std::vector<double> ClusteringComponent::findClusteringCenters(std::vector<traj_elem_t>& origTrajs, std::vector<traj_elem_t>& centerTrajs, 
	std::vector<int>& labels, std::vector<int>& centerLabels, std::vector<cv::Scalar> colors, int numberOfClusters) {

	std::vector<double> percentages;

	cv::Mat image = cv::imread(imageName, CV_LOAD_IMAGE_UNCHANGED);
	/*if (image.empty) {
		std::cout << "Fail to load image ..." << std::endl;
	}*/

	for (int id = 0; id < numberOfClusters; id++) {
		//compDrawCluster(origTrajs, labels, colors, id, imageName, "Cluster " + std::to_string(id));
		/*traj_elem_t currCenter = computeClusterCenter(origTrajs, labels, id);
		centerTrajs.push_back(currCenter);
		centerLabels.push_back(id);*/

		std::vector<traj_elem_t> clusterTrajs;
		for (size_t i = 0; i < labels.size(); i++) {
			if (id == labels[i]) {
				clusterTrajs.push_back(origTrajs[i]);
			}
		}

		// compute centers and draw clusters
		if (clusterTrajs.size() >= 10) {

			traj_elem_t currCenter = computeClusterCenter(origTrajs, labels, id);
			//traj_elem_t currCenter = computeClusterCenter(clusterTrajs, labels, id);
			double currPercentage = (double)clusterTrajs.size() / (double)origTrajs.size() * 100.0;
			centerTrajs.push_back(currCenter);
			centerLabels.push_back(id);
			percentages.push_back(currPercentage);

			for (size_t i = 0; i < clusterTrajs.size(); i++) {
				for (size_t j = 0; j < clusterTrajs[i].points.size() - 1; j++) {
					cv::line(image, cv::Point2d(clusterTrajs[i].points[j].x_coordinate, clusterTrajs[i].points[j].y_coordinate),
						cv::Point2d(clusterTrajs[i].points[j + 1].x_coordinate, clusterTrajs[i].points[j + 1].y_coordinate), colors[id], 2, 8);
				}
			}
		}
	}
	cv::namedWindow("Clusters", cv::WINDOW_AUTOSIZE);
	cv::imshow("Clusters", image);
	//cv::waitKey(0);

	return percentages;
}

std::vector<double> ClusteringComponent::findClusteringMedroids(std::vector<traj_elem_t>& origTrajs, std::vector<traj_elem_t>& centerTrajs,
	std::vector<int>& labels, std::vector<int>& centerLabels, std::vector<cv::Scalar> colors, int numberOfClusters, std::vector<std::vector<float>>& distMatrix) {

	std::vector<double> percentages;

	cv::Mat image = cv::imread(imageName, CV_LOAD_IMAGE_UNCHANGED);
	/*if (image.empty) {
	std::cout << "Fail to load image ..." << std::endl;
	}*/

	for (int id = 0; id < numberOfClusters; id++) {
		//compDrawCluster(origTrajs, labels, colors, id, imageName, "Cluster " + std::to_string(id));
		/*traj_elem_t currCenter = computeClusterCenter(origTrajs, labels, id);
		centerTrajs.push_back(currCenter);
		centerLabels.push_back(id);*/

		std::vector<traj_elem_t> clusterTrajs;
		for (size_t i = 0; i < labels.size(); i++) {
			if (id == labels[i]) {
				clusterTrajs.push_back(origTrajs[i]);
			}
		}

		// compute centers and draw clusters
		if (clusterTrajs.size() >= 10) {

			//traj_elem_t currCenter = computeClusterCenter(origTrajs, labels, id);
			traj_elem_t currCenter = findClusterMedroid(origTrajs, labels, id, distMatrix);
			//traj_elem_t currCenter = computeClusterCenter(clusterTrajs, labels, id);
			double currPercentage = (double)clusterTrajs.size() / (double)origTrajs.size() * 100.0;
			centerTrajs.push_back(currCenter);
			centerLabels.push_back(id);
			percentages.push_back(currPercentage);

			for (size_t i = 0; i < clusterTrajs.size(); i++) {
				for (size_t j = 0; j < clusterTrajs[i].points.size() - 1; j++) {
					cv::line(image, cv::Point2d(clusterTrajs[i].points[j].x_coordinate, clusterTrajs[i].points[j].y_coordinate),
						cv::Point2d(clusterTrajs[i].points[j + 1].x_coordinate, clusterTrajs[i].points[j + 1].y_coordinate), colors[id], 2, 8);
				}
			}
		}
	}
	cv::namedWindow("Clusters", cv::WINDOW_AUTOSIZE);
	cv::imshow("Clusters", image);
	//cv::waitKey(0);

	return percentages;
}