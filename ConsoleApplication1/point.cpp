#include <iostream>
#include "point.h"

void printPoint(point_t& point) {
	std::cout << "###################" << std::endl;
	std::cout << "Current point is: " << std::endl;
	std::cout << "trajId: " << point.trajId << std::endl;
	std::cout << "x_coordinate: " << point.x_coordinate << std::endl;
	std::cout << "y_coordinate: " << point.y_coordinate << std::endl;
	std::cout << "speed_x: " << point.speed_x << std::endl;
	std::cout << "speed_y: " << point.speed_y << std::endl;
	std::cout << "start_x: " << point.start_x << std::endl;
	std::cout << "start_y: " << point.start_y << std::endl;
	std::cout << "end_x: " << point.end_x << std::endl;
	std::cout << "end_y: " << point.end_y << std::endl;
}

float pointDifference(point_t& firstPoint, point_t& secondPoint) {
	float dist = (float)sqrt(pow(firstPoint.x_coordinate - secondPoint.x_coordinate, 2.0) + pow(firstPoint.y_coordinate - secondPoint.y_coordinate, 2.0)) + 
		(float)sqrt(pow(firstPoint.speed_x - secondPoint.speed_x, 2.0) + pow(firstPoint.speed_y - secondPoint.speed_y, 2.0)) + 
		(float)sqrt(pow(firstPoint.start_x - secondPoint.start_x, 2.0) + pow(firstPoint.start_y - secondPoint.start_y, 2.0)) + 
		(float)sqrt(pow(firstPoint.end_x - secondPoint.end_x, 2.0) + pow(firstPoint.end_y - secondPoint.end_y, 2.0));
	return dist;
}