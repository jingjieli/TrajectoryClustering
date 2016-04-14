#ifndef POINT_H
#define POINT_H

struct point_t {
	int trajId;
	double x_coordinate;
	double y_coordinate;
	double speed_x;
	double speed_y;
	double start_x;
	double start_y;
	double end_x;
	double end_y;
};

void printPoint(point_t& point);

double pointDifference(point_t& firstPoint, point_t& secondPoint);

#endif 