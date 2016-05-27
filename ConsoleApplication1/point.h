#ifndef POINT_H
#define POINT_H

struct point_t {
	int trajId;
	float x_coordinate;
	float y_coordinate;
	float speed_x;
	float speed_y;
	float start_x;
	float start_y;
	float end_x;
	float end_y;
};

void printPoint(point_t& point);

float pointDifference(point_t& firstPoint, point_t& secondPoint);

#endif 