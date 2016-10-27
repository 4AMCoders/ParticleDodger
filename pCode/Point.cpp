#include "Point.h"

double Point::getX() const {
	return x;
}

double Point::getY() const {
	return y;
}

Point::Point() {
	x = 0.0;
	y = 0.0;
}

Point::Point(double xi, double yi) {
	x = xi;
	y = yi;
}
