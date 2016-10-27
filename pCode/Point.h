#ifndef POINT_H
#define POINT_H

class Point {

private:

double x;
double y;

public:

double getX() const;
double getY() const;

Point();
Point(double xi, double yi);

};

#endif
