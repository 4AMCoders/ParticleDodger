#ifndef PARTICLE_H
#define PARTICLE_H
#include "Point.h"

class Particle {

private:

Point p;
Point v;
double mass;
double radius;

public:

Particle();
Particle(Point pInit);
Particle(Point pInit, double massInit);
Particle(Point pInit, double massInit, double radiusInit, Point vInit);


Point getPos() const;
Point getV() const;
double getMass() const;
double getRadius() const;

void setP(double xf, double yf);
void setV(double xf, double yf);
void setP(Point pt);
void setV(Point pt);

};
#endif
