#include "Particle.h"

Particle::Particle() {
	p = Point();
	v = Point();
	mass = 1.0;
	radius = mass/5; //dependent on mass
}

Particle::Particle(Point pInit) {
	p = pInit;
	v = Point();
	mass = 1.0;
	radius = mass/5;
}

Particle::Particle(Point pInit, double massInit) {
	p = pInit;
	v = Point();
	mass = massInit;
	radius =  mass/(5);
}

Particle::Particle(Point pInit, double massInit, double radiusInit, Point vInit) {
	p = pInit;
	v = vInit;
	mass = massInit;
	radius = radiusInit;
}

Point Particle::getPos() const {
	return p;
}

Point Particle::getV() const {
	return v;
}

double Particle::getMass() const {
	return mass;
}

double Particle::getRadius() const {
	return radius;
}

void Particle::setP(double xf, double yf) {
	p = Point(xf, yf);
}

void Particle::setV(double xf, double yf) {
	v = Point(xf, yf);
}

void Particle::setP(Point pt) {
	p = pt;
}

void Particle::setV(Point pt) {
	v = pt;
}
