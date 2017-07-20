#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>

class Vehicle
{
public:
	Vehicle(double x, double y, double vx, double vy, double s, double d, double dist)
	 { _x = x; _y = y; _vx = vx; _vy = vy; _s = s; _d = d; _v = sqrt(vx*vx + vy*vy); _dist = dist; };
	~Vehicle() {};

    static bool faster(const Vehicle &a, const Vehicle &b){ return a._v < b._v; };
    static bool closer(const Vehicle &a, const Vehicle &b){ return a._dist > b._dist; };

	/* data */
	double _x;
    double _y;
    double _vx;
    double _vy;
    double _s;
    double _d;
    double _v;
    double _dist;

};


#endif // VEHICLE_H


