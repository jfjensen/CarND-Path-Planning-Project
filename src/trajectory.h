#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include "Eigen-3.3/Eigen/Core"
#include "returncode.h"

using namespace std;

class Trajectory {
protected:
	double goal_s_dot;
  	double start_s_dot;
  	double curr_s_dot;

  	double goal_d;
  	double start_d;
  	double curr_d;

  	double p;
public:
	Trajectory() {};
	~Trajectory() {};
	
	void init_s_dot(double, double);
	void init_d(double, double);

	virtual ReturnCode generate_s_dot() {};
	virtual ReturnCode generate_d() {};

	double get_s_dot() { return curr_s_dot; };
	double get_d() { return curr_d; };
};

class EasingTrajectory : public Trajectory
{
public:
	ReturnCode generate_s_dot();
	ReturnCode generate_d();
	
};

#endif // TRAJECTORY_H
