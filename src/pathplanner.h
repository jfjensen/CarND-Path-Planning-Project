#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <string>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "vehicle.h"
#include <iostream>
#include "returncode.h"
#include "trajectory.h"

using namespace std;

class PathPlanner
{

private:

	enum MODL_STATUS { INIT, RUNNING, COMPLETE };
	MODL_STATUS trajectory_status;

	double car_speed;
	double max_speed;
	double ref_speed;

	double start_car_d;
	double goal_car_d;

	Vehicle this_veh;

	vector<Vehicle> veh_vec;

    vector<Vehicle> leftlane_infront;
    vector<Vehicle> midlane_infront;
    vector<Vehicle> rightlane_infront;

    vector<Vehicle> leftlane_behind;
    vector<Vehicle> midlane_behind;
    vector<Vehicle> rightlane_behind;

    Trajectory *trajectory;

    enum LANE_ID { LEFT, MID, RIGHT };
    LANE_ID car_lane;

public:
	PathPlanner();
	~PathPlanner() {};

	void setMaxSpeed(double max_speed) { this->max_speed = max_speed; };
	void setVehicle(Vehicle);
	void setVehicleVector(vector<Vehicle> veh_vec) { this->veh_vec = veh_vec; };
	void findClosestVeh(int);

	bool predCarInFront();
	bool predCarInFrontDiffSpeed();
	bool predLessThMaxSpeed();

	bool predExistLaneToLeft();
	bool predExistLaneToRight();

	bool predChangeToLeft();
	bool predChangeToRight();

	ReturnCode actKeepSpeed();
	ReturnCode actChangeToMaxSpeed();
	ReturnCode actChangeToRefSpeed();
	ReturnCode actChangeToLeft();
	ReturnCode actChangeToRight();

	ReturnCode actChangeSpeed(double, double);
	ReturnCode actChangeLane(double, double);

  	double dist_inc;
  	double d;

  	int ticks_ahead;
  	
};

#endif // PATHPLANNER_H
