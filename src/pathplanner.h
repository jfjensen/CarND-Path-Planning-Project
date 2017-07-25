#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <string>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"
#include "vehicle.h"
#include <iostream>
#include "returncode.h"

using namespace std;

class PathPlanner
{

private:

	enum MODL_STATUS { INIT, RUNNING, COMPLETE };
	MODL_STATUS mod_status;

	enum SPEED_CHG_STATUS { SLOW_DOWN, SPEED_UP };
	SPEED_CHG_STATUS sp_status;

	enum LANE_CHG_STATUS {  CHG_LEFT, CHG_RIGHT };
	LANE_CHG_STATUS	lc_status;
	
	double p;

	double max_speed;
	double ref_speed;

	Vehicle this_veh;

	vector<Vehicle> veh_vec;

    vector<Vehicle> leftlane_infront;
    vector<Vehicle> midlane_infront;
    vector<Vehicle> rightlane_infront;

    vector<Vehicle> leftlane_behind;
    vector<Vehicle> midlane_behind;
    vector<Vehicle> rightlane_behind;

    enum LANE_ID { LEFT, MID, RIGHT };
    LANE_ID car_lane;

public:
	PathPlanner();
	~PathPlanner() {};

	bool IsNoChange();

	bool IsChangeSpeed();
	void SetChangeSpeed(double goal_inc);
	void ChangeSpeed();

	bool IsChangeLane();
	void SetChangeLane(double goal_d);
	void ChangeLane();

	void setMaxSpeed(double max_speed) { this->max_speed = max_speed; };
	void setVehicle(Vehicle);
	void setVehicleVector(vector<Vehicle> veh_vec) { this->veh_vec = veh_vec; };
	void findClosestVeh();

	bool predCarInFront();
	bool predCarInFrontDiffSpeed();
	bool predLessThMaxSpeed();

	bool predExistLaneToLeft();
	bool predExistLaneToRight();

	bool predChangeToLeft();
	bool predChangeToRight();

	ReturnCode actKeepSpeed(){ cout << "Keep Speed" << endl; return ReturnCode::SUCCESS;};
	ReturnCode actChangeToMaxSpeed(){ cout << "Chg to max Speed" << endl; return ReturnCode::SUCCESS;};
	ReturnCode actChangeToRefSpeed(){ cout << "Chg to ref Speed" << endl; return ReturnCode::SUCCESS;};
	ReturnCode actChangeToLeft(){cout << "Change lane to left" << endl; return ReturnCode::SUCCESS;};
	ReturnCode actChangeToRight(){cout << "Change lane to right" << endl; return ReturnCode::SUCCESS;};

	/* data */
	enum STATUS { CHNG_SPEED, CHNG_LANE, NO_CHNG };
	STATUS status;

  	double goal_inc;
  	double start_inc;
  	double dist_inc;

  	double goal_d;
  	double start_d;
  	double d;
  	
  	double subd;
};

#endif // PATHPLANNER_H
