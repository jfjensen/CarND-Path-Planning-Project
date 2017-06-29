#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <vector>
#include <string>
#include <fstream>
#include "Eigen-3.3/Eigen/Core"

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
