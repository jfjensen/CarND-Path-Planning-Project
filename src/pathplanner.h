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

	double p;

public:
	PathPlanner();
	~PathPlanner() {};

	bool IsNoChange();

	bool IsChangeSpeed();
	void SetChangeSpeed(double goal_inc);
	void ChangeSpeed();

	/* data */
	enum STATUS { CHNG_SPEED, CHNG_LANE, NO_CHNG };
	STATUS status;



  	double goal_inc;
  	double start_inc;
  	double dist_inc;
};

#endif // PATHPLANNER_H
