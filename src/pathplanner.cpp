#include "pathplanner.h"
#include <iostream>

using namespace std;


PathPlanner::PathPlanner()
{
	
  	// status = CHNG_SPEED;
  	status = NO_CHNG;
  	mod_status = INIT;

  	
}

bool PathPlanner::IsNoChange()
{
	return status == NO_CHNG;
}

bool PathPlanner::IsChangeSpeed()
{
	return status == CHNG_SPEED;
}

void PathPlanner::SetChangeSpeed(double goal_inc)
{
	status = CHNG_SPEED;
	mod_status = INIT;
	this->goal_inc = goal_inc;

	start_inc = dist_inc;
	mod_status = RUNNING;

	if (goal_inc > start_inc)
	{
		sp_status = SPEED_UP;
		subd = (goal_inc - start_inc) * 500;
	}
	else
	{
		sp_status = SLOW_DOWN;
		subd = (start_inc - goal_inc) * 500;
	}

	p = 0.0;
 
}

void PathPlanner::ChangeSpeed()
{
    
    // cout << " dist_inc: " << dist_inc << " goal_inc: " << goal_inc << " start_inc: " << start_inc << " p: " << p << endl;

    if ((sp_status == SPEED_UP) and (p < 1.0))
    {
      
    	p = p + (1.0/subd);

    	dist_inc = start_inc + (0.5 * (1 - cos(p * M_PI)) * (goal_inc - start_inc) );

    	// dist_inc += 0.005;
      
    }
    else if ((sp_status == SLOW_DOWN) and (p < 1.0))
    {

    	p = p + (1.0/subd);

    	dist_inc = start_inc - (0.5 * (1 - cos(p * M_PI)) * (start_inc - goal_inc) );
    }

    else
    {
      status = NO_CHNG;
      mod_status = INIT;
    }
}


bool PathPlanner::IsChangeLane()
{
	return status == CHNG_LANE;
}

void PathPlanner::SetChangeLane(double goal_d)
{

	status = CHNG_LANE;
	mod_status = INIT;
	this->goal_d = goal_d;

	start_d = d;
	mod_status = RUNNING;

	if (goal_d > start_d)
	{
		lc_status = CHG_RIGHT;
		subd = (goal_d - start_d) * 20;
	}
	else
	{
		lc_status = CHG_LEFT;
		subd = (start_d - goal_d) * 20;
	}

	p = 0.0;


}
void PathPlanner::ChangeLane()
{

	cout << " dist_inc: " << dist_inc << " goal_inc: " << goal_inc << " start_inc: " << start_inc << " p: " << p << endl;

    if ((lc_status == CHG_RIGHT) and (p < 1.0))
    {
      
    	// p = p + (1.0/50.0);
    	p = p + (1.0/subd);

    	d = start_d + (0.5 * (1 - cos(p * M_PI)) * (goal_d - start_d) );

    	// dist_inc += 0.005;
      
    }
    else if ((lc_status == CHG_LEFT) and (p < 1.0))
    {

    	// p = p + (1.0/50.0);
    	p = p + (1.0/subd);

    	d = start_d - (0.5 * (1 - cos(p * M_PI)) * (start_d - goal_d) );
    }

    else
    {
      status = NO_CHNG;
      mod_status = INIT;
    }


}