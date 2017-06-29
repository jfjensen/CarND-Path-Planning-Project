#include "pathplanner.h"
#include <iostream>

using namespace std;


PathPlanner::PathPlanner()
{
	
  	status = CHNG_SPEED;
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
	}
	else
	{
		sp_status = SLOW_DOWN;
	}

	p = 0.0;
 
}

void PathPlanner::ChangeSpeed()
{
    
    cout << " dist_inc: " << dist_inc << " goal_inc: " << goal_inc << " start_inc: " << start_inc << " p: " << p << endl;

    if ((sp_status == SPEED_UP) and (p < 1.0))
    {
      
    	p = p + (1.0/50.0);

    	dist_inc = start_inc + (0.5 * (1 - cos(p * M_PI)) * (goal_inc - start_inc) );

    	// dist_inc += 0.005;
      
    }
    else if ((sp_status == SLOW_DOWN) and (p < 1.0))
    {

    	p = p + (1.0/50.0);

    	dist_inc = start_inc - (0.5 * (1 - cos(p * M_PI)) * (start_inc - goal_inc) );
    }

    else
    {
      status = NO_CHNG;
      mod_status = INIT;
    }
}