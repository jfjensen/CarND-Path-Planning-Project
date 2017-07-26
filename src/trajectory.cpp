#include "trajectory.h"

void Trajectory::init_s_dot(double start_s_dot, double goal_s_dot)
{

	this->start_s_dot = start_s_dot;
	this->goal_s_dot = goal_s_dot;

	p = 0.0;
}

void Trajectory::init_d(double start_d, double goal_d)
{
	this->start_d = start_d;
	this->goal_d = goal_d;

	p = 0.0;
}

ReturnCode EasingTrajectory::generate_s_dot()
{
	cout << "generate_s_dot: " << start_s_dot << " " << goal_s_dot << endl;
	if ((goal_s_dot > start_s_dot) and (p < 1.0))
    {
      
      	double subd = (goal_s_dot - start_s_dot) * 500;
    	p = p + (1.0/subd);

    	curr_s_dot = start_s_dot + (0.5 * (1 - cos(p * M_PI)) * (goal_s_dot - start_s_dot) );

    	return ReturnCode::RUNNING;
         
    }
    else if ((goal_s_dot < start_s_dot) and (p < 1.0))
    {
    	double subd = (start_s_dot - goal_s_dot) * 500;
    	p = p + (1.0/subd);

    	curr_s_dot = start_s_dot - (0.5 * (1 - cos(p * M_PI)) * (start_s_dot - goal_s_dot) );

    	return ReturnCode::RUNNING;
    }

    else
    {
      return ReturnCode::SUCCESS;
    }

}

ReturnCode EasingTrajectory::generate_d()
{

}