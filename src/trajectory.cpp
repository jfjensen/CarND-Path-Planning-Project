#include "trajectory.h"

void Trajectory::init_s_dot(double start, double goal)
{

	start_s_dot = start;
	goal_s_dot = goal;

	p = 0.0;
}

void Trajectory::init_d(double start, double goal)
{
	start_d = start;
	goal_d = goal;

	p = 0.0;
}

ReturnCode EasingTrajectory::generate_s_dot()
{
	double subd = (abs(goal_s_dot - start_s_dot)) * 500;

	p = p + (1.0/subd);

	cout << "generate_s_dot: " << start_s_dot << " " << goal_s_dot << " "<< subd << " " << p << endl;
	if ((goal_s_dot > start_s_dot) and (p < 1.0))
    {
      
      	// double subd = (goal_s_dot - start_s_dot) * 500;
    	// p = p + (1.0/subd);

    	curr_s_dot = start_s_dot + (0.5 * (1 - cos(p * M_PI)) * (goal_s_dot - start_s_dot) );

    	return ReturnCode::RUNNING;
         
    }
    else if ((goal_s_dot < start_s_dot) and (p < 1.0))
    {
    	// double subd = (start_s_dot - goal_s_dot) * 500;
    	// p = p + (1.0/subd);

    	curr_s_dot = start_s_dot - (0.5 * (1 - cos(p * M_PI)) * (start_s_dot - goal_s_dot) );

    	return ReturnCode::RUNNING;
    }

    else
    {
      // curr_s_dot = goal_s_dot;
      return ReturnCode::SUCCESS;
    }

}

ReturnCode EasingTrajectory::generate_d()
{

    double subd = (abs(goal_d - start_d)) * 30;

    p = p + (1.0/subd);

	cout << "generate_d_dot: " << start_d << " " << goal_d << " "<< subd << " " << p << endl;
	if ((goal_d > start_d) and (p < 1.0))
    {
      
     //  	double subd = (goal_d - start_d) * 30;
    	// p = p + (1.0/subd);

    	curr_d = start_d + (0.5 * (1 - cos(p * M_PI)) * (goal_d - start_d) );

    	return ReturnCode::RUNNING;
         
    }
    else if ((goal_d < start_d) and (p < 1.0))
    {
    	// double subd = (start_d - goal_d) * 30;
    	// p = p + (1.0/subd);

    	curr_d = start_d - (0.5 * (1 - cos(p * M_PI)) * (start_d - goal_d) );

    	return ReturnCode::RUNNING;
    }

    else
    {
      return ReturnCode::SUCCESS;
    }

}