#include "pathplanner.h"



PathPlanner::PathPlanner()
{
	
  	// status = CHNG_SPEED;
  	status = NO_CHNG;
  	mod_status = INIT;
    trajectory_status = INIT;
  	
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
 
  cout << "SET CHNG_SPEED" << endl;

}

void PathPlanner::ChangeSpeed()
{
    
    cout << " dist_inc: " << dist_inc << " goal_inc: " << goal_inc << " start_inc: " << start_inc << " p: " << p << endl;

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
		subd = (goal_d - start_d) * 30;
	}
	else
	{
		lc_status = CHG_LEFT;
		subd = (start_d - goal_d) * 30;
	}

	p = 0.0;
  cout << "SET CHNG_LANE" << endl;

}
void PathPlanner::ChangeLane()
{

	cout << " d: " << d << " goal_d: " << goal_d << " start_d: " << start_d << " p: " << p << endl;

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

void PathPlanner::findClosestVeh()
{
  
  double car_s = this_veh._s;

  leftlane_infront.clear();
  midlane_infront.clear();
  rightlane_infront.clear();

  leftlane_behind.clear();
  midlane_behind.clear();
  rightlane_behind.clear();

  for (int i = 0; i < veh_vec.size(); ++i)
  {

    Vehicle item = veh_vec[i];
    double item_s = item._s;
    double item_d = item._d;
    double item_dist = item._dist;

    // only look at car if close...
    if (item_dist < 40) //60)
    {
        
        //veh_vector.push_back(veh);
        // std::cout << item << " v: " << item_v << " dist: " << item_dist << std::endl;

        // is this car left lane?
        if ((item_d < 4.0))
        {
          
          // is the car in front of us?
          if (item_s > car_s)
          {
              leftlane_infront.push_back(item);                      
          }
          // or behind us?
          else
          {
              leftlane_behind.push_back(item);
          }

        }
        // or is this car in middle lane?
        else if ((item_d >= 4.0) and (item_d < 8.0) )
        {
          
          // is the car in front of us?
          if (item_s > car_s)
          {
              midlane_infront.push_back(item);                      
          }
          // or behind us?
          else
          {
              midlane_behind.push_back(item);
          }

        }
        // or is this car in right lane?
        else
        {
          
          // is the car in front of us?
          if (item_s > car_s)
          {
              rightlane_infront.push_back(item);                      
          }
          // or behind us?
          else
          {
              rightlane_behind.push_back(item);
          }

        }
    }
  }

  cout << endl;
  cout << leftlane_infront.size()<< "  ";
  cout << midlane_infront.size()<< "  ";
  cout << rightlane_infront.size()<< endl;

  switch(car_lane)
  {
    case LEFT:
      cout << "*" << endl;
      break;

    case MID:
      cout << " " << "  " << "*" << endl;
      break;
      
    case RIGHT:
      cout << " " << "  " << " " << "  " << "*" << endl;
      break;
  }

  cout << leftlane_behind.size()<< "  ";
  cout << midlane_behind.size()<< "  ";
  cout << rightlane_behind.size()<< endl << endl;
}


void PathPlanner::setVehicle(Vehicle veh)
{
  
            
  this->this_veh = veh;

  double car_d = veh._d;

  if (car_d < 4.0)
  {
    car_lane = LEFT;
  }
  else if ((car_d >= 4.0) and (car_d < 8.0))
  {
    car_lane = MID;
  }
  else
  {
    car_lane = RIGHT;
  }

}




bool PathPlanner::predCarInFront()
{

  switch(car_lane)
  {
    case LEFT:
      if (leftlane_infront.size() > 0)
      {
        return true;
      }
      else
      {
        break;
      }

    case MID:
      if (midlane_infront.size() > 0)
      {
        return true;
      }
      else
      {
        break;
      }

    case RIGHT:
      if (rightlane_infront.size() > 0)
      {
        return true;
      }
      else
      {
        break;
      }

  }

  return false;

}

bool PathPlanner::predCarInFrontDiffSpeed()
{

  double car_speed = this->this_veh._v;

  double veh_speed = car_speed;


  switch(car_lane)
  {
    case LEFT:
      if (leftlane_infront.size() > 0)
      {
        
        Vehicle closest_veh = *min_element(leftlane_infront.begin(), leftlane_infront.end(), Vehicle::closer);
        veh_speed = closest_veh._v;
      
      }
      else
      {
        break;
      }

    case MID:
      if (midlane_infront.size() > 0)
      {
        
        Vehicle closest_veh = *min_element(midlane_infront.begin(), midlane_infront.end(), Vehicle::closer);
        veh_speed = closest_veh._v;
      }
      else
      {
        break;
      }

    case RIGHT:
      if (rightlane_infront.size() > 0)
      {
        Vehicle closest_veh = *min_element(rightlane_infront.begin(), rightlane_infront.end(), Vehicle::closer);
        veh_speed = closest_veh._v;
      }
      else
      {
        break;
      }

  }

  cout << "car_speed: " << car_speed << endl;
  cout << "veh_speed: " << veh_speed << endl;

  if(abs(car_speed - veh_speed) > 0.5)
  {
    this->ref_speed = veh_speed;
    return true;  
  }
  else
  {
    return false;
  }

}

bool PathPlanner::predLessThMaxSpeed()
{
  

  double car_speed = this->this_veh._v;

  cout << "car_speed: " << car_speed << endl;
  cout << "max_speed: " << max_speed << endl;

  if (abs(car_speed-max_speed) > 0.5)
  {
    return true;
  }
  else
  {
    return false;
  }
}


bool PathPlanner::predExistLaneToLeft()
{

  if (car_lane == LEFT)
  {
    return false;
  }
  else
  {
    return true;
  }

}

bool PathPlanner::predExistLaneToRight()
{
  if (car_lane == RIGHT)
  {
    return false;
  }
  else
  {
    return true;
  }
}

bool PathPlanner::predChangeToLeft()
{

  switch(car_lane)
  {
    
    case MID:
      if ((leftlane_infront.size() == 0) and (leftlane_behind.size() == 0))
      {
        return true;
      }
      else
      {
        return false;
      }

    case RIGHT:
      if ((midlane_infront.size() == 0) and (midlane_behind.size() == 0))
      {
        return true;
      }
      else
      {
        return false;
      }
    default:
      return false;

  }

}

bool PathPlanner::predChangeToRight()
{
  switch(car_lane)
  {
    
    case LEFT:
      if ((midlane_infront.size() == 0) and (midlane_behind.size() == 0))
      {
        return true;
      }
      else
      {
        return false;
      }


    case MID:
      if ((rightlane_infront.size() == 0) and (rightlane_behind.size() == 0))
      {
        return true;
      }
      else
      {
        return false;
      }

   
    default:
      return false;

  }
}


ReturnCode PathPlanner::actKeepSpeed()
{ 
  cout << "Keep Speed" << endl; 
  return ReturnCode::SUCCESS;
}

ReturnCode PathPlanner::actChangeSpeed(double start_s_dot, double goal_s_dot)
{ 
 
  if (trajectory_status == INIT)
  {
    
    this->trajectory = new EasingTrajectory();

    trajectory->init_s_dot(start_s_dot, goal_s_dot);
    trajectory_status = RUNNING;
  }

  ReturnCode trajectory_result = trajectory->generate_s_dot();
  cout << "Trajectory generated s_dot: " << trajectory->get_s_dot() << endl;

  if (trajectory_result == ReturnCode::SUCCESS)
  {
    delete this->trajectory;
    trajectory_status = INIT;
  }

  return trajectory_result;
  // return ReturnCode::SUCCESS;
}

ReturnCode PathPlanner::actChangeToMaxSpeed()
{
  cout << "Chg to max Speed" << endl; 

  double car_speed = this->this_veh._v;

  double car_s_dot = car_speed / 50.0;
  double max_s_dot = max_speed / 50.0;

  return actChangeSpeed(car_s_dot, max_s_dot);
}

ReturnCode PathPlanner::actChangeToRefSpeed()
{ 
  cout << "Chg to ref Speed" << endl; 

  double car_speed = this->this_veh._v;

  double car_s_dot = car_speed / 50.0;
  double ref_s_dot = ref_speed / 50.0;

  return actChangeSpeed(car_s_dot, ref_s_dot);
  
}

ReturnCode PathPlanner::actChangeToLeft()
{
  cout << "Change lane to left" << endl;
  return ReturnCode::SUCCESS;
}

ReturnCode PathPlanner::actChangeToRight()
{
  cout << "Change lane to right" << endl;
  return ReturnCode::SUCCESS;
}