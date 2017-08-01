#include "pathplanner.h"



PathPlanner::PathPlanner()
{
	
  trajectory_status = INIT;
  	
}



void PathPlanner::findClosestVeh(int ticks_ahead)
{
  
  
  this->ticks_ahead = ticks_ahead;

  double car_s = this_veh._s + (ticks_ahead * dist_inc);

  leftlane_infront.clear();
  midlane_infront.clear();
  rightlane_infront.clear();

  leftlane_behind.clear();
  midlane_behind.clear();
  rightlane_behind.clear();

  for (int i = 0; i < veh_vec.size(); ++i)
  {

    Vehicle item = veh_vec[i];
    double item_dist_inc = item._v / 50.0;
    
    double item_s = item._s + (ticks_ahead * item_dist_inc);
    double item_d = item._d;
    
    double item_dist = item._dist;


    // only look at car if close...
    if (item_dist < 40) //60)
    {
        
        // is this car left lane?
        if ((item_d < 4.0))
        {
          
          // is the car in front of us?
          if (item_s > car_s)
          {
              leftlane_infront.push_back(item);                      
          }
          // or behind us?
          else if (item_dist < 15)
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
          else if (item_dist < 15)
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
          else if (item_dist < 15)
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

  this->car_speed = dist_inc * 50.0;

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

  double veh_speed = car_speed;


  switch(car_lane)
  {
    case LEFT:
      if (leftlane_infront.size() > 0)
      {
        
        Vehicle closest_veh = *min_element(leftlane_infront.begin(), leftlane_infront.end(), Vehicle::closer);
        cout << "car distance: " << closest_veh._dist << endl;
        veh_speed = closest_veh._v;
      
      }
      
        break;
      

    case MID:
      if (midlane_infront.size() > 0)
      {
        
        Vehicle closest_veh = *min_element(midlane_infront.begin(), midlane_infront.end(), Vehicle::closer);
        cout << "car distance: " << closest_veh._dist << endl;
        veh_speed = closest_veh._v;
      }
      
        break;
      

    case RIGHT:
      if (rightlane_infront.size() > 0)
      {
        Vehicle closest_veh = *min_element(rightlane_infront.begin(), rightlane_infront.end(), Vehicle::closer);
        cout << "car distance: " << closest_veh._dist << endl;
        veh_speed = closest_veh._v;
      }
      
        break;
      

  }

  cout << "car_speed: " << car_speed << endl;
  cout << "veh_speed: " << veh_speed << endl;

  
  if((veh_speed - 0.5) < car_speed)
  {
    // this->ref_speed = veh_speed;
    this->ref_speed = veh_speed - 0.8;
    return true;  
  }
  else
  {
    return false;
  }

}

bool PathPlanner::predLessThMaxSpeed()
{
  cout << "car_speed: " << car_speed << endl;
  cout << "max_speed: " << max_speed << endl;

  if (abs(car_speed-max_speed) > 1.5)
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
        
        start_car_d = 6.0;
        goal_car_d  = 2.0;

        return true;
      }
      else
      {
        return false;
      }

    case RIGHT:
      if ((midlane_infront.size() == 0) and (midlane_behind.size() == 0))
      {
        
        start_car_d = 9.8;//10.0;
        goal_car_d  =  6.0;

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
        
        start_car_d = 2.0;
        goal_car_d  = 6.0;

        return true;
      }
      else
      {
        return false;
      }


    case MID:
      if ((rightlane_infront.size() == 0) and (rightlane_behind.size() == 0))
      {
        
        start_car_d =  6.0;
        goal_car_d  = 9.8; //10.0;

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

  this->dist_inc = trajectory->get_s_dot();

  cout << "Trajectory generated s_dot: " << dist_inc << endl;

  if (trajectory_result == ReturnCode::SUCCESS)
  {
    delete this->trajectory;
    trajectory_status = INIT;
  }

  return trajectory_result;
}

ReturnCode PathPlanner::actChangeToMaxSpeed()
{
  cout << "Chg to max Speed" << endl; 

  double car_s_dot = car_speed / 50.0;
  double max_s_dot = max_speed / 50.0;

  return actChangeSpeed(car_s_dot, max_s_dot);
}

ReturnCode PathPlanner::actChangeToRefSpeed()
{ 
  cout << "Chg to ref Speed" << endl; 

  double car_s_dot = car_speed / 50.0;
  double ref_s_dot = ref_speed / 50.0;

  return actChangeSpeed(car_s_dot, ref_s_dot);
}

ReturnCode PathPlanner::actChangeLane(double start_d, double goal_d)
{ 
 
  if (trajectory_status == INIT)
  {
    
    this->trajectory = new EasingTrajectory();

    trajectory->init_d(start_d, goal_d);
    trajectory_status = RUNNING;
  }

  ReturnCode trajectory_result = trajectory->generate_d();

  this->d = trajectory->get_d();

  cout << "Trajectory generated d: " << d << endl;

  if (trajectory_result == ReturnCode::SUCCESS)
  {
    delete this->trajectory;
    trajectory_status = INIT;
  }

  return trajectory_result;
}

ReturnCode PathPlanner::actChangeToLeft()
{
  cout << "Change lane to left" << endl;

  double car_d = this_veh._d;

  return actChangeLane(start_car_d, goal_car_d);
}

ReturnCode PathPlanner::actChangeToRight()
{
  cout << "Change lane to right" << endl;

  double car_d = this_veh._d;

  return actChangeLane(start_car_d, goal_car_d);
}