#include "Robot.h"

#define _USE_MATH_DEFINES
#include <math.h>

Robot::Robot(string         name,
             int            radius,
             Point2d        start_pos,
             double         start_orientation,
             vector<double> sensor_angles,
             vector<double> sensor_distances)
{
  this->name             = name;
  this->radius           = radius,
  this->pos              = start_pos;
  this->orientation      = start_orientation;
  this->sensor_angles    = sensor_angles;
  this->sensor_distances = sensor_distances;
  nr_sensors              = (int) sensor_angles.size();
}


double Robot::get_radius()
{
  return radius;
}

Point2d Robot::get_position()
{
  return pos;
}

double Robot::get_orientation()
{
  return orientation;
}

int Robot::get_nr_sensors()
{
  return nr_sensors;
}


vector<double> Robot::get_sensor_values()
{
  return sensor_values;
}

vector<double> Robot::get_sensor_angles()
{
  return sensor_angles;
}

vector<double> Robot::get_sensor_distances()
{
  return sensor_distances;
}


void Robot::compute_sensor_values(Mat world)
{
  // 1. clear old sensor values
  sensor_values.clear();

  // 2. for each distance sensor
  for (int sensor_nr = 0; sensor_nr < nr_sensors; sensor_nr++)
  {
    // 2.1 get (x,y) coords of current robot position
    double x = pos.x;
    double y = pos.y;

    // 2.2 get sensor orientation relative to robots orientation
    double sensor_angle = sensor_angles[sensor_nr];

    // 2.3 map robot angle + sensor_angle to a direction vector
    double sensor_dx = cos(orientation + sensor_angle);
    double sensor_dy = sin(orientation + sensor_angle);

    // 2.4 compute sensor start position
    double sensor_startx = x + sensor_dx * radius;
    double sensor_starty = y + sensor_dy * radius;

    // 2.5 now move from sensor start position into sensor direction
    //     till we reach the maximum distance or hit an obstacle in the world (white pixel)

    // 2.6 get maximum sensor distance
    double sensor_max_dist = sensor_distances[sensor_nr];

    // 2.7 test step by step whether the next pixel is a black pixel == free space
    int step;
    for (step = 0; step < sensor_max_dist; step++)
    {
      // get next pixel location on sensor ray
      double sx = sensor_startx + step*sensor_dx;
      double sy = sensor_starty + step*sensor_dy;

      // invalid coordinates?
      if ((sx >= world.cols) || (sy >= world.rows) || (sx < 0) || (sy < 0))
         break;

      // get value of world pixel at the sensor ray position (sx,sy)
      Vec3b pixel_color = world.at<Vec3b>((int)sy, (int)sx);

      // is it black or white?
      if ((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0))
      {
        // black pixel, so continue
        continue;
      }
      else
      {
        // white pixel: sensor ray reached a white pixel
        break;
      }

    } // for (move along sensor line)

    // 2.8 store final sensor value (= maximum distance or distance till we have found a non-black pixel)
    sensor_values.push_back(step);

    // 2.9 output sensor value for debugging
    //printf("sensor #%d: %d\n", sensor_nr, step);

  } // for (sensor_nr)

} // compute_sensor_values



bool Robot::test_wall_bump(Mat world)
{
   for (double angle = 0.0; angle < 2 * M_PI; angle += 0.1)
   {
      // compute world coordinates (x,y) of check point
      int x = (int) (pos.x + cos(angle) * radius);
      int y = (int) (pos.y + sin(angle) * radius);

      // is there a white pixel at (x,y)?
      Vec3b pixel_color = world.at<Vec3b>(y, x);
      if ((pixel_color.val[0] == 0) && (pixel_color.val[1] == 0) && (pixel_color.val[2] == 0))
      {
         // black pixel, so continue         
      }
      else
      {
         // white pixel: robot's outer radius has hit a wall!
         return true;
      }
   }

   // no outer point of the robot hit a wall point
   return false;

} // test_wall_bump


void constrainAngle(double& x) {   
   x = fmod(x, 2 * M_PI);   
}


void Robot::update(Mat world)
{
  compute_sensor_values(world);

  // save old position
  Point2d old_pos = pos; 

  // get sensor values
  double sensor_f   = sensor_values[0]; // front
  double sensor_lll = sensor_values[1]; // left most
  double sensor_ll  = sensor_values[2]; // left middle
  double sensor_l   = sensor_values[3]; // left
  double sensor_r   = sensor_values[4]; // right
  double sensor_rr  = sensor_values[5]; // right middle
  double sensor_rrr = sensor_values[6]; // right most
     
  const double one_radian = +M_PI / 180.0;

  constrainAngle(orientation);


  ///
  /// Behavior: Turn to goal
  ///
  if (current_behavior_mode == TURN_TO_GOAL)
  {
     // compute direction vector to the target location
     double dirx = target_location.x - pos.x;
     double diry = target_location.y - pos.y;
     
     // compute goal angle
     double angle_goal = atan2(diry,dirx);
          
     double angle_diff = abs(angle_goal - orientation);
          
     // is the goal angle and the current orientation
     // of the robot similar enough?
     if (angle_diff <=1.0*one_radian)
        if ((get_distance_to_target()<sensor_f) // goal seems to be reachable directly!
            || (sensor_f>100))            // a lot of free space in direction of goal
            switch_to_new_behavior_mode( GO_STRAIGHT_TO_GOAL );
        else
            switch_to_new_behavior_mode( OBSTACLE_AVOIDANCE );
     else
     {        
        // compute turn direction
        double turn_angle;
        if (orientation<angle_goal)
         turn_angle = +one_radian;
        else
         turn_angle = -one_radian;

        // command the robot to turn
        turn( turn_angle );
      }
  }


  ///
  /// Behavior: Go straight to goal
  ///
  if (current_behavior_mode == GO_STRAIGHT_TO_GOAL)
  {
     double D = 15;

     // too near to wall?
     if ( (sensor_f<D)   ||
          (sensor_lll<D) || (sensor_ll<D) || (sensor_l<D) ||
          (sensor_rrr<D) || (sensor_rr<D) || (sensor_r<D)
        )
        switch_to_new_behavior_mode(OBSTACLE_AVOIDANCE);

      move(1);
  }


  ///
  /// Behavior: Wall following
  ///
  if (current_behavior_mode == OBSTACLE_AVOIDANCE)
  {   
     const int METHOD = 2;

     double D; // minimum distance
     
     // OBSTACLE-AVOIDANCE #1: (Braitenberg vehicle)
     if (METHOD == 1)
     {
        D = 20;
        if (sensor_f < D) // wall in front
           turn(one_radian);  // turn right
        else
           if ((sensor_lll < D) || (sensor_ll < D) || (sensor_l < D)) // left side too near to wall
              turn(one_radian);  // turn right
           else
              if ((sensor_rrr < D) || (sensor_rr < D) || (sensor_r < D)) // right side too near to wall
                 turn(-one_radian);  // turn left
              else
                 move(1);
     }
     
     // OBSTACLE-AVOIDANCE #2: (Wall following)
     if (METHOD == 2)
     {
        D = 20;
        if (sensor_f < D) // wall in front
           turn(one_radian);  // turn right
        else
           if ((sensor_lll < D) || (sensor_ll < D) || (sensor_l < D)) // left side too near to wall
              turn(one_radian);  // turn right
           else
              if ((sensor_lll > 2 * D) || (sensor_ll > 2 * D) || (sensor_l>2 * D)) // left side too far away from wall
                 turn(-one_radian);  // turn left
        move(1);
     }
     

    
          
     // from time to time, re-orientate into goal direction
     // again to see whether we can reach it..
     // if we are near to the goal this probability is much higher
     int N = 500;
     if (get_distance_to_target()<sensor_f)
        N = 250;
     if (rand()%N==0)
        switch_to_new_behavior_mode(TURN_TO_GOAL);
  }

  
  // did the robot bump into a wall?
  bool bumped = test_wall_bump( world );
  if (bumped)
  {
    printf("B");
    pos = old_pos;
  }

} // update


void Robot::move(double pixel)
{
  // get (x,y) coords of current robot position
  double x = pos.x;
  double y = pos.y;

  // map angle to a direction vector
  double dx = cos(orientation);
  double dy = sin(orientation);

  // add direction vector to current position to compute new robot position
  x += dx * pixel;
  y += dy * pixel;

  // store new position
  pos = Point2d(x, y);

} // move



void Robot::turn(double angle)
{
  orientation += angle;

} // turn


void Robot::set_target_location(Point p)
{
   target_location = p;
}


void Robot::switch_to_new_behavior_mode(behavior_modes m)
{
   current_behavior_mode = m;
   printf("switched behavior to mode: %s\n", BEHAVIOR_MODE_STRINGS[m]);
}


double Robot::get_distance_to_target()
{
   // compute direction vector to the target location
   double dirx = target_location.x - pos.x;
   double diry = target_location.y - pos.y;
   double distance_to_target = sqrt(dirx*dirx + diry*diry);

   return distance_to_target;
}