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



void Robot::update(Mat world)
{
  compute_sensor_values(world);

  // get sensor values of sensor #0 and sensor #1
  double sensor_val0 = sensor_values[0];
  double sensor_val1 = sensor_values[1];
  
  // near to a wall?
  if ((sensor_val0 < 10) || (sensor_val1 < 10))
  {
    // turn left or right?
    if (sensor_val0<sensor_val1)
      turn(M_PI / 16);
    else
      turn(-M_PI / 16);
  }
  else
    move(1);

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
