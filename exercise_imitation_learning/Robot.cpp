#include "Robot.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>

#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <conio.h>

#include "params.h"
#include "math_tools.h"

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
  nr_sensors             = (int) sensor_angles.size();
  lfd_mode               = false;

  load_demonstration_data( DEMONSTRATION_DATA_FILE );

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

      // make sure (sx,sy) is within the world!
      // if it is not, we simulate a white pixel
      // so: the 2D world is enclosed by an imaginery
      //     white border
      if ((sx < 0) || (sx >= world_width) || (sy < 0) || (sy >= world_height))
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



void Robot::update(Mat world)
{
   world_width  = world.cols;
   world_height = world.rows;

   // 1. compute new sensor values
   compute_sensor_values(world);

   enum actions a = undefined;
     

   // 2.
   // are we in "Learning from Demonstration" mode?
   // if yes, collect an action example from the teacher
   // if not, drive according to the model learned
   // from the demonstration data   
   if (lfd_mode)
   {
      a = forward;

      // collect new (state,action) sample pair from the teacher
      bool valid_action = true;
      int c;

      c = cv::waitKeyEx();
      //printf("%d\n", c);

      switch (c)
      {
         case 2424832: // cursor left
            a = left;
            valid_action = true;
            break;

         case 2555904: // cursor right
            a = right;
            valid_action = true;
            break;

         case 2490368: // forward
            a = forward;
            valid_action = true;
            break;

         case 27: // escape
            // stop learning from demonstration
            set_lfd_mode( false );
            valid_action = false;            
            break;
      }

      // did we record a valid action?
      if (valid_action)
      {
         // save current demonstration example
         // (state, action) = (sensor_values, action):
         //
         demo_datum* d = new demo_datum;
         for (int svnr = 0; svnr < sensor_values.size(); svnr++)
         {
            d->sensor_values.push_back( sensor_values[svnr] );            
         }
         d->action = a;
         demonstration_data.push_back(d);
      }

   } // if (lfd_mode)
  
   else
   {
      // now compute an action based on the model learned
      // from the demonstration data

      double avg_distance;
      int k = USE_K_NEIGHBORS;
      a = k_nn_classifier(&avg_distance, k);

      // ask teacher for help?
      if ((INTERACTIVE_LEARNING) &&
          (avg_distance > MAX_AVG_DISTANCE_BEFORE_GOING_BACK_TO_DEMO_MODE)
         )
      {
         // forget about the action that
         // the k-NN classifier suggested
         // since we have not found similar
         // demonstration examples
         a = undefined;

         // reactivate demo mode
         set_lfd_mode(true);
      }

   } // else (use policy based on demonstration examples collected so far)


   // 3. perform the action
   switch (a)
   {
      case left: turn(-M_PI / 16);
                  break;
      case right: turn(+M_PI / 16);
                  break;
      case forward: move(3);
                     break;
   }
  
   /*
   // Braitenberg variant:
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
   */

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

  // make sure, we never leave the world!
  if (x < 0) x = 0;
  if (x >= world_width) x = world_width - 1;
  if (y < 0) y = 0;
  if (y >= world_height) y = world_height - 1;

  // store new position
  pos = Point2d(x, y);

} // move



void Robot::turn(double angle)
{
  orientation += angle;

} // turn



void Robot::set_lfd_mode(bool b)
{
   lfd_mode = b;

   if (lfd_mode)
   {        
      printf("\nHi! I'am a small robot. Please show me how to act in this world\n");
      printf("by pressing <- or -> or ^.\n");
      printf("\nIf you are finished with your demonstrations press ESC.\n");      
   }
   else
   {
      printf("\nStopped Learning from Demonstration mode");
      save_demonstration_data( DEMONSTRATION_DATA_FILE );
   }

} // set_lfd_mode



int Robot::get_size_of_demo_dataset()
{
   return (int)demonstration_data.size();

} // get_size_of_demo_dataset



void Robot::save_demonstration_data(string fname)
{
   // 1. open file for writing
   printf("\n\nsaving demonstration data to file\n%s\n", fname.c_str());
   std::ofstream f(fname);

   // 2. write nr of demonstrations to file
   f << demonstration_data.size() << endl;

   // 3. write len of sensor vec to file
   f << sensor_values.size() << endl;

   // 4. write len of action vec to file
   f << 1 << endl;
   
   // 5. for all demonstrations ...
   for (int demonr = 0; demonr < demonstration_data.size(); demonr++)
   {      
      demo_datum* d = demonstration_data[demonr];

      // 5.1 write all sensor values to file
      for (int svnr = 0; svnr < d->sensor_values.size(); svnr++)
      {
         double sensorvalue = d->sensor_values[svnr];
         f << sensorvalue << endl;
      }

      // 5.2 write action to file
      f << d->action << endl;
   }

} // save_demonstration_data



bool Robot::load_demonstration_data(string fname)
{
   // 1. open file for reading
   printf("\nTrying to load demonstration data from file\n%s\n", fname.c_str());
   std::ifstream f(fname);


   // 2. could we open this file?
   //    or does it exist at all?
   if (f.is_open() == false)
   {
      printf("\nError! Could not open file %s\n", fname.c_str());
      _getch();
      return false;
   }


   // 3. read number of demonstration data items
   int nr_demonstrations;
   f >> nr_demonstrations;
   printf("\nThere are %d sample (state,action) pairs in the demonstration data file.\n",
      nr_demonstrations);


   // 4. read length of sensor vector
   int len_sensor_vec;
   f >> len_sensor_vec;
   printf("Sensor vector length: %d\n", len_sensor_vec);


   // 5. read length of action vector
   int len_action_vec;
   f >> len_action_vec;
   printf("Action vector length: %d\n", len_action_vec);

   
   // 6. load all demonstration data items
   for (int item = 0; item < nr_demonstrations; item++)
   {
      // 6.1 create new demo data item
      demo_datum* d = new demo_datum;

      // 6.2 read all sensor values
      for (int svnr = 0; svnr < len_sensor_vec; svnr++)
      {
         double sensor_value;
         f >> sensor_value;
         d->sensor_values.push_back( sensor_value );
      }

      // 6.3 read demonstrated action
      int a;
      f >> a;
      d->action = (enum actions)a;

      // 6.4 store demo data item in vector of all demonstrations
      demonstration_data.push_back( d );
   }
   printf("\nI have read in %d data items.\n", (int)demonstration_data.size());


   _getch();

   return true;

} // load_demonstration_data


///
/// given the current (4D) sensor values
/// of the robot, this function returns an action
/// a based on the demonstration data that we collected
/// before from a teacher.
///
/// the k-Nearest Neighbor classifier approach is used to
/// to search for the k "most similar" examples in the
/// demonstrationd data and determine which action was
/// most often taken
///
/// that is the action, that the k-NN classifier suggests
/// to perform
///
enum Robot::actions Robot::k_nn_classifier(double* avg_distance, int k)
{
   double*       top_k_min_distances = new double[k];
   enum actions* top_k_actions       = new enum actions[k];


   // 1. for each demonstration data item,
   //    compute the distance between the stored state
   //    vector and the current state vector
   //    (the "state" is here defined just as the
   //    4D sensor vector)
   //    and
   //    compute the top k smallest distances
   //    and store the corresponding actions
   for (int i = 0; i < demonstration_data.size(); i++)
   {
      // 1.1 get next demo data item
      demo_datum* d = demonstration_data[i];

      // 1.2 compute & store "distance" between the
      //     demo data sensor vector and
      //     the robot's current sensor vector
      double dist = L1_norm( d->sensor_values, this->sensor_values );

      // 1.3 just store the first k min distances & actions
      if (i < k)
      {
         top_k_min_distances[i] = dist;
         top_k_actions[i]       = d->action;
      }
      else
      {
         // what is currently the largest of the k stored distances?
         int largest_dist_idx = 0;
         for (int j = 1; j < k; j++)
         {
            if (top_k_min_distances[j] > top_k_min_distances[largest_dist_idx])
               largest_dist_idx = j;
         }

         // check whether we have found an even smaller distance
         // than the k smallest distances found so far
         if (dist < top_k_min_distances[largest_dist_idx])
         {
            top_k_min_distances[largest_dist_idx] = dist;
            top_k_actions[largest_dist_idx]       = d->action;
         }
      }
         
   } // for (all demonstration data items)


   // 2. for debugging only:
   //    show the top k actions
   for (int i = 0; i < k; i++)
   {
      printf("Best action #%d: dist=%f, act=%d\n",
         i,
         top_k_min_distances[i],
         top_k_actions[i]);
   }


   // 3. which action is done most oftenly
   //    in the k examples?

   // 3.1 set all actions counters for all (3)
   //     actions to 0
   double action_counters[NR_ACTIONS];
   for (int i = 0; i < NR_ACTIONS; i++)
      action_counters[i] = 0;

   // 3.2 count how often each action occurs
   //     in the list of the top k neighbours
   for (int j = 0; j < k; j++)
      action_counters[(int)top_k_actions[j]]++;

   // 3.3 which action is done most often?
   int action_idx_most_often = 0;
   for (int a = 1; a < NR_ACTIONS; a++)
      if (action_counters[a] > action_counters[action_idx_most_often])
         action_idx_most_often = a;

   enum actions suggested_action =
      (enum actions) action_idx_most_often;

   // 3.4 compute the average distance between
   //     the best k found states and the current state
   //     in order to assess how "good" the k nearest
   //     demonstrations examples fit to the current state
   *avg_distance = 0.0;
   for (int j = 0; j < k; j++)
      *avg_distance += top_k_min_distances[j];
   *avg_distance /= (double)k;


   // 4. free memory
   delete[] top_k_min_distances;
   delete[] top_k_actions;


   // 5. return best action found
   return suggested_action;
   
} // k_nn_classifier
