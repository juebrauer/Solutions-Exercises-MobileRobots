#include "Robot.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <conio.h>
#include <fstream>



#include "math_tools.h"
#include "params.h"

Robot::Robot(string          name,
             int             radius,
             Point2d         start_pos,
             double          start_orientation,
             vector<double>  wall_sensor_angles,
             vector<double>  wall_sensor_distances,
             double          food_sensor_range
            )
{
  printf("Robot::Robot() constructor START\n");
  age = 0;

  this->name                  = name;
  this->radius                = radius,
  this->pos                   = start_pos;
  this->orientation           = start_orientation;
  this->wall_sensor_angles    = wall_sensor_angles;
  this->wall_sensor_distances = wall_sensor_distances;
  this->food_sensor_range     = food_sensor_range;
  nr_wall_sensors             = (int) wall_sensor_angles.size();

  // test for std::map as Q-value storage
  /*
  char key[100];
  sprintf(key, "%d,%d,%d,%d,%d,%d,%d",
     1, 2, 3, 4, 5, 6, 0);
  q_values[key] = 0.2;
  printf("Robot: q_values[%s]=%.2f\n", key, q_values[key]);
  */


  // check whether file already exists
  string fname = Q_VALUES_FILENAME;
  ifstream myfile( fname );
  if (myfile.is_open() == false)
  {
     printf("\t File %s does not exist!\n", fname.c_str());

     // Initialize all Q-values randomly
     // 11*11*3*3=1089
     printf("\t Initializing Q-values ...");
     int nr_q_values_stored = 0;
     for (int s0 = 0; s0 <= 10; s0++)
        for (int s1 = 0; s1 <= 10; s1++)
           for (int s2 = 0; s2 <= 2; s2++)
               for (int a = 0; a < NR_ACTIONS;  a++)
               {
                  char key[100];
                  sprintf(key, "%d,%d,%d,%d",
                     s0, s1, s2, a);
                  q_values[key] = get_rand_val_from_interval(-1.0, 1.0);
                  nr_q_values_stored++;
               }
   printf("\t Q-Values initialized randomly.\n");
     printf("\t Initialized %d q-values (%d).\n",
        nr_q_values_stored, (unsigned int)q_values.size());

     save_q_values_to_file( fname );
  }

  load_q_values_from_file( fname );

  printf("Robot::Robot() constructor END\n");

  printf("\nPress a key to start/continue learning!\n");
  _getch();

} // Robot constructor


void Robot::save_q_values_to_file(string fname)
{
   printf("\t Saving Q-values to file %s ...", fname.c_str());
   ofstream myfile(fname.c_str());

   myfile << get_age() << "\n";

   typedef std::map<std::string, double>::iterator it_type;
   for (it_type iterator = q_values.begin(); iterator != q_values.end(); iterator++)
   {
      myfile << iterator->first << "\n";
      myfile << iterator->second << "\n";
   }

   myfile.close();
   printf("\t Saving finished! Saved %d Q-Values to file\n",
      (unsigned int)q_values.size());

} // save_q_values_to_file


void Robot::load_q_values_from_file(string fname)
{
   q_values.clear();

   printf("\t Loading Q-values from file %s ...\n", fname.c_str());
   ifstream myfile(fname.c_str());

   if (myfile.is_open() == false)
   {
      printf("Fatal error!");
      _getch();
      exit(-1);
   }

   string l1, l2;
   
   getline(myfile, l1);
   int age_from_file = atoi(l1.c_str());
   set_age(age_from_file);

   printf("\t Robot has learned already for %d steps.\n", age);
      
   while (getline(myfile, l1))
   {
      getline(myfile, l2);

      double val = atof(l2.c_str());

      //printf("%s --> %.3f\n", l1.c_str(), val);

      q_values[l1] = val;
   }

   myfile.close();
   printf("\t Loading finished! Loaded %d Q-Values from file.\n",
      (unsigned int)q_values.size());
   

} // load_q_values_from_file


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

int Robot::get_nr_wall_sensors()
{
  return nr_wall_sensors;
}


vector<double> Robot::get_wall_sensor_values()
{
  return wall_sensor_values;
}

vector<double> Robot::get_wall_sensor_angles()
{
  return wall_sensor_angles;
}

vector<double> Robot::get_wall_sensor_distances()
{
  return wall_sensor_distances;
}


void Robot::compute_wall_sensor_values(Mat world)
{
  // 1. clear old sensor values
   wall_sensor_values.clear();

  // 2. for each distance sensor
  for (int sensor_nr = 0; sensor_nr < nr_wall_sensors; sensor_nr++)
  {
    // 2.1 get (x,y) coords of current robot position
    double x = pos.x;
    double y = pos.y;

    // 2.2 get sensor orientation relative to robots orientation
    double sensor_angle = wall_sensor_angles[sensor_nr];

    // 2.3 map robot angle + sensor_angle to a direction vector
    double sensor_dx = cos(orientation + sensor_angle);
    double sensor_dy = sin(orientation + sensor_angle);

    // 2.4 compute sensor start position
    double sensor_startx = x + sensor_dx * radius;
    double sensor_starty = y + sensor_dy * radius;

    // 2.5 now move from sensor start position into sensor direction
    //     till we reach the maximum distance or hit an obstacle in the world (white pixel)

    // 2.6 get maximum sensor distance
    double sensor_max_dist = wall_sensor_distances[sensor_nr];

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
    wall_sensor_values.push_back(step);

    // 2.9 output sensor value for debugging
    //printf("sensor #%d: %d\n", sensor_nr, step);

  } // for (sensor_nr)

} // compute_wall_sensor_values



bool Robot::test_wall_bump(Mat world)
{
   for (double angle = 0.0; angle < 2 * M_PI; angle += 0.1)
   {
      // compute world coordinates (x,y) of check point
      int x = (int) (pos.x + cos(angle) * radius);
      int y = (int) (pos.y + sin(angle) * radius);

      if ((x < 0) || (x >= world_width) || (y < 0) || (y >= world_height))
         continue;

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



void Robot::update(Mat world, vector<Food*> food_pieces)
{
   world_width  = world.cols;
   world_height = world.rows;

  compute_wall_sensor_values( world );
  compute_food_sensor_values( food_pieces );

  // save old position
  Point2d old_pos = pos; 

  // get sensor values
  double sensor_FRONT = wall_sensor_values[0];
  double sensor_LEFT  = wall_sensor_values[1];
  double sensor_RIGHT = wall_sensor_values[2];

  // do programmed behavior?
  if (USE_PROGRAMMED_BEHAVIOR)
  {
     double rnd_val = get_rand_val_from_interval(-M_PI / 100, +M_PI / 100);
     double cmd_turn_angle = 0.0;
     double cmd_pixel_to_drive = 0.0;
     if (sensor_FRONT < 20)
     {
        // turn right
        cmd_turn_angle = M_PI / 16 + rnd_val;
        turn(cmd_turn_angle);
     }
     else
        if ((sensor_LEFT < 10) || (sensor_RIGHT < 10))
        {
           // turn left or right?
           if (sensor_LEFT < sensor_RIGHT)
           {
              // turn right
              cmd_turn_angle = M_PI / 16 + rnd_val;
           }
           else
           {
              // turn left
              cmd_turn_angle = -M_PI / 16 + rnd_val;
           }
           turn(cmd_turn_angle);
        }
        else
        {
           // move forward in current direction
           cmd_pixel_to_drive = 1.0;
           move(cmd_pixel_to_drive);
        }

  } // if (do programmed behavior?)
  else
  {
     // do not use programmed behavior,
     // but choose behavior due to current Q-values

     // 1. fill state[6] array with current values
     set_state_vector();

     // 2. find action in current state with highest Q-value
     double highest_q_value = 0.0;
     int best_action = -1;
     for (int a = 0; a < NR_ACTIONS; a++)
     {
        // prepare retrieval key
        char key[100];
        sprintf(key, "%d,%d,%d,%d",
           state[0], state[1], state[2], a);

        // get q-value for that state-action pair (s,a)
        double q_val = q_values[key];

        // found action with higher q-value?
        if ((a == 0) || (q_val > highest_q_value))
        {
           best_action = a;
           highest_q_value = q_val;           
        }

     } // for (all actions a)

     // 3. with probability epsilon overwrite this action
     //    i.e., execute an action randomly
     double rnd_val = get_rand_val_from_interval(0, 1);
     if (rnd_val < EPSILON_EXPLORATION)
        best_action = rand() % NR_ACTIONS;
     
     // 4. execute that "best" found action
     switch (best_action)
     {
        case 0: // turn left
           turn(-M_PI / 16);
           break;
        case 1: // turn right
           turn(M_PI / 16);
           break;
        case 2: // move forward
           move(2);
           break;     
     }

     // 4. remember which action we did for the next
     //    update of the q-values
     last_action_done = best_action;


     // 5. already prepare state-action-key for upcoming
     //    q-values update
     sprintf(last_state_action_pair, "%d,%d,%d,%d",
        state[0], state[1], state[2], best_action);
     //printf("Next state action pair to be updated will be: %s\n",
     //   last_state_action_pair);

  } // if (do programmed behavior?)


  
  // did the robot bump into a wall?
  bool bumped = test_wall_bump( world );
  if (bumped)
  {
    printf("B");
    pos = old_pos;

    update_current_reward( REWARD_BUMPING_INTO_WALL );
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

  // 2D world with strange behavior ;-)
  if (x >= world_width)
     x = 0;
  if (x < 0)
     x = world_width - 1;
  if (y >= world_height)
     y = 0;
  if (y < 0)
     y = world_height - 1;

  // store new position
  pos = Point2d(x, y);

} // move



void Robot::turn(double angle)
{
  orientation += angle;

} // turn



double Robot::get_episode_return()
{
   return episode_return;
}


void Robot::set_episode_return(double ret)
{
   episode_return = ret;
}



double Robot::get_food_sensor_range()
{
   return food_sensor_range;
}



void Robot::compute_food_sensor_values(vector<Food*> food_pieces)
{
   // 1. find food item piece nearest to robot
   angle_to_next_food_piece = 0.0;
   distance_to_next_food_piece = 0.0;
   food_type = 0.0;
   double min_dist = 10000000.0;
   int    idx_nearest_item = -1;
   for (int i = 0; i < food_pieces.size(); i++)
   {
      // get position of food piece
      Point2i food_pos = food_pieces[i]->get_position();

      // compute distance between food piece and robot
      double dist = norm((Point2d)food_pos - pos);

      // this food piece is out of the range of the robot's
      // food sensor, so continue with next food piece...
      if (dist > food_sensor_range)
         continue;

      // found a food item piece nearer than any before?
      if (dist < min_dist)
      {
         min_dist = dist;
         idx_nearest_item = i;
      }

   } // for (all food item pieces)


   // 2. there is no food item within the food sensor range
   //    of our robot!
   if (idx_nearest_item == -1)
   {
      printf("no food item within the range of the food sensor!\n");
      return;
   }


   // 3. compute angle between robot's orientation
   //    and food item piece

   Point2d food_pos = (Point2d)food_pieces[idx_nearest_item]->get_position();

   // compute direction vector to the food item
   double dirx = food_pos.x - pos.x;
   double diry = food_pos.y - pos.y;

   // compute angle to food item,
   // if robots position is the origin of the coordinate system
   double angle_to_food = atan2(diry, dirx);

   //printf("angle_to_food=%.1f, robot_angle=%.1f\n", angle_to_food, orientation);

   // Get the (minimum) difference between the two angles
   // i) (robot) orientation and ii) angle_to_food
   // see
   // http://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
   //
   // note: angle_diff is not the absolute difference, but a signed difference
   //       i.e., it still preserves the information in which direction to turn!
   double angle_diff = 
      atan2(sin(orientation - angle_to_food), cos(orientation - angle_to_food));

   //printf("angle_diff = %.1f\n", angle_diff);


   // 4. set all 3 food sensor values:
   //    (angle to next food item, distance to next food item, food-type)
   //
   angle_to_next_food_piece = angle_diff;
   distance_to_next_food_piece = sqrt(dirx*dirx+diry*diry);
   if (food_pieces[idx_nearest_item]->get_reward() > 0)
      food_type = 1;
   else
      food_type = 2;

   if (0)
   {
      printf("food sensor: (%.2f, %.1f, %.0f)\n",
         angle_to_next_food_piece,
         distance_to_next_food_piece,
         food_type);
   }

} // compute_food_sensor_values



void Robot::get_food_sensor_values(double& angle, double& dist, double& ftype)
{
   angle = angle_to_next_food_piece;
   dist  = distance_to_next_food_piece;
   ftype = food_type;

} // get_food_sensor_values



/// divide an interval [min_value,max_value] into
/// nr_bins many bins
/// and compute into which bin the value <value> falls
///
int discretize(double value, double min_value, double max_value, int nr_bins)
{
   // 1. compute size of the interval [min_value, max_value]
   double interval_size = max_value - min_value;

   // 2. compute size of a single bin if we divide the interval
   //    into <nr_bins> many bins
   double bin_size = interval_size / (double)nr_bins;

   // 3. into which bin does <value> fall?
   double bin_bottom = min_value;
   int bin_nr;
   for (bin_nr = 0; bin_nr < nr_bins; bin_nr++)
   {
      double bin_top = bin_bottom + bin_size;

      if ((value >= bin_bottom) && (value < bin_top))
         break;

      bin_bottom = bin_top;

   } // for (all bins)

   // 4. return the bin_nr in which bin_nr lies
   //    the returned bin_nr will always be from the set {0,1,2,...,nr_bins-1}
   return bin_nr;

} // discretize


///
/// we have 11*11*3*4 = 1452 different states
///
void Robot::set_state_vector()
{
   state[0] = discretize(angle_to_next_food_piece, -M_PI, +M_PI, 10);
   state[1] = discretize(distance_to_next_food_piece, 0, food_sensor_range, 10);
   state[2] = (int)food_type;   
   //state[3] = discretize(wall_sensor_values[0], 0, wall_sensor_distances[0], 3);

} // set_state_vector


void Robot::show_state_vector()
{
   printf("s=(%d,%d,%d)\n", state[0], state[1], state[2]);
}


void Robot::set_current_reward(double rew)
{
   current_reward = rew;
}


void Robot::update_current_reward(double rew)
{
   current_reward += rew;
}


double Robot::get_current_reward()
{
   return current_reward;
}


void Robot::update_q_values(Mat world, vector<Food*> food_pieces)
{
   if (SHOW_INFOS)
      printf("Updating Q-values:\n");

   // 1. show for which state-action-values we will update
   //    the Q-value
   if (SHOW_INFOS)
      printf("\t we are going to update Q(s,a) with (s,a)=%s\n",
         last_state_action_pair);
   double old_q_value_s_a = q_values[last_state_action_pair];

   // 2. compute current wall sensor values & food sensor values
   //    in order to determine current state s'
   compute_wall_sensor_values(world);
   compute_food_sensor_values(food_pieces);
   set_state_vector();

   // 3. show current state s'
   if (SHOW_INFOS)
      printf("\t s'=(%d,%d,%d)\n",
         state[0], state[1], state[2]);

   // 4. find action a' which maximizes Q(s',a')
   double max_q_value;
   for (int a_prime = 0; a_prime < NR_ACTIONS; a_prime++)
   {
      char key[100];
      sprintf_s(key, "%d,%d,%d,%d",
         state[0], state[1], state[2], a_prime);

      // get q-value Q(s',a')
      double q_value_s_prime_a_prime = q_values[key];

      if ((a_prime == 0) || (q_value_s_prime_a_prime > max_q_value))
      {
         max_q_value = q_value_s_prime_a_prime;
      }
   } // for (all actions a_prime)

   // 5. compute new Q(s,a) value
   double r = get_current_reward();
   double new_q_value_s_a =
      old_q_value_s_a + LEARN_RATE * (r + GAMMA*max_q_value - old_q_value_s_a);

   // 6. store new Q(s,a) value
   q_values[last_state_action_pair] = new_q_value_s_a;

   if (SHOW_INFOS)
   {
      printf("\t old Q(s,a) = %.3f\n", old_q_value_s_a);
      printf("\t new Q(s,a) = %.3f\n", new_q_value_s_a);
   }

} // update_q_values


void Robot::set_age(int age)
{
   this->age = age;
}

int Robot::get_age()
{
   return age;
}