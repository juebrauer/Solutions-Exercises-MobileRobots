#include "Robot.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <conio.h>
#include <fstream>



#include "math_tools.h"
#include "params.h"

#include "opencv2/opencv.hpp"


Robot::Robot(  string          name,
               int             radius,
               Point2d         start_pos)
{
   this->age = 0;
   this->name = name;
   this->radius = radius,
   this->pos = start_pos;

   current_exploration_rate = EPSILON_EXPLORATION;

   // Initialize the robot's "brain"
   // of probabilities of mapping states to actions
   brain_state_action_scores.clear();
   for (int d1 = 0; d1 < NR_DIFFERENT_DISTANCES; d1++)
   {
      for (int d2 = 0; d2 < NR_DIFFERENT_DISTANCES; d2++)
      {
         for (int d3 = 0; d3 < NR_DIFFERENT_DISTANCES; d3++)
         {
            for (int d4 = 0; d4 < NR_DIFFERENT_DISTANCES; d4++)
            {
               for (int a = 0; a < NR_ACTIONS; a++)
               {

                  char state_action_pair[50];
                  set_state_action_pair_str(state_action_pair, 50, d1, d2, d3, d4, a);

                  //double some_random_ness = (double) ((rand() % 21) - 10) / 100.0; // [-0.1,0.1]

                  brain_state_action_scores[string(state_action_pair)] = 0.5;

                  //if (a == ACTION_MOVE_FORWARD)
                  //               brain_state_action_scores[string(state_action_pair)] += 0.1;
               } // for actions a

            } // for d4
         } // for d3
      } // for d2
   } // for d1

} // Robot constructor


void Robot::set_state_action_pair_str(char* str, int str_size,
   int d1, int d2,
   int d3, int d4,
   int a)   
{
   sprintf_s(str, str_size, "[s=(%d,%d,%d,%d)<->a=%s]",
      d1,d2,d3,d4, action_str[a].c_str());
}



double Robot::get_radius()
{
   return radius;
}

Point2d Robot::get_position()
{
   return pos;
}


void Robot::search_nearest_food_item(Mat world,
                                     vector<Food*> food_pieces,
                                     int foodtype_to_search_for,
                                     Point2d& vec_to_food)
{
   // 1. find food item piece nearest to robot
   double min_dist = 10000000.0;
   int    idx_nearest_item = -1;
   for (int i = 0; i < food_pieces.size(); i++)
   {
      // get position of food piece
      Point2i food_pos = food_pieces[i]->get_position();

      // compute distance between food piece and robot
      double dist = norm((Point2d)food_pos - pos);

      // found a food item piece of the desired type
      // <foodtype_to_search_for> nearer than any before?
      if ((dist < min_dist) && (food_pieces[i]->get_food_type()== foodtype_to_search_for))
      {
         min_dist = dist;
         idx_nearest_item = i;
      }

   } // for (all food item pieces)


   // 2. compute vector to food item and store it
   Point2d food_pos = (Point2d)food_pieces[idx_nearest_item]->get_position();
   vec_to_food = food_pos - pos;

} // search_nearest_food_item


///
/// Search for the nearest food type
/// to the robot of type <foodtype>
/// and return the angle
///
void Robot::compute_sensor_values(Mat world,
                                  vector<Food*> food_pieces)
{
   search_nearest_food_item(world, food_pieces, FOODTYPE_GREEN,
                            vec_to_green_food);
   search_nearest_food_item(world, food_pieces, FOODTYPE_RED,
                            vec_to_red_food);

} // compute_sensor_values


bool Robot::test_wall_bump(Mat world)
{
   for (double angle = 0.0; angle < 2 * M_PI; angle += 0.1)
   {
      // compute world coordinates (x,y) of check point
      int x = (int)(pos.x + cos(angle) * radius);
      int y = (int)(pos.y + sin(angle) * radius);

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
   world_width = world.cols;
   world_height = world.rows;

   // 1. compute new sensor values
   compute_sensor_values(world, food_pieces);


   // 2. compute current state-action-pair
   compute_state_vector();


   // 3. determine action for next step
   // choose an action randomly?
   printf("Searching for the action with the highest score:\n");  
   double best_score = -1.0;
   int action_with_highest_score = -1;
   for (int a = 0; a < NR_ACTIONS; a++)
   {
      // prepare key for dictionary, i.e.
      // a state-action-pair string
      char search_str[50];
      set_state_action_pair_str(search_str, 50,
         state[0], state[1], state[2], state[3], a);

      // get the corresponding score of that
      // state-action pair
      double score =
         brain_state_action_scores[string(search_str)];

      // found a better score?
      if ((score > best_score) || (action_with_highest_score == -1))
      {
         best_score = score;
         action_with_highest_score = a;
      }

      // show score for that state-action pair?
      printf("\t%s -> %f\n", search_str, score);
      
   } // for (all actions)

   int best_action = action_with_highest_score;
   printf("Best action found: %s\n", action_str[best_action].c_str());

   

   // 4. Some randomness for better exploration
   //    of state-action-pairs:
   //
   // with probability epsilon overwrite the computed
   // action. I.e., execute a random action
   if (1)
   {
      double rnd_val = get_rand_val_from_interval(0, 1);
      if (rnd_val < current_exploration_rate)
      {
         printf("Random action! Exploration rate=%f\n",
            current_exploration_rate);
         best_action = rand() % NR_ACTIONS;
         printf("Doing a random action: %s\n", action_str[best_action].c_str());
      }
   }
   
   printf("Press a key to perform the chosen action!\n");
   cv::waitKey(1);


   // 5. execute chosen action from last step
   printf("Performing action %s\n", action_str[best_action].c_str());
   int stepwidth = 10;
   switch (best_action)
   {
      case ACTION_MOVE_LEFT: // turn left
         move(-stepwidth, 0);
         break;
      case ACTION_MOVE_RIGHT: // turn right
         move(+stepwidth, 0);
         break;
      case ACTION_MOVE_TOP:   // move to top
         move(0, -stepwidth);
         break;
      case ACTION_MOVE_DOWN:  // move down
         move(0, +stepwidth);
         break;
   }


   // 6. prepare a string that encodes
   //    the current state and the chosen action
   set_state_action_pair_str(last_state_action_pair, 50,
                             state[0], state[1], state[2], state[3], best_action);
   //printf("Current state-action pair is: %s\n", last_state_action_pair);


   // 7. store current state-action-pair in history
   //    of state-action-pairs
   state_action_history.push_front(string(last_state_action_pair));
   if (state_action_history.size() > STATE_ACTION_HISTORY_MAX_LEN)
   {
      state_action_history.pop_back();
   }


   // 8. show last state-action-pairs
   printf("History of state-actions pairs:\n");
   for (int i = 0; i < state_action_history.size(); i++)
      printf("t=%d: %s\n", -i, state_action_history[i].c_str());
   printf("\n");
   
     

  
   // did the robot bump into a wall?
   /**
   bool bumped = test_wall_bump( world );
   if (bumped)
   {
      printf("B");
      pos = old_pos;

      update_current_reward( REWARD_BUMPING_INTO_WALL );
   }
   */

} // update


void Robot::move(double pixelsidewards, double pixelupdown)
{
  // get (x,y) coords of current robot position
  double x = pos.x;
  double y = pos.y;

  // add direction vector to current position to compute new robot position
  x += pixelsidewards;
  y += pixelupdown;

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



double Robot::get_episode_return()
{
   return episode_return;
}


void Robot::set_episode_return(double ret)
{
   episode_return = ret;
}






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

      if (value < bin_top)
         break;

      bin_bottom = bin_top;

   } // for (all bins)

   // 4. put maximum value into last bin as well,
   //    not a new bin
   if (bin_nr >= nr_bins)
      bin_nr = nr_bins - 1;

   // 5. return the bin_nr in which bin_nr lies
   //    the returned bin_nr will always be from the set {0,1,2,...,nr_bins-1}
   return bin_nr;

} // discretize


///
/// we have 5*5*5*5=625 different states
///
void Robot::compute_state_vector()
{
   printf("sensor data (%.0f,%.0f,%.0f,%.0f) --> ",
      vec_to_green_food.x, vec_to_green_food.y,
      vec_to_red_food.x, vec_to_red_food.y);

   // prepare current state vector
   state[0] = discretize(vec_to_green_food.x, -50.0, 50.0, NR_DIFFERENT_DISTANCES);
   state[1] = discretize(vec_to_green_food.y, -50.0, 50.0, NR_DIFFERENT_DISTANCES);

   state[2] = discretize(vec_to_red_food.x, -50.0, 50.0, NR_DIFFERENT_DISTANCES);
   state[3] = discretize(vec_to_red_food.y, -50.0, 50.0, NR_DIFFERENT_DISTANCES);

   show_state_vector();
  
} // compute_state_vector


void Robot::show_state_vector()
{
   printf("Current state is s=(%d,%d,%d,%d)\n", state[0], state[1], state[2], state[3]);
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


void Robot::set_age(int age)
{
   this->age = age;
}

int Robot::get_age()
{
   return age;
}


/// on the one hand,
/// the last N=<STATE_ACTION_HISTORY_MAX_LEN>
/// state action pairs are stored in
/// <state_action_history>:
///    [(distance,object-type)->action]
///    ...
///    [(distance,object-type)->action]
///
/// on the other hand,
/// we now know the reward <current_reward>
/// we have got for the last action
///
/// now increase or decrease the goodness of the
/// last N state-action-associations stored in
/// the <brain_state_action_scores> based on
/// current reward
/// 
void Robot::update_state_action_associations()
{
   map<string, double>::iterator it;

   double learn_rate_scaled_reward = current_reward * LEARN_RATE;

   printf("Reward for current step = %f\n", current_reward);
   for (int i = 0; i < state_action_history.size(); i++)      
   {
      // get score of that state-action-pair
      string state_action_pair = state_action_history[i];
      double score = brain_state_action_scores[state_action_pair];

      // increase/decrease score
      //double pos_scaling_factor = (double)(i+1) / (double)state_action_history.size();
      //score += learn_rate_scaled_reward * pos_scaling_factor;

      score += learn_rate_scaled_reward;

      // make sure scores stay in [0.0, 1.0]
      if (score < 0.0)
         score = 0.0;
      if (score > 1.0)
         score = 1.0;

      // store new score in map (dictionary)
      brain_state_action_scores[state_action_pair] = score;

      printf("New state-action-pair score %s -> score=%f\n",
             state_action_pair.c_str(), score);

   } // for (all state-action pairs)

} // update_state_action_associations



Point2d Robot::get_vec_to_green_food()
{
   return vec_to_green_food;
}


Point2d Robot::get_vec_to_red_food()
{
   return vec_to_red_food;
}
