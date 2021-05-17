#pragma once

#include "opencv2/core.hpp"
#include <deque>
#include <map>
#include "food.h"


using namespace std;
using namespace cv;

class Robot
{
public:
                   Robot(string             name,
                         int                radius,
                         Point2d            start_pos
                   );

   void            compute_sensor_values(Mat world, vector<Food*> food_pieces);

   int             get_nr_sensors();

   void            update(Mat world, vector<Food*> food_pieces);

   Point2d         get_position();

   double          get_radius();

   void            move(double pixelsidewards, double pixelupdown);

   bool            test_wall_bump(Mat world);


   /// methods related to Reinforcement-Learning:

   double          get_episode_return();

   void            set_episode_return(double ret);

   void            compute_state_vector();

   void            show_state_vector();

   void            set_current_reward(double rew);

   void            update_current_reward(double rew);

   double          get_current_reward();

   void            set_age(int age);

   int             get_age();

   void            update_state_action_associations();

   double          get_angle_to_green_food();

   double          get_angle_to_red_food();








private:

   void                 set_state_action_pair_str(char* str, int str_size,
                                                  int a1, int a2,
                                                  int act);

   void                 search_nearest_food_item(Mat world,
                                                 vector<Food*> food_pieces,
                                                 int foodtype_to_search_for,
                                                 Point2d& vec_to_food);
   double               angle_to_green_food;
   double               angle_to_red_food;
   
   string               name;
   double               radius;
   Point2d              pos;

   double               episode_return;
   double               current_reward;

   int                  state[2];
   int                  age;

   int                  world_width;
   int                  world_height;


   char                 last_state_action_pair[50];
   deque<string>        state_action_history;
   map<string, double>  brain_state_action_scores;

   double               current_exploration_rate;
   
}; // class Robot
