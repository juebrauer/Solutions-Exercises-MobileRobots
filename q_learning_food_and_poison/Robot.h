#pragma once

#include "opencv2/core.hpp"
#include <map>
#include "Food.h"


using namespace std;
using namespace cv;

class Robot
{ 
  public:
                    Robot::Robot(string             name,
                                 int                radius,
                                 Point2d            start_pos,
                                 double             start_orientation,
                                 vector<double>     wall_sensor_angles,
                                 vector<double>     wall_sensor_distances,
                                 double             food_sensor_range
                                );

    void            compute_wall_sensor_values(Mat world);

    void            update(Mat world, vector<Food*> food_pieces);

    Point2d         get_position();

    double          get_orientation();

    double          get_radius();

    int             get_nr_wall_sensors();

    vector<double>  get_wall_sensor_values();

    vector<double>  get_wall_sensor_angles();

    vector<double>  get_wall_sensor_distances();

    void            move(double pixel);

    void            turn(double angle);

    bool            test_wall_bump(Mat world);


    /// methods related to Q-learning:

    double          get_episode_return();

    void            set_episode_return(double ret);

    double          get_food_sensor_range();

    void            compute_food_sensor_values(vector<Food*> food_pieces);

    void            get_food_sensor_values(double& angle, double& dist, double& ftype);

    void            set_state_vector();

    void            show_state_vector();

    void            set_current_reward(double rew);

    void            update_current_reward(double rew);

    double          get_current_reward();

    void            update_q_values(Mat world, vector<Food*> food_pieces);

    void            save_q_values_to_file(string fname);

    void            load_q_values_from_file(string fname);

    void            set_age(int age);

    int             get_age();

    




private:

   string               name;
   double               radius;
   Point2d              pos;
   double               orientation;

   vector<double>       wall_sensor_angles;
   vector<double>       wall_sensor_distances;
   int                  nr_wall_sensors;
   vector<double>       wall_sensor_values;

   double               food_sensor_range;
   double               angle_to_next_food_piece;
   double               distance_to_next_food_piece;
   double               food_type;

   double               episode_return;
   double               current_reward;

   int                  state[3];
   map<string, double>  q_values;
   int                  last_action_done;
   char                 last_state_action_pair[100];
   int                  age;

   int                  world_width;
   int                  world_height;
   
}; // class Robot
