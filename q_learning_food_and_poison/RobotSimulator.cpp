/// Q-Learning - Food and Poison
///
/// In this example a simple 2D robot learns to
/// behave in an unknown world, such that its expected
/// rewards are maximzed.
///
/// In the world there are food pieces (green circles)
/// and poison pieces (red circles).
/// The robot does not know what they mean when it starts
/// to explore the world, but soon experiences that moving
/// into green circles is associated with a positive reward and
/// moving into red circles is associated with a negative reward.
///
/// For this, it adapts its state-action-value function Q(s,a),
/// such that it moves into green circles and avoids red circles.
///
/// ---
/// by Prof. Dr. Jürgen Brauer, www.juergenbrauer.org

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES


#include <iostream>
#include <conio.h>
#include <math.h>
#include <windows.h>

#include "opencv2/opencv.hpp"
#include "Robot.h"
#include "Food.h"
#include "params.h"



using namespace cv;
using namespace std;

bool exit_simulation;
vector<Food*> food_pieces;


void simulate_one_episode(Mat world,
                          Robot& r1,
                          vector<double> wall_sensor_angles,
                          vector<double> wall_sensor_distances)
{   
   int simulation_step = r1.get_age();
   Mat image;
   r1.set_episode_return( 0.0 );
   while (!exit_simulation)
   {
      if (SHOW_INFOS)
         printf("\n");

      // 1. initialize image with world image
      world.copyTo(image);
      
      // 2. move all food / poison pieces
      for (int i = 0; i < food_pieces.size(); i++)
      {
         food_pieces[i]->update(world);
      }

      // 3. move the robot according to its specific behavior
      r1.set_current_reward(0.0);
      r1.update(world, food_pieces);
            
      // 4. draw all food pieces
      for (int i = 0; i < food_pieces.size(); i++)
      {
         food_pieces[i]->draw( image );
      }

      // 5. show the robot's position as a circle and its orientation by a line
      Point2d pos = r1.get_position();
      double  ori = r1.get_orientation();
      double  dx = cos(ori);
      double  dy = sin(ori);
      double  r = r1.get_radius();
      circle(image, pos, (int)r, CV_RGB(255, 0, 0), 1);
      line(image, pos, pos + Point2d(dx*r, dy*r), CV_RGB(0, 255, 0), 1);


      // 6. compute all the sensor values
      vector<double> sensor_values = r1.get_wall_sensor_values();


      // 7. for each sensor draw a sensor ray
      if (0)
      {
         for (int sensor_nr = 0; sensor_nr < r1.get_nr_wall_sensors(); sensor_nr++)
         {
            double sensor_value = sensor_values[sensor_nr];

            // get (x,y) coords of current robot position
            double x = pos.x;
            double y = pos.y;

            // get sensor orientation relative to robots orientation
            double sensor_angle = wall_sensor_angles[sensor_nr];

            // map robot angle + sensor_angle to a direction vector
            double sensor_dx = cos(r1.get_orientation() + sensor_angle);
            double sensor_dy = sin(r1.get_orientation() + sensor_angle);

            // compute sensor start position
            double sensor_startx = x + sensor_dx * r1.get_radius();
            double sensor_starty = y + sensor_dy * r1.get_radius();

            // compute sensor ray end position
            double sensor_endx = sensor_startx + sensor_dx * sensor_value;
            double sensor_endy = sensor_starty + sensor_dy * sensor_value;

            // draw sensor ray line
            line(image, Point((int)sensor_startx, (int)sensor_starty), Point((int)sensor_endx, (int)sensor_endy), CV_RGB(255, 255, 0), 1);

         } // for (draw all sensor rays)
      }


      // 9. did the robot move over a food piece?
      bool collision_detected;
      do
      {
         collision_detected = false;
         for (int i = 0; i < food_pieces.size(); i++)
         {
            double dist =
               norm((Point2d)(food_pieces[i]->get_position()) - r1.get_position());
            if (dist < r1.get_radius() + food_pieces[i]->get_radius())
            {
               // robot has moved over i-th food piece!

               // change reward for current step
               r1.update_current_reward( food_pieces[i]->get_reward() );
               
               // delete food item from list of food pieces
               food_pieces.erase(food_pieces.begin() + i);
               
               if (PLAY_SOUND)
                  PlaySound(TEXT("sounds/crunch.wav"), NULL, SND_FILENAME);

               collision_detected = true;
               break;
            }
         }

      } while (collision_detected);
      

      // 9. show food sensor range & visualize food sensor values
      circle(image, r1.get_position(), (int)r1.get_food_sensor_range(), CV_RGB(0, 255, 255), 1);
      double angle, dist, food_type;
      r1.get_food_sensor_values(angle, dist, food_type);
      Point2i p1 = r1.get_position();
      double rad = r1.get_orientation() - angle;      
      Point2i p2 = p1 + Point2i((int)(cos(rad)*dist),(int)(sin(rad)*dist));
      line(image, p1, p2, CV_RGB(0, 255, 255), 1);


      // 10. each step needs a little bit energy...
      r1.update_current_reward(REWARD_ONE_STEP);


      // 11. if a robot is near to a green food piece,
      //     it already gets a small positive reward
      //     if it is near to a red food piece (=poison),
      //     it already get a small negative reward
      /*
      if (food_type == 1)
      {
         r1.update_current_reward(75.0-dist);
      }

      if (food_type == 2)
      {
         r1.update_current_reward(-(75.0 - dist));
      }
      */


      // 12. update q-values now      
      r1.update_q_values( world, food_pieces );
      

      // 13. keep track of the episode return      
      r1.set_episode_return(
         r1.get_episode_return() + r1.get_current_reward()
         );


      // 14. show simulation step nr
      char txt[100];
      sprintf_s(txt, "simulation step %d. Reward / Episode return: %.2f /%.2f",
         simulation_step,
         r1.get_current_reward(),
         r1.get_episode_return());
      putText(image,
         txt,
         Point(20, 50),
         FONT_HERSHEY_SIMPLEX, 0.7, // font face and scale
         CV_RGB(255, 0, 0), // white
         1); // line thickness and type


      // 16. show world with robot, sensor rays and additional textual information
      imshow("Robot Simulator", image);


      // 17. save image?
      if (SAVE_IMAGES)
      {
         char fname[500];
         sprintf_s(fname, "%s\\img%05d.png", SAVE_FOLDER, simulation_step);
         imwrite(fname, image);
      }


      // 18. wait for a key
      char c;
      if (WAIT_FOR_KEYPRESS)
         c = (char)waitKey(0);
      else
         c = (char)waitKey(1); // waitKey() is always needed for imshow()!


      // 19. ESC pressed?
      if (c == 27)
         exit_simulation = true;


      // 20. one step simulated more
      simulation_step++;
      r1.set_age( simulation_step );


   } // while (simulation shall continue)

} // simulate_one_episode




int main()
{
   srand((unsigned int)time(NULL));

  // 1. load image of world: change this to the folder where you store
   //   a black-white pixel coding your world
  //    white pixels mean: no drivable space
  //    black pixels mean: drivable space
  string img_filename = "world3.png";
  Mat world = imread(img_filename);


  // 2. check whether world dimensions are valid
  if ((world.cols == 0) && (world.rows == 0))
  {
    cout << "Error! Could not read the image file: " << img_filename << endl;
    _getch();
    return -1;
  }


  // 3. get world dimensions
  int WORLD_WIDTH  = world.cols;
  int WORLD_HEIGHT = world.rows;


  // 4. prepare an image (where we will draw the robot and the world)
  Mat image(WORLD_HEIGHT, WORLD_WIDTH, CV_8UC3);


  // 5. create a robot

  vector<double> wall_sensor_angles, wall_sensor_distances;
  wall_sensor_angles.push_back(0);
  wall_sensor_angles.push_back(-M_PI / 4);
  wall_sensor_angles.push_back(+M_PI / 4);
  wall_sensor_distances.push_back(100);
  wall_sensor_distances.push_back(100);
  wall_sensor_distances.push_back(100);

  Robot r1("R2D2",
           10,
           Point(WORLD_HEIGHT / 2, WORLD_WIDTH / 2),
           M_PI / 4,
           wall_sensor_angles,
           wall_sensor_distances,
           FOOD_SENSOR_RADIUS);
           

  // 6. create food pieces with positive and negative rewards  
  for (int i = 0; i < NR_FOOD_PIECES; i++)
  {
     int rnd = rand() % 2;
     double reward;
     if (rnd == 0)
        reward = REWARD_GREEN_FOOD;
     else
        reward = REWARD_RED_FOOD;
     Food* f = new Food(rand() % WORLD_WIDTH,
                        rand() % WORLD_HEIGHT,
                        FOOD_RADIUS,
                        reward);
     food_pieces.push_back( f );
  }



  // 7. let user select target locations
  //    then try to get the robot there
  exit_simulation = false;
  while (!exit_simulation)
  {
      simulate_one_episode(world, r1, wall_sensor_angles, wall_sensor_distances);
  }  

  // 8. save Q-Values learned so far
  r1.save_q_values_to_file( Q_VALUES_FILENAME );

} // main