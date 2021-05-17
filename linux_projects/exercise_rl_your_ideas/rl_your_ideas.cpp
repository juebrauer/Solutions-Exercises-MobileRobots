/// Reinforcement Learning - Your ideas
///
/// In this example a simple 2D robot learns to
/// behave in an unknown world such that situations
///   are reached more frequently where a positive reward
///   is obtained
/// and 
///   situations are avoided where a negative reward
///   is obatined
/// 
/// Core idea:
/// Log the history of last actions done.
/// Whenever a positive reward is obtained, strengthen
/// the situation-->action associations.
/// Whenever a negative reward is obtained, weaken
/// the situation-->action associations.
///
/// In the world there are food pieces (green circles)
/// and poison pieces (red circles).
/// The robot does not know what they mean when it starts
/// to explore the world, but soon experiences that moving
/// into green circles is associated with a positive reward and
/// moving into red circles is associated with a negative reward.
///
/// ---
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES


#include <iostream>
#include <math.h>

#include "opencv2/opencv.hpp"
#include "robot.h"
#include "food.h"
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
   int green_counter = 0;
   int red_counter = 0;
   while (!exit_simulation)
   {
      if (SHOW_INFOS)
      {
         system("cls");
      }
         

      // 1. initialize image with world image
      world.copyTo(image);
      

      // 2. move all food / poison pieces
      for (uint i = 0; i < food_pieces.size(); i++)
      {
         food_pieces[i]->update(world);
      }


      // 3. reset reward
      r1.set_current_reward(0.0);


      // 4. move the robot according to its specific behavior      
      r1.update(world, food_pieces);
      

      // 5. draw all food pieces
      for (uint i = 0; i < food_pieces.size(); i++)
      {
         food_pieces[i]->draw( image );
      }


      // 6.1 show the robot's position as a circle
      Point2d pos = r1.get_position();
      double  r = r1.get_radius();
      circle(image, pos, (int)r, CV_RGB(255, 255, 255), 3);

      // 6.2 draw lines into the directions of the
      //     next green and next red food item
      double veclen = 40.0;
      int diry, dirx;
      diry = (int) (sin(r1.get_angle_to_green_food()) * veclen);
      dirx = (int) (cos(r1.get_angle_to_green_food()) * veclen);
      line(image, pos, pos + Point2d(dirx,diry), CV_RGB(0,   255, 0), 1);

      diry = (int)(sin(r1.get_angle_to_red_food()) * veclen);
      dirx = (int)(cos(r1.get_angle_to_red_food()) * veclen);
      line(image, pos, pos + Point2d(dirx, diry), CV_RGB(255, 0, 0), 1);      


      // 7. did the robot move over a food piece?
      bool food_found = false;
      for (uint i = 0; i < food_pieces.size(); i++)
      {
         double dist =
            norm((Point2d)(food_pieces[i]->get_position()) - r1.get_position());

         if (dist < r1.get_radius() + food_pieces[i]->get_radius())
         {
            // robot has moved over i-th food piece!

            // change reward for current step
            r1.update_current_reward( food_pieces[i]->get_reward() );

            if (food_pieces[i]->get_reward() > 0)
               green_counter++;
            else
               red_counter++;

            // delete food item from list of food pieces
            food_pieces.erase(food_pieces.begin() + i);
            
            food_found = true;

         } // if (stepped on a food piece)

      } // for (all food pieces)

      if (!food_found)
      {
         //r1.update_current_reward( REWARD_ENERGY_LOSS * (double) (1 + rand() % 5) );
         r1.update_current_reward(REWARD_ENERGY_LOSS);
      }

      
      // 8. keep track of the episode return      
      r1.set_episode_return(
         r1.get_episode_return() + r1.get_current_reward()
         );


      // 9. update state action associations      
      r1.update_state_action_associations();

      // stop for a keypress by user, if a reward was given
      if (r1.get_current_reward() != 0.0)
      {
         printf("Robot got reward of %f\n",
            r1.get_current_reward());
      }


      // 10. show simulation step nr
      char txt[100];
      sprintf(txt, "simulation step %d. Green: %d Red:%d",
         simulation_step,
         green_counter, red_counter);
      putText(image,
         txt,
         Point(20, 50),
         FONT_HERSHEY_SIMPLEX, 0.7, // font face and scale
         CV_RGB(255, 255, 255), // white
         1); // line thickness and type


      // 11. show world with robot, sensor rays and additional textual information
      imshow("Robot Simulator", image);


      // 12. save image?
      if (SAVE_IMAGES)
      {
         char fname[500];
         sprintf(fname, "%s\\img%05d.png", SAVE_FOLDER, simulation_step);
         imwrite(fname, image);
      }


      // 13. wait for a key
      char c;
      if (WAIT_FOR_KEYPRESS_AFTER_EACH_SIMU_STEP)
         c = (char)waitKey(0);
      else
         c = (char)waitKey(1); // waitKey() is always needed for imshow()!


      // 14. ESC pressed?
      if (c == 27)
         exit_simulation = true;


      // 15. one step simulated more
      simulation_step++;
      r1.set_age( simulation_step );


      // 16. add new food to world?
      if (rand()%15==0)
      {
         int rndfoodtype = rand() % 2;
         double reward;
         if (rndfoodtype == FOODTYPE_GREEN)
            reward = REWARD_GREEN_FOOD;
         else if (rndfoodtype == FOODTYPE_RED)
            reward = REWARD_RED_FOOD;

         Food* f = new Food(rndfoodtype,
            rand() % world.cols,
            rand() % world.rows,
            FOOD_RADIUS,
            reward);
         food_pieces.push_back(f);
      }


   } // while (simulation shall continue)

} // simulate_one_episode




int main()
{
   //srand((unsigned int)time(NULL));
   srand(42);

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
      return -1;
   }


   // 3. get world dimensions
   int WORLD_WIDTH  = world.cols;
   int WORLD_HEIGHT = world.rows;


   // 4. prepare an image (where we will draw the robot and the world)
   Mat image(WORLD_HEIGHT, WORLD_WIDTH, CV_8UC3);


   // 5. create a robot
   vector<double> sensor_angles, sensor_distances;
   sensor_angles.push_back(0);
   sensor_distances.push_back(100);
   Robot r1("R2D2",
            10,
            Point(WORLD_HEIGHT / 2, WORLD_WIDTH / 2));
           

   // 6. create food pieces with positive and negative rewards  
   int food_pieces_set = 0;
   for (int y=0; y<WORLD_WIDTH; y+=50)
   {
      for (int x = 0; x < WORLD_HEIGHT; x+=50)
      {
         int rndfoodtype = rand() % 2;
         double reward;
         if (rndfoodtype == FOODTYPE_GREEN)
            reward = REWARD_GREEN_FOOD;
         else if (rndfoodtype == FOODTYPE_RED)
            reward = REWARD_RED_FOOD;

         Food* f = new Food(rndfoodtype,
            x + (-10 + rand() % 21),
            y + (-10 + rand() % 21),
            FOOD_RADIUS,
            reward);
         food_pieces.push_back(f);

         food_pieces_set++;
         if (food_pieces_set == NR_FOOD_PIECES)
            break;
      }  
   }



   // 7. simulate robot till...
   //    ... user presses ESC
   exit_simulation = false;
   while (!exit_simulation)
   {
      simulate_one_episode(world, r1, sensor_angles, sensor_distances);
   }  


} // main