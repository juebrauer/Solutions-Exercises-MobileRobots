/// Exercise: Novelty Detection
///
/// Novelty Detection means the ability for a robot
/// to detect new situations using its sensor information.
///
/// Here your task is to code a novelty detector module
/// for a simple mobile robot.
///
/// ---
/// by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#define SAVE_IMAGES 0
#define SAVE_FOLDER "~/tmp"

#include <iostream>
#include <math.h>

#include <opencv2/opencv.hpp>
#include "Robot.h"



using namespace cv;
using namespace std;


int main()
{
  // 1. load image of world: change this to the folder where you store world1.png!
  //    white pixels mean: no drivable space
  //    blavk pixels mean: drivable space
  string img_filename = "world1.png";
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
  sensor_angles.push_back(-M_PI / 4);
  sensor_angles.push_back(+M_PI / 4);
  sensor_angles.push_back(-M_PI / 2);
  sensor_angles.push_back(+M_PI / 2);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  Robot r1("R2D2", 10, Point(WORLD_HEIGHT/2, WORLD_WIDTH/2), M_PI/4, sensor_angles, sensor_distances);


  // prepare a data structure for the novelty map
  // 0 = not new
  // 1 = new
  // 2 = unknown
  int** novelty_map = new int*[WORLD_HEIGHT];
  for (int y = 0; y < WORLD_HEIGHT; y++)
  {
     novelty_map[y] = new int[WORLD_WIDTH];
     for (int x = 0; x < WORLD_WIDTH; x++)
     {
         novelty_map[y][x] = 2;
     }
  }
    

  // 6. simulation loop
  bool exit = false;
  int simulation_step = 0;
  while (!exit)
  {
    // 6.1 initialize image with world image
    world.copyTo(image);


    // 6.2 move the robot according to its specific behavior
    r1.update(world);


    // what does the robot think? is this a new situation?
    bool is_new = r1.run_novelty_detection();
    Point2d pos = r1.get_position();
    novelty_map[(int)pos.y][(int)pos.x] = is_new;

    // draw novelty information into the world image
    for (int y = 0; y < WORLD_HEIGHT; y++)
    {
       for (int x = 0; x < WORLD_WIDTH; x++)
       {
          // no novelty information regarding this location
          if (novelty_map[y][x] == 2)
            continue;

          if (novelty_map[y][x] == 1)
             image.at<Vec3b>(y,x) = Vec3b(0,255,0); // render new situations in green
          else
             image.at<Vec3b>(y, x) = Vec3b(0,0,255); // render not new situations in red
       }
    }


    // 6.3 show the robot's position as a circle and its orientation by a line    
    double  ori   = r1.get_orientation();
    double  dx    = cos(ori);
    double  dy    = sin(ori);
    double  r     = r1.get_radius();
    circle(image, pos, (int)r, CV_RGB(255, 0, 0), 2);
    line(image, pos, pos + Point2d(dx*r, dy*r), CV_RGB(0, 255, 0), 1);
    
        
    // 6.4 compute all the sensor values
    vector<double> sensor_values = r1.get_sensor_values();


    // 6.5 for each sensor draw a sensor ray
    for (int sensor_nr = 0; sensor_nr < r1.get_nr_sensors(); sensor_nr++)
    {
      double sensor_value = sensor_values[sensor_nr];

      // get (x,y) coords of current robot position
      double x = pos.x;
      double y = pos.y;

      // get sensor orientation relative to robots orientation
      double sensor_angle = sensor_angles[sensor_nr];

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


    // 6.6 show simulation step nr    
    char txt[100];
    sprintf(txt, "simu %d / %s / %d", simulation_step, is_new ? "new" : "not new", r1.get_emotion_bored());
    putText(image,
      txt,
      Point(20, 50),
      FONT_HERSHEY_SIMPLEX, 0.7, // font face and scale
      CV_RGB(255,255,255), // white
      1); // line thickness and type


    // 6.7 show world with robot, sensor rays and additional textual information
    imshow("Robot Simulator", image);


    // 6.8 save image?
    if (SAVE_IMAGES)
    {
      char fname[500];
      sprintf(fname, "%s\\img%05d.png", SAVE_FOLDER, simulation_step);
      imwrite(fname, image);
    }


    // 6.9 wait for a key
    char c = (char)waitKey(10);

    
    // 6.10 ESC pressed?
    if (c == 27)
      exit = true;


    // 6.11 one step simulated more
    simulation_step++;

  } // while (simulation shall continue)

} // main