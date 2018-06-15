/// Exercise: Imitation Learning
///
/// In this task, the user helps to show the robot what
/// are good actions in certain states.
///
/// The robot will then use the demonstration examples
/// in order to "learn" a good policy, i.e.,
/// state to action mapping.
///
/// ---
/// by Prof. Dr. Jürgen Brauer, www.juergenbrauer.org

#define _CRT_SECURE_NO_WARNINGS
#define _USE_MATH_DEFINES

#define SAVE_IMAGES 0
#define SAVE_FOLDER "V:\\tmp"

#include <iostream>
#include <conio.h>
#include <math.h>

#include "opencv2/opencv.hpp"
#include "Robot.h"
#include "params.h"



using namespace cv;
using namespace std;


int main()
{
  // 1. load image of world: change this to the folder where you store world1.png!
  //    white pixels mean: no drivable space
  //    blavk pixels mean: drivable space
  string img_filename = WORLD_FILENAME;
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
  vector<double> sensor_angles, sensor_distances;
  sensor_angles.push_back(-M_PI / 4);
  sensor_angles.push_back(+M_PI / 4);
  sensor_angles.push_back(-M_PI / 2);
  sensor_angles.push_back(+M_PI / 2);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  sensor_distances.push_back(200);
  Point startpos = Point(50, 50);
  if (RANDOM_START_POS)
     startpos += Point(-30 + rand() % 61, -30 + rand() % 61);
  Robot r1("R2D2", 10, startpos, M_PI/2, sensor_angles, sensor_distances);


  // start simulation loop with
  // Learning from Demonstrations (LfD) mode on
  r1.set_lfd_mode(true);


  // 6. simulation loop
  bool exit = false;
  int simulation_step = 0;
  while (!exit)
  {
    // 6.1 initialize image with world image
    world.copyTo(image);


    // 6.2 move the robot according to its specific behavior
    r1.update(world);
    


    // 6.3 show the robot's position as a circle and its orientation by a line    
    Point2d pos   = r1.get_position();
    double  ori   = r1.get_orientation();
    double  dx    = cos(ori);
    double  dy    = sin(ori);
    double  r     = r1.get_radius();
    circle(image, pos, (int)r, CV_RGB(255, 0, 0), 1);
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
    sprintf_s(txt, "simu %d, demo-data: %d", simulation_step, r1.get_size_of_demo_dataset());
    putText(image,
      txt,
      Point(20, 50),
      FONT_HERSHEY_SIMPLEX, 0.7, // font face and scale
      CV_RGB(255,255,0), // white
      1); // line thickness and type


    // 6.7 show world with robot, sensor rays and additional textual information
    imshow("Robot Simulator", image);


    // 6.8 save image?
    if (SAVE_IMAGES)
    {
      char fname[500];
      sprintf_s(fname, "%s\\img%05d.png", SAVE_FOLDER, simulation_step);
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