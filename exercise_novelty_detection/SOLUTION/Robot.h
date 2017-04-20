#pragma once

#include "opencv2/core.hpp"

using namespace std;
using namespace cv;

class Robot
{
  private:

    string          name;
    double          radius;
    Point2d         pos;
    double          orientation;
    vector<double>  sensor_angles;
    vector<double>  sensor_distances;
    int             nr_sensors;
    vector<double>  sensor_values;
    int             emotion_bored;

    bool            robot_memory[21][21][21][21];


    

  public:

                    Robot::Robot(string             name,
                                 int                radius,
                                 Point2d            start_pos,
                                 double             start_orientation,
                                 vector<double>     sensor_angles,
                                 vector<double>     sensor_distances);

    void            compute_sensor_values(Mat world);

    void            update(Mat world);

    Point2d         get_position();

    double          get_orientation();

    double          get_radius();

    int             get_nr_sensors();

    vector<double>  get_sensor_values();

    vector<double>  get_sensor_angles();

    vector<double>  get_sensor_distances();

    void            move(double pixel);

    void            turn(double angle);

    bool            run_novelty_detection();

    int             get_emotion_bored();

    

}; // class Robot
