#pragma once

#include "opencv2/core.hpp"

using namespace std;
using namespace cv;


class Robot
{
   public:

      enum actions { undefined, left, right, forward, NR_ACTIONS};

      struct demo_datum
      {
         vector<double>  sensor_values;
         enum actions    action;
      };


  private:

    string                 name;
    double                 radius;
    Point2d                pos;
    double                 orientation;
    vector<double>         sensor_angles;
    vector<double>         sensor_distances;
    int                    nr_sensors;
    vector<double>         sensor_values;
    bool                   lfd_mode;
    vector<demo_datum*>    demonstration_data;
    int                    world_width;
    int                    world_height;


    

  public:

                    Robot( string             name,
                           int                radius,
                           Point2d            start_pos,
                           double             start_orientation,
                           vector<double>     sensor_angles,
                           vector<double>     sensor_distances );

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

    void            set_lfd_mode(bool b);

    int             get_size_of_demo_dataset();

    void            save_demonstration_data(string fname);

    bool            load_demonstration_data(string fname);

    enum actions    k_nn_classifier(double* avg_distance, int k=3);




    

}; // class Robot
