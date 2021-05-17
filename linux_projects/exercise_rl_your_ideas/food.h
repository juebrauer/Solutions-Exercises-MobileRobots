#pragma once

#include "opencv2/core.hpp"

using namespace std;
using namespace cv;

class Food
{
   public:

                     Food(int foodtype, int x, int y, int radius, double reward);

      void           update(Mat world);

      void           draw(Mat& img);

      Point2i        get_position();

      int            get_radius();

      double         get_reward();

      int            get_food_type();


   private:

      int            x, y, radius;

      double         reward;

      int            foodtype;



}; // class Food