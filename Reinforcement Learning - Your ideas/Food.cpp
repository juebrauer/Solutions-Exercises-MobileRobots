#include "opencv2/opencv.hpp"

#include "Food.h"


Food::Food(int foodtype, int x, int y, int radius, double reward)
{
   this->foodtype = foodtype;
   this->x = x;
   this->y = y;
   this->radius = radius;
   this->reward = reward;
    
} // Food constructor



void Food::update(Mat world)
{

} // update



void Food::draw(Mat& img)
{
   Scalar col;
   if (reward < 0)
      col = Scalar(0, 0, 255);
   else
      col = Scalar(0, 255, 0);

   circle(img, Point(x, y), radius, col, -1);

} // draw



Point2i Food::get_position()
{
   return Point2i(x, y);
}


int Food::get_radius()
{
   return radius;
}


double Food::get_reward()
{
   return reward;
}


int Food::get_food_type()
{
   return foodtype;
}