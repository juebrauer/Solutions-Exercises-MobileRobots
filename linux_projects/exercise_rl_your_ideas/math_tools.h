#pragma once

#include <stdlib.h>

// compute a random double value in the interval [min,max]
double get_rand_val_from_interval(double min, double max)
{
   // compute random int in [0,100]
   int rndval = rand() % 101; // rndval is in [0,100]

                              // convert to double value in [0,1]
   double rndvald = (double)rndval / 100.0;

   // map to position in interval [min,max]
   double rndintervalpos = min + rndvald * (max - min);

   return rndintervalpos;

} // get_rand_val_from_interval



double rad_to_deg(double rad)
{
   return (rad / M_PI)*180.0;
}