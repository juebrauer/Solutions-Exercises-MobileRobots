#pragma once

#include <vector>

template<typename T>
T L1_norm(std::vector<T> v1, std::vector<T> v2)
{   
   T dist = 0.0;
   for (int i = 0; i < v1.size(); i++)
   {
      dist += abs(v1[i] - v2[i]);
   }
   return dist;
}
