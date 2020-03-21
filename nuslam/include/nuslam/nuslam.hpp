#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
#include <math.h>
#include <vector>
#include <algorithm>    // std::sort
#include <numeric>
#include <eigen3/Eigen/Dense>



/// \file
/// \brief Library for two-dimensional rigid body transformations.

class EKF
{ 
    public:
          std::vector<float> x_data;
          std::vector<float> y_data;
          std::vector<float> r_data;
          int j = 0;
          EKF();
          void circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in);
};  

#endif