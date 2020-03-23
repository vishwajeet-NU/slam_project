#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
#include <math.h>
#include <vector>
#include <algorithm>    // std::sort
#include <numeric>
#include <eigen3/Eigen/Dense>
#include<random>
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include"rigid2d/telep.h"

/// \file
/// \brief Library for two-dimensional rigid body transformations.

class EKF
{ 
    public:
          
          double x_start = 0.0;
          double y_start= 0.0;
          double theta_start = 0.0; // change these later , need to accomodate different start positions?
          bool decision = false;
          Eigen::MatrixXd mu_t_bar;
          Eigen::MatrixXd mu_t_posterior;
          Eigen::MatrixXd w_t;
          Eigen::MatrixXd v_t;
          Eigen::MatrixXd g_t;
          Eigen::MatrixXd q_t;
      
          Eigen::MatrixXd pose_var_prior;
          Eigen::MatrixXd pose_var_bar;
          Eigen::MatrixXd pose_var_posterior;
          Eigen::MatrixXd z_t;
          Eigen::MatrixXd small_h;
          Eigen::MatrixXd large_h;
          Eigen::MatrixXd k_gain;
          Eigen::MatrixXd R;

          std::vector<float> x_data;
          std::vector<float> y_data;
          std::vector<float> r_data;
          int j = 0;
          EKF();
          void initialize_matrices(int landmark_no, double sigma_r,double sigma_theta,double sigma_landmark, double r_param);
          void circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in);
          std::mt19937 & get_random();
          void ekf_predict(rigid2d::Twist2D body_v, double co_var_w, int total);
          void ekf_update(int m_land, double co_var_v, std::vector<float> x_center, std::vector<float> y_center, int total, double threshold);
          double wrap_angles(double incoming_angle);
          int which_landmark(double threshold , float x_reading, float y_reading);
          void resize_matrices(int current_size,float x_reading, float y_reading);

};  

#endif