#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP
#include <math.h>
#include <vector>
#include <algorithm>    // std::sort
#include <numeric>
#include <eigen3/Eigen/Dense>
#include<random>
#include "rigid2d/rigid2d.hpp"

/// \file
/// \brief Library to support EKF slam, contains circle fitting, ekf measurement update and ekf prediction



/// \brief A class that houses all necessary methods and 
/// initializations to perform EKF slam, including landmark classification and circle fit

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
        

          double a = rigid2d::PI;
          std::vector<float> x_data;
          std::vector<float> y_data;
          std::vector<float> r_data;
          int j = 0;
          EKF();

          /// \brief initializes matrices needed for ekf 
          /// \param number of landmarks and noise parameters
          /// \returns none
   
          void initialize_matrices(int landmark_no, double sigma_r,double sigma_theta,double sigma_landmark, double r_param);

          /// \brief A method to determine whether a list of points is a circle or not 
          /// \param vectors of vectors containing x and y locations
          /// \returns none

          std::vector<std::vector<std::vector<float>>> circle_or_not_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in);

          /// \brief used to fit a circle to given points  
          /// \param vectors of vectors containing x and y locations
          /// \returns vector of selected values

          void circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in);
          std::mt19937 & get_random();

          /// \brief performs the prediction step of an EKF algorithm 
          /// \param twist containig the forward motion twist, noise parameter and state vector size
          /// \returns none

          void ekf_predict(rigid2d::Twist2D body_v, double co_var_w);

          /// \brief performs the measurement step of an EKF algorithm 
          /// \param noise parameters, sensor x and y readings, state vector size, distance thresholds for identifying new landmarks
          /// \returns none

          void ekf_update(double co_var_v, std::vector<float> x_center, std::vector<float> y_center, double threshold , double upper_threshold);


          /// \brief wraps angles between PI -PI
          ///
          /// \tparam inputs: takes angle to be wrapped in degrees
          /// \returns wrapped angle
          double wrap_angles(double incoming_angle);


          /// \brief decides which landmark the current measurement belongs to. Does data association using only 
          /// cartesian logic
          ///
          /// \tparam inputs: distance thresholds, and the current sensor reading for comparing with state vector
          /// \returns index of state vector which corresponds to the sensor reading
          int which_landmark(double threshold , double upper_threshold, float x_reading, float y_reading);

          /// \brief used to resize and redifine the matrices that need to be updated after a new landmark is added to
          /// the state
          ///
          /// \tparam current size of state vector, and sensor readings to be appended to vector
          /// \returns none
          
          void resize_matrices(int current_size,float x_reading, float y_reading);

};  

#endif