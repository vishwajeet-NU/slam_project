#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects

/// \file
/// \brief Library for two-dimensional rigid body transformations.

class EKF
{ 
    public:
        ros::NodeHandle n;

        double odom_x = 0.0;
        double odom_y = 0.0;
        double odom_theta = 0.0;
        geometry_msgs::Quaternion quat;        
        double roll, pitch, yaw;
        bool odom_call = false;

        std::vector<float> x_center;
        std::vector<float> y_center; 

        bool land_called = false;

        geometry_msgs::PoseStamped current_pose_map;
        nav_msgs::Path path_taken_map;

        ros::Publisher path_pub_map;
        double co_var_w;
        double co_var_v;

        double sigma_r = 0 , sigma_theta = 0, sigma_landmark = 0;
        double r_param;
        double x_start = 0.0;
        double y_start= 0.0;
        double theta_start = 0.0; // change these later , need to accomodate different start positions?
        int m_land =12 ;      // number of landmarks 
        int n_land = m_land * 2;   // 2m for x and y location of landmark
        int total = n_land + 3;

        Eigen::MatrixXd mu_t_bar;
        Eigen::MatrixXd mu_t_posterior;
        Eigen::MatrixXd w_t;
        Eigen::MatrixXd v_t;
        Eigen::MatrixXd g_t;
        Eigen::MatrixXd q_t;
        Eigen::MatrixXd pose_var_prior;
        Eigen::MatrixXd pose_var_bar;
        Eigen::MatrixXd pose_var_posterior;
        Eigen::MatrixXd temp_covar;
        Eigen::VectorXd temp_temp;
  
        Eigen::MatrixXd z_t;
        Eigen::MatrixXd small_h;
        Eigen::MatrixXd large_h;
  
        Eigen::MatrixXd k_gain;
        Eigen::MatrixXd R;

        EKF();
        float wrap_angles(float incoming_angle);
        void ekf_predict(Twist2D &body_v);
        void ekf_update();
        void do_tf(const ros::TimerEvent&);



#endif