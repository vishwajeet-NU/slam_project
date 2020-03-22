/// \file
/// \brief finds odometry of the robot and publishes it 

/// Publishers : odom
/// Subscribers : jointstate

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include"rigid2d/telep.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>    // std::sort
#include <vector>
#include <turtlesim/Pose.h> 

#include <nuslam/turtle_map.h>
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Path.h> 
#include "nuslam/nuslam.hpp"

#include<random>

bool start = true;

namespace positions{
      float x_loc = 0.0;
      float y_loc = 0.0;
      float orient = 0.0;
}
namespace turtle_odom_pose{
    float x = 0.0;
    float y = 0.0;
    float th = 0.0;
}
double roll,pitch,yaw= 0.0;    
geometry_msgs::Quaternion quat; 

static double sigma_r = 0 , sigma_theta = 0, sigma_landmark = 0;
static double r_param;
static double incoming_left_wheel;
static double incoming_right_wheel;
static double left_wheel = 0;
static double right_wheel = 0;


static bool called = false;

static int m_land = 12;      // number of landmarks 
static int n_land = m_land * 2;   // 2m for x and y location of landmark
static int total = n_land + 3;

static double co_var_w;
static double co_var_v;
std::vector<float> x_center;
std::vector<float> y_center; 

bool land_called = false;
bool odom_call = false;
void landmarks(const nuslam::turtle_map &coordinates)
{
    x_center = coordinates.x_center;

    y_center = coordinates.y_center;
    land_called = true;
}

double wrap_angles(double incoming_angle)
{
    incoming_angle = atan2(sin(incoming_angle),cos(incoming_angle));

    return incoming_angle;
}

 void jt_callback(const sensor_msgs::JointState JT)
 {
       incoming_left_wheel= JT.position[0];
       incoming_right_wheel= JT.position[1];
 }

bool do_teleport(rigid2d::telep::Request  &req, rigid2d::telep::Response &res)
{     

     positions::x_loc = req.x;
     positions::y_loc = req.y;
     positions::orient = req.theta; 
     called = true;
     return true;
}


void pose_odom_Callback(const nav_msgs::Odometry &odom_pose)
    {
        turtle_odom_pose::x = odom_pose.pose.pose.position.x ;
        turtle_odom_pose::y = odom_pose.pose.pose.position.y;
        quat = odom_pose.pose.pose.orientation;

        double quatx= odom_pose.pose.pose.orientation.x;
        double quaty= odom_pose.pose.pose.orientation.y;
        double quatz= odom_pose.pose.pose.orientation.z;
        double quatw= odom_pose.pose.pose.orientation.w;
        tf::Quaternion q(quatx, quaty, quatz, quatw);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
    }


int main(int argc, char** argv)
{
    ros::init(argc, argv, "slam");
    ros::NodeHandle n;

    ros::param::get("sigma_r",sigma_r);
    ros::param::get("sigma_theta",sigma_theta);
    ros::param::get("sigma_landmark",sigma_landmark);
    ros::param::get("co_var_w",co_var_w);
    ros::param::get("co_var_v",co_var_v);
    ros::param::get("r_param",r_param);

     
    ros::ServiceServer service = n.advertiseService("/set_pose", do_teleport);
    Twist2D starting_position;
    starting_position.v_x =0.0;
    starting_position.v_y =0.0;
    starting_position.w =0.0;
    DiffDrive turtle_odo(starting_position,0.160,0.033);

    Twist2D Vb;
    ros::Publisher path_pub_map = n.advertise<nav_msgs::Path>("/path_map", 1);
    ros::Subscriber joint_state_subsciber = n.subscribe("/joint_states", 1, jt_callback);
    ros::Subscriber read_landmarks = n.subscribe("landmarks", 1, landmarks);
    ros::Subscriber odom_sub = n.subscribe("odom", 1,pose_odom_Callback);

    tf::TransformBroadcaster map_broadcaster;

    geometry_msgs::PoseStamped current_pose_map;
    nav_msgs::Path path_taken_map;

    EKF new_bot;
    new_bot.initialize_matrices(m_land, sigma_r, sigma_theta, sigma_landmark, r_param);
    ros::Rate r(100);
      while(n.ok())
      {
          ros::spinOnce();               // check for incoming messages
          if(called)
          {
              starting_position.v_x =positions::x_loc;
              starting_position.v_y =positions::y_loc;
              starting_position.w =positions::orient;    
              turtle_odo.reset(starting_position);
              called = false;
          }
          
          if(land_called)
          {
              if(start)
              {
                  for(int i =0; i<m_land; i++)
                  {
                      new_bot.mu_t_bar.row(2+2*(i+1)-1) << x_center[i];
                      new_bot.mu_t_bar.row(2+2*(i+1)) << y_center[i];
                  }
                  start = false;  
              }
              left_wheel = incoming_left_wheel- left_wheel;
              right_wheel = incoming_right_wheel - right_wheel;

              Twist2D body_v;
              WheelVelocities body_in; 
              body_in.U1 = left_wheel;
              body_in.U2 = right_wheel;
              body_v = turtle_odo.wheelsToTwist(body_in);
              left_wheel = incoming_left_wheel;
              right_wheel = incoming_right_wheel;

              new_bot.ekf_predict(body_v,co_var_w, total); 
              new_bot.ekf_update(m_land,co_var_v,x_center,y_center,total); 
           
              // std::cout<<"mu posteriro"<<mu_t_posterior<<"\n";
              new_bot.k_gain.fill(0.0);
              new_bot.large_h.fill(0.0);
              new_bot.pose_var_prior = new_bot.pose_var_posterior;
              
              geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(wrap_angles(new_bot.mu_t_bar.coeff(0,0)) - yaw);   
              geometry_msgs::TransformStamped map_trans;
              map_trans.header.stamp = ros::Time::now();
              map_trans.header.frame_id = "map";
              map_trans.child_frame_id = "odom";

              std::cout<<"coeff x = "<<new_bot.mu_t_bar.coeff(1,0) <<"\n";
              std::cout<<"coeff y = "<<new_bot.mu_t_bar.coeff(2,0) <<"\n";
              std::cout<<"x = "<<turtle_odom_pose::x<<"\n";
              std::cout<<"y = "<<turtle_odom_pose::y<<"\n";
        
              map_trans.transform.translation.x = new_bot.mu_t_bar.coeff(1,0) - turtle_odom_pose::x;
              map_trans.transform.translation.y = new_bot.mu_t_bar.coeff(2,0) - turtle_odom_pose::y;
              map_trans.transform.translation.z = 0.0;
              map_trans.transform.rotation = map_quat;
   
              map_broadcaster.sendTransform(map_trans);

             current_pose_map.header.stamp = ros::Time::now();
             current_pose_map.header.frame_id = "map";

             path_taken_map.header.stamp = ros::Time::now();
             path_taken_map.header.frame_id = "map";
             current_pose_map.pose.position.x = new_bot.mu_t_bar.coeff(1,0);
             current_pose_map.pose.position.y = new_bot.mu_t_bar.coeff(2,0);
             current_pose_map.pose.orientation = map_quat;
            
             path_taken_map.poses.push_back(current_pose_map);
             path_pub_map.publish(path_taken_map);


             land_called = false;

       }
        r.sleep();

    } 
}


