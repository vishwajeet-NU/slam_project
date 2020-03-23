/// \file
/// \brief This file makes a turtle travel in a pentagon, using a feedforward
/// strategy of control. It also prints the error between its expected position and 
/// the actual position, and also plots it
/// PARAMETERS:
/// x positions 
/// y positions 

/// PUBLISHES:
///     vel_pub (cmd_vel): publishes velocity commands to move the turtle 
///     err_pub (pose_err): this topic takes in error in actual and expected position of the turtle
/// SUBSCRIBES:
///     sub (pose): reads actual position data of the turtle
///     sub (odom): odometry readings from the /odom topic 
/// SERVICES:
///     client2 (SetPen): can be used to change color and transperency of turtle marker
///     client   (TeleportAbsolute) used to teleport the turtle to desired point and orientation
///     

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h> 
#include "tsim/PoseError.h"
#include <math.h>
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include "tf/transform_broadcaster.h"
#include <vector>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/LaserScan.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>    // std::sort
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

#include <nuslam/turtle_map.h>
#include <nuslam/test.h>
#include <numeric>
#include "std_msgs/Empty.h"

#include "nuslam/nuslam.hpp"
using namespace Eigen;

static double threshold;
static int cut_off;
static double radius_threshold;
ros::Publisher map_pub;
nuslam::turtle_map container;


void fit_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{
  EKF cir;
  cir.circle(x_in, y_in);

  std::vector<float> x_holder;
  std::vector<float> y_holder;
  std::vector<float> r_holder;

  ros::NodeHandle n;
  map_pub= n.advertise<nuslam::turtle_map>("landmarks", 1);
  
  tf::TransformListener listener(ros::Duration(10));

  for(unsigned int i =0; i<cir.x_data.size(); i ++)
  { 
  if(cir.r_data[i] < radius_threshold)
  {
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "base_scan";
    laser_point.header.stamp = ros::Time();
    laser_point.point.x = cir.x_data[i];
    laser_point.point.y = cir.y_data[i];
    laser_point.point.z = 0.0;
    try
    {
      geometry_msgs::PointStamped base_point;
      listener.transformPoint("odom", laser_point, base_point);
    }

    catch(tf::TransformException& ex)
    {
      ROS_ERROR("Received an exception trying to transform a point from \"base_scan\" to \"odom\": %s", ex.what()); 
    }
    x_holder.push_back(laser_point.point.x);
    y_holder.push_back(laser_point.point.y);
    r_holder.push_back(cir.r_data[i]);
  }

}
std::cout<<"holdersize "<< x_holder.size() << "\n";

container.radius = r_holder;
container.x_center = x_holder;
container.y_center = y_holder;
map_pub.publish(container);
cir.x_data.clear();
cir.y_data.clear();
cir.r_data.clear();
x_holder.clear();
y_holder.clear();
r_holder.clear();

}

void scanCallback(const sensor_msgs::LaserScan & scan_in)
{
  std::cout<<"inside main callback"<<"\n";
  
    std::vector<float> readings;
    std::vector<std::vector<float>> all_x;
    std::vector<std::vector<float>> all_y;
    readings = scan_in.ranges;


   double current_val = readings[0];
   double angle_increment = scan_in.angle_increment;
   double angle_hold = 0.0;
    //double threshold = 0.05; // change as a parameter later
    std::vector<float> temp_x;
    std::vector<float> temp_y;
    
    //handles for inf readings , for the test world
    for(unsigned int i=0; i<readings.size(); i++ )
    {
      if(readings[i] > 3)
      {
        readings[i] = 10;
      }
    }

    for(unsigned int i=0; i<readings.size(); i++ )
    {
//      std::cout<<"reading = "<<i<<" "<< readings[i]<<"\n";
    }

    for(unsigned int i=0; i<readings.size(); i++ )
    {   

      
    while(abs(current_val- readings[i])<=threshold)
    {
      
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);

       temp_x.push_back(x);
       temp_y.push_back(y);
       i = i + 1;
       angle_hold = angle_hold + angle_increment;
    }
      if(temp_x.size() ==0 )
      {
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);
       //std::cout<<"x from 2 "<<x<<"\n";
       //std::cout<<"y from 2 "<<y<<"\n";

       temp_x.push_back(x);
       temp_y.push_back(y);
        }
      current_val = readings[i];
      all_x.push_back(temp_x);
      all_y.push_back(temp_y);
      temp_x.clear();
      temp_y.clear();


       angle_hold = angle_hold + angle_increment;
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);
       temp_x.push_back(x);
       temp_y.push_back(y);

      }

//    std::cout<<"clusters formed prior"<<all_x.size()<<"\n";

    for(unsigned int jj=0; jj<all_x.size(); jj++)
    {
      int sz = all_x[jj].size();
      if(sz<cut_off)
      {

        all_x.erase(all_x.begin() + jj);
        all_y.erase(all_y.begin() + jj);
        jj=-1;
      }

    }


  //  std::cout<<"clusters formed post "<<all_x.size()<<"\n";
    for( unsigned int boredom = 0; boredom<all_x.size(); boredom++)
    {
               
        for(unsigned int shit =0; shit<all_x[boredom].size(); shit++)
        {
        // std::cout<<"current clsuter"<<boredom<<" ";
        // std::cout<<"range = "<< sqrt(all_x[boredom][shit] * all_x[boredom][shit] + all_y[boredom][shit] * all_y[boredom][shit])<<"\n";
    //    std::cout<<"x_post sort =" <<all_x[boredom][shit]<<"  ";
    //    std::cout<<"y post sort ="<<all_y[boredom][shit] <<"\n";
          
        }
      //  std::cout<<"\n";
    }
   fit_circle(all_x,all_y);
   // circle_or_not_circle(all_x,all_y);
    all_x.clear();
    all_y.clear();

}   


int main(int argc, char **argv)
{
    ros::init(argc,argv,"landmarks");
    ros::NodeHandle n;
    ros::param::get("/threshold",threshold);
    ros::param::get("/cut_off_points",cut_off);
    ros::param::get("/radius_threshold",radius_threshold);
    ros::Subscriber laser_sub = n.subscribe("scan", 1, scanCallback);
        
    ros::Rate rate(5);
 
    while (ros::ok())
    {
    ros::spinOnce();

    rate.sleep();
    }   
}

/*
void circle_or_not_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{


  float first_x,last_x,first_y,last_y;
  std::vector<std::vector<float>> selected_x;
  std::vector<std::vector<float>> selected_y;
  
 
  for (int k =0;k<x_in.size(); k++)
  {
    std::vector<float> angle_list;
    std::vector<float> current_x_cluster = x_in[k];
    std::vector<float> current_y_cluster = y_in[k];
    
//    std::cout<<"size = "<<current_y_cluster.size()<<"\n";
    first_x = *(current_x_cluster.begin());
    last_x  = *(current_x_cluster.end()-1);

    first_y = *(current_y_cluster.begin());
    last_y  = *(current_y_cluster.end()-1);

    for(int l = 1;  l< (current_x_cluster.size()-1); l++ )
    {

      std::cout<<"k = "<< k<<"\n";
      std::cout<<"l = "<< l<<"\n";

      std::cout<< " current x "<<current_x_cluster[l]<<" "<< "current y "<<current_y_cluster[l]<<" "<<"\n";
      std::cout<<"first x "<< first_x<<" "<< "first_y"<< first_y <<"\n";

      std::cout<<"last x "<< last_x<<" "<< "last y"<< last_y <<"\n";

      float y_delta_first =  (current_y_cluster[l] - first_y);
      float x_delta_first = (current_x_cluster[l] - first_x);

      float y_delta_last = (current_y_cluster[l] - last_y);
      float x_delta_last = (current_x_cluster[l] - last_x);
      
      std::cout<<"y_delta_first "<<y_delta_first<<"\n";
      std::cout<<"x_delta_first "<<x_delta_first<<"\n";
      

      std::cout<<"y_delta_last "<<y_delta_last<<"\n";
      std::cout<<"x_delta_last "<<x_delta_last<<"\n";

      float first_ang = atan2(x_delta_first,y_delta_first);
      float last_ang = atan2(x_delta_last,y_delta_last);
      std::cout<<"first ang" <<first_ang<<"\n";
      std::cout<<"last ang" << last_ang <<"\n";

      angle_list.push_back(first_ang-last_ang);      
      std::cout<<"angle = " <<angle_list[l-1]<<"\n";
      
    }

    double sum = std::accumulate(std::begin(angle_list), std::end(angle_list), 0.0);
    double m =  sum / angle_list.size();
    double accum = 0.0;
    std::for_each (std::begin(angle_list), std::end(angle_list), [&](const double d) 
    {
    accum += (d - m) * (d - m);
    });
    double stdev = sqrt(accum / (angle_list.size()));

    if(m<0.0)
    {
      m = m + 2.0*rigid2d::PI;
    }
//    std::cout<<"m out ="<<m<<"\n";

    if((m>=(rigid2d::PI)/2.0) && (m<= (3.0* rigid2d::PI)/4.0)  && stdev <0.15)
    {
      std::cout<<"found a circle \n";
      std::cout<< "itr np = "<< k<<"\n";
      std::cout<<"m = "<<m<<"\n";
      std::cout<<"stddev = "<<stdev<<"\n";

      selected_x.push_back(x_in[k]);
      selected_y.push_back(y_in[k]);
    }

    angle_list.clear();
  }


}
*/