/// \file
/// \brief This file reads laser scanner data, fits circles that matches landmarks, to this data
/// and then publishes them to a landmark topic
/// PARAMETERS:
/// threshold : distance threshold to classify points 
/// cut_off_points : value below which to ignore point clouds
/// radius_threshold : value beyond which to stop reporting circles 

/// PUBLISHES:
///     map_pub (nuslam::turtle_map): publishes calculated center and radius of landmark 
/// SUBSCRIBES:
///     laser_sub (sensor_msgs::LaserScan): reads data from the laser scanner
/// SERVICES:
///    none



#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include"rigid2d/rigid2d.hpp"
#include "tf/transform_broadcaster.h"
#include <vector>

#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/LaserScan.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>    // std::sort

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
std::string base_scan = "base_scan";

static double red = 50.0;
static double blue = 100.0;
static double green = 50.0;
static double alpha = 0.7;

/// \brief A function to fit point to a circle using 
/// https://nu-msr.github.io/navigation_site/circle_fit.html

/// \tparam inputs: a vector of vectors containing x and y coordinates 
/// \returns none

void fit_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{
  EKF cir;
  cir.circle(x_in, y_in);

  std::vector<float> x_holder;
  std::vector<float> y_holder;
  std::vector<float> r_holder;

  ros::NodeHandle n;
  map_pub= n.advertise<nuslam::turtle_map>("landmarks", 1);
  
  // tf::TransformListener listener(ros::Duration(10));

  for(unsigned int i =0; i<cir.x_data.size(); i ++)
  { 
  if(cir.r_data[i] < radius_threshold)
  {
    x_holder.push_back(cir.x_data[i]);
    y_holder.push_back(cir.y_data[i]);
    r_holder.push_back(cir.r_data[i]);
  }

}

container.radius = r_holder;
container.x_center = x_holder;
container.y_center = y_holder;
container.red = red;
container.blue = blue;
container.green = green;
container.alpha = alpha;
container.frame_name = base_scan;
map_pub.publish(container);
cir.x_data.clear();
cir.y_data.clear();
cir.r_data.clear();
x_holder.clear();
y_holder.clear();
r_holder.clear();

}

/// \brief callback for reading laser scan data
///
/// \tparam inputs: laser scan message
/// \returns none
void scanCallback(const sensor_msgs::LaserScan & scan_in)
{
  
    std::vector<float> readings;
    std::vector<std::vector<float>> all_x;
    std::vector<std::vector<float>> all_y;
    readings = scan_in.ranges;


   double current_val = readings[0];
   double angle_increment = scan_in.angle_increment;

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

    while(abs(current_val- readings[i])<=threshold)
    {
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);

       temp_x.push_back(x);
       temp_y.push_back(y);
       i = i + 1;
    }
      if(temp_x.size() ==0 )
      {
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);

       temp_x.push_back(x);
       temp_y.push_back(y);
      }
      current_val = readings[i];
      all_x.push_back(temp_x);
      all_y.push_back(temp_y);
      temp_x.clear();
      temp_y.clear();

       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);
       temp_x.push_back(x);
       temp_y.push_back(y);

      }


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
