/// \file
/// \brief This file takes gazebo state and publishes a landmark data that 
/// resembles that of landmark node. It run in debug mode to verify EKF slam
/// PARAMETERS:
/// detection radius: landmarks beyond this radius are ignored  
/// noise_variance: this is used to add gaussian noise to the data 

/// PUBLISHES:
///     map_pub (nuslam::turtle_map): publishes location of landmark wrt to the robot, in x y along with radius 
///     
/// SUBSCRIBES:
///     landmark_gt (gazebo::model_states): reads actual position data of the bot as reported by gazebo
///     sub (odom): odometry readings from the /odom topic 
/// SERVICES:
///     client (get_world_properties): used to read model names reported by gazebo
///     
#include "ros/ros.h"
#include "std_msgs/String.h"
#include<std_srvs/Empty.h>
#include <math.h>
#include"rigid2d/rigid2d.hpp"
#include "tf/transform_broadcaster.h"
#include <vector>
#include <string>

#include "gazebo_msgs/GetWorldProperties.h"
#include "gazebo_msgs/ModelStates.h"


#include <nuslam/turtle_map.h>

#include <numeric>
#include<random>

#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Path.h> 


 std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }

double detection_radius = 0.0;
double nvar = 0.0; 
static std::vector<float> x_coordinates;
static std::vector<float> y_coordinates;
static std::vector<float> radii;

static double red = 0.0;
static double blue = 100.0;
static double green = 255.0;
static double alpha = 0.8;

static std::vector<std::string> names;

std::string base_scan = "base_scan";

ros::Publisher map_pub;
nuslam::turtle_map container;

std::string cylinder = "cylinder";
std::string robot = "diff_drive";

double wrap_angles(float incoming_angle)
{
    incoming_angle = atan2(sin(incoming_angle),cos(incoming_angle));
    
    return incoming_angle;

}
void gazebo_callback(const gazebo_msgs::ModelStates &in_var)
{
    
    std::normal_distribution<double> d(0.0, nvar);
    ros::NodeHandle n;
    map_pub= n.advertise<nuslam::turtle_map>("landmarks", 1);
   
    std::vector<int> index_pos;
    unsigned int length = in_var.name.size();
    int robot_index;

   for(unsigned int i = 0; i< length; i++ )
   {

     std::string name = in_var.name[i];
     std::string temp = name.std::string::substr(0,8);
    
    if(temp.compare(cylinder) == 0)
    {
        index_pos.push_back(i);
    }
   if(name.compare(robot) == 0)
   {
       robot_index = i;
   }
   }
    double roll,pitch,yaw;
    tf::Quaternion quat(in_var.pose[robot_index].orientation.x,\
                           in_var.pose[robot_index].orientation.y,\
                           in_var.pose[robot_index].orientation.z,\
                           in_var.pose[robot_index].orientation.w);

    tf::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw);
  
  for(unsigned int i = 0; i< index_pos.size();i++)
  {
       rigid2d::Vector2D bot;
       bot.x = in_var.pose[robot_index].position.x;
       bot.y = in_var.pose[robot_index].position.y;

       rigid2d::Vector2D land;
       land.x = in_var.pose[index_pos[i]].position.x;
       land.y = in_var.pose[index_pos[i]].position.y;

       double x_delta = in_var.pose[index_pos[i]].position.x - in_var.pose[robot_index].position.x;
       double y_delta = in_var.pose[index_pos[i]].position.y - in_var.pose[robot_index].position.y;
       double range = land.distance(bot,land);
      
       double bearing =  wrap_angles(atan2(y_delta,x_delta)) - wrap_angles(yaw) ;       

       std::cout<<"atan = "<< atan2(y_delta,x_delta)<<"\n";
       std::cout<<"yaw = "<<yaw<<"\n";
       std::cout<<"bearing ="<<bearing<<"\n";
       if(abs(range)<detection_radius)
       {
       std::cout<<"x global = "<<in_var.pose[index_pos[i]].position.x<<"\t";
       std::cout<<"y global = "<<in_var.pose[index_pos[i]].position.y<<"\n"; 

            std::cout<<"range = "<<range<<"\n";
            x_coordinates.push_back(range * cos(bearing) + d(get_random()));
            y_coordinates.push_back(range * sin(bearing) + d(get_random()));
            radii.push_back(0.035);

       std::cout<<"x from range = "<<range * cos(bearing)<<"\t";
       std::cout<<"y as range = "<<range * sin(bearing)<<"\n"; 


       }
  
  }

    container.radius = radii;
    container.x_center = x_coordinates;
    container.y_center = y_coordinates;
    container.red = red;
    container.blue = blue;
    container.green = green;
    container.alpha = alpha;
    container.frame_name = base_scan; 
    map_pub.publish(container);

    x_coordinates.clear();
    y_coordinates.clear();
    radii.clear();

}
int main(int argc, char **argv)
{
    ros::init(argc,argv,"landmarks");
    ros::NodeHandle n;

    ros::Subscriber landmark_gt = n.subscribe("gazebo/model_states", 1, gazebo_callback);
    ros::Rate rate(5);
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("gazebo/get_world_properties");
    gazebo_msgs::GetWorldProperties srv;
//    gazebo_msgs::ModelStates 

     ros::param::get("detection_radius",detection_radius);
     ros::param::get("noise_variance", nvar);

    client.call(srv);
    for(unsigned int i = 0; i< srv.response.model_names.size(); i++ )
    {
        names.push_back(srv.response.model_names[i]);
    }

    while (ros::ok())
    {
    ros::spinOnce();
    rate.sleep();
    }   
}


             