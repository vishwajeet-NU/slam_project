/// \file
/// \brief Used to plot robots path as reported by gazebo, aka - ground truth
/// PARAMETERS:
/// none

/// PUBLISHES:
///     path_pub_gt (nav_msgs::Path): publishes path for plotting in rviz
/// SUBSCRIBES:
///     landmark_gt (gazebo_msgs::ModelStates): reads robots pose reported by gazebo
/// SERVICES:
///     client   (gazebo_msgs::GetWorldProperties) reads model names reported by gazebo

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
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


static std::vector<std::string> names;

geometry_msgs::PoseStamped ground_truth;
nav_msgs::Path ground_path;

ros::Publisher path_pub_gt;

std::string robot = "diff_drive";


/// \brief callback for gazebo states 
///
/// \tparam inputs: model states 
/// \returns none
void gazebo_callback(const gazebo_msgs::ModelStates &in_var)
{
    ros::NodeHandle n;
    path_pub_gt = n.advertise<nav_msgs::Path>("/path_ground", 1);
   
    std::vector<int> index_pos;
    unsigned int length = in_var.name.size();
    int robot_index;

   for(unsigned int i = 0; i< length; i++ )
   {
   
     std::string name = in_var.name[i];
     std::string temp = name.std::string::substr(0,8);
    // std::cout<<"temp =  = "<< temp<<"\n";
    
   if(name.compare(robot) == 0)
   {
       robot_index = i;
   }
   }
   std::cout<<"robot idnex = "<<robot_index<<"\n";
   std::cout<<"x pos"<<in_var.pose[robot_index].position.x<<"\n";

    ground_truth.header.stamp = ros::Time::now();
    ground_truth.header.frame_id = "map";
    ground_path.header.stamp = ros::Time::now();
    ground_path.header.frame_id = "odom";
    ground_truth.pose.position.x =  in_var.pose[robot_index].position.x;
    ground_truth.pose.position.y =  in_var.pose[robot_index].position.y;
    ground_truth.pose.orientation= in_var.pose[robot_index].orientation;           
    ground_path.poses.push_back(ground_truth);
    path_pub_gt.publish(ground_path);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"landmarks");
    ros::NodeHandle n;

    ros::Subscriber landmark_gt = n.subscribe("gazebo/model_states", 1, gazebo_callback);
    ros::Rate rate(5);
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetWorldProperties>("gazebo/get_world_properties");
    gazebo_msgs::GetWorldProperties srv;

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


             