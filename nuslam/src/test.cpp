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
#include <iostream>
#include "nav_msgs/Odometry.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h> 
#include "tsim/PoseError.h"
#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include "tf/transform_broadcaster.h"
#include <vector>
#include "sensor_msgs/LaserScan.h"


int main(int argc, char **argv)
{
     ros::init(argc,argv,"whatnode");

    ros::NodeHandle n;
    sensor_msgs::LaserScan what;
    //what.ranges = {0.113851,0.112796,0.111762,0.110782,0.109854,0.108977,0.108149,0.107368,0.106633,0.105942,0.105294,0.07388,0.071378,0.069749,0.068557,0.06765,0.066954,0.066429,0.066049,0.065799,0.065669,0.065655,0.065758,0.065979,0.066327,0.066817,0.067469,0.068322,0.069437,0.070941,0.073157};
    what.ranges = {0.1,0.100015,0.100063,0.100142,0.100254,0.100398,0.100575,0.100785,0.101028,0.101305,0.101616,0.101961,0.102342,0.102759,0.080445,0.077828,0.076145,0.074926,0.074012,0.073326,0.072825,0.072484,0.072289,0.072231,0.072308};
    what.angle_increment = 0.01750192232;
//    what.ranges = {};
    ros::Publisher _pub = n.advertise<sensor_msgs::LaserScan>("/scan", 1);

    ros::Rate rate(60);

    while (ros::ok())
    {

    _pub.publish(what);
//    rate.sleep();
    }   

}

