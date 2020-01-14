#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h>   


/// \file
/// \brief This file creates a service that resets the turtle to the parameter defined
/// start position 
/// SERVICES:
///     client   (TeleportAbsolute) used to teleport the turtle to desired point and orientation


   int main(int argc, char **argv)
   {
     int abs_x,abs_y;
     ros::init(argc, argv, "traj_reset");
     ros::NodeHandle n;
     ros::service::waitForService("spawn");


     n.getParam("abs_x",abs_x);
     n.getParam("abs_y",abs_y);

    ros::ServiceClient client2 = n.serviceClient<turtlesim::SetPen>("turtle1/set_pen");
    turtlesim::SetPen srv2;
    srv2.request.r =0;
    srv2.request.g =0;
    srv2.request.b =0;
    srv2.request.width =1;
    srv2.request.off =1;
    client2.call(srv2);
    ros::ServiceClient client = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
    turtlesim::TeleportAbsolute srv;
    srv.request.x =abs_x;
    srv.request.y =abs_y;
    srv.request.theta =0;
    client.call(srv);
    srv2.request.off =0;
    client2.call(srv2);

   
     return 0;
   }