#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h>   




   int main(int argc, char **argv)
   {

     ros::init(argc, argv, "traj_reset");
     ros::NodeHandle n;
     ros::ServiceClient client = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");

    turtlesim::TeleportAbsolute srv;
    srv.request.x =3;
    srv.request.y =2;
    srv.request.theta =0;
    client.call(srv);

   
     return 0;
   }