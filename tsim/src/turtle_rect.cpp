#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>


geometry_msgs::Twist msg;


int main(int argc, char **argv)
{

ros::init(argc, argv, "turtle_rect");
ros::NodeHandle n;

ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);

int local_a;

ros::Rate rate(1);
n.getParam("param2",local_a);

ros::service::waitForService("spawn");

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
srv.request.x =5;
srv.request.y =5;
srv.request.theta =0;
client.call(srv);

srv2.request.off =0;
client2.call(srv2);

while (ros::ok())
{
ros::service::waitForService("spawn");

/*

distance = speed * time 
speed = known. 
start position = known. 



*/


ros::Time start = ros::Time::now();

while(ros::Time::now() - start < ros::Duration(2.0)){

msg.linear.x = 1;
msg.angular.z = 0;
vel_pub.publish(msg);

}

ros::Time start2 = ros::Time::now();

while(ros::Time::now() - start2 < ros::Duration(5.0)){

msg.linear.x = 0;
msg.angular.z = 0.314;
vel_pub.publish(msg);

}

ros::Time start3 = ros::Time::now();

while(ros::Time::now() - start3 < ros::Duration(5.0)){

msg.linear.x = 1;
msg.angular.z = 0;
vel_pub.publish(msg);

}
ros::Time start4 = ros::Time::now();

while(ros::Time::now() - start4 < ros::Duration(5.0)){

msg.linear.x = 0;
msg.angular.z = 0.314;
vel_pub.publish(msg);

}



ros::spinOnce();
rate.sleep();
}

/*
ros::Rate loop_rate(10);
int count = 0;
while (ros::ok())
{
std_msgs::String msg;
std::stringstream ss;
ss << "hello world " << count;
msg.data = ss.str();
ROS_INFO("%s", msg.data.c_str());

++count;
}
*/

}    