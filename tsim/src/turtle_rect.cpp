#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h> 
#include "tsim/PoseError.h"

/// \file
/// \brief This file makes a turtle travel in a rectangle, using a feedforward
/// strategy of control. It also prints the error between its expected position and 
/// the actual position, and also plots it
/// PARAMETERS:
///     width (int): width of the rectangle 
///     height (int): height of the rectangle 
///     trans_vel(int) : linear vel of turtle
///     rot_vel(int): angular vel of turtle
///     abs_x(int) : starting position of turtle in x
///     abs_y(int) : starting position of turtle in y
///     frequency(int): Update rate of velocity signal
/// PUBLISHES:
///     vel_pub (cmd_vel): this topic takes in velocity commands to move the turtle
///     err_pub (pose_err): this topic takes in error in actual and expected position of the turtle
/// SUBSCRIBES:
///     sub (pose): reads actual position data of the turtle
/// SERVICES:
///     client2 (SetPen): can be used to change color and transperency of turtle marker
///     client   (TeleportAbsolute) used to teleport the turtle to desired point and orientation
///     
geometry_msgs::Twist msg;
tsim::PoseError er;
turtlesim::Pose pose;

bool up= true;
bool right = true;
namespace turtle_pose{
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
}
/// \brief is called at every ros::spin. passes all information of that particular
/// topic
/// \returns turtle_pose

void poseCallback(const turtlesim::Pose &pose){
   turtle_pose::x = pose.x;
   turtle_pose::y = pose.y;
   turtle_pose::theta = pose.theta;
}

/// \brief Makes the turtle move horizontally, while also publishing
/// error to the pose_error topic
/// \params the various parameters that define the rectangle dimensions 
/// along with its motion// height, width, linear velocity, rotational velocity
/// x,y position and error buffers. A marker is used to decide whether its moving 
/// right or left 
/// \returns void
/// <template typename T>
/// horizontal(h,w,v,r,x,y,pub,sub,x_err,y_err,the_err,marker)
void horizontal(int height,int width,int trans_vel,int rot_vel,int abs_x,int abs_y, 
ros::Publisher vel_pub, ros::Publisher err_pub, float x_error, float y_error,float theta_error, int marker ){

ros::NodeHandle n;

ros::Time start = ros::Time::now();
while(ros::Time::now() - start < ros::Duration(2.0)){
msg.linear.x = trans_vel;
msg.angular.z = 0;
vel_pub.publish(msg);

ros::Duration inner_time1 = ros::Time::now() - start;
ros::spinOnce();

float secs;
secs = inner_time1.toSec(); 
if(marker == 0){
x_error = turtle_pose::x - (secs*trans_vel + abs_x);
y_error = turtle_pose::y - abs_y;
theta_error = turtle_pose::theta - 0;
}
else{
x_error = turtle_pose::x - (abs_x + width - secs*trans_vel );
y_error = turtle_pose::y - (abs_y+height);
theta_error = turtle_pose::theta - 3.14159;
}
er.x_error = x_error;
er.y_error = y_error;
er.theta_error = theta_error;
err_pub.publish(er);

}
ros::Time start2 = ros::Time::now();

while(ros::Time::now() - start2 < ros::Duration(1.57079)){

msg.linear.x = 0;
msg.angular.z = rot_vel;
vel_pub.publish(msg);

}

}


/// \brief Makes the turtle move vertically, while also publishing
/// error to the pose_error topic
/// \params the various parameters that define the rectangle dimensions 
/// along with its motion// height, width, linear velocity, rotational velocity
/// x,y position and error buffers. A marker is used to decide whether its moving 
/// right or left 
/// \returns void
/// <template typename T>
/// vertical(h,w,v,r,x,y,pub,sub,x_err,y_err,the_err,marker)
void vertical(int height,int width,int trans_vel,int rot_vel,int abs_x,int abs_y, 
ros::Publisher vel_pub, ros::Publisher err_pub, float x_error, float y_error,float theta_error, int marker ){

ros::NodeHandle n;

ros::Time start = ros::Time::now();
while(ros::Time::now() - start < ros::Duration(2.5)){
msg.linear.x = trans_vel;
msg.angular.z = 0;
vel_pub.publish(msg);

ros::Duration inner_time1 = ros::Time::now() - start;
ros::spinOnce();

float secs;
secs = inner_time1.toSec(); 
if(marker == 0){
x_error = turtle_pose::x - (abs_x+width) ;
y_error = turtle_pose::y - (secs*trans_vel + abs_y);
theta_error = turtle_pose::theta -1.57079 ;

}
else{
x_error = turtle_pose::x - (abs_x) ;
y_error = turtle_pose::y - (abs_y+ height - secs*trans_vel );
theta_error = turtle_pose::theta +1.57079 ;

}
er.x_error = x_error;
er.y_error = y_error;
er.theta_error = theta_error;
err_pub.publish(er);

}
ros::Time start2 = ros::Time::now();

while(ros::Time::now() - start2 < ros::Duration(1.57079)){

msg.linear.x = 0;
msg.angular.z = rot_vel;
vel_pub.publish(msg);

}

}

int main(int argc, char **argv)
{
int width, height, trans_vel, rot_vel, abs_x , abs_y,frequency;
float x_error, y_error, theta_error;


ros::init(argc,argv,"turtle_rect");
ros::NodeHandle n;

n.getParam("width",width);
n.getParam("height",height);
n.getParam("trans_vel",trans_vel);
n.getParam("rot_vel",rot_vel);
n.getParam("abs_x",abs_x);
n.getParam("abs_y",abs_y);
n.getParam("frequency",frequency);
ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
ros::Publisher err_pub = n.advertise<tsim::PoseError>("turtle1/pose_err", 100);
ros::Subscriber sub = n.subscribe("turtle1/pose", 1, poseCallback);

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
srv.request.x =abs_x;
srv.request.y =abs_y;
srv.request.theta =0;
client.call(srv);
srv2.request.off =0;
client2.call(srv2);


ros::Rate rate(frequency);
while (ros::ok())
{
/// calling all functions to complete the rectangle 
horizontal(height,width,trans_vel,rot_vel,abs_x,abs_y,vel_pub, err_pub,x_error,y_error, theta_error,0);
vertical(height,width,trans_vel,rot_vel,abs_x,abs_y,vel_pub, err_pub,x_error,y_error,theta_error,0);
horizontal(height,width,trans_vel,rot_vel,abs_x,abs_y,vel_pub, err_pub,x_error,y_error,theta_error,1);
vertical(height,width,trans_vel,rot_vel,abs_x,abs_y,vel_pub, err_pub,x_error,y_error,theta_error,1);
rate.sleep();
}

}

