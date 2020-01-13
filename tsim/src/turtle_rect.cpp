#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <sstream>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h> 
#include "tsim/PoseError.h"


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

void poseCallback(const turtlesim::Pose &pose){
   turtle_pose::x = pose.x;
   turtle_pose::y = pose.y;
   turtle_pose::theta = pose.theta;
}

 


int main(int argc, char **argv)
{
int width;
int height;
int trans_vel ;
int rot_vel;
int abs_x ;
int abs_y;


ros::init(argc, argv, "turtle_rect");
ros::NodeHandle n;

n.getParam("width",width);
n.getParam("height",height);
n.getParam("trans_vel",trans_vel);
n.getParam("rot_vel",rot_vel);
n.getParam("abs_x",abs_x);
n.getParam("abs_y",abs_y);


ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
ros::Publisher err_pub = n.advertise<tsim::PoseError>("turtle1/pose_err", 100);
ros::Subscriber sub = n.subscribe("turtle1/pose", 1, poseCallback);
ros::Rate rate(1);

int local_a;
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
srv.request.x =abs_x;
srv.request.y =abs_y;
srv.request.theta =0;
client.call(srv);

srv2.request.off =0;
client2.call(srv2);




while (ros::ok())
{

float x_error;
float y_error;
int which_part = 1; // think where this should be properly defined.. maybe it should come in at reset?


// going right 

ros::Time start = ros::Time::now();
while(ros::Time::now() - start < ros::Duration(2.0)){
msg.linear.x = trans_vel;
msg.angular.z = 0;
vel_pub.publish(msg);

ros::Duration inner_time1 = ros::Time::now() - start;
ros::spinOnce();

float secs;
secs = inner_time1.toSec(); 
x_error = turtle_pose::x - (secs*trans_vel + abs_x);
y_error = turtle_pose::y - abs_y;
//ROS_INFO_STREAM(x_error);
//ROS_INFO_STREAM(y_error);
er.x_error = x_error;
er.y_error = y_error;
er.theta_error = 0;
err_pub.publish(er);

}
ros::Time start2 = ros::Time::now();

while(ros::Time::now() - start2 < ros::Duration(1.57079)){

msg.linear.x = 0;
msg.angular.z = rot_vel;
vel_pub.publish(msg);

}

// going up 
ros::Time start1 = ros::Time::now();
while(ros::Time::now() - start1 < ros::Duration(2.5)){
msg.linear.x = trans_vel;
msg.angular.z = 0;
vel_pub.publish(msg);

ros::Duration inner_time2 = ros::Time::now() - start1;
ros::spinOnce();

float secs2;
secs2 = inner_time2.toSec(); 
x_error = turtle_pose::x - (abs_x+width) ;
y_error = turtle_pose::y - (secs2*trans_vel + abs_y);
//ROS_INFO_STREAM(x_error);
//ROS_INFO_STREAM(y_error);

er.x_error = x_error;
er.y_error = y_error;
er.theta_error = 0;
err_pub.publish(er);

}
ros::Time start4 = ros::Time::now();

while(ros::Time::now() - start4 < ros::Duration(1.57079)){

msg.linear.x = 0;
msg.angular.z = rot_vel;
vel_pub.publish(msg);

}

ros::Time start5 = ros::Time::now();
while(ros::Time::now() - start5 < ros::Duration(2.0)){
msg.linear.x = trans_vel;
msg.angular.z = 0;
vel_pub.publish(msg);

ros::Duration inner_time5 = ros::Time::now() - start5;
ros::spinOnce();

float secs5;
secs5 = inner_time5.toSec(); 
x_error = turtle_pose::x - (abs_x + width - secs5*trans_vel );
y_error = turtle_pose::y - (abs_y+height);
//ROS_INFO_STREAM(x_error);
//ROS_INFO_STREAM(y_error);
er.x_error = x_error;
er.y_error = y_error;
er.theta_error = 0;
err_pub.publish(er);


}

ros::Time start6 = ros::Time::now();

while(ros::Time::now() - start6 < ros::Duration(1.57079)){

msg.linear.x = 0;
msg.angular.z = rot_vel;
vel_pub.publish(msg);

}
ros::Time start7 = ros::Time::now();
while(ros::Time::now() - start7 < ros::Duration(2.5)){
msg.linear.x = trans_vel;
msg.angular.z = 0;
vel_pub.publish(msg);

ros::Duration inner_time7 = ros::Time::now() - start7;
ros::spinOnce();

float secs7;
secs7 = inner_time7.toSec(); 
x_error = turtle_pose::x - (abs_x) ;
y_error = turtle_pose::y - (abs_y+ height - secs7*trans_vel );
//ROS_INFO_STREAM(x_error);
//ROS_INFO_STREAM(y_error);
er.x_error = x_error;
er.y_error = y_error;
er.theta_error = 0;
err_pub.publish(er);


}

ros::Time start8 = ros::Time::now();

while(ros::Time::now() - start8 < ros::Duration(1.57079)){

msg.linear.x = 0;
msg.angular.z = rot_vel;
vel_pub.publish(msg);

}


}




}












/*





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

while (ros::ok())
{
ros::service::waitForService("spawn");


distance = speed * time 
speed = known. 
start position = known. 





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
