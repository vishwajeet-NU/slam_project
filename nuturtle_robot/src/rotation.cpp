#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include<iostream>
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include "tf/transform_broadcaster.h"

class rotate_bot 
{
    public:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;      
        DiffDrive turtle_real;
        geometry_msgs::Twist speed_out;

    rotate_bot()
    {
        cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        speed_out.linear.x = 0;
        speed_out.linear.y=0;
        speed_out.linear.z = 20;
      }
    
};

int main(int argc, char **argv)
{

ros::init(argc,argv,"rotation");

rotate_bot turn_me;
ros::Rate rate(60);

while(ros::ok())
{
//ros::spinOnce();
turn_me.cmd_pub.publish(turn_me.speed_out);
rate.sleep();
       
}
}
