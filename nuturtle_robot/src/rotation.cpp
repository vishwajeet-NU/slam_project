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
#include "nuturtle_robot/which_way.h"
#include "rigid2d/telep.h"

//max rotation vel = 2.84 rad/s
#define max_rotation_vel 2.84;
class rotate_bot 
{
    public:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;     
        ros::ServiceServer service;
        ros::ServiceClient client;
        rigid2d::telep srv;


        DiffDrive turtle_real;
        geometry_msgs::Twist speed_out;
        int rotation_vector = 1;
        bool istrue = true;
        
   rotate_bot()
    {
        
        
        cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        speed_out.linear.x = 0;
        speed_out.linear.y = 0;
        speed_out.linear.z = 0.5* rotation_vector * max_rotation_vel; // remember to change 0.5 to param reading 
        ros::spinOnce();
        
    }


    bool where(nuturtle_robot::which_way::Request &req, nuturtle_robot::which_way::Response &res)
    {     

        srv.request.x = 0.0;
        srv.request.y = 0.0;
        srv.request.theta= 0.0;
        client.call(srv);

        if (client.call(srv))
       {
         ROS_INFO("called ");
       }
       else
       {
         ROS_ERROR("Failed to call service");
       }
        if(req.direction == -1)
        {
            rotation_vector = -1;
        }
        if(req.direction == 1)
        {
            rotation_vector = 1;
        }
        
        return true;
     }

};

int main(int argc, char **argv)
{

ros::init(argc,argv,"rotation");
ros::NodeHandle n;
rotate_bot turn_me;
ros::Rate rate(60);

ros::ServiceServer service = n.advertiseService("/start", &rotate_bot::where, &turn_me);
ros::ServiceClient client = n.serviceClient<rigid2d::telep>("/set_pose");
while(ros::ok())
{

turn_me.cmd_pub.publish(turn_me.speed_out);
ros::spinOnce();
rate.sleep();
}
}
