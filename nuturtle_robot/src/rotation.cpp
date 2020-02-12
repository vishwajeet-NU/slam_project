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
#define max_rotation_vel 2.84
class rotate_bot 
{
    public:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;     
        ros::ServiceServer service;
        ros::ServiceClient client;
        rigid2d::telep srv;
      
        int timer_count = 0;
        DiffDrive turtle_real;
        geometry_msgs::Twist speed_out;
        int rotation_vector = 1;
        bool istrue = true;
        float stop = 1.0;
        float frac;
        
   rotate_bot()
    {
        client = n.serviceClient<rigid2d::telep>("/set_pose");
        cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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

void timerCallback(const ros::TimerEvent&)
{
    speed_out.linear.x = 0;
    speed_out.linear.y = 0;
    speed_out.angular.z = stop * frac * rotation_vector * max_rotation_vel; // remember to change 0.5 to param reading 

    cmd_pub.publish(speed_out);
    timer_count = timer_count + 1;
    std::cout<<"i am timer counts = "<< timer_count << "\n";
}


};


void delay(double wait_time, rotate_bot temp)
{
  geometry_msgs::Twist speed;
  speed.linear.x =0.0;
  speed.linear.y =0.0;
  speed.angular.z =0.0;
  temp.cmd_pub.publish(speed);
  ros::Duration(wait_time/(100*20)).sleep();
}


int main(int argc, char **argv)
{
int no_rot= 0;
ros::init(argc,argv,"rotation");
ros::NodeHandle n;



float frac;
ros::param::get("/frac_vel",frac);
rotate_bot turn_me;
turn_me.frac = frac;
//ros::Rate rate(100);

ros::Timer timer = n.createTimer(ros::Duration(0.01), &rotate_bot::timerCallback, &turn_me);

ros::ServiceServer service = n.advertiseService("/start", &rotate_bot::where, &turn_me);

ros::Duration(7).sleep();

while(ros::ok())
{
    while( no_rot <20)
    {
        ROS_INFO("%d \n",no_rot);
        double rotation_counts_needed = 2*PI*100.0 / (frac*max_rotation_vel);

        ROS_INFO("%f \n",rotation_counts_needed);

        while (turn_me.timer_count < (rotation_counts_needed+1) )
        {
            // 1 rotation done
            ros::spinOnce();
        }
        
        delay(rotation_counts_needed, turn_me);
      

        turn_me.timer_count = 0;
        no_rot = no_rot + 1;
    }
turn_me.stop = 0.0;
ros::spinOnce();
turn_me.timer_count = 0;

}

}
