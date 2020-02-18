/// \file
/// \brief This file gives commands to make a diff drive robot rotate 20 full cicrles, 
///        while stopping for 1/20th time requried to do one full circle
/// PARAMETERS:
///     It  reads max possible velocity as a fraction of robots maximum , which are also
///     read as params 
/// PUBLISHES:
///     publishes velocities to make the turtle follow waypoints 
/// SUBSCRIBES:
///     nothing
/// SERVICES:
///    start services which decides when to start the bot. This calls set pose which resets the bot to 0,0

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

/// \brief A class that creates all subscribers and publishers 
/// while also providing methods and variables to do the tasks 
class rotate_bot 
{
    public:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;     
        ros::ServiceServer service;
        ros::ServiceClient client;
        rigid2d::telep srv;
        double max_rotation_vel;

        int no_rot= 0;

        float start_choice_service = 0.0;
      
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
    /// \brief callback that checks start service and calls set pose service, to set turtle to first waypoint
    ///
    /// \tparam service request and response
    /// \returns boolean 

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
        res.vector = rotation_vector;
        start_choice_service = 1.0;
        timer_count = 0;
        stop = 1;
        no_rot = 0;
        return true;
     }
    /// \brief timer callback that is used to publish messages at the desired freq(100 hz this case)
    ///
    /// \tparam timer event
    /// \returns nothing 

    void timerCallback(const ros::TimerEvent&)
    {
        speed_out.linear.x = 0;
        speed_out.linear.y = 0;
        speed_out.angular.z = start_choice_service* stop * frac * rotation_vector * max_rotation_vel; // remember to change 0.5 to param reading 

        cmd_pub.publish(speed_out);
        timer_count = timer_count + 1;
        std::cout<<"i am timer counts = "<< timer_count << "\n";
    }

};
    /// \brief a delay function to stop the bot between every full rotation
    ///
    /// \tparam wait time and the robot class
    /// \returns nothing 


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
    ros::init(argc,argv,"rotation");
    ros::NodeHandle n;
    double mrv;

    float frac;
    ros::param::get("/frac_vel",frac);
    ros::param::get("/max_rvel_robot",mrv);

    rotate_bot turn_me;
    turn_me.frac = frac;
    turn_me.max_rotation_vel = mrv;

    ros::ServiceServer service = n.advertiseService("/start", &rotate_bot::where, &turn_me);

    ros::Timer timer = n.createTimer(ros::Duration(0.01), &rotate_bot::timerCallback, &turn_me);

    while(ros::ok())
    {
        ros::spinOnce();

        while( turn_me.no_rot <20)
        {
            ROS_INFO("%d \n",turn_me.no_rot);
            double rotation_counts_needed = 2.0*PI*100.0 / (frac*turn_me.max_rotation_vel);

            ROS_INFO("%f \n",rotation_counts_needed);

            while (turn_me.timer_count < (rotation_counts_needed+1) )
            {
                // 1 rotation done
                ros::spinOnce();
            }
        
            delay(rotation_counts_needed, turn_me);
     
            turn_me.timer_count = 0;
            turn_me.no_rot = turn_me.no_rot + 1;
        }
    turn_me.stop = 0.0;
    ros::spinOnce();
    turn_me.timer_count = 0;
    
}
}
