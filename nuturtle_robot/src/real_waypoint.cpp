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
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"


class waypoint_bot 
{
    public:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;     
        ros::ServiceServer service;
        ros::ServiceServer stopservice;
        ros::ServiceClient client;
        ros::Subscriber odom_sub;
        rigid2d::telep srv;
        geometry_msgs::Twist msg;
        double kp_linear;
        double kp_angular;
        double max_rotation_vel;
        double max_linear_vel;
        double roll = 0.0;
        double pitch = 0.0; 
        double yaw = 0.0;
        
        double x_starting =0.0;
        double y_starting=0.0;

        float x_pose = 0.0;
        float y_pose = 0.0;
        float theta_pose = 0.0;
        geometry_msgs::Quaternion quat; 
        
       
        bool start_choice_service = false;
        bool stop_choice_service = false;
      

        DiffDrive turtle_real;
        geometry_msgs::Twist speed_out;
        int rotation_vector = 1;
        float frac;
        
   waypoint_bot()
    {
        client = n.serviceClient<rigid2d::telep>("/set_pose");
        cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        odom_sub = n.subscribe("odom", 1, &waypoint_bot::pose_odom_Callback, this);

        ros::spinOnce();
        
    }
    bool dontmove(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {

        stop_choice_service = true;
        return true;
    }
    bool where(nuturtle_robot::which_way::Request &req, nuturtle_robot::which_way::Response &res)
    {     

        srv.request.x = x_starting;
        srv.request.y = y_starting;
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
        start_choice_service = true;
        return true;
     }


void move_angular(float ang_speed)
{
    std::cout<<"stop val"<<!stop_choice_service;
    if(!stop_choice_service)
    {
    ros::spinOnce();
    double angle_tolerance = 0.1;
    double drive_vel_ang;
    while(abs(ang_speed- yaw)>angle_tolerance)
    {
        ros::spinOnce();    
        if( abs(ang_speed - yaw) > PI)
        {
            drive_vel_ang = kp_angular *(ang_speed- yaw + 2.0*PI);

        }
        else if (abs(ang_speed - yaw)<=PI)
        {
            drive_vel_ang = kp_angular *(ang_speed- yaw);
        }

        drive_vel_ang = std::clamp(drive_vel_ang,(-1.0*max_rotation_vel*frac),(1.0*max_rotation_vel*frac));

        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.angular.z = drive_vel_ang;
        cmd_pub.publish(msg);

    }
    msg.angular.z =  0.0;
    cmd_pub.publish(msg);
   }
}



void move_linear(float linear_speed, double x_p, double y_p)
{
    std::cout<<"stop val"<<!stop_choice_service;
    if(!stop_choice_service)
    {
    ros::spinOnce();
    double tolerance = 0.01;
    while( abs(linear_speed - sqrt((x_pose-x_p)*(x_pose-x_p) + (y_pose-y_p)*(y_pose-y_p))) >tolerance)
    {
        ros::spinOnce();    
        double drive_vel_linear = kp_linear* (linear_speed - sqrt((x_pose-x_p)*(x_pose-x_p) + (y_pose-y_p)*(y_pose-y_p)));
        
        drive_vel_linear = std::clamp(drive_vel_linear,(-1.0*max_linear_vel*frac),(1.0*max_linear_vel*frac));
        msg.linear.x = drive_vel_linear;
        msg.linear.y = 0.0;
        msg.angular.z =  0.0;
        cmd_pub.publish(msg);

    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_pub.publish(msg);
    }
    
}
void pose_odom_Callback(const nav_msgs::Odometry &odom_pose){
   x_pose = odom_pose.pose.pose.position.x ;
   y_pose = odom_pose.pose.pose.position.y;
   quat = odom_pose.pose.pose.orientation;

   double quatx= odom_pose.pose.pose.orientation.x;
   double quaty= odom_pose.pose.pose.orientation.y;
   double quatz= odom_pose.pose.pose.orientation.z;
   double quatw= odom_pose.pose.pose.orientation.w;
   tf::Quaternion q(quatx, quaty, quatz, quatw);
   tf::Matrix3x3 m(q);
   m.getRPY(roll, pitch, yaw);
}
};



int main(int argc, char **argv)
{
ros::init(argc,argv,"real_waypoint");
ros::NodeHandle n;

Waypoints whereto;
Twist2D iamhere;    
std::vector<double> x_cors;
std::vector<double> y_cors;

n.getParam("waypoint_x",x_cors);
n.getParam("waypoint_y",y_cors);

    
float frac;
float k_lin;
float k_ang;
double mrv;
double mlv;

ros::param::get("/frac_vel",frac);
ros::param::get("/kp_linear",k_lin);
ros::param::get("/kp_angular",k_ang);
ros::param::get("/max_lvel_robot",mlv);
ros::param::get("/max_rvel_robot",mrv);


waypoint_bot move;
move.frac = frac;
move.kp_linear = k_lin;
move.kp_angular = k_ang;
move.max_linear_vel = mlv;
move.max_rotation_vel = mrv;
move.x_starting = x_cors[0];
move.y_starting = y_cors[0];

ros::ServiceServer service = n.advertiseService("/start", &waypoint_bot::where, &move);
ros::ServiceServer stopservice = n.advertiseService("/stop", &waypoint_bot::dontmove, &move);

std::cout<<"start "<<move.start_choice_service<<"\n";
std::cout<<"stop "<<move.stop_choice_service<<"\n";

ros::Rate rate(100);

while(ros::ok())
{
    if(move.start_choice_service && !move.stop_choice_service)
    {
        unsigned int i =0;    
        for(i = 0; i<(x_cors.size()-1); i ++)
        {
            Vector2D temp_t;
            temp_t.x = x_cors[i+1];
            temp_t.y = y_cors[i+1];
            whereto.updatePose(temp_t);    
            iamhere.v_x = x_cors[i];
            iamhere.v_y = y_cors[i];
            iamhere.w=move.yaw;
        
  
        Twist2D whatsthespeed = whereto.nextWaypoint(iamhere);

        move.move_angular(whatsthespeed.w);
        move.move_linear(whatsthespeed.v_x, iamhere.v_x, iamhere.v_y);
        }
    i = 0;
    } 
    else
    {
        std::cout<<"nothing started \n";
    }
        ros::spinOnce();
      

    rate.sleep();       
    
}
}