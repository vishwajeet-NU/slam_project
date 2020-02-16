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


class waypoint_bot 
{
    public:
        ros::NodeHandle n;
        ros::Publisher cmd_pub;     
        ros::ServiceServer service;
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
        

        float x_pose = 0.0;
        float y_pose = 0.0;
        float theta_pose = 0.0;
        geometry_msgs::Quaternion quat; 
        
       
        bool start_choice_service = false;
      
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
        start_choice_service = true;
        return true;
     }


void move_angular(float ang_speed)
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

        drive_vel_ang = std::clamp(drive_vel_ang,(-1.0*max_rotation_vel),(1.0*max_rotation_vel));

        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.angular.z = drive_vel_ang;
        //std::cout<<"inside yaw = "<<yaw<<"\n";
        //std::cout<<"inside ang_speed =  "<<ang_speed<<"\n";
        //std::cout<<"w speed = "<< msg.angular.z<<"\n";
        cmd_pub.publish(msg);

    }
    msg.angular.z =  0.0;
    cmd_pub.publish(msg);
   
}



void move_linear(float linear_speed, double x_p, double y_p)
{
    ros::spinOnce();    
    double tolerance = 0.01;
    //std::cout<<"req dist = "<< linear_speed<<"\n";
    
    //std::cout<<"req dist = "<< linear_speed<<"\n";

    //std::cout<<"travel = "<< sqrt((x_pose-x_p)*(x_pose-x_p) + (y_pose-y_p)*(y_pose-y_p))<<"\n";
    

    while( abs(linear_speed - sqrt((x_pose-x_p)*(x_pose-x_p) + (y_pose-y_p)*(y_pose-y_p))) >tolerance)
    {
        ros::spinOnce();    
        double drive_vel_linear = kp_linear* (linear_speed - sqrt((x_pose-x_p)*(x_pose-x_p) + (y_pose-y_p)*(y_pose-y_p)));
        
        drive_vel_linear = std::clamp(drive_vel_linear,(-1.0*max_linear_vel),(1.0*max_linear_vel));
        
        
//       double error = (linear_speed - sqrt((x_pose-x_p)*(x_pose-x_p) + (y_pose-y_p)*(y_pose-y_p)));
        msg.linear.x = drive_vel_linear;
        msg.linear.y = 0.0;
        msg.angular.z =  0.0;
        cmd_pub.publish(msg);
        //std::cout<<"error = "<<error<<"\n";

        //std::cout<<"inside linear x pose = "<<x_pose<<"/n";
        //std::cout<<"inside linear x p = = "<<x_p<<"/n";
        //std::cout<<"vx speed = "<< msg.linear.x<<"\n";

    }

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_pub.publish(msg);

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
//   std::cout<<"yaw value = "<<yaw<<"\n";
}
};



int main(int argc, char **argv)
{
ros::init(argc,argv,"real_waypoint");
ros::NodeHandle n;

double x0,x1,x2,x3,x4,x5;
double y0,y1,y2,y3,y4,y5;

Waypoints whereto;
Twist2D iamhere;    

n.getParam("x0",x0);
n.getParam("x1",x1);
n.getParam("x2",x2);
n.getParam("x3",x3);
n.getParam("x4",x4);
n.getParam("x5",x5);
n.getParam("y0",y0);
n.getParam("y1",y1);
n.getParam("y2",y2);
n.getParam("y3",y3);
n.getParam("y4",y4);
n.getParam("y5",y5);

std::vector<double> x_cors = {x0,x1,x2,x3,x4,x5};
std::vector<double>y_cors = {y0,y1,y2,y3,y4,y5};
    
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


ros::ServiceServer service = n.advertiseService("/start", &waypoint_bot::where, &move);


ros::Rate rate(100);

while(ros::ok())
{
    if(move.start_choice_service)
    {
        unsigned int i =0;    
        for(i = 0; i<(x_cors.size()-1); i ++)
        {
            Vector2D temp_t;
            temp_t.x = x_cors[i+1];
            temp_t.y = y_cors[i+1];
        //std::cout<<"temp x"<<temp_t.x<<"\n";
        //std::cout<<"temp y"<<temp_t.y<<"\n";

            whereto.updatePose(temp_t);    
            iamhere.v_x = x_cors[i];
            iamhere.v_y = y_cors[i];
            iamhere.w=move.yaw;
        
        //std::cout<<"base x"<<iamhere.v_x<<"\n";
        //std::cout<<"base y"<<iamhere.v_y<<"\n";
        
        //std::cout<<"nex x"<<temp_t.x<<"\n";
        //std::cout<<"next y"<<temp_t.y<<"\n";
  
   //     std::cout<<"max linear"<<move.max_linear_vel<<"\n";
   //     std::cout<<"max angular"<<move.max_rotation_vel<<"\n";
  
        Twist2D whatsthespeed = whereto.nextWaypoint(iamhere);

//        std::cout<<whatsthespeed.w<<"\n";
//        std::cout<<whatsthespeed.v_x<<"\n";
        move.move_angular(whatsthespeed.w);
        move.move_linear(whatsthespeed.v_x, iamhere.v_x, iamhere.v_y);
        }
    i = 0;
    } 
    else
    {
        //std::cout<<"nothing started \n";
    }
        ros::spinOnce();
      

    rate.sleep();       
    
}
}