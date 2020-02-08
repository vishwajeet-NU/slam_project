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

class ros_stuff 
{
    public:
        ros::Publisher wheel_pub;
        ros::Publisher jt_pub;
        ros::Subscriber in_vel; 
        ros::Subscriber sens;
        ros::NodeHandle n;
        ros::Time current_time, last_time;
        nuturtlebot::WheelCommands msg;
        sensor_msgs::JointState j_out;
       
        DiffDrive turtle_real;

    ros_stuff()
    {
       // current_time = ros::Time::now();
       // last_time = ros::Time::now();


        wheel_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 1);
        jt_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
        
        in_vel = n.subscribe("cmd_vel", 1, &ros_stuff::cmd_callback, this);
        sens = n.subscribe("sensor_data", 1, &ros_stuff::sensor_callback, this);
        
    }
    void cmd_callback(const geometry_msgs::Twist &twist)
    {   
        Twist2D body_twist;
        WheelVelocities go;
        body_twist.v_x = twist.linear.x;
        body_twist.v_y = twist.linear.y;
        body_twist.w= twist.linear.z;
        go = turtle_real.twistToWheels(body_twist);
        double high = 44.0;
        double low = -44.0;
        go.U1 = std::clamp(go.U1,low,high);
        go.U2 = std::clamp(go.U2,low,high);
        
        
        msg.left_velocity = go.U1;
        msg.right_velocity = go.U2;
        wheel_pub.publish(msg);
    }

    void sensor_callback(const nuturtlebot::SensorData &sensor)
    {        
        j_out.header.stamp = ros::Time::now();
        //current_time = ros::Time::now();
        //double dt = (current_time - last_time).toSec();
        //double left_speed = (sensor.left_encoder - j_out.position[0])/dt;
        //double right_speed = (sensor.right_encoder - j_out.position[1])/dt;

        j_out.name.resize(2);
        j_out.name[0] = "left_wheel_axle";
        j_out.name[1] = "right_wheel_axle";

        j_out.position.resize(2);
        j_out.position[0] = sensor.left_encoder;
        j_out.position[1] = sensor.right_encoder;

        //j_out.velocity[0] = left_speed;
        //j_out.velocity[1] = right_speed;

        jt_pub.publish(j_out);
        //last_time = current_time;

    }

};

int main(int argc, char **argv)
{

ros::init(argc,argv,"turtle_interface");

ros_stuff burger_turtle;

while(ros::ok())
{
ros::spinOnce();

}
}
