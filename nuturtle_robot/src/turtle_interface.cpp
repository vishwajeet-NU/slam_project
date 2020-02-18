/// \file
/// \brief This node does the processing needed to generate appropriate wheel velocities for a given 
///         commanded velocity. It also publishes joint states
/// PARAMETERS:
///     It  reads max possible velocity as a fraction of robots maximum , which are also
///     read as params. It also reads the encoder ticks per 1 rotation
/// PUBLISHES:
///     publishes wheel velocities and joint state messages  
/// SUBSCRIBES:
///     subscribes to sensor readings (encoders) and the commanded body twist
/// SERVICES:
///    none
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


/// \brief A class that creates all subscribers and publishers 
/// while also providing methods and variables to do the tasks 
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
        bool first_time = true;
        double first_time_value_left;
        double first_time_value_right;

        double position_left;
        double position_right;
        
        double max_pwr;
        double max_rv;
        double encoderticks;
        
        DiffDrive turtle_real;

    ros_stuff()
    {
        current_time = ros::Time::now();
        last_time = ros::Time::now();

        wheel_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd", 1);
        jt_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
        
        in_vel = n.subscribe("cmd_vel", 1, &ros_stuff::cmd_callback, this);
        sens = n.subscribe("sensor_data", 1, &ros_stuff::sensor_callback, this);
        
    }

    /// \brief callback that reads commnaded velocity ,and converts them to appropriate wheel commanded 
    ///         based on the motor power limitations and geometry of the diff drive robot
    /// \tparam reads twist messages 
    /// \returns nothing 
    void cmd_callback(const geometry_msgs::Twist &twist)
    {   
        Twist2D body_twist;
        WheelVelocities go;
        body_twist.v_x = twist.linear.x;
        body_twist.v_y = twist.linear.y;
        body_twist.w= twist.angular.z;
        go = turtle_real.twistToWheels(body_twist);
        double high = max_pwr;
        double low = -1.0* max_pwr;
        go.U1 = (go.U1*max_pwr)/max_rv;
        go.U2 = (go.U2*max_pwr)/max_rv;
        
        go.U1 = std::clamp(go.U1,low,high);
        go.U2 = std::clamp(go.U2,low,high);
        
        msg.left_velocity = go.U1;
        msg.right_velocity = go.U2;
        wheel_pub.publish(msg);
    }
    /// \brief callback that reads encoder readings and publishes appropriate joint positions and velocities
    /// \tparam reads sensor messages 
    /// \returns nothing

    void sensor_callback(const nuturtlebot::SensorData &sensor)
    {   

        if(first_time) 
        {
          first_time_value_left = sensor.left_encoder;
          first_time_value_right = sensor.right_encoder;
          position_left = sensor.left_encoder;
          position_right = sensor.right_encoder;
          first_time = false;
        }    

        j_out.header.stamp = ros::Time::now();
        double dt = 0.01;
        double left_speed = (sensor.left_encoder - position_left)/(encoderticks*dt);
        double right_speed = (sensor.right_encoder - position_right)/(encoderticks*dt);
        j_out.name.resize(2);
        j_out.name[0] = "left_wheel_axle";
        j_out.name[1] = "right_wheel_axle";
        
        j_out.position.resize(2);
        j_out.position[0] = 2.0*PI* ((sensor.left_encoder- first_time_value_left)/encoderticks);
        j_out.position[1] = 2.0*PI* ((sensor.right_encoder - first_time_value_right)/encoderticks);

        j_out.velocity.resize(2);
        j_out.velocity[0] = left_speed;
        j_out.velocity[1] = right_speed;

        jt_pub.publish(j_out);
        position_left = sensor.left_encoder;
        position_right = sensor.right_encoder;

    }

};

int main(int argc, char **argv)
{

    ros::init(argc,argv,"turtle_interface");

    ros_stuff burger_turtle;
    double power;
    double rvel;
    double ticks;
    ros::param::get("/max_motor_power",power);
    ros::param::get("/max_rvel_motor",rvel);
    ros::param::get("/encoder_ticks_per_rev",ticks);

    burger_turtle.max_pwr = power;
    burger_turtle.max_rv = rvel;
    burger_turtle.encoderticks = ticks;

    while(ros::ok())
    {
        ros::spinOnce();
    }
}
