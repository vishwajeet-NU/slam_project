/// \file
/// \brief Simulates readings of encoders on wheels

/// Publishers : Jointstate of the two wheels
/// Subscribers : commanded velocity 
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"

/// global variables
sensor_msgs::JointState jt_output;
Twist2D forward_velocity_body;
WheelVelocities wheel_rotations;



/// \brief callback to capture twist

void get_vel(const geometry_msgs::Twist velo)
{
    forward_velocity_body.v_x = velo.linear.x;
    forward_velocity_body.w = velo.angular.z;
    forward_velocity_body.v_y=0;
}

int main(int argc, char **argv)

{
    DiffDrive turtle_go;

    ros::init(argc,argv,"fake_diff_encoder");
    ros::NodeHandle n;
    ros::Subscriber cmd_sub = n.subscribe("turtle1/cmd_vel", 1, get_vel);
    ros::Publisher joint_state_publisher = n.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Rate rate(60);

    while(n.ok())
    {
        ros::spinOnce();
        wheel_rotations = turtle_go.twistToWheels(forward_velocity_body);
        jt_output.header.stamp = ros::Time::now();
        jt_output.name.resize(2);
        jt_output.name[0] = "left_wheel_axle";
        jt_output.name[1] = "right_wheel_axle";

        jt_output.position.resize(2);
        jt_output.position[0] = turtle_go.left_wheel_angle;
        jt_output.position[1] = turtle_go.right_wheel_angle;

        joint_state_publisher.publish(jt_output);
        rate.sleep();

    }
}
