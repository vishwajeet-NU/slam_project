// tests.cpp
#include "ros/ros.h"
#include "rigid2d/rigid2d.hpp"
#include <gtest/gtest.h>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
using namespace rigid2d;
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"


class testbot
{
        public:
        ros::NodeHandle n;
        DiffDrive test_bot;
        geometry_msgs::Twist twi;
        nuturtlebot::SensorData ss;
        WheelVelocities actual_vel;
        WheelVelocities joint_pos;
        WheelVelocities joint_vels;

        
        bool istrue = true;
        bool istrue2 = true;
        
        int i = 0;
        testbot(double a, double b, double c)
        {
        ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        twi.linear.x = a;
        twi.linear.y = b;
        twi.angular.z = c;

        cmd_pub.publish(twi);

        ros::Subscriber wheel_speeds = n.subscribe("/wheel_cmd", 1, &testbot::callback, this);

        while(istrue)
        {
        ros::spinOnce();
        }
        }

        testbot(double d, double e)
        {
        ros::Publisher fake_sensor = n.advertise<nuturtlebot::SensorData>("sensor_data",1,true);

        ss.left_encoder = d;
        ss.right_encoder = e;

        fake_sensor.publish(ss);

        ros::Subscriber in_joint = n.subscribe("/joint_states",1,&testbot::jtcallback,this);

        while(istrue2)
        {
        ros::spinOnce();
        }
        
        }
        
    void jtcallback(const sensor_msgs::JointState &sens)
    {
        joint_pos.U1 = sens.position[0];
        joint_pos.U2 = sens.position[1];

        joint_vels.U1 = sens.velocity[0];
        joint_vels.U2 = sens.velocity[1];
        istrue2 = false;

    }
    void callback(const nuturtlebot::WheelCommands &wheel)
    {   
        actual_vel.U1 = wheel.left_velocity;
        actual_vel.U2 = wheel.right_velocity;
        istrue = false;
    }


};

TEST(Real_Turtle, no_rotation)
	{
        testbot test1(0.22,0.0,0.0);
        ASSERT_EQ(test1.actual_vel.U1, 265);
       	ASSERT_EQ(test1.actual_vel.U2, 265);
	
    }	

TEST(Real_Turtle, no_translation)
	{
        testbot test1(0.0,0.0,2.84);
       	ASSERT_EQ(test1.actual_vel.U1, -265);
       	ASSERT_EQ(test1.actual_vel.U2, 265);   
	
    }	

TEST(Real_Turtle, trans_and_rot)
	{
        testbot test1(0.1,0.0,1.0);
       	ASSERT_EQ(test1.actual_vel.U1, 25);
       	ASSERT_EQ(test1.actual_vel.U2, 227);
	
    }	

TEST(Real_Turtle, encoder_check)
{
    testbot test1(0.0,0.0);
    ASSERT_EQ(test1.joint_pos.U1, 0.0);
    ASSERT_EQ(test1.joint_pos.U2, 0.0);

    ASSERT_EQ(test1.joint_vels.U1, 0.0);
    ASSERT_EQ(test1.joint_vels.U2, 0.0);
 	

}
int main(int argc, char * argv[])
{
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");
    ros::NodeHandle n;

    return RUN_ALL_TESTS();
   

}
