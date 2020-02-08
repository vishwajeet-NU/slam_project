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
        WheelVelocities actual_vel;
        WheelVelocities expected_vel;
        bool istrue = true;
        int i = 0;
        testbot(int a, int b, int c)
        {
        ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
        twi.linear.x = a;
        twi.linear.y = b;
        twi.linear.z = c;
        cmd_pub.publish(twi);

        ros::Subscriber wheel_speeds = n.subscribe("/wheel_cmd", 1, &testbot::callback, this);
        Twist2D check_twist;
        check_twist.v_x  = twi.linear.x;
        check_twist.v_y  = twi.linear.y;
        check_twist.w    = twi.linear.z;
        expected_vel = test_bot.twistToWheels(check_twist);
        actual_vel.U1 = 0;
        actual_vel.U2 = 0;
        while(istrue){
        ros::spinOnce();
        }
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
        testbot test1(10,10,0);
       	ASSERT_EQ(test1.actual_vel.U1, test1.expected_vel.U1);
       	ASSERT_EQ(test1.actual_vel.U2, test1.expected_vel.U2);
	
    }	

TEST(Real_Turtle, no_translation)
	{
        testbot test1(0,0,10);
       	ASSERT_EQ(test1.actual_vel.U1, test1.expected_vel.U1);
       	ASSERT_EQ(test1.actual_vel.U2, test1.expected_vel.U2);
	
    }	

TEST(Real_Turtle, trans_and_rot)
	{
        testbot test1(10,10,10);
       	ASSERT_EQ(test1.actual_vel.U1, test1.expected_vel.U1);
       	ASSERT_EQ(test1.actual_vel.U2, test1.expected_vel.U2);
	
    }	


int main(int argc, char * argv[])
{
    
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "turtle_interface_test");
    ros::NodeHandle n;
    return RUN_ALL_TESTS();
   

}
