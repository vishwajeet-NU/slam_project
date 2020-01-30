// tests.cpp
#include "ros/ros.h"
#include "rigid2d/rigid2d.hpp"
#include <gtest/gtest.h>
#include <iostream>
#include<math.h>
using namespace rigid2d;
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"


Transform2D TT;
Transform2D T,T2;

Twist2D twist;
Vector2D vec1;
Vector2D vec2;
DiffDrive test_diff;
rigid2d::WheelVelocities testvel;

TEST (rigid2d, adjoint)
{
	Transform2D test;
	test.Adjoint();
	ASSERT_EQ(1,test.z5);
	ASSERT_EQ(0,test.z6);
	ASSERT_EQ(0,test.z8);
	ASSERT_EQ(1,test.z9);
		
}
TEST (rigid2d, inv)
{

	Transform2D test2, inv_m;
	test2.m1 = 0.8766;
	test2.m2 = -0.4794;
	test2.m3 = 3;
	test2.m4 = 0.4794;
	test2.m5 = 0.8766;
	test2.m6 = 1;
	inv_m = test2.inv();
	ASSERT_NEAR(0.8766,inv_m.m1,1e-2);
	ASSERT_NEAR(0.4793,inv_m.m2,1e-2);
	ASSERT_NEAR(-3.1122,inv_m.m3,1e-2);
	ASSERT_NEAR(0.5607,inv_m.m6,1e-2);


}
TEST(addtest, twos) { 
    ASSERT_EQ(5, TT.dummy(2,3));
}

TEST(integration_twistcheck, m1term)
{
	ASSERT_GT(0.01, abs(0.5403-T2.m1));
}
TEST(integration_twistcheck, m2term)
{
	ASSERT_GT(0.01, abs(-0.8415 - T2.m2));
}

TEST(integration_twistcheck, m3term)
{
	ASSERT_GT(0.01, abs(0.3038- T2.m3));
}

TEST(integration_twistcheck, m4term)
{
	ASSERT_GT(0.01, abs(3.4438- T2.m6));
}

TEST(VectorCheck, length_check)
{
	ASSERT_EQ(5, vec1.length());
}
TEST(VectorCheck, angle_check)
{
	ASSERT_GT(0.01 , abs(0.9273- vec1.angle()));
}

TEST(VectorCheck, distance_check)
{
	ASSERT_EQ(5, vec1.distance(vec1,vec2));

}


TEST(VectorCheck, addition_check)
{
	Vector2D dummy;
	Vector2D vec_dummy;
	dummy.x = 9;
	dummy.y= 12;    
	vec_dummy = operator+(vec1,vec2);
	ASSERT_EQ(dummy.x,vec_dummy.x );
	ASSERT_EQ(dummy.y,vec_dummy.y );

}
TEST(VectorCheck, subtraction_check)
{
	Vector2D dummy;
	Vector2D vec_dummy;
	dummy.x = -3;
	dummy.y= -4;    
	vec_dummy = operator-(vec1,vec2);
	ASSERT_EQ(dummy.x,vec_dummy.x );
	ASSERT_EQ(dummy.y,vec_dummy.y );
}

TEST(VectorCheck, scalar_product_check)
{
ASSERT_EQ(50, operator*(vec1,vec2));
}


TEST(DiffDrive, constructor)
	{
		Twist2D pose = test_diff.pose();
		ASSERT_EQ(testvel.U1, 0);
		ASSERT_EQ(testvel.U2, 0);
	    ASSERT_EQ(pose.v_x, 0);
		ASSERT_EQ(pose.v_y, 0);
		ASSERT_EQ(pose.w, 0);
	}
TEST(DiffDrive, twistToWheels)
	{
		Twist2D Vb;
		test_diff.wheel_base = 1;
		test_diff.wheel_radius = 0.02;
		Vb.v_x = 0;
		Vb.v_y =0;
		Vb.w = PI/4;
		WheelVelocities vels = test_diff.twistToWheels(Vb);
		ASSERT_NEAR(vels.U1, -19.6349, 1.0e-4);
		ASSERT_NEAR(vels.U2, 19.6349, 1.0e-4);
		Vb.v_x = 1;
		vels = test_diff.twistToWheels(Vb);
		ASSERT_NEAR(vels.U1, 30.36504, 1.0e-4);
		ASSERT_NEAR(vels.U2, 69.6349, 1.0e-4);
	}
	
TEST(DiffDrive,wheelsToTwist)
	{
		WheelVelocities vels;
		vels.U1 = -19.6349;
		vels.U2 = 19.6349;
		Twist2D twist = test_diff.wheelsToTwist(vels);
		ASSERT_EQ(twist.v_x, 0);
		ASSERT_EQ(twist.v_y, 0);
        ASSERT_NEAR(twist.w, PI/4, 1.0e-4);

		vels.U1 = 30.36504;
		vels.U2 = 69.6349;
		twist = test_diff.wheelsToTwist(vels);
		ASSERT_NEAR(twist.w, PI/4, 1.0e-4);
		ASSERT_NEAR(twist.v_x, 1,1.0e-4);
		ASSERT_EQ(twist.v_y, 0);

	}

TEST(DiffDrive, updateOdometry)
	{	
		DiffDrive test;
		test.updateOdometry(10.0,10.0);
		ASSERT_NEAR(test.left_wheel_angle,0.1666,1.0e-4); 
		ASSERT_NEAR(test.right_wheel_angle, 0.1666,1.0e-4); 

	}

TEST(DiffDrive, feedforward)
	{
        Twist2D Vb;
		DiffDrive feedcheck;
		feedcheck.wheel_base = 1;
		feedcheck.wheel_radius = 0.02;
		Vb.w = PI/2;
		Vb.v_x = 1;
		Vb.v_y = 0;
		WheelVelocities in;
		in = feedcheck.twistToWheels(Vb);
		feedcheck.feedforward(in.U1, in.U2);
		ASSERT_NEAR(feedcheck.position.v_x, 0.63662, 1.0e-4);
		ASSERT_NEAR(feedcheck.position.v_y, 0.63662, 1.0e-4);
	    ASSERT_NEAR(feedcheck.position.w, PI/2, 1.0e-4);
	}	

int main(int argc, char **argv) 
{
		testing::InitGoogleTest(&argc, argv);
 
	    twist.w = 1;
    	twist.v_x=2;
	    twist.v_y=3;
	    vec1.x = 3;
    	vec1.y = 4;
    	vec2.x = 6;
    	vec2.y = 8;

	    T2 = T.integrateTwist(twist);

    	return RUN_ALL_TESTS();
}
