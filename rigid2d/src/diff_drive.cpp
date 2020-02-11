/// \file
/// \brief Creates a class to model a diff drive robot

#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
namespace rigid2d



{
DiffDrive::DiffDrive(){
    position.v_x =0;
    position.v_y=0;
    position.w=0;
    wheel_base = 0.16;
    wheel_radius = 0.033;
    T.m1 = cos(position.w);
    T.m2 = -sin(position.w);
    T.m3 = position.v_x;
    T.m4 = sin(position.w);
    T.m5 = cos(position.w);
    T.m6 = position.v_y;
}


DiffDrive::DiffDrive(Twist2D ps, double wb , double wr)
{
    position.v_x =ps.v_x;
    position.v_y=ps.v_y;
    position.w=ps.w;
    wheel_base = wb;
    wheel_radius = wr;
    T.m1 = cos(position.w);
    T.m2 = -sin(position.w);
    T.m3 = position.v_x;
    T.m4 = sin(position.w);
    T.m5 = cos(position.w);
    T.m6 = position.v_y;
}

/// \brief Computes wheel speeds from body twist for diff drive robot
///
/// Takes twist  
/// \returns left and right wheel speeds
WheelVelocities DiffDrive::twistToWheels(Twist2D Vbs)
{
    double D;
    D = wheel_base/ 2.0;
    WheelVelocities vels;
    vels.U1 = (1/wheel_radius)*(-D*Vbs.w + Vbs.v_x);
    vels.U2 = (1/wheel_radius)*(D*Vbs.w + Vbs.v_x);
    updateOdometry(vels.U1, vels.U2);
    return vels;
}

/// \brief Computes a body twist from wheel speeds
///
/// Takes wheel speeds  
/// \returns body twist

Twist2D DiffDrive::wheelsToTwist(WheelVelocities vel)
{
    double D;
    D = wheel_base/ 2.0;

    Twist2D Vbb;
    Vbb.w = wheel_radius * ((-1 * vel.U1)/(2*D) + (1*vel.U2)/(2*D));
    Vbb.v_x = wheel_radius * (0.5* vel.U1 + 0.5*vel.U2);
    Vbb.v_y = 0;
    return Vbb;
}

/// \brief Computes odometry of the diff bot
///
/// Takes wheel speeds  
/// \returns total angle travelled 

void DiffDrive::updateOdometry(double left, double right)
{
    left_wheel_angle = left/60 + left_wheel_angle;
    right_wheel_angle = right/60 + right_wheel_angle; 
}

/// \brief Computes odometry to estimate body in the world frame
///
/// Takes wheel speeds  
/// \returns nothing but updates position of the body in the world frame (of the class)

void DiffDrive::feedforward(float left_wheel, float right_wheel)
{
    Twist2D Vb;
    WheelVelocities in; 
    Twist2D delta_qb;
    in.U1 = left_wheel;
    in.U2 = right_wheel;
    Vb = wheelsToTwist(in);


if(Vb.w ==0)
{
  T_bb.m1 = T.m1;
  T_bb.m2= T.m2;
  T_bb.m3 = Vb.v_x;  
  T_bb.m4 = T.m4;
  T_bb.m5= T.m5;
  T_bb.m6 = Vb.v_y;  
}

else{

T_bb = T.integrateTwist(Vb);
}

delta_qb.v_x = T_bb.m3;
delta_qb.v_y = T_bb.m6;
delta_qb.w = Vb.w;


position.v_x = position.v_x + cos(position.w) * delta_qb.v_x - sin(position.w)* delta_qb.v_y;
position.v_y = position.v_y+ sin(position.w) * delta_qb.v_x + cos(position.w)* delta_qb.v_y;
position.w = position.w + delta_qb.w;
}

/// \brief gives body pose when required 
///
/// no inputs  
/// \returns body pose


Twist2D DiffDrive::pose()
{
Twist2D current;

current.v_x = position.v_x;
current.v_y = position.v_y;
current.w = position.w;
return current;
}

void DiffDrive::reset(Twist2D ps)
{
  position.v_x = ps.v_x;
  position.v_y = ps.v_y;
  position.w   = ps.w;
}
}