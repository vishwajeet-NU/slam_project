#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP
#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include"rigid2d/rigid2d.hpp"

/// \file
/// \brief Library for two-dimensional rigid body transformations.


namespace rigid2d
{
struct WheelVelocities{
double U1 = 0;
double U2 = 0;
};  

class DiffDrive
{
public:
    double left_wheel_angle = 0;
    double right_wheel_angle = 0;
    
    /// \brief the default constructor creates a robot at (0,0,0), with a fixed wheel base and wheel radius
    Twist2D position;
    Transform2D T;
    Transform2D T_bb;
    Transform2D T_out;

    double wheel_base;
    double wheel_radius;
    
    DiffDrive();

     /// \brief determine the wheel velocities required to make the robot
    ///        move with the desired linear and angular velocities
    /// \param twist - the desired twist in the body frame of the robot
    /// \returns - the wheel velocities to use
    /// \throws std::exception

    explicit DiffDrive(Twist2D ps, double wb , double wr);

    /// \brief determine the body twist of the robot from its wheel velocities
    /// \param vel - the velocities of the wheels, assumed to be held constant
    ///  for one time unit
    /// \returns twist in the original body frame of the
    WheelVelocities twistToWheels(Twist2D Vbs);

    /// \brief determine the body twist of the robot from its wheel velocities
    /// \param vel - the velocities of the wheels, assumed to be held constant
    ///  for one time unit
    /// \returns twist in the original body frame of the
    Twist2D wheelsToTwist(WheelVelocities vel);

    /// \brief Update the robot's odometry based on the current encoder readings
    /// \param left - the left encoder angle (in radians)
    /// \param right - the right encoder angle (in radians)
    void updateOdometry( double left, double right );

    // \brief update the odometry of the diff drive robot, assuming that
    /// it follows the given body twist for one time  unit
    /// \param cmd - the twist command to send to the robot
    void feedforward(float left_wheel, float right_wheel);

    /// \brief get the current pose of the robot
    Twist2D pose();

     /// \brief get the wheel speeds, based on the last encoder update
    /// \returns the velocity of the wheels, which is equivalent to
    /// displacement because \Delta T = 1

    WheelVelocities wheelVelocities() const;

    /// \brief reset the robot to the given position/orientation
    void reset(Twist2D ps);
    
};
}
#endif