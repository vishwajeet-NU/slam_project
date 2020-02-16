/// \file
/// \brief Creates a class to find the next waypoints
/// waypoints are taken in as a vector 

#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include<stdio.h>
#include<vector>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/waypoints.hpp"
#include "rigid2d/diff_drive.hpp"
using namespace rigid2d;

DiffDrive bandit;
Vector2D xx; 
Waypoints::Waypoints()
{
    Vector2D zero_vec;
    zero_vec.x=0;
    zero_vec.y=0;
    waypoints.push_back(zero_vec);
}

Waypoints::Waypoints(Vector2D input)
{
    waypoints.push_back(input);    
}


/// \brief find the speed that will take the robot to the next waypoint
/// inputs: the current position of the bot
/// outputs: the body velocity 

Twist2D Waypoints::nextWaypoint(Twist2D current_pose)
{
    Twist2D output_twist;
    std::cout<< "next_x" << waypoints[0].x<<"\n";
    std::cout<< "next_y" << waypoints[0].y<<"\n";

    std::cout<< "current_x" <<  current_pose.v_x<<"\n";
    std::cout<< "current_y" << current_pose.v_y<<"\n";

    double angle_req = atan2((waypoints[0].y - current_pose.v_y), (waypoints[0].x - current_pose.v_x));

    double v_x_world = waypoints[0].x - current_pose.v_x;
    double v_x_body = v_x_world/cos(atan2((waypoints[0].y - current_pose.v_y), (waypoints[0].x - current_pose.v_x)));
    output_twist.v_x = v_x_body;
    output_twist.v_y = 0;
    output_twist.w = angle_req;
    return output_twist;
}

/// \brief updates the current position of the robot inside the waypoint class
/// inputs: the current position 
/// outputs: nothings, but self-updates its knowledge 

void Waypoints::updatePose(Vector2D position)
{
    waypoints.clear();    
    waypoints.push_back(position);

}
