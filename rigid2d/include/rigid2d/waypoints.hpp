#ifndef waypoints_INCLUDE_GUARD_HPP
#define waypoints_INCLUDE_GUARD_HPP
#include <math.h>
#include <iostream>
#include<iosfwd> // contains forward definitions for iostream objects
#include<stdio.h>
#include<vector>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/diff_drive.hpp"



using namespace rigid2d;
class Waypoints
{

public:
        std::vector<Vector2D> waypoints;

        Waypoints();
        Waypoints(Vector2D input );
        Twist2D nextWaypoint(Twist2D current_pose);
        void updatePose(Vector2D Vd);
        float test(float a);
};

#endif