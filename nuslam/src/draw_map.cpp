/// \file
/// \brief This file creates cylindrical markers for visualizing waypoints on rviz
///
/// PARAMETERS:
///     it takes in waypoints x and y as parameter. 
/// PUBLISHES:
///     publishes visualization messages 
/// SUBSCRIBES:
///     no subscribers 
/// SERVICES:
///     no services used

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nuslam/turtle_map.h>


std::vector<float> x_vals;
std::vector<float> y_vals;

void Callback(const nuslam::turtle_map &coordinates)
{
    x_vals = coordinates.x_vals;
    y_vals = coordinates.y_vals;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "markers");
  ros::NodeHandle n;

  ros::Rate r(5);
  
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker_scan", 1,true);
  ros::Subscriber marker_sub = n.subscribe("landmarks", 1, Callback);


  while (ros::ok())
  {
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type =  visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.id = 0;

    marker.lifetime = ros::Duration();
    float x_pos;
    float y_pos;
    for(unsigned int i = 0; i<x_vals.size(); i ++)
    {
      x_pos = x_vals[i];
      y_pos =  y_vals[i];
      geometry_msgs::Point p;
      p.x = x_pos;
      p.y = y_pos;
      p.z = 0;
      marker.points.push_back(p);    

    }
    
    marker_pub.publish(marker);

    r.sleep();

     }
}
