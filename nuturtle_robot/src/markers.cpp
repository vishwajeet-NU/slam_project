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
#include <visualization_msgs/MarkerArray.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "markers");
  ros::NodeHandle n;

  ros::service::waitForService("/start");
  
  std::vector<double> x_cors;
  std::vector<double> y_cors;

  n.getParam("waypoint_x",x_cors);
  n.getParam("waypoint_y",y_cors);

  
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1,true);
  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    unsigned int i =0;    
    for(i = 0; i<(x_cors.size()-1); i ++)
    {
      marker.id = i;
      marker.pose.position.x = x_cors[i];
      marker.pose.position.y = y_cors[i];
      marker.pose.position.z = 0;
    

     marker_pub.publish(marker);
     r.sleep();

    }
     }
}
