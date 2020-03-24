/// \file
/// \brief This file publishes markers at the reported locations at the landmark topic
/// it reads frame name and colors from the landmark topic it subscrbes to
/// PARAMETERS:
/// none
/// PUBLISHES:
///     marker_pub (visualization_msgs::MarkerArray): publishes an array of cylinders at landmark location
///     
/// SUBSCRIBES:
///     marker_sub (nuslam::turtle_map): reads landmark location, and receives color and frame of reference
/// SERVICES:
///     none
///     

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nuslam/turtle_map.h>


std::vector<float> x_center;
std::vector<float> y_center;
std::vector<float> radius;

bool gotreading = false;
static double red;
static double green;
static double blue;
static double alpha;
std::string frame;

/// \brief callback for reading landmark topic
///
/// \tparam inputs: turtlemap message
/// \returns none
void Callback(const nuslam::turtle_map &coordinates)
{
    x_center = coordinates.x_center;
    y_center = coordinates.y_center;
    radius = coordinates.radius;
    red = coordinates.red;
    blue = coordinates.blue;
    green = coordinates.green;
    alpha = coordinates.alpha;
    frame = coordinates.frame_name;
    gotreading = true;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "markers");
  ros::NodeHandle n;

  
  ros::Rate r(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_scan_array", 1,true);
  ros::Subscriber marker_sub = n.subscribe("landmarks", 1, Callback);
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

   

  while (ros::ok())
  {
    while(gotreading)
    {
    visualization_msgs::MarkerArray array;  
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;     
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue ;
    marker.color.a = alpha;
    marker.lifetime = ros::Duration();

    unsigned int i =0;    
    for(i = 0; i<(x_center.size()); i ++)
    {
      marker.scale.x = radius[i];
      marker.scale.y = radius[i];
      marker.scale.z = 0.5;

      marker.id = i;
      marker.pose.position.x = x_center[i];
      marker.pose.position.y = y_center[i];
      marker.pose.position.z = 0;
      array.markers.push_back(marker);
     
    }
    marker_pub.publish(array);
    gotreading = false;
    }

    ros::spinOnce();
    r.sleep();

  }
}










