/// \file
/// \brief This file makes a turtle travel in a pentagon, using a feedforward
/// strategy of control. It also prints the error between its expected position and 
/// the actual position, and also plots it
/// PARAMETERS:
/// x positions 
/// y positions 

/// PUBLISHES:
///     vel_pub (cmd_vel): publishes velocity commands to move the turtle 
///     err_pub (pose_err): this topic takes in error in actual and expected position of the turtle
/// SUBSCRIBES:
///     sub (pose): reads actual position data of the turtle
///     sub (odom): odometry readings from the /odom topic 
/// SERVICES:
///     client2 (SetPen): can be used to change color and transperency of turtle marker
///     client   (TeleportAbsolute) used to teleport the turtle to desired point and orientation
///     

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/SetPen.h>
#include<std_srvs/Empty.h>
#include <turtlesim/Pose.h> 
#include "tsim/PoseError.h"
#include <math.h>
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include "tf/transform_broadcaster.h"
#include <vector>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "sensor_msgs/LaserScan.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>    // std::sort
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <numeric>

using namespace Eigen;

void fit_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{
   for (int k =0;k<x_in.size(); k++)
  {
  float x_mean;
  float y_mean;
  float z_mean;
  float x_centroid;
  float y_centroid;

  std::vector<float> current_x_cluster = x_in[k];
  std::vector<float> current_y_cluster = y_in[k];

  std::vector<float> current_z_cluster;
  int n = current_x_cluster.size();

  float sum_x = std::accumulate(std::begin(current_x_cluster), std::end(current_x_cluster), 0.0);
  x_mean =  sum_x / current_x_cluster.size();
  float sum_y = std::accumulate(std::begin(current_y_cluster), std::end(current_y_cluster), 0.0);
  y_mean =  sum_y / current_y_cluster.size();


  for(int l = 0;  l< (current_x_cluster.size()); l++ )
  {
    current_z_cluster.push_back(current_x_cluster[l] * current_x_cluster[l] + current_y_cluster[l] * current_y_cluster[l] ); 
    current_x_cluster[l] = current_x_cluster[l] - x_mean;   
    current_y_cluster[l] = current_y_cluster[l] - y_mean;   

  }

  float sum_z = std::accumulate(std::begin(current_z_cluster), std::end(current_z_cluster), 0.0);
  z_mean =  sum_z / current_z_cluster.size();
  
  MatrixXf Z(n,4);
  Map<VectorXf> temp2((&current_x_cluster[0]),current_x_cluster.size());
  Map<VectorXf> temp1((&current_z_cluster[0]),current_z_cluster.size());
  Map<VectorXf> temp3((&current_y_cluster[0]),current_y_cluster.size());
  VectorXf temp4(n-1,1);
  temp4 << MatrixXf::Ones(n-1,1);
  
  Z << temp1, temp2, temp3, temp4; 
  
  Matrix<float,Dynamic,Dynamic> M;
  
  M =  (Z.transpose() * Z )/n;

  Matrix<float,4,4> H;
  
  H << 8.0*z_mean,0.0,0.0,2,
       0.0,1.0,0.0,0.0,
       0.0,0.0,1.0,0.0,
       2.0,0.0,0.0,0.0;

 Matrix<float,4,4> H_inv;

 H_inv << 0.0,0.0,0.0,0.5,
          0.0,1.0,0.0,0.0,
          0.0,0.0,1.0,0.0,
          0.5,0.0,0.0,(-2.0*z_mean);

Matrix<float,Dynamic,Dynamic> U;
Matrix<float,Dynamic,Dynamic> V;
std::vector<float> values;

JacobiSVD<MatrixXf> svd(Z, ComputeThinU | ComputeThinV);
U = svd.matrixU();
V = svd.matrixV();

VectorXf temp_(4,1);
MatrixXf ma(4,4);
for(int i =0; i<svd.singularValues().size(); i++)
{
  values.push_back(svd.singularValues()[i]);

}
temp_ << svd.singularValues();
std::sort(values.begin(),values.end());

ma = temp_.asDiagonal();

VectorXf A;

 
if(values[0] < pow(10,-12))
{
  A = V.col(3);
  std::cout<<"yes"<<"\n";
}

else 

{

MatrixXf Y;
MatrixXf Q;

Y = V * ma * V.adjoint();
Q = Y * H_inv * Y;

SelfAdjointEigenSolver<MatrixXf> es(Q);

std::vector<float> temp_a; 

for(int i =0; i<es.eigenvalues().size(); i++)
{
  temp_a.push_back(es.eigenvalues()[i]);

}
int minElementIndex = std::min_element(temp_a.begin(),temp_a.end()) - temp_a.begin();
VectorXf A_star;
A_star = es.eigenvectors().col(minElementIndex);

VectorXf A = Y.colPivHouseholderQr().solve(A_star);

std::cout<<"in else \n";
}

}



}
void circle_or_not_circle(std::vector<std::vector<float>> &x_in, std::vector<std::vector<float>> &y_in)
{


  float first_x,last_x,first_y,last_y;
  std::vector<std::vector<float>> selected_x;
  std::vector<std::vector<float>> selected_y;
  
 
  for (int k =0;k<x_in.size(); k++)
  {
    std::vector<float> angle_list;
    std::vector<float> current_x_cluster = x_in[k];
    std::vector<float> current_y_cluster = y_in[k];
    
//    std::cout<<"size = "<<current_y_cluster.size()<<"\n";
    first_x = *(current_x_cluster.begin());
    last_x  = *(current_x_cluster.end()-1);

    first_y = *(current_y_cluster.begin());
    last_y  = *(current_y_cluster.end()-1);

    for(int l = 1;  l< (current_x_cluster.size()-1); l++ )
    {

      std::cout<<"k = "<< k<<"\n";
      std::cout<<"l = "<< l<<"\n";

      std::cout<< " current x "<<current_x_cluster[l]<<" "<< "current y "<<current_y_cluster[l]<<" "<<"\n";
      std::cout<<"first x "<< first_x<<" "<< "first_y"<< first_y <<"\n";

      std::cout<<"last x "<< last_x<<" "<< "last y"<< last_y <<"\n";

      float y_delta_first =  (current_y_cluster[l] - first_y);
      float x_delta_first = (current_x_cluster[l] - first_x);

      float y_delta_last = (current_y_cluster[l] - last_y);
      float x_delta_last = (current_x_cluster[l] - last_x);
      
      std::cout<<"y_delta_first "<<y_delta_first<<"\n";
      std::cout<<"x_delta_first "<<x_delta_first<<"\n";
      

      std::cout<<"y_delta_last "<<y_delta_last<<"\n";
      std::cout<<"x_delta_last "<<x_delta_last<<"\n";

      float first_ang = atan2(x_delta_first,y_delta_first);
      float last_ang = atan2(x_delta_last,y_delta_last);
      std::cout<<"first ang" <<first_ang<<"\n";
      std::cout<<"last ang" << last_ang <<"\n";

      angle_list.push_back(first_ang-last_ang);      
      std::cout<<"angle = " <<angle_list[l-1]<<"\n";
      
    }

    double sum = std::accumulate(std::begin(angle_list), std::end(angle_list), 0.0);
    double m =  sum / angle_list.size();
    double accum = 0.0;
    std::for_each (std::begin(angle_list), std::end(angle_list), [&](const double d) 
    {
    accum += (d - m) * (d - m);
    });
    double stdev = sqrt(accum / (angle_list.size()));

    if(m<0.0)
    {
      m = m + 2.0*rigid2d::PI;
    }
//    std::cout<<"m out ="<<m<<"\n";

    if((m>=(rigid2d::PI)/2.0) && (m<= (3.0* rigid2d::PI)/4.0)  && stdev <0.15)
    {
      std::cout<<"found a circle \n";
      std::cout<< "itr np = "<< k<<"\n";
      std::cout<<"m = "<<m<<"\n";
      std::cout<<"stddev = "<<stdev<<"\n";

      selected_x.push_back(x_in[k]);
      selected_y.push_back(y_in[k]);
    }

    angle_list.clear();
  }

//-=publish_points(selected_x, selected_y);
}
void scanCallback(const sensor_msgs::LaserScan & scan_in)
{
    std::vector<float> readings;
    std::vector<std::vector<float>> all_x;
    std::vector<std::vector<float>> all_y;

    readings = scan_in.ranges;
    double current_val = readings[0];
    double angle_increment = scan_in.angle_increment;
    double threshold = 0.019; // change as a parameter later
    std::vector<float> temp_x;
    std::vector<float> temp_y;

    for(int i=0; i<readings.size(); i++ )
    {   
       bool flag = false;
    
    while((abs(readings[i] - current_val))<=threshold)
    {
      
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);

       temp_x.push_back(x);
       temp_y.push_back(y);
       i = i + 1;
      
    }
      if(temp_x.size() ==0 )
      {
       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);
       //std::cout<<"x from 2 "<<x<<"\n";
       //std::cout<<"y from 2 "<<y<<"\n";

       temp_x.push_back(x);
       temp_y.push_back(y);
        }
      current_val = readings[i];
      all_x.push_back(temp_x);
      all_y.push_back(temp_y);
      temp_x.clear();
      temp_y.clear();



       float x = readings[i] * cos(i*angle_increment);
       float y = readings[i] * sin(i*angle_increment);
       temp_x.push_back(x);
       temp_y.push_back(y);

      }

    for(int jj=0; jj<all_x.size(); jj++)
    {
      std::vector<float> temp_a = all_x[jj];
      if(temp_a.size()<4)
      {
        all_x.erase(all_x.begin() + jj);
        all_y.erase(all_y.begin() + jj);
        jj=0;
      }
    }

    for( int boredom = 0; boredom<all_x.size(); boredom++)
    {
        for(int shit =0; shit<all_x[boredom].size(); shit++)
        {
        //  std::cout<<"x =" <<all_x[boredom][shit]<<"  ";
        //  std::cout<<"y ="<<all_y[boredom][shit] <<"\n";
          
        }
       // std::cout<<"\n";
    }
   fit_circle(all_x,all_y);
   // circle_or_not_circle(all_x,all_y);
    all_x.clear();
    all_y.clear();
}   




int main(int argc, char **argv)
{
    ros::init(argc,argv,"landmarks");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("scan", 1, scanCallback);
    ros::Rate rate(5);

    while (ros::ok())
    {
    rate.sleep();
    ros::spinOnce();
    }   
}

