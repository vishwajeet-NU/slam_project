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

using namespace Eigen;
VectorXf v;
static std::vector<float> input ;

void fit_curve(MatrixXd mat_in);
void do_clustering(MatrixXd sorted_mat);
void scanCallback(const sensor_msgs::LaserScan & scan_in);

bool compare_head(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
{
    return lhs(0) < rhs(0);
}
void do_clustering(MatrixXd sorted_mat)
{
    double threshold = 0.052;
    int group_number = 1;
    //int counter = 0;
    int current_pose = 0;
    double current_pose_val;
    int next_current_pose = 0;

while(group_number < 17)
{
current_pose = next_current_pose;
for(int i=0;i<=359;i++)
{
current_pose_val = sorted_mat(current_pose,0);
double distance = abs(sorted_mat(i,0)- current_pose_val);
if(distance < threshold)
{

    sorted_mat(i,2)= group_number;
    next_current_pose = i+1; 
}

}

group_number = group_number + 1;
}


//std::cout<<"MAT = "<< sorted_mat << "\n";

fit_curve(sorted_mat);
}

void fit_curve(MatrixXd mat_in)
{
    int current_group_no = 0;
    std::vector<double> cat;
    std::vector<double> index_hold;
    Matrix<double,Dynamic,Dynamic> all_cat ;
    std::vector<Eigen::MatrixXd> all_vecs;    
//      std::vector<std::vector<double>> all_vecs;
//      std::vector<std::vector<int>> all_index;
    while(current_group_no < 17)
    {
        for(int i=0;i<=359;i++)
       {
          // std::cout<<"outer i = "<< i<< "\n";
           while(mat_in(i,2) == current_group_no && i<359)
           {
              // std::cout<<"inner i = "<< i<< "\n";
               cat.push_back(mat_in(i,0));
               index_hold.push_back(mat_in(i,1));
               i=i+1;
           }
        }
        int len = cat.size();
        int len2 = index_hold.size();
        Map<VectorXd> cat_vec((&cat[0]),len);
        Map<VectorXd> len_vec((&index_hold[0]),len2);
   //     all_cat.cols(0) << len_vec;
   //     all_cat.cols(1) << cat_vec;
        all_cat << len_vec, cat_vec;
        current_group_no  = current_group_no + 1;
        all_vecs.push_back(all_cat);
//        all_index.push_back(index_hold);
        
        cat.clear();
        index_hold.clear();
    }

//MatrixXd temp;
//VectorXd t1;
//VectorXd t2;
//std::cout<<"size of big vec"<<all_vecs.size()<<"\n";
//std::cout << "size of 1 ele" << all_vecs[0].size()<<"\n";
//std::copy(all_vecs.begin(), all_vecs.end(), std::ostream_iterator<int>(std::cout, " "));
//    for( int i = 0; i<all_vecs.size(); i++)
//    {
//         std::cout<<"all vecs i = "<< all_vecs[i] << "\n";

//        Map<VectorXd> t1(&(all_index[i][0]),all_index[i][0].size());  

//    }
    
//std::cout<<all_vecs[0]<<"\n";

std::cout<<"i am out"<<"\n";
}




//static Eigen::Matrix<float, Eigen::Dynamic,2> A;
void scanCallback(const sensor_msgs::LaserScan & scan_in)
{
MatrixXd A(360,4);
float temp = 0.0;
input = scan_in.ranges;
//std::sort(input.begin(),input.end());
double angle_increment = 0.01750192232;
double angle =0.0;
for(int i = 0; i<= 359; i++)
{
temp = temp + 1.0;
angle = angle + angle_increment;
A(i,0) = input[i];
A(i,1) = temp;
A(i,2) = 0;
A(i,3) = angle;
}
//std::cout<<"MAT 1= "<< A << "\n";
std::vector<Eigen::VectorXd> vec;
for (int i = 0; i < A.rows(); ++i)
{
    vec.push_back(A.row(i));
}
std::sort(vec.begin(), vec.end(),&compare_head);
for (int i = 0; i < A.rows(); ++i)
{   
    A.row(i) = vec[i];
}
//std::cout<<"MAT 1= "<< A << "\n";

do_clustering(A);
}   


int main(int argc, char **argv)
{
    ros::init(argc,argv,"landmarks");
    ros::NodeHandle n;

    ros::Subscriber laser_sub = n.subscribe("scan", 1, scanCallback);
    ros::Rate rate(60);

    while (ros::ok())
    {
    rate.sleep();
    ros::spinOnce();
    }   
}

