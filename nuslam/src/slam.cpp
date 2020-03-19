/// \file
/// \brief finds odometry of the robot and publishes it 

/// Publishers : odom
/// Subscribers : jointstate

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include"rigid2d/waypoints.hpp"
#include"rigid2d/telep.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>    // std::sort
#include <vector>
#include <turtlesim/Pose.h> 

#include <nuslam/turtle_map.h>
#include <geometry_msgs/PoseStamped.h> 
#include <nav_msgs/Path.h> 

#include<random>

 std::mt19937 & get_random()
 {
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     return mt;
 }

bool start = true;

namespace positions{
      float x_loc = 0.0;
      float y_loc = 0.0;
      float orient = 0.0;
}
namespace turtle_odom_pose{
    float x = 0.0;
    float y = 0.0;
    float theta = 0.0;
    geometry_msgs::Quaternion quat; 
    
}
static double sigma_r = 0 , sigma_theta = 0, sigma_landmark = 0;
static double r_param;
static double incoming_left_wheel;
static double incoming_right_wheel;
static double left_wheel = 0;
static double right_wheel = 0;

static double x_start = 0.0;
static double y_start= 0.0;
static double theta_start = 0.0; // change these later , need to accomodate different start positions?
static bool called = false;

static int m_land =12 ;      // number of landmarks 
static int n_land = m_land * 2;   // 2m for x and y location of landmark
static int total = n_land + 3;

static double co_var_w;
static double co_var_v;
std::vector<float> x_center;
std::vector<float> y_center; 

static double roll, pitch, yaw;


bool land_called = false;
bool odom_call = false;
void landmarks(const nuslam::turtle_map &coordinates)
{
    x_center = coordinates.x_center;

    y_center = coordinates.y_center;
    land_called = true;
}

float wrap_angles(float incoming_angle)
{
    if(incoming_angle>=(rigid2d::PI))
    {
        incoming_angle = incoming_angle - 2.0*rigid2d::PI;
    }
    else
    {
        incoming_angle = incoming_angle;
    }
    return incoming_angle;
}

 void jt_callback(const sensor_msgs::JointState JT)
 {
       incoming_left_wheel= JT.position[0];
       incoming_right_wheel= JT.position[1];
 }

bool do_teleport(rigid2d::telep::Request  &req, rigid2d::telep::Response &res)
{     

     positions::x_loc = req.x;
     positions::y_loc = req.y;
     positions::orient = req.theta; 
     called = true;
     return true;
}

void pose_odom_Callback(const nav_msgs::Odometry &odom_pose){
   turtle_odom_pose::x = odom_pose.pose.pose.position.x ;
   turtle_odom_pose::y = odom_pose.pose.pose.position.y;
   turtle_odom_pose::quat = odom_pose.pose.pose.orientation;

   double quatx= odom_pose.pose.pose.orientation.x;
   double quaty= odom_pose.pose.pose.orientation.y;
   double quatz= odom_pose.pose.pose.orientation.z;
   double quatw= odom_pose.pose.pose.orientation.w;
   tf::Quaternion q(quatx, quaty, quatz, quatw);
   tf::Matrix3x3 m(q);
   m.getRPY(roll, pitch, yaw);
   odom_call = true;
}


int main(int argc, char** argv)
{
      ros::init(argc, argv, "slam");
      ros::NodeHandle n;

     ros::param::get("sigma_r",sigma_r);
     ros::param::get("sigma_theta",sigma_theta);
     ros::param::get("sigma_landmark",sigma_landmark);
     ros::param::get("co_var_w",co_var_w);
     ros::param::get("co_var_v",co_var_v);
     ros::param::get("r_param",r_param);

     geometry_msgs::PoseStamped current_pose_map;
     nav_msgs::Path path_taken_map;

    ros::ServiceServer service = n.advertiseService("/set_pose", do_teleport);
    Twist2D starting_position;
    starting_position.v_x =0.0;
    starting_position.v_y =0.0;
    starting_position.w =0.0;
    DiffDrive turtle_odo(starting_position,0.160,0.033);

    Twist2D Vb;
    ros::Publisher path_pub_map = n.advertise<nav_msgs::Path>("/path_map", 1);
    ros::Subscriber joint_state_subsciber = n.subscribe("/joint_states", 1, jt_callback);
    ros::Subscriber read_landmarks = n.subscribe("landmarks", 1, landmarks);
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, pose_odom_Callback);

    tf::TransformBroadcaster map_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();


      Eigen::MatrixXd mu_t_bar(total,1);
      mu_t_bar.fill(0.0);
      Eigen::MatrixXd mu_t_posterior(total,1);
      Eigen::MatrixXd w_t(3,1);
      Eigen::MatrixXd v_t(2,1);
      Eigen::MatrixXd g_t(total,total);
      g_t.fill(0.0);
      g_t = g_t + Eigen::MatrixXd::Identity(total,total); 
      
      Eigen::MatrixXd q_t(total,total);
      q_t.fill(0.0);
      q_t.block(0,0,3,3) << sigma_r*sigma_r,0,0,0,sigma_theta*sigma_theta,0,0,0,sigma_landmark*sigma_landmark;
     
      Eigen::MatrixXd pose_var_prior(total,total);
      Eigen::MatrixXd pose_var_bar(total,total);
      Eigen::MatrixXd pose_var_posterior(total,total);
      pose_var_prior.fill(0.0);
      Eigen::MatrixXd temp_covar(n_land,n_land);
      Eigen::VectorXd temp_temp(n_land);
      temp_temp.fill(10000);
      temp_covar.fill(0.0);
      temp_covar = temp_temp.asDiagonal(); 
      pose_var_prior.bottomRightCorner(n_land,n_land) = temp_covar;

      Eigen::MatrixXd z_t(2,1);
      Eigen::MatrixXd small_h(2,1);
      Eigen::MatrixXd large_h(2,total);
      large_h.fill(0.0);

      Eigen::MatrixXd k_gain(total,2);
      Eigen::MatrixXd R(2,2);
      R << r_param,r_param,r_param,r_param; //parametrize this later

      ros::Rate r(100);
      while(n.ok())
      {
          ros::spinOnce();               // check for incoming messages
          if(called)
          {
              starting_position.v_x =positions::x_loc;
              starting_position.v_y =positions::y_loc;
              starting_position.w =positions::orient;    
              turtle_odo.reset(starting_position);
              called = false;
          }
          
          if(land_called && odom_call)
          {

              if(start)
              {
                  for(int i =0; i<m_land; i++)
                  {
                      mu_t_bar.row(2+2*(i+1)-1) << x_center[i];
                      mu_t_bar.row(2+2*(i+1)) << y_center[i];
                  }
                //   std::cout<<"starting mu_t = \n"<<mu_t_bar<<"\n";
                start = false;  
              }
              left_wheel = incoming_left_wheel- left_wheel;
              right_wheel = incoming_right_wheel - right_wheel;

              Twist2D body_v;
              WheelVelocities body_in; 
              body_in.U1 = left_wheel;
              body_in.U2 = right_wheel;
              body_v = turtle_odo.wheelsToTwist(body_in);

             left_wheel = incoming_left_wheel;
             right_wheel = incoming_right_wheel;

            theta_start = mu_t_bar.coeff(0,0);
            x_start = mu_t_bar.coeff(1,0);
            y_start = mu_t_bar.coeff(2,0);

            //    std::cout<<"x_start going in = "<<x_start<<"\n";
            //    std::cout<<"y_start going in = "<<y_start<<"\n";

            //    std::cout<<"w pos = "<<turtle_odo.position.w<<"\n"; 
              if(body_v.w == 0)
              {   
                  x_start = x_start + body_v.v_x*cos(theta_start);
                  y_start = y_start + body_v.v_x*sin(theta_start);

              }
              else
              {
                  x_start = x_start - body_v.v_x*sin(theta_start)/body_v.w + body_v.v_x*sin(body_v.w + theta_start)/body_v.w;
                  y_start = y_start + body_v.v_x*cos(theta_start)/body_v.w - body_v.v_x*cos(body_v.w + theta_start)/body_v.w;
                  theta_start = theta_start + body_v.w;
                  
              }

            //    std::cout<<"x_start coming out = "<<x_start<<"\n";
            //    std::cout<<"y_start coming out = "<<y_start<<"\n";
            // std::cout<<"mu_t_bar before << = "<<mu_t_bar<<"\n";
            mu_t_bar.row(0) << theta_start;
            mu_t_bar.row(1) << x_start;
            mu_t_bar.row(2) << y_start;
// std::cout<<"xstart"<<x_start<<"\n";
// std::cout<<"ystart"<<y_start<<"\n";
// std::cout<<"thetastart"<<theta_start<<"\n";

            // std::cout<<"mu_t_bar after << = "<<mu_t_bar<<"\n";

            std::normal_distribution<double> d(0.0, co_var_w);
            std::normal_distribution<double> e(0.0, co_var_v);

            std::vector<double> random_nos_w;
            std::vector<double> random_nos_v;
            for( int i = 0; i < 3 ;i++)
            {
             random_nos_w.push_back(d(get_random()));   
             
            }

            for( int i = 1; i < 3 ;i++)
            {
                random_nos_v.push_back(e(get_random()));   
            }
             

            Eigen::Map<Eigen::VectorXd> temp_rand_w((&random_nos_w[0]),random_nos_w.size());
            Eigen::Map<Eigen::VectorXd> temp_rand_v((&random_nos_v[0]),random_nos_v.size());

            w_t << temp_rand_w;
            v_t << temp_rand_v;


            mu_t_bar.row(0) = mu_t_bar.row(0) + w_t.row(0);
            mu_t_bar.row(1) = mu_t_bar.row(1) + w_t.row(1);
            mu_t_bar.row(2) = mu_t_bar.row(2) + w_t.row(2);


            Eigen::VectorXd temp_for_gt(total,1);
            temp_for_gt.fill(0.0);
            if(body_v.w == 0)
            {   
                temp_for_gt.row(1) <<  -body_v.v_x * sin(theta_start);
                temp_for_gt.row(2) <<  body_v.v_x * cos(theta_start);
            }
            else
            {
                temp_for_gt.row(1)<< (body_v.v_x * (-cos(theta_start) + cos(theta_start + body_v.w)))/body_v.w;
                temp_for_gt.row(2)<< (body_v.v_x * (-sin(theta_start) + sin(theta_start + body_v.w)))/body_v.w;
            }

            g_t.col(0) << temp_for_gt;

             std::cout<<"g_t \n= "<<g_t<<"\n \n"; 

   
            std::cout<<"mu_t_bar before loop\n"<<mu_t_bar<<"\n \n";
            std::cout<<"pose_var_prior before loop \n ="<<pose_var_prior<<"\n \n";
            
            for( int i = 0; i<m_land; i++ )
            {
                 pose_var_bar = g_t*pose_var_prior*(g_t.transpose()) + q_t ;
      
                 std::cout<<"landmark number = "<<i<<"\n ";
                // std::cout<<"x val of land"<<x_center[i]<<"\n";
                // std::cout<<"y val of land"<<y_center[i]<<"\n";
              std::cout<<"mu_t_bar in loop \n"<<mu_t_bar<<"\n \n";
            
               std::cout<<"pose_var_bar in loop \n="<<pose_var_bar<<"\n \n";
                
/// expected values z   
                 double y_del = (mu_t_bar.coeff(2*(i+1)+2,0) - mu_t_bar.coeff(2,0)); 
                 double x_del = (mu_t_bar.coeff(2*(i+1)+1,0) - mu_t_bar.coeff(1,0));
                  std::cout<<"x_del ="<< x_del <<"\n";
                  std::cout<<"y_del ="<< y_del <<"\n";


                 double known_range = sqrt(x_del * x_del + y_del * y_del); 
                 double known_bearing = ( wrap_angles(atan2(y_del,x_del)) - wrap_angles(mu_t_bar.coeff(0,0)));
                 
                std::cout<<"known_range = "<<known_range<<"\n"; 
                std::cout<<"known_bearing = "<<known_bearing<<"\n"; 

                 
/// sensor values z_hat
                double range = sqrt((x_center[i] - mu_t_bar.coeff(1,0)) * (x_center[i] - mu_t_bar.coeff(1,0)) + (y_center[i] - mu_t_bar.coeff(2,0)) * (y_center[i] - mu_t_bar.coeff(2,0)));
                double bearing = (wrap_angles(atan2((y_center[i] - mu_t_bar.coeff(2,0)) ,(x_center[i] - mu_t_bar.coeff(1,0))))  - wrap_angles(mu_t_bar.coeff(0,0)));


                std::cout<<"range = "<<range<<"\n"; 
                std::cout<<"bearing = "<<bearing<<"\n"; 

                small_h.row(0) << range;
                small_h.row(1) << wrap_angles( bearing);

                small_h = small_h + v_t;

                 std::cout<<"small_h = "<<small_h<<"\n";  

                double x_delta = mu_t_bar.coeff(1,0) - x_center[i];
                double y_delta = mu_t_bar.coeff(2,0) - y_center[i];

                double delta = x_delta * x_delta + y_delta * y_delta;

                std::cout<<"x_delta = "<<x_delta<<"\n"; 

                std::cout<<"y_delta = "<<y_delta<<"\n"; 
                std::cout<<"delta = "<<delta<<"\n"; 
                large_h.col(0) << 0,-1;
                large_h.col(1) << (-x_delta/sqrt(delta)), (y_delta/(delta));
                large_h.col(2) << (-y_delta/sqrt(delta)), -(x_delta/(delta));
                large_h.col(2+2*(i+1)-1) << (x_delta/sqrt(delta)), (-y_delta/delta);
                large_h.col(2+2*(i+1)) << (y_delta/sqrt(delta)),(x_delta/delta);

                std::cout<<"large h \n="<<large_h<<"\n \n";
                // for(int i =0; i<m_land; i++)
                // {
                //     std::cout<<"i = "<< i <<"\t";
                //     std::cout<<"large_h = "<< large_h.col(i)<<"\n";
                // }
                

                z_t << known_range,wrap_angles(known_bearing);

                z_t(1) = wrap_angles(z_t(1));
                std::cout<<"z_t = \n"<<z_t<<"\n"; 
                std::cout<<"small_h = \n"<<small_h<<"\n";   
                Eigen::MatrixXd diff(2,1);
                diff = z_t - small_h;
                diff(1) = wrap_angles(diff(1));
                std::cout<<"diff= "<<diff<<"\n";
                k_gain = pose_var_bar * large_h.transpose() * (large_h * pose_var_bar * large_h.transpose() + R ).inverse();
        
                std::cout<<"k_gain = \n"<<k_gain<<"\n"; 

                mu_t_posterior = mu_t_bar + k_gain* diff;

                std::cout<<"mu_t_posterior = \n"<<mu_t_posterior<<"\n \n"; 


                Eigen::MatrixXd identity_temp;
                identity_temp.setIdentity(total,total);
                pose_var_posterior = (identity_temp - k_gain*large_h) * pose_var_bar;

                 std::cout<<"pose_var_posterior = \n"<<pose_var_posterior<<"\n \n"; 
            mu_t_bar = mu_t_posterior;
            pose_var_prior = pose_var_posterior;
                
            }   
            // std::cout<<"mu posteriro"<<mu_t_posterior<<"\n";
            k_gain.fill(0.0);
            large_h.fill(0.0);

            geometry_msgs::Quaternion map_quat = tf::createQuaternionMsgFromYaw(wrap_angles(mu_t_bar.coeff(0,0)) - wrap_angles(yaw));   
            
            geometry_msgs::TransformStamped map_trans;
            map_trans.header.stamp = current_time;
            map_trans.header.frame_id = "map";
            map_trans.child_frame_id = "odom";


            // std::cout<<"coeff x = "<<mu_t_bar.coeff(1,0) <<"\n";
            // std::cout<<"coeff y = "<<mu_t_bar.coeff(2,0) <<"\n";
            // std::cout<<"x = "<<x<<"\n";
            // std::cout<<"y = "<<y<<"\n";
        
            map_trans.transform.translation.x = mu_t_bar.coeff(1,0) - turtle_odom_pose::x;
            map_trans.transform.translation.y = mu_t_bar.coeff(2,0) - turtle_odom_pose::y;
            map_trans.transform.translation.z = 0.0;
            map_trans.transform.rotation = map_quat;
   
            map_broadcaster.sendTransform(map_trans);

            current_pose_map.header.stamp = ros::Time::now();
            current_pose_map.header.frame_id = "map";

            path_taken_map.header.stamp = ros::Time::now();
            path_taken_map.header.frame_id = "map";


            current_pose_map.pose.position.x = mu_t_bar.coeff(1,0);
            current_pose_map.pose.position.y = mu_t_bar.coeff(2,0);
            current_pose_map.pose.orientation = map_quat;
            
            path_taken_map.poses.push_back(current_pose_map);
            path_pub_map.publish(path_taken_map);


            land_called = false;
            last_time = current_time;

      }
          r.sleep();

     }

    
 
}




// void do_odometry(WheelVelocities &temp_wheel, DiffDrive &turtle_odo,ros::Publisher odom_pub ,tf::TransformBroadcaster odom_broadcaster)
// {   
//     ros::NodeHandle n;
//     ros::Time current_time;

//     current_time = ros::Time::now();
//     Twist2D Vb;
//     Twist2D current_position;
//     current_position = turtle_odo.pose(); 
//     x = current_position.v_x;
//     y = current_position.v_y;
//     th = current_position.w;
// //          th = wrap_angles(th);
//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);   
    
//     geometry_msgs::TransformStamped odom_trans;
//     odom_trans.header.stamp = current_time;
//     odom_trans.header.frame_id = "odom";
//     odom_trans.child_frame_id ="base_link";

//     odom_trans.transform.translation.x = x;
//     odom_trans.transform.translation.y = y;
//     odom_trans.transform.translation.z = 0.0;
//     odom_trans.transform.rotation = odom_quat;   
//     odom_broadcaster.sendTransform(odom_trans);
   
//     nav_msgs::Odometry odom;
//     odom.header.stamp = current_time;
//     odom.header.frame_id = "odom";
//     odom.pose.pose.position.x = x;
//     odom.pose.pose.position.y = y;
//     odom.pose.pose.position.z = 0.0;
//     odom.pose.pose.orientation = odom_quat;
//     odom.child_frame_id = "base_link";
//     Vb = turtle_odo.wheelsToTwist(temp_wheel);
//     odom.twist.twist.linear.x = Vb.v_x;
//     odom.twist.twist.linear.y = Vb.v_y;
//     odom.twist.twist.angular.z = Vb.w;
//     odom_pub.publish(odom);
//     ros::spinOnce();               // check for incoming messages

// }
