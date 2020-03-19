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

#include <nuslam/turtle_map.h>

// namespace positions{
//       float x_loc = 0.0;
//       float y_loc = 0.0;
//       float orient = 0.0;
// }

static double alpha1 =0.1, alpha2 =0.1, alpha3 = 0.1, alpha4 =0.1;

static double sigma_r = 0.1 , sigma_theta = 0.1, sigma_landmark = 0.1;
static double incoming_left_wheel;
static double incoming_right_wheel;
static double left_wheel = 0;
static double right_wheel = 0;

// static bool called = false;

std::vector<float> x_center ={0.1};
std::vector<float> y_center ={0.1}; 

static double x_known = 0.719942;
static double y_known = -0.02845;

void landmarks(const nuslam::turtle_map &coordinates)
{
    x_center = coordinates.x_center;
//     std::cout<<"x_cent = "<<x_center[0] <<"\n";
//     std::cout<<"y cent ="<<y_center[0]<<"\n";

    y_center = coordinates.y_center;
//    gotreading = true;
}




 void jt_callback(const sensor_msgs::JointState JT)
 {
       incoming_left_wheel= JT.position[0];
       incoming_right_wheel= JT.position[1];

 }

// bool do_teleport(rigid2d::telep::Request  &req, rigid2d::telep::Response &res)
// {     

//      positions::x_loc = req.x;
//      positions::y_loc = req.y;
//      positions::orient = req.theta; 
//      called = true;
//      return true;
// }

int main(int argc, char** argv)
{
      ros::init(argc, argv, "slam");
      ros::NodeHandle n;
      std::string world;
      std::string base;
      n.getParam("world", world);
      n.getParam("base", base);
      

  //    ros::ServiceServer service = n.advertiseService("/set_pose", do_teleport);
      Twist2D starting_position;
      starting_position.v_x =0.0;
      starting_position.v_y =0.0;
      starting_position.w =0.0;

      DiffDrive turtle_odo(starting_position,0.160,0.033);


      Twist2D Vb;

//      ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);

      ros::Subscriber joint_state_subsciber = n.subscribe("/joint_states", 1, jt_callback);
      ros::Subscriber read_landmarks = n.subscribe("landmarks", 1, landmarks);


//      tf::TransformBroadcaster odom_broadcaster;

      // double x = 0.0;
      // double y = 0.0;
      // double th = 0.0;
//      ros::Time current_time, last_time;
//      current_time = ros::Time::now();
//      last_time = ros::Time::now();
      ros::Rate r(100);
      while(n.ok())
      {
            ros::spinOnce();               // check for incoming messages
//             if(called)
//             {
//                   starting_position.v_x =positions::x_loc;
//                   starting_position.v_y =positions::y_loc;
//                   starting_position.w =positions::orient;    
//                   turtle_odo.reset(starting_position);
//                   called = false;
//             }

// //            current_time = ros::Time::now();
            left_wheel = incoming_left_wheel- left_wheel;
            right_wheel = incoming_right_wheel - right_wheel;


            Twist2D body_v;
            WheelVelocities body_in; 
            body_in.U1 = left_wheel;
            body_in.U2 = right_wheel;
            body_v = turtle_odo.wheelsToTwist(body_in);
    
            Eigen::MatrixXd mu_t_bar(3,1);
            Eigen::MatrixXd mu_t_prior(3,1);
            Eigen::MatrixXd mu_t_update(3,1);
            Eigen::MatrixXd g_t(3,3);
            Eigen::MatrixXd m_t(2,2);
            Eigen::MatrixXd v_t(3,2);
            Eigen::MatrixXd pose_var_prior(3,3);
            Eigen::MatrixXd pose_var_bar(3,3);
            pose_var_prior <<0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01,0.01;

            mu_t_prior << turtle_odo.position.v_x,turtle_odo.position.v_y,turtle_odo.position.w;
            mu_t_update<< body_v.v_x*cos(turtle_odo.position.w + body_v.w/2) , body_v.v_x*sin(turtle_odo.position.w + body_v.w/2) , body_v.w;
            g_t<<1,0,-body_v.v_x*sin(turtle_odo.position.w+body_v.w/2),0,1,body_v.v_x*cos(turtle_odo.position.w+body_v.w/2),0,0,1;
            m_t<<alpha1*body_v.v_x * body_v.v_x + alpha2 * body_v.w*body_v.w , 0,0, alpha3*body_v.v_x * body_v.v_x + alpha4 * body_v.w*body_v.w;
            v_t<< cos(turtle_odo.position.w + body_v.w/2) , (-0.5)* sin(turtle_odo.position.w + body_v.w/2) , sin(turtle_odo.position.w + body_v.w/2)  , 0.5* cos(turtle_odo.position.w + body_v.w/2),0,1;

            mu_t_bar = mu_t_prior + mu_t_update;
            // std::cout<<"mu_t_bar = "<<mu_t_bar<<"\n";
            // std::cout<<"g_t = "<<g_t<<"\n";
            // std::cout<<"m_t = "<<m_t<<"\n";
            // std::cout<<"v_t = "<<v_t<<"\n";

            pose_var_bar = g_t*pose_var_prior*g_t.transpose() + v_t*m_t*v_t.transpose();
            
            // std::cout<<"x_center = "<< x_center[0]<<"\n";
            // std::cout<<"y_center = "<< y_center[0]<<"\n";

            // std::cout<<"mu11"<<mu_t_bar.coeff(1,1) <<"\n";
            // std::cout<<"mu21"<<mu_t_bar.coeff(2,1) <<"\n";
            // std::cout<<"mu31"<<mu_t_bar.coeff(3,1) <<"\n";

            double range = sqrt((x_center[0] - mu_t_bar.coeff(1,1)) * (x_center[0] - mu_t_bar.coeff(1,1)) + (y_center[0] - mu_t_bar.coeff(2,1)) * (y_center[0] - mu_t_bar.coeff(2,1)));
            double bearing = (atan2((y_center[0] - mu_t_bar.coeff(2,1)) ,(x_center[0] - mu_t_bar.coeff(1,1))) - mu_t_bar.coeff(3,1));

            // std::cout<<"range = "<<range<<"\n";
            // std::cout<<"bearing = "<< bearing <<"\n";

            double known_range = sqrt((x_known - mu_t_bar.coeff(1,1)) * (x_known - mu_t_bar.coeff(1,1)) + (y_known - mu_t_bar.coeff(2,1)) * (y_known - mu_t_bar.coeff(2,1)));
            double known_bearing = (atan2((y_known - mu_t_bar.coeff(2,1)) ,(x_known - mu_t_bar.coeff(1,1))) - mu_t_bar.coeff(3,1));

            // std::cout<<"known_range = "<<known_range<<"\n";
            // std::cout<<"known_bearing = "<< known_bearing <<"\n";

            Eigen::MatrixXd z_t(3,1); 
            z_t << range, bearing, 1;

            // std::cout<<"z_t = "<< z_t<<"\n";

            Eigen::MatrixXd h_t(3,3);
            h_t << -(x_center[0]-mu_t_bar.coeff(1,1)/range), -(y_center[0]-mu_t_bar.coeff(2,1)/range),0, (y_center[0]-mu_t_bar.coeff(2,1)/(range*range)), (x_center[0]-mu_t_bar.coeff(1,1)/(range*range)) , -1, 0,0,0 ;

            // std::cout<<"h_t = "<< h_t<<"\n";
            
            Eigen::MatrixXd s_t(3,3);
            Eigen::MatrixXd q_t(3,3);

            q_t<< sigma_r*sigma_r,0,0,0,sigma_theta*sigma_theta,0,0,0,sigma_landmark*sigma_landmark;

//            std::cout<<"q_t = "<< q_t<<"\n";

            s_t = h_t * pose_var_bar * h_t.transpose() + q_t;

            // std::cout<<"s_t = "<< s_t<<"\n";

            Eigen::MatrixXd k_gain(3,3);
            k_gain = pose_var_bar * h_t.transpose() * s_t.inverse();
            
            // std::cout<<"k_gain = "<< k_gain<<"\n";

            Eigen::MatrixXd mu_posterior(3,1);
            Eigen::MatrixXd pose_var_posterior(3,3);
            Eigen::MatrixXd I(3,3);
            I.Identity(3,3);
            Eigen::MatrixXd z_expected(3,1);
            z_expected<<known_range,known_bearing,1;

            // std::cout<<"z_expected = "<< z_expected<<"\n";

            mu_posterior = mu_t_bar + k_gain * ( z_expected - z_t);
            pose_var_posterior = (I - k_gain * h_t) * pose_var_bar;

             std::cout<<"mu_posterior = "<< mu_posterior<<"\n";
            // std::cout<<"pose_var_posterior = "<< pose_var_posterior<<"\n";
            
            // std::cout<<"stuff is happening"<<"\n";

            r.sleep();
     }
    
 
}






//            turtle_odo.feedforward(left_wheel, right_wheel);

            // WheelVelocities temp_wheel;
            // temp_wheel.U1 = left_wheel;
            // temp_wheel.U2 = right_wheel;


            // left_wheel = incoming_left_wheel;
            // right_wheel = incoming_right_wheel;

            // Twist2D current_position;
            // current_position = turtle_odo.pose();

            // x = current_position.v_x;
            // y = current_position.v_y;
            // th = current_position.w;
            // std::cout<<"x"<<x<<"\n";
            // std::cout<<"y"<<y<<"\n";
            // std::cout<<"theta"<<th<<"\n";
            
            // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);   
            
            // geometry_msgs::TransformStamped odom_trans;
            // odom_trans.header.stamp = current_time;
            // odom_trans.header.frame_id = world;
            // odom_trans.child_frame_id = base;
   
            // odom_trans.transform.translation.x = x;
            // odom_trans.transform.translation.y = y;
            // odom_trans.transform.translation.z = 0.0;
            // odom_trans.transform.rotation = odom_quat;
   
            // odom_broadcaster.sendTransform(odom_trans);
   
            // nav_msgs::Odometry odom;
            // odom.header.stamp = current_time;
            // odom.header.frame_id = "odom";
   
            // odom.pose.pose.position.x = x;
            // odom.pose.pose.position.y = y;
            // odom.pose.pose.position.z = 0.0;
            // odom.pose.pose.orientation = odom_quat;

            // odom.child_frame_id = "base_link";
            // Vb = turtle_odo.wheelsToTwist(temp_wheel);
            // odom.twist.twist.linear.x = Vb.v_x;
            // odom.twist.twist.linear.y = Vb.v_y;
            // odom.twist.twist.angular.z = Vb.w;
   
            // odom_pub.publish(odom);
   
            // last_time = current_time;