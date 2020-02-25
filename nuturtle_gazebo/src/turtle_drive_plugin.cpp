#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include"rigid2d/rigid2d.hpp"
#include"rigid2d/diff_drive.hpp"
#include <boost/bind.hpp>
#include <gazebo/common/common.hh>
#include <gazebo/physics/Model.hh>

#include <gazebo/physics/Link.hh>

#include <gazebo/physics/World.hh>

#include <gazebo/physics/Joint.hh>

#include <gazebo/physics/PhysicsIface.hh>

#include <gazebo/physics/PhysicsEngine.hh>


#define LEFT_WHEEL_JOINT  0
#define RIGHT_WHEEL_JOINT 1


namespace gazebo
{
  class ModelPush : public ModelPlugin
  {

    private: std::unique_ptr<ros::NodeHandle> rosNode;
    private: ros::Subscriber wheel_sub;
    private: ros::Publisher sensor_pub;
    private: double power = 0.0;
    private: double rvel = 0.0;
    private: double ticks = 0.0;
    private: rigid2d::WheelVelocities incoming_speeds;
    private: common::Time prevUpdateTime;
    private: common::Time currTime;
    

//parent
    public: std::string wheel_cmd;
    public: std::string sensor_data; 
    public: std::string right_wheel_joint;
    public: std::string left_wheel_joint;
    public: double sf = 0.0; 
    public: double update_period = 0.0;



    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
      // Store the pointer to the model
      this->model = _parent;


      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      if (!ros::isInitialized())
    {
      
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    ROS_INFO("Hello World!"); 


    if (_sdf->HasElement("wheel_cmd_topic"))
     {
       wheel_cmd = _sdf->GetElement("wheel_cmd_topic")->Get<std::string>();
     }
     else
     {
       wheel_cmd = "standard";
     }
    if (_sdf->HasElement("sensor_data_topic"))
     {
       sensor_data = _sdf->GetElement("sensor_data_topic")->Get<std::string>();
     }
     else
     {
       sensor_data = "standard";
     }
    if (_sdf->HasElement("right_wheel_joint"))
     {
       right_wheel_joint = _sdf->GetElement("right_wheel_joint")->Get<std::string>();
     }
     else
     {
       ROS_FATAL("no right wheel joint");
      ros::shutdown();
     }
    if (_sdf->HasElement("left_wheel_joint"))
     {
       left_wheel_joint = _sdf->GetElement("left_wheel_joint")->Get<std::string>();
     }
     else
     {
       ROS_FATAL("no left wheel joint");
      ros::shutdown();
     }

    if (_sdf->HasElement("sensor_frequency"))
     {
       sf = std::stod(_sdf->GetElement("sensor_frequency")->Get<std::string>());
     }
     else
     {
       sf = 200.0;
     }

    update_period= 1/sf;
    std::cout<< "sf = "<<sf<<"\n";
    std::cout<< "update period = "<<update_period<<"\n";

    this->prevUpdateTime = this->model->GetWorld()->SimTime();

    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
    this->wheel_sub= this->rosNode->subscribe(wheel_cmd, 1, &ModelPush::callback_wheel_cmd, this);
    this->sensor_pub= this->rosNode->advertise<nuturtlebot::SensorData>(sensor_data, 1);
 
    this->rosNode->getParam("/max_motor_power",this->power);
    this->rosNode->getParam("/max_rvel_motor",this->rvel);
    this->rosNode->getParam("/encoder_ticks_per_rev",this->ticks);


    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&ModelPush::OnUpdate, this));

//    double last_time = common::Time::GetWallTime();
   // std::cout<<"last time = "<< last_time<<"\n";
    }


    public: void callback_wheel_cmd(const nuturtlebot::WheelCommands &wheelcmd)
    {
      this->incoming_speeds.U1 = wheelcmd.left_velocity * rvel/power ;
      this->incoming_speeds.U2 = wheelcmd.right_velocity * rvel/power ;

    }
    
    // Called by the world update start event
    public: void OnUpdate()
    {
    // model->GetLink("body_link")->SetLinearVel({0.1, 0, 0});
    this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0, 100.0);
    this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0, this->incoming_speeds.U1);

    this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0, 100.0);
    this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0, this->incoming_speeds.U2);

    double right = this->model->GetJoint(right_wheel_joint)->Position(0);
    double left = this->model->GetJoint(left_wheel_joint)->Position(0);

    nuturtlebot::SensorData msg;
    msg.right_encoder = right * ticks/(2*rigid2d::PI);
    msg.left_encoder = left * ticks/(2*rigid2d::PI);
    
    
    common::Time currTime = this->model->GetWorld()->SimTime();
    common::Time stepTime = currTime - this->prevUpdateTime;

    std::cout<<"step = "<<stepTime.Double()<<"\n";
    if ( stepTime.Double() > update_period ) 
    {
      this->sensor_pub.publish(msg);
      this->prevUpdateTime = currTime;

    }

    
    }

 
    // Pointer to the model
    private: physics::ModelPtr model;

    private: physics::ModelPtr joint; 

    

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}