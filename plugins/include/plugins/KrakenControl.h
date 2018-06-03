#ifndef KRAKENCONTROL_H
#define KRAKENCONTROL_H

#include<ros/ros.h>
#include<kraken_msgs/thrusterData6Thruster.h>
#include<gazebo/gazebo.hh>
#include<gazebo/common/Event.hh>
#include<gazebo/common/Plugin.hh>
#include<gazebo/physics/physics.hh>
#include<sensor_msgs/Imu.h>
#include<geometry_msgs/Vector3Stamped.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Matrix3x3.h"


namespace gazebo{
  class GAZEBO_VISIBLE KrakenControlPlugin : public ModelPlugin{
  public:
    KrakenControlPlugin();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Init();

  protected:
    virtual void Update();
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    physics::PhysicsEnginePtr physicsEngine;
    sdf::ElementPtr sdf;
    void thrust6Callback(const kraken_msgs::thrusterData6Thruster::ConstPtr &msg);
    int argc = 0;
    char **argv = NULL;
    ros::Subscriber _thrust6dataSub;
    ros::Publisher DVLPub, PosePub, ImuPub;
  };
}

#endif
