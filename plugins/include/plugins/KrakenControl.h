#ifndef KRAKENCONTROL_H
#define KRAKENCONTROL_H

#include<ros/ros.h>
#include<msgs_stack/thrusterData6.h>
#include<gazebo/gazebo.hh>
#include<gazebo/common/Event.hh>
#include<gazebo/common/Plugin.hh>
#include<gazebo/physics/physics.hh>


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
    void thrust6Callback(const msgs_stack::thrusterData6::ConstPtr &msg);
    int argc = 0;
    char **argv = NULL;
    ros::Subscriber _thrust6dataSub;
    ros::Publisher DVLPub, PosePub;
  };
}

#endif
