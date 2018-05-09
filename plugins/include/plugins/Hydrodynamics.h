#ifndef HYDRODYNAMICS_H
#define HYDRODYNAMICS_H

#include "gazebo/common/Event.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/physics.hh"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "gazebo/common/Assert.hh"
#include "map"
namespace gazebo{
  class VolumeProperties
  {
    public: VolumeProperties() : volume(0) {}
    public: math::Vector3 cov;
    public: double volume;
  };
  class GAZEBO_VISIBLE Hydrodynamics : public ModelPlugin{
  public:
    Hydrodynamics();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Init();
    double volume;
    double fluidDensity;
    math::Vector3 cov;

  protected:
    virtual void Update();
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    physics::PhysicsEnginePtr physicsEngine;
    sdf::ElementPtr sdf;
    geometry_msgs::Twist COM_twist;
    double Damping[6] = {0.0, 0.0, 0.0, -130.0, -130.0, -130.0};
    double quadratic_Damping[6] = {-148.0, -148.0, -148.0, -180.0, -180.0, -180.0};
    void Damping_matrix(double *damp);
    void apply_Dampingforce(double *damp);
    void getTwist();
    void Buoyancy();
    std::map<int, VolumeProperties> volPropsMap;

  };
}

#endif
