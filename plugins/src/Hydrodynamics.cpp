#include "plugins/Hydrodynamics.h"
#include "iostream"
#include "string"
using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(Hydrodynamics)

Hydrodynamics::Hydrodynamics(){}

void Hydrodynamics::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  this->model = _model;
  this->sdf = _sdf;
  physics::WorldPtr world = _model->GetWorld();
  this->physicsEngine = world->GetPhysicsEngine();
  GZ_ASSERT(world != NULL, "MODEL IS IN A NULL WORLD");
  GZ_ASSERT(this->physicsEngine != NULL, "PHYSICS ENGINE IS NULL");
  GZ_ASSERT(_sdf != NULL, "RECEIVED NULL SDF POINTER");

  if(this->sdf->HasElement("fluid_density")){
    this->fluidDensity = this->sdf->Get<double>("fluid_density");
  }
  for(int i = 0; i < this->sdf->Get<int>("no_of_links"); i++){
    if(this->sdf->HasElement("link"+ std::to_string(i))){
      sdf::ElementPtr linkElem = this->sdf->GetElement("link"+ std::to_string(i));
      int id = -1;
      std::string name = "";
      if(linkElem->HasAttribute("name")){
          name = linkElem->Get<std::string>("name");
          physics::LinkPtr link = this->model->GetLink(name);
          id = link->GetId();
        }
      if(linkElem->HasElement("center_of_volume")){
          math::Vector3 cov = linkElem->GetElement("center_of_volume")->Get<math::Vector3>();
          this->volPropsMap[id].cov = cov;
          std::cout<<cov<<" ";
        }
      if(linkElem->HasElement("volume")){
          double volume = linkElem->GetElement("volume")->Get<double>();
          this->volPropsMap[id].volume = volume;
          std::cout<<volume<<"\n";
        }
        std::cout<<linkElem->Get<std::string>("name")<<"\n";
      }
  }
}

void Hydrodynamics::Init(){
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Hydrodynamics::Update, this));
}

void Hydrodynamics::Update(){
  this->getTwist();
  double Damping_force[6];
  this->Damping_matrix(Damping_force);
  //this->apply_Dampingforce(Damping_force);
  this->Buoyancy();
}

void Hydrodynamics::Damping_matrix(double *damp){
  damp[0] = this->Damping[0] + this->quadratic_Damping[0]*this->COM_twist.linear.x;
  damp[1] = this->Damping[1] + this->quadratic_Damping[1]*this->COM_twist.linear.y;
  damp[2] = this->Damping[2] + this->quadratic_Damping[2]*this->COM_twist.linear.z;
  damp[3] = this->Damping[3] + this->quadratic_Damping[3]*this->COM_twist.angular.x;
  damp[4] = this->Damping[4] + this->quadratic_Damping[4]*this->COM_twist.angular.y;
  damp[5] = this->Damping[5] + this->quadratic_Damping[5]*this->COM_twist.angular.z;
}

void Hydrodynamics::getTwist(){
  this->COM_twist.linear.x = this->model->GetRelativeLinearVel()[0];
  this->COM_twist.linear.y = this->model->GetRelativeLinearVel()[1];
  this->COM_twist.linear.z = this->model->GetRelativeLinearVel()[2];
  this->COM_twist.angular.x = this->model->GetRelativeAngularVel()[0];
  this->COM_twist.angular.y = this->model->GetRelativeAngularVel()[1];
  this->COM_twist.angular.z = this->model->GetRelativeAngularVel()[2];
}

void Hydrodynamics::Buoyancy(){
  for(auto link : this->model->GetLinks()){
    //double buoyancy = this->fluidDensity * this->volume * 9.81;
    math::Vector3 buoyancy = -this->fluidDensity * this->volPropsMap[link->GetId()].volume * this->physicsEngine->GetGravity();
    //std::cout<<this->physicsEngine->GetGravity()<<"\n";
    math::Pose linkFrame = link->GetWorldPose();
    // rotate buoyancy into the link frame before applying the force.
    math::Vector3 buoyancyLinkFrame = linkFrame.rot.GetInverse().RotateVector(buoyancy);
    //std::cout<<buoyancyLinkFrame<<"\n";
    link->AddLinkForce(buoyancyLinkFrame, this->volPropsMap[link->GetId()].cov);
  }
}

void Hydrodynamics::apply_Dampingforce(double *damp){
  math::Vector3 linacc;
  math::Vector3 angacc;
  linacc.x = -damp[0]*this->COM_twist.linear.x/23.15124;   //mass = 23.15124
  linacc.y = -damp[1]*this->COM_twist.linear.y/23.15124;
  linacc.z = -damp[2]*this->COM_twist.linear.z/23.15124;
  angacc.x = -damp[3]*this->COM_twist.angular.x/23.15124;
  angacc.y = -damp[4]*this->COM_twist.angular.y/23.15124;
  angacc.z = -damp[5]*this->COM_twist.angular.z/23.15124;
  this->model->SetLinearAccel(linacc);
  this->model->SetAngularAccel(angacc);
}
