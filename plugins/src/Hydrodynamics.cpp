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
  this->physicsEngine = world->Physics();
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
          ignition::math::Vector3<double> cov = linkElem->GetElement("center_of_volume")->Get<ignition::math::Vector3<double>>();
          this->volPropsMap[id].cov = cov;
          //std::cout<<cov<<" ";
        }
      if(linkElem->HasElement("volume")){
          double volume = linkElem->GetElement("volume")->Get<double>();
          this->volPropsMap[id].volume = volume;
          std::cout<<volume<<"\n";
        }
        //std::cout<<linkElem->Get<std::string>("name")<<"\n";
      }
  }
}

void Hydrodynamics::Init(){
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&Hydrodynamics::Update, this));
}

void Hydrodynamics::Update(){
  this->getTwist();
  this->Buoyancy();
  this->Damping();
  //this->AddedMass();
}

void Hydrodynamics::Damping_matrix(double *damp){
  damp[0] = this->linear_Damping[0] + this->quadratic_Damping[0]*this->COM_twist.linear.x;
  damp[1] = this->linear_Damping[1] + this->quadratic_Damping[1]*this->COM_twist.linear.y;
  damp[2] = this->linear_Damping[2] + this->quadratic_Damping[2]*this->COM_twist.linear.z;
  damp[3] = this->linear_Damping[3] + this->quadratic_Damping[3]*this->COM_twist.angular.x;
  damp[4] = this->linear_Damping[4] + this->quadratic_Damping[4]*this->COM_twist.angular.y;
  damp[5] = this->linear_Damping[5] + this->quadratic_Damping[5]*this->COM_twist.angular.z;

  //**SET THE DAMP EQUAL TO THE DAMPING MATRIX** THEY SHOULD BE +ve//
}

void Hydrodynamics::getTwist(){
  this->COM_twist.linear.x = this->model->GetLink("base_link")->RelativeLinearVel()[0];
  this->COM_twist.linear.y = this->model->GetLink("base_link")->RelativeLinearVel()[1];
  this->COM_twist.linear.z = this->model->GetLink("base_link")->RelativeLinearVel()[2];
  this->COM_twist.angular.x = this->model->GetLink("base_link")->RelativeAngularVel()[0];
  this->COM_twist.angular.y = this->model->GetLink("base_link")->RelativeAngularVel()[1];
  this->COM_twist.angular.z = this->model->GetLink("base_link")->RelativeAngularVel()[2];
}

void Hydrodynamics::Buoyancy(){
  for(auto link : this->model->GetLinks()){
    //double buoyancy = this->fluidDensity * this->volume * 9.81;
    ignition::math::Vector3<double> buoyancy = -this->fluidDensity * this->volPropsMap[link->GetId()].volume * this->model->GetWorld()->Gravity();
    //std::cout<<this->physicsEngine->GetGravity()<<"\n";
    ignition::math::Pose3<double> linkFrame = link->WorldPose();
    // rotate buoyancy into the link frame before applying the force.
    ignition::math::Vector3<double> buoyancyLinkFrame = linkFrame.Rot().Inverse().RotateVector(buoyancy);
    //std::cout<<buoyancyLinkFrame<<"\n";
    link->AddLinkForce(buoyancyLinkFrame, this->volPropsMap[link->GetId()].cov);
  }
}

void Hydrodynamics::Damping(){
  double damp[6];
  this->Damping_matrix(damp);

  ignition::math::Vector3<double> relforcedamp;
  relforcedamp.X() = damp[0]*this->COM_twist.linear.x;
  relforcedamp.Y() = damp[1]*this->COM_twist.linear.y;
  relforcedamp.Z() = damp[2]*this->COM_twist.linear.z;

  ignition::math::Vector3<double> reltorquedamp;
  reltorquedamp.X() = damp[3]*this->COM_twist.angular.x;
  reltorquedamp.Y() = damp[4]*this->COM_twist.angular.y;
  reltorquedamp.Z() = damp[5]*this->COM_twist.angular.z;

  this->model->GetLink("base_link")->AddRelativeForce(relforcedamp);
  this->model->GetLink("base_link")->AddRelativeTorque(reltorquedamp);
}

void Hydrodynamics::AddedMass(){
  ignition::math::Vector3<double> linAcc;
  linAcc.X() = this->model->GetLink("base_link")->RelativeLinearAccel()[0];
  linAcc.Y() = this->model->GetLink("base_link")->RelativeLinearAccel()[1];
  linAcc.Z() = this->model->GetLink("base_link")->RelativeLinearAccel()[2];
  ignition::math::Vector3<double> Force;
  Force.X() = this->added_mass[0] * linAcc.X();
  Force.Y() = this->added_mass[1] * linAcc.Y();
  Force.Z() = this->added_mass[2] * linAcc.Z();

  ignition::math::Vector3<double> rotAcc;
  rotAcc.X() = this->model->GetLink("base_link")->RelativeAngularAccel()[0];
  rotAcc.Y() = this->model->GetLink("base_link")->RelativeAngularAccel()[1];
  rotAcc.Z() = this->model->GetLink("base_link")->RelativeAngularAccel()[2];
  ignition::math::Vector3<double> Torque;
  Torque.X() = this->added_mass[3] * rotAcc.X();
  Torque.Y() = this->added_mass[4] * rotAcc.Y();
  Torque.Z() = this->added_mass[5] * rotAcc.Z();

  this->model->GetLink("base_link")->AddRelativeForce(Force);
  this->model->GetLink("base_link")->AddRelativeTorque(Torque);
}
