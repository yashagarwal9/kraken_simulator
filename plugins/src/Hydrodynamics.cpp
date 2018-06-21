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
  this->COM_twist.linear.x = this->model->GetLink("base_link")->GetRelativeLinearVel()[0];
  this->COM_twist.linear.y = this->model->GetLink("base_link")->GetRelativeLinearVel()[1];
  this->COM_twist.linear.z = this->model->GetLink("base_link")->GetRelativeLinearVel()[2];
  this->COM_twist.angular.x = this->model->GetLink("base_link")->GetRelativeAngularVel()[0];
  this->COM_twist.angular.y = this->model->GetLink("base_link")->GetRelativeAngularVel()[1];
  this->COM_twist.angular.z = this->model->GetLink("base_link")->GetRelativeAngularVel()[2];
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

void Hydrodynamics::Damping(){
  double damp[6];
  this->Damping_matrix(damp);

  math::Vector3 relforcedamp;
  relforcedamp.x = damp[0]*this->COM_twist.linear.x;
  relforcedamp.y = damp[1]*this->COM_twist.linear.y;
  relforcedamp.z = damp[2]*this->COM_twist.linear.z;

  math::Vector3 reltorquedamp;
  reltorquedamp.x = damp[3]*this->COM_twist.angular.x;
  reltorquedamp.y = damp[4]*this->COM_twist.angular.y;
  reltorquedamp.z = damp[5]*this->COM_twist.angular.z;

  this->model->GetLink("base_link")->AddRelativeForce(relforcedamp);
  this->model->GetLink("base_link")->AddRelativeTorque(reltorquedamp);
}

void Hydrodynamics::AddedMass(){
  math::Vector3 linAcc;
  linAcc.x = this->model->GetLink("base_link")->GetRelativeLinearAccel()[0];
  linAcc.y = this->model->GetLink("base_link")->GetRelativeLinearAccel()[1];
  linAcc.z = this->model->GetLink("base_link")->GetRelativeLinearAccel()[2];
  math::Vector3 Force;
  Force.x = this->added_mass[0] * linAcc.x;
  Force.y = this->added_mass[1] * linAcc.y;
  Force.z = this->added_mass[2] * linAcc.z;

  math::Vector3 rotAcc;
  rotAcc.x = this->model->GetLink("base_link")->GetRelativeAngularAccel()[0];
  rotAcc.y = this->model->GetLink("base_link")->GetRelativeAngularAccel()[1];
  rotAcc.z = this->model->GetLink("base_link")->GetRelativeAngularAccel()[2];
  math::Vector3 Torque;
  Torque.x = this->added_mass[3] * rotAcc.x;
  Torque.y = this->added_mass[4] * rotAcc.y;
  Torque.z = this->added_mass[5] * rotAcc.z;

  this->model->GetLink("base_link")->AddRelativeForce(Force);
  this->model->GetLink("base_link")->AddRelativeTorque(Torque);
}
