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
  double v[6];
  v[0]=this->COM_twist.linear.x;
  v[1]=this->COM_twist.linear.y;
  v[2]=this->COM_twist.linear.z;
  v[3]=this->COM_twist.angular.x;
  v[4]=this->COM_twist.angular.y;
  v[5]=this->COM_twist.angular.z;
  double Damping[36];
  this->Damping_matrix(Damping,v);
  double added_mass[36];
  this->setAddedMass(added_mass);
  //this->apply_Dampingforce(Damping_force);
  this->Buoyancy();
  this->apply_allforce(Damping,added_mass);
}

void Hydrodynamics::Damping_matrix(double *damp,double *v){
  //damp[0] = this->Damping[0] + this->quadratic_Damping[0]*this->COM_twist.linear.x;
  //damp[1] = this->Damping[1] + this->quadratic_Damping[1]*this->COM_twist.linear.y;
  //damp[2] = this->Damping[2] + this->quadratic_Damping[2]*this->COM_twist.linear.z;
  //damp[3] = this->Damping[3] + this->quadratic_Damping[3]*this->COM_twist.angular.x;
  //damp[4] = this->Damping[4] + this->quadratic_Damping[4]*this->COM_twist.angular.y;
  //damp[5] = this->Damping[5] + this->quadratic_Damping[5]*this->COM_twist.angular.z;

  //**SET THE DAMP EQUAL TO THE DAMPING MATRIX** THEY SHOULD BE +ve//
}

void Hydrodynamics::setAddedMass(double *added_mass){
  //**SET THE VALUE OF ADDED MASS HERE**//??THEY SHOULD BE +ve??
  added_mass[0]=2.082654;
  added_mass[1]=0;
  added_mass[2]=0;
  added_mass[3]=0;
  added_mass[4]=0;
  added_mass[5]=0;
  added_mass[6]=0;
  added_mass[7]=19.6210693;
  added_mass[8]=0;
  added_mass[9]=0;
  added_mass[10]=0;
  added_mass[11]=0;
  added_mass[12]=0;
  added_mass[13]=0;
  added_mass[14]=19.6210693;
  added_mass[15]=0;
  added_mass[16]=0;
  added_mass[17]=0;
  added_mass[18]=0;
  added_mass[19]=0;
  added_mass[20]=0;
  added_mass[21]=0;
  added_mass[22]=0;
  added_mass[23]=0;
  added_mass[24]=0;
  added_mass[25]=0;
  added_mass[26]=0;
  added_mass[27]=0;
  added_mass[28]=4.76528099;
  added_mass[29]=0;
  added_mass[30]=0;
  added_mass[31]=0;
  added_mass[32]=0;
  added_mass[33]=0;
  added_mass[34]=0;
  added_mass[35]=4.76528099;


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

//void Hydrodynamics::apply_Dampingforce(double *damp){
//  //math::Vector3 linacc;
//  //math::Vector3 angacc;
//  //linacc.x = (this->model->GetRelativeLinearAccel().x)-(damp[0]*this->COM_twist.linear.x/23.15124);   //mass = 23.15124
//  //linacc.y = (this->model->GetRelativeLinearAccel().y)-(damp[1]*this->COM_twist.linear.y/23.15124);
//  //linacc.z = (this->model->GetRelativeLinearAccel().z)-(damp[2]*this->COM_twist.linear.z/23.15124);
//  //angacc.x = (this->model->GetRelativeAngularAccel().x)-(damp[3]*this->COM_twist.angular.x/23.15124);
//  //angacc.y = (this->model->GetRelativeAngularAccel().y)-(damp[4]*this->COM_twist.angular.y/23.15124);
//  //angacc.z = (this->model->GetRelativeAngularAccel().z)-(damp[5]*this->COM_twist.angular.z/23.15124);
//  //this->model->SetRelativeLinearAccel(linacc);
//  //this->model->SetRelativeAngularAccel(angacc);
//
//  math::Vector3 relforcedamp;
//  relforcedamp.x=damp[0]*this->COM_twist.linear.x;
//  relforcedamp.y=damp[1]*this->COM_twist.linear.y;
//  relforcedamp.z=damp[2]*this->COM_twist.linear.z;
//  math::Vector3 reltorquedamp;
//  reltorquedamp.x=damp[3]*this->COM_twist.angular.x;
//  reltorquedamp.y=damp[4]*this->COM_twist.angular.x;
//  reltorquedamp.z=damp[5]*this->COM_twist.angular.x;
//  std::cout<<reltorquedamp.x<<","<<reltorquedamp.y<<","<<reltorquedamp.z<<"\n";
//  this->model->GetLink("base_link")->AddRelativeForce(relforcedamp);
//  this->model->GetLink("base_link")->AddRelativeTorque(reltorquedamp);
//}
void Hydrodynamics::apply_allforce(double *damp,double *added_mass)
{
  math::Vector3 relforcedamp;
  relforcedamp.x=(damp[0]*(this->COM_twist.linear.x))+(damp[1]*(this->COM_twist.linear.y))+(damp[2]*(this->COM_twist.linear.z))+(damp[3]*(this->COM_twist.angular.x))+(damp[4]*(this->COM_twist.angular.y))+(damp[5]*(this->COM_twist.angular.z));
  relforcedamp.y=(damp[6]*this->COM_twist.linear.x)+(damp[7]*this->COM_twist.linear.y)+(damp[8]*this->COM_twist.linear.z)+(damp[9]*this->COM_twist.angular.x)+(damp[10]*this->COM_twist.angular.y)+(damp[11]*this->COM_twist.angular.z);
  relforcedamp.z=(damp[12]*this->COM_twist.linear.x)+(damp[13]*this->COM_twist.linear.y)+(damp[14]*this->COM_twist.linear.z)+(damp[15]*this->COM_twist.angular.x)+(damp[16]*this->COM_twist.angular.y)+(damp[17]*this->COM_twist.angular.z);
  math::Vector3 reltorquedamp;
  reltorquedamp.x=(damp[18]*this->COM_twist.linear.x)+(damp[19]*this->COM_twist.linear.y)+(damp[20]*this->COM_twist.linear.z)+(damp[21]*this->COM_twist.angular.x)+(damp[22]*this->COM_twist.angular.y)+(damp[23]*this->COM_twist.angular.z);
  reltorquedamp.y=(damp[24]*this->COM_twist.linear.x)+(damp[25]*this->COM_twist.linear.y)+(damp[26]*this->COM_twist.linear.z)+(damp[27]*this->COM_twist.angular.x)+(damp[28]*this->COM_twist.angular.y)+(damp[29]*this->COM_twist.angular.z);
  reltorquedamp.z=(damp[30]*this->COM_twist.linear.x)+(damp[31]*this->COM_twist.linear.y)+(damp[32]*this->COM_twist.linear.z)+(damp[33]*this->COM_twist.angular.x)+(damp[34]*this->COM_twist.angular.y)+(damp[35]*this->COM_twist.angular.z);

  math::Vector3 relforcemass;
  math::Vector3 reltorquemass;
  relforcemass.x=(added_mass[0]*(this->COM_twist.linear.x))+(added_mass[1]*(this->COM_twist.linear.y))+(added_mass[2]*(this->COM_twist.linear.z))+(added_mass[3]*(this->COM_twist.angular.x))+(added_mass[4]*(this->COM_twist.angular.y))+(added_mass[5]*(this->COM_twist.angular.z));
  relforcemass.y=(added_mass[6]*this->COM_twist.linear.x)+(added_mass[7]*this->COM_twist.linear.y)+(added_mass[8]*this->COM_twist.linear.z)+(added_mass[9]*this->COM_twist.angular.x)+(added_mass[10]*this->COM_twist.angular.y)+(added_mass[11]*this->COM_twist.angular.z);
  relforcemass.z=(added_mass[12]*this->COM_twist.linear.x)+(added_mass[13]*this->COM_twist.linear.y)+(added_mass[14]*this->COM_twist.linear.z)+(added_mass[15]*this->COM_twist.angular.x)+(added_mass[16]*this->COM_twist.angular.y)+(added_mass[17]*this->COM_twist.angular.z);
  reltorquemass.x=(added_mass[18]*this->COM_twist.linear.x)+(added_mass[19]*this->COM_twist.linear.y)+(added_mass[20]*this->COM_twist.linear.z)+(added_mass[21]*this->COM_twist.angular.x)+(added_mass[22]*this->COM_twist.angular.y)+(added_mass[23]*this->COM_twist.angular.z);
  reltorquemass.y=(added_mass[24]*this->COM_twist.linear.x)+(added_mass[25]*this->COM_twist.linear.y)+(added_mass[26]*this->COM_twist.linear.z)+(added_mass[27]*this->COM_twist.angular.x)+(added_mass[28]*this->COM_twist.angular.y)+(added_mass[29]*this->COM_twist.angular.z);
  reltorquemass.z=(added_mass[30]*this->COM_twist.linear.x)+(added_mass[31]*this->COM_twist.linear.y)+(added_mass[32]*this->COM_twist.linear.z)+(added_mass[33]*this->COM_twist.angular.x)+(added_mass[34]*this->COM_twist.angular.y)+(added_mass[35]*this->COM_twist.angular.z);
  //std::cout<<reltorquedamp.x<<","<<reltorquedamp.y<<","<<reltorquedamp.z<<"\n";

  math::Vector3 relforce;
  math::Vector3 reltorque;
  relforce=-relforcedamp-relforcemass;
  reltorque=-reltorquedamp-reltorquemass;
  this->model->GetLink("base_link")->AddRelativeForce(relforce);
  this->model->GetLink("base_link")->AddRelativeTorque(reltorque);

}
