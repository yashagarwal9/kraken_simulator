#include<plugins/KrakenControl.h>
#include<gazebo/common/Assert.hh>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
#include<iostream>
using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KrakenControlPlugin)

KrakenControlPlugin::KrakenControlPlugin(){}

void KrakenControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  this->model = _model;
  this->sdf = _sdf;
  physics::WorldPtr world = _model->GetWorld();
  this->physicsEngine = world->GetPhysicsEngine();
  GZ_ASSERT(world != NULL, "MODEL IS IN A NULL WORLD");
  GZ_ASSERT(this->physicsEngine != NULL, "PHYSICS ENGINE WAS NULL");
  GZ_ASSERT(_sdf != NULL, "RECEIVED NULL SDF POINTER");
  ros::init(KrakenControlPlugin::argc, KrakenControlPlugin::argv, "KrakenControlPlugin");
  ros::NodeHandle n;
  KrakenControlPlugin::DVLPub = n.advertise<geometry_msgs::Twist>("KrakenSimulator/DVL", 1);
  KrakenControlPlugin::PosePub = n.advertise<geometry_msgs::Pose>("KrakenSimulator/Pose", 1);
  _thrust6dataSub = n.subscribe<msgs_stack::thrusterData6>("krakenSimulator_thrusterData6", 10, &KrakenControlPlugin::thrust6Callback, this);
}

void KrakenControlPlugin::Init(){
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KrakenControlPlugin::Update, this));
}

void KrakenControlPlugin::Update(){
 geometry_msgs::Twist DVL_Twist;
 DVL_Twist.linear.x = this->model->GetLink("DVL")->GetRelativeLinearVel()[0];
 DVL_Twist.linear.y = this->model->GetLink("DVL")->GetRelativeLinearVel()[1];
 DVL_Twist.linear.z = this->model->GetLink("DVL")->GetRelativeLinearVel()[2];
 DVLPub.publish(DVL_Twist);
 geometry_msgs::Pose _pose;
 physics::ModelState modelState(this->model);
 _pose.position.x = modelState.GetPose().pos.x;
 _pose.position.y = modelState.GetPose().pos.y;
 _pose.position.z = modelState.GetPose().pos.z;
 _pose.orientation.x = modelState.GetPose().rot.x;
 _pose.orientation.y = modelState.GetPose().rot.y;
 _pose.orientation.z = modelState.GetPose().rot.z;
 _pose.orientation.w = modelState.GetPose().rot.w;
 PosePub.publish(_pose);
 ros::spinOnce();

}

void KrakenControlPlugin::thrust6Callback(const msgs_stack::thrusterData6::ConstPtr &msg){
  msgs_stack::thrusterData6::Type thrust;
  for(int i = 0; i<6; i++){
    if((msg->data[i] < 50000) && (msg->data[i] >= 0)){
          thrust.data[i] = msg->data[i];
          thrust.data[i] = 0.0006835*thrust.data[i]*thrust.data[i];
        }

    else if((msg->data[i] > -50000) && (msg->data[i] < 0)){
          thrust.data[i] = msg->data[i];
          thrust.data[i] = -0.0006835*thrust.data[i]*thrust.data[i];
        }

    else{
          if((msg->data[i] > 0))
            thrust.data[i] = 0.0006835*50000*50000;
          if((msg->data[i] < 0))
            thrust.data[i] = -0.0006835*50000*50000;
        }
      }
  std::cout<<thrust<<"\n";
  this->model->GetLink("thruster_surge_left")->AddRelativeForce({thrust.data[0], 0, 0});
  this->model->GetLink("thruster_surge_right")->AddRelativeForce({thrust.data[1], 0, 0});
  this->model->GetLink("thruster_sway_back")->AddRelativeForce({thrust.data[2], 0, 0});
  this->model->GetLink("thruster_sway_front")->AddRelativeForce({thrust.data[3], 0, 0});
  this->model->GetLink("thruster_depth_back")->AddRelativeForce({thrust.data[4], 0, 0});
  this->model->GetLink("thruster_depth_front")->AddRelativeForce({thrust.data[5], 0, 0});
}
