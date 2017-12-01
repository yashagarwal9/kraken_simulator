#include<plugins/KrakenControl.h>
#include<gazebo/common/Assert.hh>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Pose.h>
using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KrakenControlPlugin)

KrakenControlPlugin::KrakenControlPlugin(){

}

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
 _pose.position.x = this->model->GetRelativePose().pos.x;
 _pose.position.y = this->model->GetRelativePose().pos.y;
 _pose.position.z = this->model->GetRelativePose().pos.z;
 _pose.orientation.x = this->model->GetRelativePose().rot.x;
 _pose.orientation.y = this->model->GetRelativePose().rot.y;
 _pose.orientation.z = this->model->GetRelativePose().rot.z;
 _pose.orientation.w = this->model->GetRelativePose().rot.w;
 PosePub.publish(_pose);
 ros::spinOnce();

}

void KrakenControlPlugin::thrust6Callback(const msgs_stack::thrusterData6::ConstPtr &msg){
  this->model->GetLink("thruster_surge_left")->SetForce({msg->data[0], 0, 0});
  this->model->GetLink("thruster_surge_right")->SetForce({msg->data[1], 0, 0});
  this->model->GetLink("thruster_sway_back")->SetForce({0, msg->data[2], 0});
  this->model->GetLink("thruster_sway_front")->SetForce({0, msg->data[3], 0});
  this->model->GetLink("thruster_depth_back")->SetForce({0, 0, msg->data[4]});
  this->model->GetLink("thruster_depth_front")->SetForce({0, 0, msg->data[5]});
}
