#include<plugins/KrakenControl.h>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KrakenControlPlugin)

KrakenControlPlugin::KrakenControlPlugin(){}

void KrakenControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  this->model = _model;
  this->sdf = _sdf;
  physics::WorldPtr world = _model->GetWorld();
  this->physicsEngine = world->Physics();
  GZ_ASSERT(world != NULL, "MODEL IS IN A NULL WORLD");
  GZ_ASSERT(this->physicsEngine != NULL, "PHYSICS ENGINE WAS NULL");
  GZ_ASSERT(_sdf != NULL, "RECEIVED NULL SDF POINTER");
  ros::init(KrakenControlPlugin::argc, KrakenControlPlugin::argv, "KrakenControlPlugin");
  ros::NodeHandle n;
  KrakenControlPlugin::DVLPub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("/kraken/sensors/DVL", 1);
  KrakenControlPlugin::PosePub = n.advertise<geometry_msgs::Pose>("KrakenSimulator/Pose", 1);
  KrakenControlPlugin::ImuPub = n.advertise<sensor_msgs::Imu>("/kraken/sensors/IMU", 1);
  _thrust6dataSub = n.subscribe<kraken_msgs::thrusterData6Thruster>("/kraken/control/thruster6pid", 10, &KrakenControlPlugin::thrust6Callback, this);

}

void KrakenControlPlugin::Init(){
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&KrakenControlPlugin::Update, this));

}

void KrakenControlPlugin::Update(){
 geometry_msgs::TwistWithCovarianceStamped DVL_Twist;
 DVL_Twist.twist.twist.linear.x = this->model->GetLink("DVL")->RelativeLinearVel()[0];
 DVL_Twist.twist.twist.linear.y = this->model->GetLink("DVL")->RelativeLinearVel()[1];
 DVL_Twist.twist.twist.linear.z = this->model->GetLink("DVL")->RelativeLinearVel()[2];
 DVLPub.publish(DVL_Twist);


 //tf::StampedTransform transform;
 //try{
     //listener.lookupTransform("base_link", "odom", ros::Time(0), transform);
 //}
 //catch(tf::TransformException ex){
 //ROS_ERROR("%s",ex.what());
 //ros::Duration(1.0).sleep();
 //}

 ignition::math::Vector3d grav = this->model->GetWorld()->Gravity();
 //grav.x=0;
 //grav.y=0;
 //grav.z=9.81;
 physics::LinkPtr link = this->model->GetLink("Imu");
 ignition::math::Pose3d linkFrame = link->WorldPose();
 ignition::math::Vector3d tempo = linkFrame.Rot().Inverse().RotateVector(grav);

 //listener.transformVector("/base_link",grav,tempo);

 sensor_msgs::Imu Imu_data;
 Imu_data.linear_acceleration.x = tempo.X() - this->model->GetLink("Imu")->RelativeLinearAccel()[0];
 Imu_data.linear_acceleration.y = tempo.Y() - this->model->GetLink("Imu")->RelativeLinearAccel()[1];
 Imu_data.linear_acceleration.z = tempo.Z() - this->model->GetLink("Imu")->RelativeLinearAccel()[2];

 Imu_data.angular_velocity.x = this->model->GetLink("Imu")->RelativeAngularVel()[0];
 Imu_data.angular_velocity.y = this->model->GetLink("Imu")->RelativeAngularVel()[1];
 Imu_data.angular_velocity.z = this->model->GetLink("Imu")->RelativeAngularVel()[2];

 //tf::Quaternion q(this->model->GetLink("Imu")->GetRelativePose.orientation[0],this->model->GetLink("Imu")->GetRelativePose[1],this->model->GetLink("Imu")->GetRelativePose[2]);
 Imu_data.orientation.x = this->model->GetLink("Imu")->RelativePose().Rot().X();
 Imu_data.orientation.y = this->model->GetLink("Imu")->RelativePose().Rot().Y();
 Imu_data.orientation.z = this->model->GetLink("Imu")->RelativePose().Rot().Z();
 Imu_data.orientation.w = this->model->GetLink("Imu")->RelativePose().Rot().W();

 ImuPub.publish(Imu_data);

 geometry_msgs::Pose _pose;
 physics::ModelState modelState(this->model);
 _pose.position.x = modelState.Pose().Pos().X();
 _pose.position.y = modelState.Pose().Pos().Y();
 _pose.position.z = modelState.Pose().Pos().Z();
 _pose.orientation.x = modelState.Pose().Rot().X();
 _pose.orientation.y = modelState.Pose().Rot().Y();
 _pose.orientation.z = modelState.Pose().Rot().Z();
 _pose.orientation.w = modelState.Pose().Rot().W();
 PosePub.publish(_pose);
 ros::spinOnce();

}

void KrakenControlPlugin::thrust6Callback(const kraken_msgs::thrusterData6Thruster::ConstPtr &msg){
  kraken_msgs::thrusterData6Thruster::Type thrust;
  for(int i = 0; i<6; i++){
    if((msg->data[i] < 50000) && (msg->data[i] >= 0)){
          thrust.data[i] = msg->data[i];
          thrust.data[i] = 0.0625*thrust.data[i]*thrust.data[i];
        }

    else if((msg->data[i] > -50000) && (msg->data[i] < 0)){
          thrust.data[i] = msg->data[i];
          thrust.data[i] = -0.0625*thrust.data[i]*thrust.data[i];
        }

    else{
          if((msg->data[i] > 0))
            thrust.data[i] = 0.0625*50000*50000;
          if((msg->data[i] < 0))
            thrust.data[i] = -0.0625*50000*50000;
        }
      }
  std::cout<<thrust<<"\n";
  this->model->GetLink("thruster_surge_left")->AddRelativeForce({-thrust.data[2], 0, 0});//{thrust.data[0], 0, 0}
  this->model->GetLink("thruster_surge_right")->AddRelativeForce({-thrust.data[3], 0, 0});//({thrust.data[1], 0, 0
  this->model->GetLink("thruster_sway_back")->AddRelativeForce({-thrust.data[4], 0, 0});//thrust.data[2], 0, 0})
  this->model->GetLink("thruster_sway_front")->AddRelativeForce({-thrust.data[5], 0, 0});//{thrust.data[3], 0, 0}
  this->model->GetLink("thruster_depth_back")->AddRelativeForce({-thrust.data[0], 0, 0});//{thrust.data[4], 0, 0}
  this->model->GetLink("thruster_depth_front")->AddRelativeForce({-thrust.data[1], 0, 0});//({thrust.data[5], 0, 0
}
