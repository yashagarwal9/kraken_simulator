#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
void poseCallback(const geometry_msgs::Pose::ConstPtr &msg){
  geometry_msgs::Pose pose;
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg->position.x, msg->position.y, msg->position.z) );
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  //std::cout<<msg->orientation.w<<"\n";//<<msg->orientation.y<<"\n"<<msg->orientation.z<<"\n"<<msg->orientation.w;
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/KrakenSimulator/Pose", 30, &poseCallback);
  ros::spin();
  return 0;
};
