#include <ros/ros.h>
#include <iostream>
#include <kraken_msgs/thrusterData6Thruster.h>


using namespace std;
kraken_msgs::thrusterData6Thruster _force_sent;
void storePresentForceData(const kraken_msgs::thrusterData6Thruster force){
    _force_sent.data[0] = -force.data[0];
    _force_sent.data[1] = -force.data[1];
    _force_sent.data[2] = -force.data[2];
    _force_sent.data[3] = -force.data[3];
    _force_sent.data[4] = force.data[4];
    _force_sent.data[5] = force.data[5];
}

int main(int argc, char **argv){
    ros::init(argc, argv, "KeyboardControl_cpp");
    ros::NodeHandle _nh;
    ros::Publisher _force_pub = _nh.advertise<kraken_msgs::thrusterData6Thruster>("/kraken/control/thruster6pid", 10);
    ros::Subscriber sub1 = _nh.subscribe("Keyboard_Control", 10, storePresentForceData);
    ros::Rate _looprate(25);

    while(ros::ok()){
        _force_pub.publish(_force_sent);
        ros::spinOnce();
        _looprate.sleep();
    }

    return 0;
}
