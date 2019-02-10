//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <Scheduler.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include "BrickPi3Motor.h"
using namespace std;

class BrickPi3Ros {
private:
    ros::NodeHandle nodeHandle;

public:
    BrickPi3Ros(int argc, char** argv) {}
    virtual ~BrickPi3Ros() {}
    void run() {ros::spin();}
    void init();
};

void BrickPi3Ros::init()
{
    // Parse the ros_robot.yaml file and create the appropriate sensors and actuators
    XmlRpc::XmlRpcValue brickpi3_robot;
    nodeHandle.getParam("/brickpi3_ros/brickpi3_robot", brickpi3_robot);
    ROS_ASSERT(brickpi3_robot.getType() == XmlRpc::XmlRpcValue::TypeArray);
    cout << "---------------------------------" << endl;
    for (int i = 0; i < brickpi3_robot.size(); i++) {
        XmlRpc::XmlRpcValue brickpi3_device = brickpi3_robot[i];
        string type = brickpi3_device["type"];
        if (type == "motor") {
            string name = brickpi3_device["name"];
            string port = brickpi3_device["port"];
            double freq = brickpi3_device["desired_frequency"];
            cout << "Motor: " << name << ", " << port << ", " << freq << endl;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "brickpi3_ros"); // Name of this node.
    BrickPi3Ros brickPi3Ros(argc, argv);
    brickPi3Ros.init();
    brickPi3Ros.run();
}
