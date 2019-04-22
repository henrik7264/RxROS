//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <Scheduler.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include "Node.h"
#include "BrickPi3Motor.h"
#include "BrickPi3Ultrasonic.h"
#include "BrickPi3Color.h"
#include "BrickPi3Touch.h"

class BrickPi3Ros {
private:
    Bosma::Scheduler scheduler;

public:
    BrickPi3Ros(int argc, char** argv);
    virtual ~BrickPi3Ros() {}
    void run() {ros::spin();}
};

BrickPi3Ros::BrickPi3Ros(int argc, char** argv):
    scheduler(8) // BrickPi3 supports 8 devices.
{
    // Parse the ros_robot.yaml file and create the appropriate sensors and actuators
    XmlRpc::XmlRpcValue brickpi3_robot;
    Node::getHandle().getParam("/brickpi3/brickpi3_robot", brickpi3_robot);
    ROS_ASSERT(brickpi3_robot.getType() == XmlRpc::XmlRpcValue::TypeArray);
    std::cout << "---------------------------------" << std::endl;
    for (int i = 0; i < brickpi3_robot.size(); i++) {
        XmlRpc::XmlRpcValue brickpi3_device = brickpi3_robot[i];
        std::string type = brickpi3_device["type"];
        if (type == "motor") {
            std::string name = brickpi3_device["name"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];
            std::cout << type << ", " << name << ", " << port << ", " << freq << std::endl;

            BrickPi3Motor* motor = new BrickPi3Motor(name, port, freq); //todo: Needs better resource handling. Should be freed when the BrickPi3Ros object is deleted.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Motor::schedulerCB, motor));
        }
        else if (type == "ultrasonic") {
            std::string name = brickpi3_device["name"];
            std::string frame_id = brickpi3_device["frame_id"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];
            double min_range = brickpi3_device["min_range"];
            double max_range = brickpi3_device["max_range"];
            double spread_angle = brickpi3_device["spread_angle"];

            BrickPi3Ultrasonic* ultrasonic = new BrickPi3Ultrasonic(name, frame_id, port, freq, min_range, max_range, spread_angle); //todo: Needs better resource handling.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Ultrasonic::schedulerCB, ultrasonic));
        }
        else if (type ==  "color")
        {
            std::string name = brickpi3_device["name"];
            std::string frame_id = brickpi3_device["frame_id"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];

            BrickPi3Color* color = new BrickPi3Color(name, frame_id, port, freq); //todo: Needs better resource handling.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Color::schedulerCB, color));
        }
        else if (type == "touch")
        {
            std::string name = brickpi3_device["name"];
            std::string frame_id = brickpi3_device["frame_id"];
            std::string port = brickpi3_device["port"];
            double freq = brickpi3_device["frequency"];

            BrickPi3Touch* touch = new BrickPi3Touch(name, frame_id, port, freq); //todo: Needs better resource handling.
            scheduler.every(std::chrono::milliseconds(static_cast<int >(1000.0/freq)), boost::bind(&BrickPi3Touch::schedulerCB, touch));
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "brickpi3"); // Name of this node.
    BrickPi3Ros brickPi3Ros(argc, argv);
    brickPi3Ros.run();
}
