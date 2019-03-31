//
// Created by hl on 2/10/19.
//

#include <rxros.h>
#include "BrickPi3Ros_rx.h"

int main(int argc, char** argv) {
    rxros::init(argc, argv, "brickpi3"); // Name of this node.

    auto jointCmdObserv = rxros::Observable::fromTopic<brickpi3_msgs::JointCommand>("/joint_command");

    rxros::Observable::fromRobotYaml("/brickpi3/brickpi3_robot").subscribe(
        [=](auto device) { // on_next event
            if (device.getType() == "motor") {
                rxros::Logging().info() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();
//                brickpi3::Observable::motor(device.getName(), device.getPort(), device.getFrequency())
//                | scan(std::make_tuple(ros::Time::now(), 0.0, 0.0), motorEvent2PosVelTimeTuple)
//                | join_with_latest_from(jointCmdObserv.filter([=](const auto jointCmd){ return (device.getName() == jointCmd.name); }))
//                | map(posVelTimeTuple2JointStateMsg, device.getName())
//                | publish_to_topic<sensor_msgs::JointState>("/joint_state");
//                //brickPi3.set_motor_power(port, static_cast<int8_t>(effort * 100.0));
            }
            else if (device.getType() == "ultrasonic") {
                rxros::Logging().info() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();
//                brickpi3::Observable::ultrasonicSensor(device.getName(), device.getPort(), device.getFrequency())
//                | map(ultrasonicEvent2RangeMsg, device.getFrameId(), device.getMinRange(), device.getMaxRange(), device.getSpreadAngle())
//                | publish_to_topic<brickpi3_msgs::Range>("/" + device.getName());
            }
            else if (device.getType() == "color") {
                rxros::Logging().info() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();

//                brickpi3::Observable::colorSensor(device.getName(), device.getPort(), device.getFrequency())
//                | map(colorEvent2ColorMsg, device.getFrameId())
//                | publish_to_topic<brickpi3_msgs::Color>("/" + device.getName());
            }
            else if (device.getType() == "touch") {
                rxros::Logging().info() << device.getType() << ", " << device.getName() << ", " << device.getPort() << ", " << device.getFrequency();

//                brickpi3::Observable::tuchSensor(device.getName(), device.getPort(), device.getFrequency())
//                | map(touchEvent2ContactMsg, device.getFrameId())
//                | publish_to_topic<brickpi3_msgs::Contact>("/" + device.getName());
            }},
        [](){}); // on completed event

    rxros::spin();
}
