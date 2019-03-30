//
// Created by hl on 2/10/19.
//

#include <rxros.h>
#include "BrickPi3Ros_rx.h"

int main(int argc, char** argv) {
    rxros::init(argc, argv, "brickpi3"); // Name of this node.

    auto jointCmdObserv = rxros::Observable::fromTopic<brickpi3_msgs::JointCommand>("/joint_command");

    rxros::Observable::fromRobotYaml("/brickpi3/brickpi3_robot").subscribe(
        [=](const auto& device){ // on_next event
            if (device["type"] == "motor") {
                const std::string name = device["name"];
                const std::string port = device["port"];
                const double frequency = device["frequency"];

                brickpi3::Observable::motor(name, port, frequency)
                | scan(std::make_tuple(ros::Time::now(), 0.0, 0.0), motorEvent2PosVelTimeTuple)
                | join_with_latest_from(jointCmdObserv.filter([=](const auto jointCmd){ return (name == jointCmd.name); }))
                | map(posVelTimeTuple2JointStateMsg, name)
                | publish_to_topic<sensor_msgs::JointState>("/joint_state");
                //brickPi3.set_motor_power(port, static_cast<int8_t>(effort * 100.0));
            }
            else if (device["type"] == "ultrasonic") {
                const std::string name = device["name"];
                const std::string frameId = device["frame_id"];
                const std::string port = device["port"];
                const double frequency = device["frequency"];
                const double minRange = device["min_range"];
                const double maxRange = device["max_range"];
                const double spreadAngle = device["spread_angle"];

                brickpi3::Observable::ultrasonicSensor(name, port, frequency)
                | map(ultrasonicEvent2RangeMsg, frameId, minRange, maxRange, spreadAngle)
                | publish_to_topic<brickpi3_msgs::Range>("/" + name);
            }
            else if (device["type"] == "color")
            {
                const std::string name = device["name"];
                const std::string frameId = device["frame_id"];
                const std::string port = device["port"];
                const double frequency = device["frequency"];

                brickpi3::Observable::colorSensor(name, port, frequency)
                | map(colorEvent2ColorMsg, frameId)
                | publish_to_topic<brickpi3_msgs::Color>("/" + name);
            }},
        [](){}); // on completed event

    rxros::spin();
}
