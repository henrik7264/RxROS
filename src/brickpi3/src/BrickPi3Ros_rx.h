//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3ROS_RX_H
#define BRICKPI3_BRICKPI3ROS_RX_H

#include <sensor_msgs/JointState.h>
#include <brickpi3_msgs/Contact.h>
#include <brickpi3_msgs/Color.h>
#include <brickpi3_msgs/Range.h>
#include <brickpi3_msgs/JointCommand.h>
#include "BrickPi3Device.h"
#include "BrickPi3Observable_rx.h"


auto motorEvent2PosVelTimeTuple = [](const auto& prevPosVelTimeTuple, const actuator_motor_t& motorEvent) {
    const auto prevTime = std::get<0>(prevPosVelTimeTuple);
    const auto prevPos = std::get<1>(prevPosVelTimeTuple);
    const auto prevVel = std::get<2>(prevPosVelTimeTuple);
    const auto currTime = ros::Time::now();
    const auto currPos = static_cast<double>(motorEvent.motorPosition) * M_PI / 180.0 / 3.0; // 3.0 is gearing ratio
    const auto currVel = (currPos - prevPos)/(currTime - prevTime).toSec();
    return std::make_tuple(currTime, currPos, currVel);};


auto posVelTimeTuple2JointStateMsg = [](const auto& tuple, const std::string& name) {
    static int seqNo = 0;
    const auto posVelTimeTuple = std::get<0>(tuple);
    const auto position = std::get<1>(posVelTimeTuple);
    const auto velocity = std::get<2>(posVelTimeTuple);
    const auto effort = std::get<1>(tuple).effort;
    sensor_msgs::JointState jointState;
    //jointState.header.frame_id = name;
    jointState.header.stamp = ros::Time::now();
    jointState.header.seq = seqNo++;
    jointState.name.push_back(name);
    jointState.effort.push_back(effort);
    jointState.position.push_back(position); // rad
    jointState.velocity.push_back(velocity); // rad/s
    return jointState;};


auto touchEvent2ContactMsg = [](const sensor_touch_t& sensorColor, const std::string& frameId) {
    static int seqNo = 0;
    brickpi3_msgs::Contact contact;
    contact.header.frame_id = frameId;
    contact.header.stamp = ros::Time::now();
    contact.header.seq = seqNo++;
    contact.contact = sensorColor.pressed;
    return contact;};


auto colorEvent2ColorMsg = [](const sensor_color_t& sensorColor, const std::string& frameId) {
    static int seqNo = 0;
    brickpi3_msgs::Color color;
    color.header.frame_id = frameId;
    color.header.stamp = ros::Time::now();
    color.header.seq = seqNo++;
    color.r = static_cast<double>(sensorColor.reflected_red);
    color.g = static_cast<double>(sensorColor.reflected_green);
    color.b = static_cast<double>(sensorColor.reflected_blue);
    color.intensity = static_cast<double>(sensorColor.ambient);
    return color;};


auto ultrasonicEvent2RangeMsg = [](const sensor_ultrasonic_t& sensorUltrasonic, const std::string& frameId, const double minRange, const double maxRange, const double spreadAngle) {
    static int seqNo = 0;
    brickpi3_msgs::Range range;
    range.header.frame_id = frameId;
    range.header.stamp = ros::Time::now();
    range.header.seq = seqNo++;
    range.range = sensorUltrasonic.cm / 100.0;
    range.range_min = minRange;
    range.range_max = maxRange;
    range.spread_angle = spreadAngle;
    return range;};


#endif //BRICKPI3_BRICKPI3ROS_RX_H
