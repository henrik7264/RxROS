//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <brickpi3_ros/JointCommand.h>
#include "BrickPi3Device.h"
#include "BrickPi3Motor.h"
using namespace std;

#define POWER_MAX 1.25


BrickPi3Motor::BrickPi3Motor(const string& aName, const string& aPort, const double aFrequency):
    jointStatePublisher(nodeHandle.advertise<sensor_msgs::JointState>("/joint_state", 10)),
    jointCommandSubscriber(nodeHandle.subscribe("/joint_command", 10, &BrickPi3Motor::jointCommandCB, this))
{
    name = aName;
    port = port2id(aPort);
    freq = aFrequency;

    brickPi3.reset_motor_encoder(port);
}

BrickPi3Motor::~BrickPi3Motor()
{
    brickPi3.reset_motor_encoder(port);
}

void BrickPi3Motor::jointCommandCB(const brickpi3_ros::JointCommand& jointCommand)
{
    lock_guard<std::mutex> guard(mutex);

    if (name == jointCommand.name) {
        //Store the commanded power value, limited to the range +/- POWER_MAX

        effort = jointCommand.effort;
        if (effort > POWER_MAX) {
            effort = POWER_MAX;
        } else if (effort < -POWER_MAX) {
            effort = -POWER_MAX;
        }
    }
}

void BrickPi3Motor::schedulerCB()
{
    lock_guard<std::mutex> guard(mutex);
    uint8_t motorState = 0;// Variables for reading motor motorState
    int8_t motorPower = 0;// Variables for reading motor powers
    int32_t motorPosition = 0;  // Variables for reading motor encoder positions
    int16_t motorDPS = 0; // Variables for reading motor speeds (Degrees Per Second)

//    // Use the encoder value from motor A to control motors B, C, and D
//    BP.set_motor_power(PORT_B, PositionA < 100 ? PositionA > -100 ? PositionA : -100 : 100);
//    BP.set_motor_dps(PORT_C, PositionA);
//    BP.set_motor_position(PORT_D, PositionA);

    int rc = brickPi3.get_motor_status(PORT_A, motorState, motorPower, motorPosition, motorDPS);;
    if (rc == 0) {
        printf("State A: %d Power A: %4d  Encoder A: %6d  DPS A: %6d  \n", motorState, motorPower, motorPosition, motorDPS);

        sensor_msgs::JointState jointState;
        jointState.header.frame_id = name;
        jointState.header.stamp = ros::Time::now();
        jointState.header.seq = seqNo++;
        jointState.name = name;
        jointState.effort = effort;
        jointState.position = 1;
        jointState.velocity = 1;

        jointStatePublisher.publish(jointState);
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Motor status %s, rc %d", name.c_str(), rc);
    }
}
