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

#define POWER_TO_NM 0.01
#define POWER_MAX 125


static uint8_t port2id(const string& port)
{
    int id = ((port == "PORT_A") ? PORT_A :
             ((port == "PORT_B") ? PORT_B :
             ((port == "PORT_C") ? PORT_C :
             ((port == "PORT_D") ? PORT_D : INT_MAX))));
    return static_cast<uint8_t>(id);
}


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
        //Store the commanded power value,
        // limited to the range +/- POWER_MAX

        effort = jointCommand.effort / POWER_TO_NM;
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


}
