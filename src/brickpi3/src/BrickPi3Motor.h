//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3MOTOR_H
#define BRICKPI3_BRICKPI3MOTOR_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <brickpi3_msgs/JointCommand.h>
#include "BrickPi3Device.h"


class BrickPi3Motor: public BrickPi3Device
{
private:
    ros::Publisher jointStatePublisher;
    ros::Subscriber jointCommandSubscriber;
    std::string name; // The name of the wheel. Defined in yaml file.
    uint8_t port; // The BrickPi3 port number. Defined in yaml file.
    double freq; // Frequency the schedulerCB will be called with. Defined in yaml file.
    double effort; // The desired effort the motor should use.
    unsigned int seqNo;
    ros::Time lastTime;
    double lastPosition;

    void jointCommandCB(const brickpi3_msgs::JointCommand& jointCommand);

public:
    BrickPi3Motor(const std::string& name, const std::string& port, const double frequency);
    virtual ~BrickPi3Motor();

    const std::string getName() const {return name;}
    uint8_t getPort() const {return port;}
    double getFreq() const {return freq;}
    double getEffort() const {return effort;}

    void schedulerCB();
};

#endif //BRICKPI3_BRICKPI3MOTOR_H
