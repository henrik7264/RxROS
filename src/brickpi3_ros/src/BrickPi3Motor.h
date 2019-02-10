//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_ROS_BRICKPI3MOTOR_H
#define BRICKPI3_ROS_BRICKPI3MOTOR_H

#include <string>
#include <brickpi3_ros/JointCommand.h>
#include "BrickPi3Device.h"


class BrickPi3Motor: public BrickPi3Device
{
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher jointStatePublisher;
    ros::Subscriber jointCommandSubscriber;
    std::string name; // The name of the wheel. Defined in yaml file.
    uint8_t port; // The BrickPi3 port number. Defined in yaml file.
    double freq; // Frequency the schedulerCB will be called with. Defined in yaml file.
    double effort; // The desired effort the motor should use.

    void jointCommandCB(const brickpi3_ros::JointCommand& jointCommand);

public:
    BrickPi3Motor(const std::string& name, const std::string& port, const double frequency);
    virtual ~BrickPi3Motor();

    const std::string getName() const {return name;}
    uint8_t getPort() const {return port;}
    double getFreq() const {return freq;}
    double getEffort() const {return effort;}

    void schedulerCB();
};

#endif //BRICKPI3_ROS_BRICKPI3MOTOR_H
