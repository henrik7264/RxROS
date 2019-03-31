//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3TOUCH_H
#define BRICKPI3_BRICKPI3TOUCH_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include "BrickPi3Device.h"


class BrickPi3Touch: public BrickPi3Device
{
private:
    ros::Publisher touchPublisher;
    std::string name;
    std::string frameId;
    uint8_t port;
    double freq;
    unsigned int seqNo;

public:
    BrickPi3Touch(const std::string& name, const std::string& frameId, const std::string& port, const double frequency);
    virtual ~BrickPi3Touch() {}

    const std::string& getName() const {return name;}
    const std::string& getFrameId() const {return frameId;}
    const uint8_t getPort() const {return port;}
    const double getFreq() const {return freq;}

    void schedulerCB();
};


#endif //BRICKPI3_BRICKPI3TOUCH_H
