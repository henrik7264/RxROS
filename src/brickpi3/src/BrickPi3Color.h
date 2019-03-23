//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3COLOR_H
#define BRICKPI3_BRICKPI3COLOR_H

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include "BrickPi3Device.h"


class BrickPi3Color: public BrickPi3Device
{
private:
    ros::Publisher colorPublisher;
    std::string name;
    std::string frameId;
    uint8_t port;
    double freq;
    unsigned int seqNo;

public:
    BrickPi3Color(const std::string& name, const std::string& frameId, const std::string& port, const double frequency);
    virtual ~BrickPi3Color() {}

    const std::string& getName() const {return name;}
    const std::string& getFrameId() const {return frameId;}
    const uint8_t getPort() const {return port;}
    const double getFreq() const {return freq;}

    void schedulerCB();
};


#endif //BRICKPI3_BRICKPI3COLOR_H
