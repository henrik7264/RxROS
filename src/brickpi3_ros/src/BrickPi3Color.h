//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_ROS_BRICKPI3COLOR_H
#define BRICKPI3_ROS_BRICKPI3COLOR_H

#include <string>
#include "BrickPi3Device.h"

class BrickPi3Color: public BrickPi3Device
{
private:
    std::string name;
    std::string frameId;
    std::string port;
    double freq;

public:
    BrickPi3Color(const std::string& name, const std::string& frameId, const std::string& port, const double frequency);
    virtual ~BrickPi3Color() {}

    const std::string &getName() const {return name;}
    const std::string &getFrameId() const {return frameId;}
    const std::string &getPort() const {return port;}
    double getFreq() const {return freq;}

    void schedulerCB();
};


#endif //BRICKPI3_ROS_BRICKPI3COLOR_H
