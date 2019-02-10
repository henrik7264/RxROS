//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_ROS_BRICKPI3ULTRASONIC_H
#define BRICKPI3_ROS_BRICKPI3ULTRASONIC_H

#include <string>
#include "BrickPi3Device.h"

class BrickPi3Ultrasonic: public BrickPi3Device
{
private:
    std::string name;
    std::string frameId;
    std::string port;
    double freq;
    double minRange;
    double maxRange;
    double spreadAngle;

public:
    BrickPi3Ultrasonic(const std::string& name, const std::string& frameId,const std::string& port, const double frequency, const double minRange, const double maxRange, const double spreadAngle);
    virtual ~BrickPi3Ultrasonic() {}

    const std::string &getName() const {return name;}
    const std::string &getFrameId() const {return frameId;}
    const std::string &getPort() const {return port;}
    double getFreq() const {return freq;}
    double getMinRange() const {return minRange;}
    double getMaxRange() const {return maxRange;}
    double getSpreadAngle() const {return spreadAngle;}

    void schedulerCB();
};


#endif //BRICKPI3_ROS_BRICKPI3ULTRASONIC_H
