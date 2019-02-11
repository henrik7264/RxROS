//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_ROS_BRICKPI3DEVICE_H
#define BRICKPI3_ROS_BRICKPI3DEVICE_H

#include <mutex>
#include "BrickPi3.h"


class BrickPi3Device
{
protected:
    BrickPi3 brickPi3;
    std::mutex mutex;

public:
    BrickPi3Device() {};
    virtual ~BrickPi3Device() {}

    virtual void schedulerCB() = 0;
};

#endif //BRICKPI3_ROS_BRICKPI3DEVICE_H
