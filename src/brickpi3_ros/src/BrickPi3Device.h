//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_ROS_BRICKPI3DEVICE_H
#define BRICKPI3_ROS_BRICKPI3DEVICE_H

#include <brickpi3/BrickPi3.h>
#include <mutex>

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
