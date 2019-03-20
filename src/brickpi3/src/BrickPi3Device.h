//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3DEVICE_H
#define BRICKPI3_BRICKPI3DEVICE_H

#include <climits>
#include <string>
#include <mutex>
#include "BrickPi3.h"


static uint8_t port2id(const std::string& port)
{
    int id = ((port == "PORT_A") ? PORT_A :
             ((port == "PORT_B") ? PORT_B :
             ((port == "PORT_C") ? PORT_C :
             ((port == "PORT_D") ? PORT_D :
             ((port == "PORT_1") ? PORT_1 :
             ((port == "PORT_2") ? PORT_2 :
             ((port == "PORT_3") ? PORT_3 :
             ((port == "PORT_4") ? PORT_4 : INT_MAX))))))));
    return static_cast<uint8_t>(id);
}


class BrickPi3Device
{
protected:
    BrickPi3 brickPi3;
    std::mutex mutex;

public:
    BrickPi3Device() {brickPi3.detect();}
    virtual ~BrickPi3Device() {brickPi3.reset_all();}

    virtual void schedulerCB() = 0;
};

#endif //BRICKPI3_BRICKPI3DEVICE_H
