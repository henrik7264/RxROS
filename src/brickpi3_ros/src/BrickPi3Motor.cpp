//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <Scheduler.h>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include "BrickPi3Motor.h"
using namespace std;


BrickPi3Motor::BrickPi3Motor(const std::string& aName, const std::string& aPort, const int aFrequency)
{
    name = aName;
    port = aPort;
    freq = aFrequency;
}

void BrickPi3Motor::schedulerCB()
{
}
