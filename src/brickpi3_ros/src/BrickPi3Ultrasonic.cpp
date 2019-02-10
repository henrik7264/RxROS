//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include "BrickPi3Ultrasonic.h"
using namespace std;


BrickPi3Ultrasonic::BrickPi3Ultrasonic(const std::string& aName, const std::string& aFrameId,const std::string& aPort, const double aFrequency, const double aMinRange, const double aMaxRange, const double aSpreadAngle)
{
    name = aName;
    port = aPort;
    frameId = aFrameId;
    freq = aFrequency;
    minRange = aMinRange;
    maxRange = aMaxRange;
    spreadAngle = aSpreadAngle;
}


void BrickPi3Ultrasonic::schedulerCB()
{
    std::lock_guard<std::mutex> guard(mutex);
    cout << "I am ultrasonic device " << name << endl;
}
