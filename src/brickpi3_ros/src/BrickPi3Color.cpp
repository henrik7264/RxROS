//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include "BrickPi3Color.h"
using namespace std;


BrickPi3Color::BrickPi3Color(const string& aName, const string& aFrameId, const string& aPort, const double aFrequency)
{
    name = aName;
    port = aPort;
    frameId = aFrameId;
    freq = aFrequency;
}


void BrickPi3Color::schedulerCB()
{
    lock_guard<std::mutex> guard(mutex);
    cout << "I am color device " << name << endl;
}
