//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <brickpi3_ros/Range.h>
#include "BrickPi3Ultrasonic.h"
using namespace std;


BrickPi3Ultrasonic::BrickPi3Ultrasonic(const std::string& aName, const std::string& aFrameId,const std::string& aPort, const double aFrequency, const double aMinRange, const double aMaxRange, const double aSpreadAngle):
    ultrasonicPublisher(nodeHandle.advertise<brickpi3_ros::Range>("/" + aName, 10))
{
    name = aName;
    frameId = aFrameId;
    port = port2id(aPort);
    freq = aFrequency;
    minRange = aMinRange;
    maxRange = aMaxRange;
    spreadAngle = aSpreadAngle;
    seqNo = 0;

    brickPi3.set_sensor_type(port, SENSOR_TYPE_NXT_ULTRASONIC);
}


void BrickPi3Ultrasonic::schedulerCB()
{
    sensor_ultrasonic_t sensorUltrasonic;
    int rc = brickPi3.get_sensor(port, &sensorUltrasonic);
    if (rc == 0) {
        printf("Ultrasonic sensor: CM %5.1f Inches %5.1f   ", sensorUltrasonic.cm, sensorUltrasonic.inch);

        brickpi3_ros::Range range;
        range.header.frame_id = frameId;
        range.header.stamp = ros::Time::now();
        range.header.seq = seqNo++;
        range.range = sensorUltrasonic.cm / 100.0;
        range.range_min = minRange;
        range.range_max = maxRange;
        range.spread_angle = spreadAngle;

        ultrasonicPublisher.publish(range);
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Ultrasonic sensor %s, rc %d", name.c_str(), rc);
    }
}
