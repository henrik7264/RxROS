//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <brickpi3_msgs/Color.h>
#include "Node.h"
#include "BrickPi3Color.h"


BrickPi3Color::BrickPi3Color(const std::string& aName, const std::string& aFrameId, const std::string& aPort, const double aFrequency):
    colorPublisher(Node::getHandle().advertise<brickpi3_msgs::Color>("/" + aName, 10))
{
    name = aName;
    frameId = aFrameId;
    port = port2id(aPort);
    freq = aFrequency;
    seqNo = 0;

    brickPi3.set_sensor_type(port, SENSOR_TYPE_NXT_COLOR_FULL);
}


void BrickPi3Color::schedulerCB()
{
    std::lock_guard<std::mutex> guard(mutex);

    sensor_color_t sensorColor;
    int rc = brickPi3.get_sensor(port, &sensorColor);
    if (rc == 0) {
        ROS_DEBUG("Color sensor: detected %d red %4d green %4d blue %4d ambient %4d", sensorColor.color, sensorColor.reflected_red, sensorColor.reflected_green, sensorColor.reflected_blue, sensorColor.ambient);

        brickpi3_msgs::Color color;
        color.header.frame_id = frameId;
        color.header.stamp = ros::Time::now();
        color.header.seq = seqNo++;
        color.r = static_cast<double>(sensorColor.reflected_red);
        color.g = static_cast<double>(sensorColor.reflected_green);
        color.b = static_cast<double>(sensorColor.reflected_blue);
        color.intensity = static_cast<double>(sensorColor.ambient);

        colorPublisher.publish(color);
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Color sensor %s, rc %d", name.c_str(), rc);
    }
}
