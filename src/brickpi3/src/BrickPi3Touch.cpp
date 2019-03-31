//
// Created by hl on 2/10/19.
//

#include <string>
#include <iostream>
#include <brickpi3_msgs/Contact.h>
#include "Node.h"
#include "BrickPi3Touch.h"


BrickPi3Touch::BrickPi3Touch(const std::string& aName, const std::string& aFrameId, const std::string& aPort, const double aFrequency):
    touchPublisher(Node::getHandle().advertise<brickpi3_msgs::Contact>("/" + aName, 10))
{
    name = aName;
    frameId = aFrameId;
    port = port2id(aPort);
    freq = aFrequency;
    seqNo = 0;

    brickPi3.set_sensor_type(port, SENSOR_TYPE_TOUCH_NXT);
}


void BrickPi3Touch::schedulerCB()
{
    std::lock_guard<std::mutex> guard(mutex);

    sensor_touch_t sensorTouch;
    int rc = brickPi3.get_sensor(port, &sensorTouch);
    if (rc == 0) {
        ROS_DEBUG("Touch sensor: pressed %s", (sensorTouch.pressed) ? "True" : "False");

        brickpi3_msgs::Contact contact;
        contact.header.frame_id = frameId;
        contact.header.stamp = ros::Time::now();
        contact.header.seq = seqNo++;
        contact.contact = sensorTouch.pressed;

        touchPublisher.publish(contact);
    }
    else {
        ROS_ERROR("BrickPi3 failed to read Color sensor %s, rc %d", name.c_str(), rc);
    }
}
