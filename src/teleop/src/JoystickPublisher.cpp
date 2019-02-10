//
// Created by hl on 8/22/18.
//

#include <fcntl.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <teleop/Joystick.h>
#include "teleop/JoystickPublisher.h"

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */

struct joystick_event {
    unsigned int time;      /* event timestamp in milliseconds */
    short value;            /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joystick_publisher"); // Name of this Node.
    ros::NodeHandle nodeHandle;
    ros::Publisher publisher = nodeHandle.advertise<teleop::Joystick>("/joystick", 10); // Publish Topic /joystick

    // Read parameter device
    std::string joystickDevice;
    nodeHandle.param<std::string>("/joystick_publisher/device", joystickDevice, "/dev/input/js0");

    int fd = open(joystickDevice.c_str(), O_RDONLY);
    if( fd < 0 ) {
        printf("Cannot open joystick device.\n");
        exit(1);
    }

    struct joystick_event joystickEvent;
    while (ros::ok()) {
        read(fd, &joystickEvent, sizeof(joystickEvent));
        if (joystickEvent.type == JS_EVENT_BUTTON || joystickEvent.type == JS_EVENT_AXIS) {
            ros::Time rosTime(joystickEvent.time, 0);
            teleop::Joystick joystick;
            joystick.time = rosTime;

            unsigned char type = joystickEvent.type;
            unsigned char number = joystickEvent.number;
            short value = joystickEvent.value;
            joystick.event = JS_EVENT_NEUTRAL;
            if (type == JS_EVENT_BUTTON) {
                if (number == 0 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON0_UP;
                }
                else if (number == 0 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON0_DOWN;
                }
                else if (number == 1 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON1_UP;
                }
                else if (number == 1 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON1_DOWN;
                }
                else if (number == 2 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON2_UP;
                }
                else if (number == 2 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON2_DOWN;
                }
                else if (number == 3 && value == 0) {
                    joystick.event = JS_EVENT_BUTTON3_UP;
                }
                else if (number == 3 && value == 1) {
                    joystick.event = JS_EVENT_BUTTON3_DOWN;
                }
            }
            else if (type == JS_EVENT_AXIS) {
                if (number==0 && value == 32767)
                {
                    joystick.event = JS_EVENT_AXIS_RIGHT;
                }
                else if (number==0 && value==-32767)
                {
                    joystick.event = JS_EVENT_AXIS_LEFT;
                }
                else if (number==1 && value==32767)
                {
                    joystick.event = JS_EVENT_AXIS_DOWN;
                }
                else if (number==1 && value==-32767) {
                    joystick.event = JS_EVENT_AXIS_UP;
                }
            }

            if (joystick.event != JS_EVENT_NEUTRAL) {
                publisher.publish(joystick);
            }
        }
    }
}
