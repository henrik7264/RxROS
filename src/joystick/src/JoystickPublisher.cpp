//
// Created by hl on 8/22/18.
//

#include <fcntl.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <joystick/Joystick.h>
#include "JoystickPublisher.h"

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */

struct joystick_event {
    unsigned int time;      /* event timestamp in milliseconds */
    short value;            /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

int main(int argc, char** argv) {
    int fd = open("/dev/input/js0", O_RDONLY);
    if( fd < 0 ) {
        printf("Cannot open joystick device.\n");
        exit(1);
    }

    ros::init(argc, argv, "JoystickPublisher"); // Name of this Node.
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<joystick::Joystick>("/joystick", 10); // Publish Topic /joystick

    struct joystick_event joystickEvent;
    while (ros::ok()) {
        read(fd, &joystickEvent, sizeof(joystickEvent));
        if (joystickEvent.type == JS_EVENT_BUTTON || joystickEvent.type == JS_EVENT_AXIS) {
            ros::Time rosTime(joystickEvent.time, 0);
            joystick::Joystick joystick;
            joystick.time = rosTime;

            unsigned char type = joystickEvent.type;
            unsigned char number = joystickEvent.number;
            short value = joystickEvent.value;
            joystick.event = JOYSTICK_NEUTRAL;
            if (type == JS_EVENT_BUTTON) {
                if (number == 0 && value == 0) {
                    joystick.event = JOYSTICK_BUTTON0_UP;
                }
                else if (number == 0 && value == 1) {
                    joystick.event = JOYSTICK_BUTTON0_DOWN;
                }
                else if (number == 1 && value == 0) {
                    joystick.event = JOYSTICK_BUTTON1_UP;
                }
                else if (number == 1 && value == 1) {
                    joystick.event = JOYSTICK_BUTTON1_DOWN;
                }
                else if (number == 2 && value == 0) {
                    joystick.event = JOYSTICK_BUTTON2_UP;
                }
                else if (number == 2 && value == 1) {
                    joystick.event = JOYSTICK_BUTTON2_DOWN;
                }
                else if (number == 3 && value == 0) {
                    joystick.event = JOYSTICK_BUTTON3_UP;
                }
                else if (number == 3 && value == 1) {
                    joystick.event = JOYSTICK_BUTTON3_DOWN;
                }
            }
            else if (type == JS_EVENT_AXIS) {
                if (number==0 && value == 32767)
                {
                    joystick.event = JOYSTICK_AXIS_RIGHT;
                }
                else if (number==0 && value==-32767)
                {
                    joystick.event = JOYSTICK_AXIS_LEFT;
                }
                else if (number==1 && value==32767)
                {
                    joystick.event = JOYSTICK_AXIS_DOWN;
                }
                else if (number==1 && value==-32767) {
                    joystick.event = JOYSTICK_AXIS_UP;
                }
            }

            if (joystick.event != JOYSTICK_NEUTRAL) {
                pub.publish(joystick);
            }
        }
    }
}
