//
// Created by hl on 8/22/18.
//

#include <fcntl.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <joystick/Joystick.h>

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
            joystick.value = joystickEvent.value;
            joystick.type = joystickEvent.type;
            joystick.number = joystickEvent.number;
            pub.publish(joystick);
        }
    }
}
