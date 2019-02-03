//
// Created by hl on 8/22/18.
//

#include <fcntl.h>
#include <linux/input.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <keyboard/Keyboard.h>
#include "KeyboardPublisher.h"

int main(int argc, char** argv) {
    int fd = open("/dev/input/event1", O_RDONLY | O_NONBLOCK);
    if( fd < 0 ) {
        printf("Cannot open keyboard device (/dev/input/event1).\n");
        exit(1);
    }

    ros::init(argc, argv, "KeyboardPublisher"); // Name of this Node.
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<keyboard::Keyboard>("/keyboard", 10); // Publish Topic /keyboard

    input_event keyboardEvent;
    while (ros::ok()) {
        read(fd, &keyboardEvent, sizeof(keyboardEvent));

        keyboard::Keyboard keyboard;
        ros::Time rosTime(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
        keyboard.time = rosTime;



    }
}
