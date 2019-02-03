//
// Created by hl on 8/22/18.
//
#include <linux/input.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <ros/ros.h>
#include <ros/console.h>
#include <keyboard/Keyboard.h>

int main(int argc, char** argv) {
    int fd = open("/dev/input/event1", O_RDONLY | O_NONBLOCK);
    if( fd < 0 ) {
        printf("Cannot open keyboard device.\n");
        exit(1);
    }

    ros::init(argc, argv, "KeyboardPublisher"); // Name of this Node.
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<keyboard::Keyboard>("/keyboard", 10); // Publish Topic /keyboard

    bool doloop = true;
    input_event keyboardEvent;
    while (doloop && ros::ok()) {
        size_t rc = read(fd, &keyboardEvent, sizeof(keyboardEvent));

        if (rc == -1 && errno == EINTR) {
            doloop = false;
        }
        else if (rc == -1) {
            printf("Reading keyboard device returned error %d.\n", errno);
            doloop = false;
        }
        else if (rc == sizeof(keyboardEvent)) {
            if(keyboardEvent.type == EV_KEY) {
                keyboard::Keyboard keyboard;
                ros::Time rosTime(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
                keyboard.time = rosTime;
                keyboard.key = keyboardEvent.code;

                pub.publish(keyboard);
            }
        }
    }
}
