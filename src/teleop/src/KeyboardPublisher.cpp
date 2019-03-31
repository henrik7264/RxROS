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
#include <teleop_msgs/Keyboard.h>
#include "KeyboardPublisher.h"

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "keyboard_publisher"); // Name of this Node.
    ros::NodeHandle nodeHandle;
    ros::Publisher pub = nodeHandle.advertise<teleop_msgs::Keyboard>("/keyboard", 10); // Publisher Topic /keyboard

    // Read parameter device
    std::string keyboardDevice;
    nodeHandle.param<std::string>("/keyboard_publisher/device", keyboardDevice, "/dev/input/event3");
//    nodeHandle.param<std::string>("/keyboard_publisher/device", keyboardDevice, "/dev/input/event1");

    printf("keyboard device: %s\n", keyboardDevice.c_str());

    // Open specified device. Needs to be in the group input to get access to the device!
    int fd = open(keyboardDevice.c_str(), O_RDONLY | O_NONBLOCK);
    if( fd < 0 ) {
        printf("Cannot open keyboard device %s!\n", keyboardDevice.c_str());
        exit(1);
    }

    fd_set readfds; // initialize file descriptor set.
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    bool doLoop = true;
    input_event keyboardEvent;
    while (doLoop && ros::ok()) {
        int rc = select(fd+1, &readfds, nullptr, nullptr, nullptr);  // wait for input on keyboard device
        if (rc == -1 && errno == EINTR) {
            close(fd);
            doLoop = false;
        }
        else if (rc == -1 || rc == 0) {
            printf("Failed to read keyboard. Exception: %d.\n", errno);
            close(fd);
            doLoop = false;
        }
        else {
            if (FD_ISSET(fd, &readfds)) {
                ssize_t sz = read(fd, &keyboardEvent, sizeof(keyboardEvent)); // read pressed key
                if (sz == -1) {
                    printf("Failed to read keyboard. Exception: %d.\n", errno);
                    close(fd);
                    doLoop = false;
                }
                else if (sz == sizeof(keyboardEvent)) {
                    if ((keyboardEvent.type == EV_KEY) && (keyboardEvent.value != REP_DELAY)) {
                        //printf("Key: T:%d, C:%d, V:%d\n", keyboardEvent.type, keyboardEvent.code, keyboardEvent.value);

                        teleop_msgs::Keyboard keyboard;
                        ros::Time rosTime(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
                        keyboard.time = rosTime;
                        switch (keyboardEvent.code) {
                            case KEY_UP:
                                keyboard.event = KB_EVENT_UP;
                                break;
                            case KEY_LEFT:
                                keyboard.event = KB_EVENT_LEFT;
                                break;
                            case KEY_RIGHT:
                                keyboard.event = KB_EVENT_RIGHT;
                                break;
                            case KEY_DOWN:
                                keyboard.event = KB_EVENT_DOWN;
                                break;
                            case KEY_SPACE:
                                keyboard.event = KB_EVENT_SPACE;
                                break;
                            default:
                                keyboard.event = KB_EVENT_NONE;
                                break;
                        }

                        pub.publish(keyboard);
                    }
                }
            }
        }
    }
}
