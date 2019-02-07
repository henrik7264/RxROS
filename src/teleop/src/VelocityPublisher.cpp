//
// Created by hl on 2/7/19.
//

#include <string>
#include <stdio.h>
#include <Scheduler.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <JoystickPublisher.h>
#include <joystick/Joystick.h>
#include <KeyboardPublisher.h>
#include <keyboard/Keyboard.h>
#include <geometry_msgs/Twist.h>

class VelocityPublisher {
private:
    Bosma::Scheduler scheduler;
    ros::NodeHandle nodeHandle;
    ros::Publisher cmdVelPublisher;
    ros::Subscriber joystickSubscriber;
    ros::Subscriber keyboardSubscriber;

    // Callback functions for ROS topics.
    void schedulerCB(const std::string& msg);
    void joystickCB(const joystick::Joystick& joy);
    void keyboardCB(const keyboard::Keyboard& key);

public:
    VelocityPublisher(int argc, char** argv);
    virtual ~VelocityPublisher() {};
    void run() {ros::spin();}
};

VelocityPublisher::VelocityPublisher(int argc, char** argv) :
    cmdVelPublisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
    joystickSubscriber(nodeHandle.subscribe("/joystick", 10, &VelocityPublisher::joystickCB, this)),
    keyboardSubscriber(nodeHandle.subscribe("/keyboard", 10, &VelocityPublisher::keyboardCB, this))
{
    scheduler.every(std::chrono::milliseconds(100), &VelocityPublisher::schedulerCB, "every second");
}


void VelocityPublisher::schedulerCB(const std::string& msg)
{
    std::cout << msg << std::endl;
}


void VelocityPublisher::joystickCB(const joystick::Joystick& joy)
{
    JoystickEvents event = static_cast<JoystickEvents>(joy.event);
    switch (event) {
        case JS_EVENT_BUTTON0_DOWN:
            break;
        case JS_EVENT_BUTTON1_DOWN:
            break;
        case JS_EVENT_AXIS_UP:
            break;
        case JS_EVENT_AXIS_DOWN:
            break;
        case JS_EVENT_AXIS_LEFT:
            break;
        case JS_EVENT_AXIS_RIGHT:
            break;
        default:
            break;
    }
}


void VelocityPublisher::keyboardCB(const keyboard::Keyboard& key)
{
    KeyboardEvents event = static_cast<KeyboardEvents>(key.event);
    switch (event) {
        case KB_EVENT_UP:
            break;
        case KB_EVENT_LEFT:
            break;
        case KB_EVENT_RIGHT:
            break;
        case KB_EVENT_DOWN:
            break;
        default:
            break;
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "velocityPublisher"); // Name of this node.
    VelocityPublisher velocityPublisher(argc, argv);
    velocityPublisher.run();
}
