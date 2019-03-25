//
// Created by hl on 8/22/18.
//

#include <rxros.h>
#include <teleop_msgs/Joystick.h>
#include "JoystickPublisher.h"

#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */

struct joystick_event
{
    unsigned int time;      /* event timestamp in milliseconds */
    short value;            /* value */
    unsigned char type;     /* event type */
    unsigned char number;   /* axis/button number */
};

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "joystick_publisher"); // Name of this Node.

    const auto joystickDevice = rxros::Parameter::get("/joystick_publisher/device", "/dev/input/js0");

    rxros::Logging().info() << "Joystick device: " << joystickDevice;

    auto joystickEventToJoystickMsg = [=](const auto joystickEvent) {
        auto makeJoystickMsg = [=] (auto event) {
            teleop_msgs::Joystick joystickMsg;
            joystickMsg.time = ros::Time(joystickEvent.time, 0);
            joystickMsg.event = event;
            return joystickMsg;};
        if (joystickEvent.type == JS_EVENT_BUTTON) {
            if (joystickEvent.number == 0 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON0_UP);
            else if (joystickEvent.number == 0 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON0_DOWN);
            else if (joystickEvent.number == 1 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON1_UP);
            else if (joystickEvent.number == 1 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON1_DOWN);
            else if (joystickEvent.number == 2 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON2_UP);
            else if (joystickEvent.number == 2 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON2_DOWN);
            else if (joystickEvent.number == 3 && joystickEvent.value == 0)
                return makeJoystickMsg(JS_EVENT_BUTTON3_UP);
            else if (joystickEvent.number == 3 && joystickEvent.value == 1)
                return makeJoystickMsg(JS_EVENT_BUTTON3_DOWN);
        }
        else if (joystickEvent.type == JS_EVENT_AXIS) {
            if (joystickEvent.number == 0 && joystickEvent.value == 32767)
                return makeJoystickMsg(JS_EVENT_AXIS_RIGHT);
            else if (joystickEvent.number == 0 && joystickEvent.value == -32767)
                return makeJoystickMsg(JS_EVENT_AXIS_LEFT);
            else if (joystickEvent.number == 1 && joystickEvent.value == 32767)
                return makeJoystickMsg(JS_EVENT_AXIS_DOWN);
            else if (joystickEvent.number == 1 && joystickEvent.value == -32767)
                return makeJoystickMsg(JS_EVENT_AXIS_UP);
        }
        else
            return makeJoystickMsg(JS_EVENT_NEUTRAL);};

    rxros::Observable::fromDevice<joystick_event>(joystickDevice)
        | map(joystickEventToJoystickMsg)
        | publish_to_topic<teleop_msgs::Joystick>("/joystick");

    rxros::Logging().info() << "Spinning joystick_publisher ...";
    rxros::spin();
}
