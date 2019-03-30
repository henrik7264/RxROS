//
// Created by hl on 8/22/18.
//

#include <rxros.h>
#include <teleop_msgs/Keyboard.h>
#include "KeyboardPublisher.h"

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "keyboard_publisher"); // Name of this Node.

    const auto keyboardDevice = rxros::Parameter::get("/keyboard_publisher/device", "/dev/input/event4"); // Use event4 for dell and event1 for others

    rxros::Logging().info() << "Keyboard device: " << keyboardDevice;

    auto keyboardEvent2KeyboardMsg = [](const auto keyboardEvent) {
        auto makeKeyboardMsg = [=] (auto event) {
            teleop_msgs::Keyboard keyboardMsg;
            keyboardMsg.time = ros::Time(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
            keyboardMsg.event = event;
            return keyboardMsg;};
        if ((keyboardEvent.type == EV_KEY) && (keyboardEvent.value != REP_DELAY)) {
            if (keyboardEvent.code==KEY_UP)
                return makeKeyboardMsg(KB_EVENT_UP);
            else if (keyboardEvent.code==KEY_LEFT)
                return makeKeyboardMsg(KB_EVENT_LEFT);
            else if (keyboardEvent.code==KEY_RIGHT)
                return makeKeyboardMsg(KB_EVENT_RIGHT);
            else if (keyboardEvent.code==KEY_DOWN)
                return makeKeyboardMsg(KB_EVENT_DOWN);
            else if (keyboardEvent.code==KEY_SPACE)
                return makeKeyboardMsg(KB_EVENT_SPACE);
        }
        return makeKeyboardMsg(KB_EVENT_NONE);};

    rxros::Observable::fromDevice<input_event>(keyboardDevice)
        | map(keyboardEvent2KeyboardMsg)
        | publish_to_topic<teleop_msgs::Keyboard>("/keyboard");

    rxros::Logging().info() << "Spinning keyboard_publisher...";
    rxros::spin();
}
