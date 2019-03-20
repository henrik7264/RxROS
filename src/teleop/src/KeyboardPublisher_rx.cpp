//
// Created by hl on 8/22/18.
//

#include <rxros.h>
#include <teleop_msgs/Keyboard.h>
#include "KeyboardPublisher.h"

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "keyboard_publisher"); // Name of this Node.

    const auto keyboardDevice = rxros::Parameter::get("/keyboard_publisher/device", "/dev/input/event1"); // for other PCs
    //const auto keyboardDevice = rxros::Parameter::get("/keyboard_publisher/device", "/dev/input/event4"); // for Dell PC

    rxros::Logging().info() << "Keyboard device: " << keyboardDevice;

    auto keyboardEventToKeyboardMsg = [](const auto keyboardEvent) {
        auto makeKeyboardMsg = [=] (auto event) {
            teleop_msgs::Keyboard keyboardMsg;
            keyboardMsg.time = ros::Time(keyboardEvent.time.tv_sec, keyboardEvent.time.tv_usec);
            keyboardMsg.event = event;
            return keyboardMsg;
        };
        if (keyboardEvent.code == KEY_UP)
            return makeKeyboardMsg(KB_EVENT_UP);
        else if (keyboardEvent.code == KEY_LEFT)
            return makeKeyboardMsg(KB_EVENT_LEFT);
        else if (keyboardEvent.code == KEY_RIGHT)
            return makeKeyboardMsg(KB_EVENT_RIGHT);
        else if (keyboardEvent.code == KEY_DOWN)
            return makeKeyboardMsg(KB_EVENT_DOWN);
        else if (keyboardEvent.code == KEY_SPACE)
            return makeKeyboardMsg(KB_EVENT_SPACE);
        else
            return makeKeyboardMsg(KB_EVENT_NONE);};

    rxros::Observable<input_event>::fromDevice(keyboardDevice)
        | map(keyboardEventToKeyboardMsg)
        | publish_to_topic<teleop_msgs::Keyboard>("/keyboard");

    rxros::Logging().info() << "Spinning keyboard_publisher...";
    rxros::spin();
}
