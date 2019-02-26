//
// Created by hl on 8/27/18.
//

#ifndef TELEOP_KEYBOARDPUBLISHER_H
#define TELEOP_KEYBOARDPUBLISHER_H

enum KeyboardEvents
{
    KB_EVENT_NONE = 100, // to avoid conflicts with JoystickEvents
    KB_EVENT_UP,
    KB_EVENT_LEFT,
    KB_EVENT_RIGHT,
    KB_EVENT_DOWN,
    KB_EVENT_SPACE
};

#endif //TELEOP_KEYBOARDPUBLISHER_H
