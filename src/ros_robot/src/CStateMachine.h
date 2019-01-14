//
// Created by hl on 12/29/18.
//

#ifndef ROS_ROBOT_CSTATEMACHINE_H
#define ROS_ROBOT_CSTATEMACHINE_H
#include "CState.h"

class CStateMachine {
private:
    CState* currentState;

public:
    CStateMachine();
    virtual ~CStateMachine() {}

    void changeState(CState* newState);

    void handleEvent(int e);
};


#endif //ROS_ROBOT_CSTATEMACHINE_H
