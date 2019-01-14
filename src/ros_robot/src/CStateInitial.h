//
// Created by hl on 12/29/18.
//

#ifndef ROS_ROBOT_CSTATEINITIAL_H
#define ROS_ROBOT_CSTATEINITIAL_H
#include "CState.h"

class CStateInitial: public CState
{
private:
    CStateInitial() {} // Singleton

public:
    virtual ~CStateInitial() {}
    static CStateInitial* getInstance(); // Singleton

    void event1(CStateMachine* const stateMachine) {};
    void event2(CStateMachine* const stateMachine) {};
};


#endif //ROS_ROBOT_CSTATEINITIAL_H
