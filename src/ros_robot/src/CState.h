//
// Created by hl on 12/29/18.
//

#ifndef ROS_ROBOT_CSTATE_H
#define ROS_ROBOT_CSTATE_H

class CStateMachine;
class CState {
private:
public:
    CState() {};
    virtual ~CState() {}

    virtual void event1(CStateMachine* const stateMachine) = 0;
    virtual void event2(CStateMachine* const stateMachine) = 0;
};


#endif //ROS_ROBOT_CSTATE_H
