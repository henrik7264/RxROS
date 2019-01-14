//
// Created by hl on 12/29/18.
//

#include "CStateInitial.h"
#include "CStateMachine.h"

CStateMachine::CStateMachine():
    currentState(CStateInitial::getInstance())
{
}

void CStateMachine::changeState(CState* newState)
{
    currentState = newState;
}


void CStateMachine::handleEvent(int event)
{
    switch (event)
    {
        case 1:
            currentState->event1(this);
            break;
        case 2:
            currentState->event2(this);
            break;
        default:
            //todo: Print error message.
            break;
    }
}
