//
// Created by hl on 12/29/18.
//

#include "CStateInitial.h"

CStateInitial* CStateInitial::getInstance()
{
    static CStateInitial self;
    return &self;
}
