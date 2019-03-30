//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_NODE_H
#define BRICKPI3_NODE_H

#include <ros/ros.h>

class Node
{
private:
    Node() = default;

public:
    ~Node() = default;

    static ros::NodeHandle& getHandle() {
        static ros::NodeHandle nodehandle;
        return nodehandle;
    }
};

#endif //BRICKPI3_NODE_H
