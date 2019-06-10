//
// Created by hl on 2/13/19.
//

#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include "Node.h"


class BrickPi3JointStatesPublisher
{
private:
    ros::Subscriber jointStateSubscriber;
    ros::Publisher jointStatesPublisher;
    sensor_msgs::JointState jointStates;
    unsigned int seqNo;

    void jointStateSubscriberCB(const sensor_msgs::JointState& jointState);

public:
    BrickPi3JointStatesPublisher(int argc, char** argv);
    virtual ~BrickPi3JointStatesPublisher() {}
    void run() {ros::spin();}
};


BrickPi3JointStatesPublisher::BrickPi3JointStatesPublisher(int argc, char** argv):
    jointStatesPublisher(Node::getHandle().advertise<sensor_msgs::JointState>("/joint_states", 10)),
    jointStateSubscriber(Node::getHandle().subscribe("/joint_state", 10, &BrickPi3JointStatesPublisher::jointStateSubscriberCB, this)),
    seqNo(0)
{
}

void BrickPi3JointStatesPublisher::jointStateSubscriberCB(const sensor_msgs::JointState& jointState)
{
    if (std::find(jointStates.name.begin(), jointStates.name.end(), jointState.name[0]) != jointStates.name.end()) { // name already exists in the makeJointStates
        jointStates.name.clear();
        jointStates.effort.clear();
        jointStates.position.clear();
        jointStates.velocity.clear();
    }
//    for (int i = 0; i < makeJointStates.name.size(); i++) {
//        if (makeJointStates.name[i] == jointState.name[0]) {
//            makeJointStates.name.erase(makeJointStates.name.begin() + i);
//            makeJointStates.effort.erase(makeJointStates.effort.begin() + i);
//            makeJointStates.position.erase(makeJointStates.position.begin() + i);
//            makeJointStates.velocity.erase(makeJointStates.velocity.begin() + i);
//        }
//    }
    jointStates.name.push_back(jointState.name[0]);
    jointStates.effort.push_back(jointState.effort[0]);
    jointStates.position.push_back(jointState.position[0]); // rad
    jointStates.velocity.push_back(jointState.velocity[0]); // rad/s

    if (jointStates.name.size() == 2) // todo: Find way to determine the number of motors.
    {
        //makeJointStates.header.frame_id = name;
        jointStates.header.stamp = ros::Time::now();
        jointStates.header.seq = seqNo++;

        jointStatesPublisher.publish(jointStates);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brickpi3_joint_states_publisher"); // Name of this node.
    BrickPi3JointStatesPublisher jointStatesPublisher(argc, argv);
    jointStatesPublisher.run();
}
