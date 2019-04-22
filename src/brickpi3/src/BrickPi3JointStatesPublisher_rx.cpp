//
// Created by hl on 2/13/19.
//

#include <algorithm>
#include <rxros.h>
#include <sensor_msgs/JointState.h>
#include "Node.h"
using namespace rxcpp::operators;
using namespace rxros::operators;


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
    if (std::find(jointStates.name.begin(), jointStates.name.end(), jointState.name[0]) != jointStates.name.end()) { // name already exists in the jointStates
        jointStates.name.clear();
        jointStates.effort.clear();
        jointStates.position.clear();
        jointStates.velocity.clear();
    }
//    for (int i = 0; i < jointStates.name.size(); i++) {
//        if (jointStates.name[i] == jointState.name[0]) {
//            jointStates.name.erase(jointStates.name.begin() + i);
//            jointStates.effort.erase(jointStates.effort.begin() + i);
//            jointStates.position.erase(jointStates.position.begin() + i);
//            jointStates.velocity.erase(jointStates.velocity.begin() + i);
//        }
//    }
    jointStates.name.push_back(jointState.name[0]);
    jointStates.effort.push_back(jointState.effort[0]);
    jointStates.position.push_back(jointState.position[0]); // rad
    jointStates.velocity.push_back(jointState.velocity[0]); // rad/s

    if (jointStates.name.size() == 2) // todo: Find way to determine the number of motors.
    {
        //jointStates.header.frame_id = name;
        jointStates.header.stamp = ros::Time::now();
        jointStates.header.seq = seqNo++;

        jointStatesPublisher.publish(jointStates);
    }
}

int main(int argc, char** argv)
{
    rxros::init(argc, argv, "brickpi3_joint_states_publisher"); // Name of this node.

    auto js = rxros::observable::from_topic<sensor_msgs::JointState>("/joint_state");
    auto as = js.filter([](auto& js){return (js.name[0] == "a");});
    auto bs = js.filter([](auto& js){return (js.name[0] == "b");});
//    as.combine_latest(
//        [=](const auto& a, const auto& b) {
//            return std::make_tuple(a, b);}, bs)
//
//
//        | group_by(
//            [] (auto& js) {if () },
//            [] (auto& js) {})
//        | publish_to_topic<sensor_msgs::JointState>("/joint_states");



    rxros::spin();
}
