//
// Created by hl on 2/13/19.
//

#include <algorithm>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>

class JointStatesAggregator
{
private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber jointStateSubscriber;
    ros::Publisher jointStatesPublisher;
    sensor_msgs::JointState jointStates;
    unsigned int seqNo;

    void jointStateSubscriberCB(const sensor_msgs::JointState& jointState);

public:
    JointStatesAggregator(int argc, char** argv);
    virtual ~JointStatesAggregator() {}
    void run() {ros::spin();}
};


JointStatesAggregator::JointStatesAggregator(int argc, char** argv):
    jointStatesPublisher(nodeHandle.advertise<sensor_msgs::JointState>("/joint_states", 10)),
    jointStateSubscriber(nodeHandle.subscribe("/joint_state", 10, &JointStatesAggregator::jointStateSubscriberCB, this)),
    seqNo(0)
{
}

void JointStatesAggregator::jointStateSubscriberCB(const sensor_msgs::JointState& jointState)
{

    for (int i = 0; i < jointStates.name.size(); i++) {
        if (jointStates.name[i] == jointState.name[0]) {
            jointStates.name.erase(jointStates.name.begin() + i);
            jointStates.effort.erase(jointStates.effort.begin() + i);
            jointStates.position.erase(jointStates.position.begin() + i);
            jointStates.velocity.erase(jointStates.velocity.begin() + i);
        }
    }
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

        jointStates.name.clear();
        jointStates.effort.clear();
        jointStates.position.clear();
        jointStates.velocity.clear();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joint_state_aggregator"); // Name of this node.
    JointStatesAggregator jointStatesAggregator(argc, argv);
    jointStatesAggregator.run();
}
