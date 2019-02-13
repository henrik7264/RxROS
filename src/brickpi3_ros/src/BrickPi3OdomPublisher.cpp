//
// Created by hl on 2/13/19.
//

#include <stdio.h>
#include <string>
#include <Scheduler.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class BrickPi3OdomPublisher {
private:
    Bosma::Scheduler scheduler;
    ros::NodeHandle nodeHandle;
    ros::Subscriber jointStatesSubscriber;
    unsigned int seqNo;

    void schedulerCB();
    void jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates);

public:
    BrickPi3OdomPublisher(int argc, char** argv);
    virtual ~BrickPi3OdomPublisher() {};
    void run() {ros::spin();}
};

BrickPi3OdomPublisher::BrickPi3OdomPublisher(int argc, char** argv) :
    jointStatesSubscriber(nodeHandle.subscribe("/joint_states", 10, &BrickPi3OdomPublisher::jointStatesSubscriberCB, this)),
    seqNo(0)
{

}

void BrickPi3OdomPublisher::jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates)
{
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "brickpi3_odom_publisher"); // Name of this node.
    BrickPi3OdomPublisher brickPi3OdomPublisher(argc, argv);
    brickPi3OdomPublisher.run();
}
