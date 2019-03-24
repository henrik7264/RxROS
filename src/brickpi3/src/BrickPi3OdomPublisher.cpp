//
// Created by hl on 2/13/19.
//

#include <mutex>
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
#include "Node.h"


class BrickPi3OdomPublisher
{
private:
    ros::Subscriber jointStatesSubscriber;
    ros::Publisher odomPublisher;
    tf::TransformBroadcaster transformBroadcaster;
    unsigned int seqNo;
    std::string lWheelJoint;
    std::string rWheelJoint;
    double wheelRadius;
    double wheelBasis;
    double lastLWheelPosition;
    double lastRWheelPosition;
    double x, y, th;
    ros::Time lastTime;
    bool isInitialized;

    void jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates);

public:
    BrickPi3OdomPublisher(int argc, char** argv);
    virtual ~BrickPi3OdomPublisher() {};
    void run() {ros::spin();}
};

BrickPi3OdomPublisher::BrickPi3OdomPublisher(int argc, char** argv) :
    odomPublisher(Node::getHandle().advertise<nav_msgs::Odometry>("/odom", 10)),
    jointStatesSubscriber(Node::getHandle().subscribe("/joint_states", 10, &BrickPi3OdomPublisher::jointStatesSubscriberCB, this)),
    transformBroadcaster(),
    seqNo(0),
    lastLWheelPosition(0.0),
    lastRWheelPosition(0.0),
    x(0.0),
    y(0.0),
    th(0.0),
    lastTime(ros::Time::now()),
    isInitialized(false)
{
    Node::getHandle().param<std::string>("/brickpi3_odom_publisher/l_wheel_joint", lWheelJoint, "l_wheel_joint");
    Node::getHandle().param<std::string>("/brickpi3_odom_publisher/r_wheel_joint", rWheelJoint, "r_wheel_joint");
    Node::getHandle().param("/brickpi3_odom_publisher/wheel_radius", wheelRadius, 0.028); // m
    Node::getHandle().param("/brickpi3_odom_publisher/wheel_basis", wheelBasis, 0.0625); // m

    ROS_DEBUG("l_wheel_joint: %s\n", lWheelJoint.c_str());
    ROS_DEBUG("r_wheel_joint: %s m/s\n", rWheelJoint.c_str());
    ROS_DEBUG("wheel_radius: %f rad/s\n", wheelRadius);
    ROS_DEBUG("wheel_basis: %f rad/s\n", wheelBasis);
}

void BrickPi3OdomPublisher::jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates)
{
    ros::Time currTime = ros::Time::now();

    // find current position of left and right wheel
    double currLWheelPosition = 0.0;
    double currRWheelPosition = 0.0;
    for (int i = 0; i < jointStates.name.size(); i++) {
        if (lWheelJoint == jointStates.name[i]) {
            currLWheelPosition = jointStates.position[i];
        }
        else if (rWheelJoint == jointStates.name[i]) {
            currRWheelPosition = jointStates.position[i];
        }
    }

    if (isInitialized)
    {
        double deltaRWheelPosition = currRWheelPosition - lastRWheelPosition;
        double deltaLWheelPosition = currLWheelPosition - lastLWheelPosition;
        double velocityX = (deltaRWheelPosition + deltaLWheelPosition) * wheelRadius / 2.0;
        double velocityY = 0.0;
        double velocityTheta = (deltaRWheelPosition - deltaLWheelPosition) * wheelRadius / (2.0 * wheelBasis);

        double dt = (currTime - lastTime).toSec();
        double dx = (velocityX * cos(th) - velocityY * sin(th)) * dt;
        double dy = (velocityX * sin(th) + velocityY * cos(th)) * dt;
        double dth = velocityTheta * dt;

        x += dx;
        y += dy;
        th += dth;

        nav_msgs::Odometry odom;
        odom.header.stamp = currTime;
        odom.header.seq = seqNo++;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        odom.pose.covariance = {0.00001, 0.0,     0.0,     0.0,     0.0,     0.0,
                                0.0,     0.00001, 0.0,     0.0,     0.0,     0.0,
                                0.0,     0.0,     10.0000, 0.0,     0.0,     0.0,
                                0.0,     0.0,     0.0,     1.00000, 0.0,     0.0,
                                0.0,     0.0,     0.0,     0.0,     1.00000, 0.0,
                                0.0,     0.0,     0.0,     0.0,     0.0,     (velocityTheta == 0.0) ? 0.00000000001 : 1.00000};

        //set the velocity
        odom.twist.twist.linear.x = velocityX;
        odom.twist.twist.angular.z = velocityTheta;

        //publish the message
        odomPublisher.publish(odom);

        // broadcast transformation /tf
        tf::Quaternion orientation = tf::Quaternion(odom.pose.pose.orientation.x,odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
        tf::Vector3 position = tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
        transformBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(orientation, position), currTime, "odom", "base_footprint"));
    }

    lastLWheelPosition = currLWheelPosition;
    lastRWheelPosition = currRWheelPosition;
    lastTime = currTime;
    isInitialized = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brickpi3_odom_publisher"); // Name of this node.
    BrickPi3OdomPublisher brickPi3OdomPublisher(argc, argv);
    brickPi3OdomPublisher.run();
}
