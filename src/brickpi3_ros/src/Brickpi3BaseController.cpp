//
// Created by hl on 2/13/19.
//

#include <mutex>
#include <stdio.h>
#include <string>
#include <Scheduler.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <brickpi3_ros/JointCommand.h>


class BrickPi3BaseController {
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher jointCommandPublisher;
    ros::Subscriber cmdVelSubscriber;
    ros::Subscriber jointStatesSubscriber;
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

    void cmdVelSubscriberCB(const geometry_msgs::Twist& jointStates);
    void jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates);

public:
    BrickPi3BaseController(int argc, char** argv);
    virtual ~BrickPi3BaseController() {};
    void run() {ros::spin();}
};

BrickPi3BaseController::BrickPi3BaseController(int argc, char** argv) :
    jointCommandPublisher(nodeHandle.advertise<brickpi3_ros::JointCommand>("/joint_command", 10)),
    cmdVelSubscriber(nodeHandle.subscribe("/cmd_vel", 10, &BrickPi3BaseController::cmdVelSubscriberCB, this)),
    jointStatesSubscriber(nodeHandle.subscribe("/joint_states", 10, &BrickPi3BaseController::jointStatesSubscriberCB, this)),
    seqNo(0),
    lastLWheelPosition(0.0),
    lastRWheelPosition(0.0),
    x(0.0),
    y(0.0),
    th(0.0),
    lastTime(ros::Time::now()),
    isInitialized(false)
{
    nodeHandle.param<std::string>("/brickpi3_base_controller/l_wheel_joint", lWheelJoint, "l_wheel_joint");
    nodeHandle.param<std::string>("/brickpi3_base_controller/r_wheel_joint", rWheelJoint, "r_wheel_joint");
    nodeHandle.param("/brickpi3_base_controller/wheel_radius", wheelRadius, 0.028); // m
    nodeHandle.param("/brickpi3_base_controller/wheel_basis", wheelBasis, 0.0625); // m

    printf("l_wheel_joint: %s\n", lWheelJoint.c_str());
    printf("r_wheel_joint: %s m/s\n", rWheelJoint.c_str());
    printf("wheel_radius: %f rad/s\n", wheelRadius);
    printf("wheel_basis: %f rad/s\n", wheelBasis);
}

void BrickPi3BaseController::cmdVelSubscriberCB(const geometry_msgs::Twist& twist)
{
}


void BrickPi3BaseController::jointStatesSubscriberCB(const sensor_msgs::JointState& jointStates)
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brickpi3_base_controller"); // Name of this node.
    BrickPi3BaseController BrickPi3BaseController(argc, argv);
    BrickPi3BaseController.run();
}

