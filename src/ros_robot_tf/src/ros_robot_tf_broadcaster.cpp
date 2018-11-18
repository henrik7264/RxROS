#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


class RosRobotTfBroadcaster {
private:
    ros::NodeHandle nodeHandle;
    ros::Subscriber odomSubscriber;
    tf::TransformBroadcaster transformBroadcaster;

    void odomCB(const nav_msgs::Odometry& odom);

public:
    RosRobotTfBroadcaster(int argc, char** argv);
    virtual ~RosRobotTfBroadcaster() {};
    void run() {ros::spin();}
};

RosRobotTfBroadcaster::RosRobotTfBroadcaster(int argc, char** argv)
{
    odomSubscriber = nodeHandle.subscribe("/odom", 10, &RosRobotTfBroadcaster::odomCB, this);
}

void  RosRobotTfBroadcaster::odomCB(const nav_msgs::Odometry& odom)
{
    tf::Quaternion orientation = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    tf::Vector3 position = tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    transformBroadcaster.sendTransform(tf::StampedTransform(tf::Transform(orientation, position), ros::Time::now(), "odom", "base_link"));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_robot_tf_broadcaster"); // Name of this node.
    RosRobotTfBroadcaster rosRobotTfBroadcaster(argc, argv);
    rosRobotTfBroadcaster.run();
}



/*
int main(int argc, char** argv){
    ros::init(argc, argv, "ros_robot_tf_publisher");
    ros::Subscriber odomSubscriber;
    ros::Rate rate(10);
    tf::TransformBroadcaster transformBroadcaster;



    while(nodeHandle.ok()) {
        transformBroadcaster.sendTransform(
            tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), ros::Time::now(), "base_link", "map"));

        rate.sleep();
    }
}
*/