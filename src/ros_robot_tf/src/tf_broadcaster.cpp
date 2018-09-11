#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ros_robot_tf_publisher");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(10);
    tf::TransformBroadcaster transformBroadcaster;

    while(nodeHandle.ok()) {
        transformBroadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.07)),
                ros::Time::now(),
                "base_link",
                "laser_link"));
        rate.sleep();
    }
}
