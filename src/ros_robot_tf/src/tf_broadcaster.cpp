#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ros_robot_tf_publisher");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    tf::TransformBroadcaster transformBroadcaster;

    while(nodeHandle.ok()) {
        transformBroadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                ros::Time::now(),
                "base_frame",
                "laser_frame"));
        rate.sleep();
    }
}
