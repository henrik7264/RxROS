#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <nxt_msgs/JointCommand.h>
#include <nxt_msgs/Contact.h>
#include <nxt_msgs/Color.h>
#include <nxt_msgs/Range.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#define MAX_EFFORT 12
#define ZERO_EFFORT 6
#define MIN_EFFORT 0
static float Efforts[] = {-1.0F, -0.9F, -0.8F, -0.7F, -0.6F, -0.5F, 0.0F, 0.5F, 0.6F, 0.7F, 0.8F, 0.9F, 1.0F};

class RosRobot {
private:
    ros::NodeHandle nodeHandle;
    ros::Publisher motorPublisher;
    ros::Publisher cmdVelPublisher;
//    ros::Publisher laserScanPublisher;
//    ros::Publisher pointCloudPublisher;
    ros::Subscriber joystickSubscriber;
    ros::Subscriber colorSubscriber;
    ros::Subscriber rangeSubscriber;
    ros::Subscriber motorSubscriber;
    ros::Subscriber motorsSubscriber;
    ros::Subscriber twistSubscriber;
    ros::Subscriber odomSubscriber;

    int effortMotor1 = ZERO_EFFORT;
    int effortMotor2 = ZERO_EFFORT;
    // Callback functions for ROS topics.
    void colorCB(const nxt_msgs::Color& col);
    void rangeCB(const nxt_msgs::Range& rng);
    void motorCB(const sensor_msgs::JointState& mot);
    void motorsCB(const sensor_msgs::JointState& mot);
    void twistCB(const geometry_msgs::Twist& twist);
    void odomCB(const nav_msgs::Odometry& odom);

public:
    RosRobot(int argc, char** argv);
    virtual ~RosRobot() {};
    void run() {ros::spin();}
};

RosRobot::RosRobot(int argc, char** argv) :
    motorPublisher(nodeHandle.advertise<nxt_msgs::JointCommand>("/joint_command", 10)),
    cmdVelPublisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10)),
//    laserScanPublisher(nodeHandle.advertise<sensor_msgs::LaserScan>("/scan", 10)),
//    pointCloudPublisher(nodeHandle.advertise<sensor_msgs::PointCloud>("/cloud", 10)),
    colorSubscriber(nodeHandle.subscribe("/color_sensor", 10, &RosRobot::colorCB, this)),
    rangeSubscriber(nodeHandle.subscribe("/ultrasonic_sensor", 10, &RosRobot::rangeCB, this)),
    motorSubscriber(nodeHandle.subscribe("/joint_state", 10, &RosRobot::motorCB, this)),
    motorsSubscriber(nodeHandle.subscribe("/joint_states", 10, &RosRobot::motorsCB, this)),
    twistSubscriber(nodeHandle.subscribe("/cmd_vel", 10, &RosRobot::twistCB, this)),
    odomSubscriber(nodeHandle.subscribe("/odom", 10, &RosRobot::odomCB, this))
{
}

void RosRobot::colorCB(const nxt_msgs::Color& col)
{
    static bool firstTime = true;
    static bool goLeft = true;
    double r = col.r;
    double g = col.g;
    double b = col.b;
    double intensity = col.intensity;
/*
    if (r==0.0 && g==0.0 && b==0.0) { // black
        if (goLeft || firstTime) {
            geometry_msgs::Twist turnLeft;
            turnLeft.linear.x = 0.035 / 2.0;
            turnLeft.angular.z = 0.55 / 4.0;
            cmdVelPublisher.publish(turnLeft);
            goLeft = false;
            firstTime = false;
        }
    }
    else if (r == 1.0 && g == 1.0 && b == 1.0) {
        if (!goLeft || firstTime)
        {
            geometry_msgs::Twist turnRight;
            turnRight.linear.x = 0.035 / 2.0;
            turnRight.angular.z = -0.55 / 10.0;
            cmdVelPublisher.publish(turnRight);
            goLeft = true;
            firstTime = false;
        }
        else {

        }
    }
*/
    ROS_INFO( "Color r:%lf g:%lf b:%lf, i:%lf\n", r, g, b, intensity);
}

void RosRobot::rangeCB(const nxt_msgs::Range& rng)
{
    double range = rng.range;
    double range_min = rng.range_min;
    double range_max = rng.range_max;
    double spread_angle = rng.spread_angle;

    //ROS_INFO( "Range range:%lf min:%lf max:%lf spread:%lf\n", range, range_min, range_max, spread_angle);

    // Turn the Ultrasonic range into a LaserScan and publish it.
    /*
    sensor_msgs::LaserScan laserScan;
    laserScan.header.stamp = ros::Time::now();
    laserScan.header.frame_id = "laser_link";
    laserScan.angle_min = static_cast<float>(-spread_angle/2);
    laserScan.angle_max = static_cast<float>(spread_angle/2);
    laserScan.angle_increment = static_cast<float>(spread_angle);
    laserScan.time_increment = 0.0;
    laserScan.range_min = static_cast<float>(range_min);
    laserScan.range_max = static_cast<float>(range_max);
    laserScan.ranges.resize(1);
    laserScan.intensities.resize(1);
    laserScan.ranges[0] = static_cast<float>(range);
    laserScan.intensities[0] = 100;
    laserScanPublisher.publish(laserScan);

    // Turn the Ultrasonic range into a PointCloud and publish it.
    sensor_msgs::PointCloud pointCloud;
    pointCloud.header.stamp = ros::Time::now();
    pointCloud.header.frame_id = "sensor_frame";
    pointCloud.channels.resize(1);
    pointCloud.channels[0].name = "intensities";
    pointCloud.channels[0].values.resize(1);
    pointCloud.points.resize(1);
    pointCloud.points[0].x = range;
    pointCloud.points[0].y = 0;
    pointCloud.points[0].z = 0.07;
    pointCloud.channels[0].values[0] = 100;
    pointCloudPublisher.publish(pointCloud);
    */
}

void RosRobot::motorCB(const sensor_msgs::JointState& mot)
{
    std::string nam0 = mot.name[0];
    double eff0 = mot.effort[0];
    double vel0 = mot.velocity[0];
    double pos0 = mot.position[0];

//    ROS_INFO("name0:%s effect:%lf velocity:%lf position:%lf %lu\n", nam0.c_str(), eff0, vel0, pos0, mot.name.size());
}


void RosRobot::motorsCB(const sensor_msgs::JointState& mot)
{
    std::string nam0 = mot.name[0]; // motor1
    double eff0 = mot.effort[0];
    double vel0 = mot.velocity[0];
    double pos0 = mot.position[0];

    std::string nam1 = mot.name[1]; // motor2
    double eff1 = mot.effort[1];
    double vel1 = mot.velocity[1];
    double pos1 = mot.position[1];

//    ROS_INFO("name0:%s effect:%lf velocity:%lf position:%lf %lu\n", nam0.c_str(), eff0, vel0, pos0, mot.name.size());
//    ROS_INFO("name1:%s effect:%lf velocity:%lf position:%lf %lu\n", nam1.c_str(), eff1, vel1, pos1, mot.name.size());
}

void RosRobot::twistCB(const geometry_msgs::Twist& twist)
{
    double angularX = twist.angular.x;
    double angularY = twist.angular.y;
    double angularZ = twist.angular.z;
    double linearX = twist.linear.x;
    double linearY = twist.linear.y;
    double linearZ = twist.linear.z;

//    ROS_INFO("ax:%lf, ay:%lf, az:%lf, lx:%lf, ly:%lf, lz:%lf\n", angularX, angularY, angularZ, linearX, linearY, linearZ);
}

void  RosRobot::odomCB(const nav_msgs::Odometry& odom)
{
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_robot"); // Name of this node.
    RosRobot rosRobot(argc, argv);
    rosRobot.run();
}
