#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <nxt_msgs/JointCommand.h>
#include <JoystickPublisher.h>
#include <joystick/Joystick.h>
#include <nxt_msgs/Contact.h>
#include <nxt_msgs/Color.h>
#include <nxt_msgs/Range.h>
#include <sensor_msgs/JointState.h>

#define MAX_EFFORT 12
#define ZERO_EFFORT 6
#define MIN_EFFORT 0
static float Efforts[] = {-1.0F, -0.9F, -0.8F, -0.7F, -0.6F, -0.5F, 0.0F, 0.5F, 0.6F, 0.7F, 0.8F, 0.9F, 1.0F};

class RosRobot {
private:
    ros::Publisher motorPublisher;
    int effortMotor1 = ZERO_EFFORT;
    int effortMotor2 = ZERO_EFFORT;
    void joystickCB(const joystick::Joystick& joy);
    void colorCB(const nxt_msgs::Color& col);
    void rangeCB(const nxt_msgs::Range& rng);
    void touchCB1(const nxt_msgs::Contact& cnt);
    void touchCB2(const nxt_msgs::Contact& cnt);
    void motorCB(const sensor_msgs::JointState& mot);

public:
    RosRobot(int argc, char** argv);
    virtual ~RosRobot() {};
    void run() {ros::spin();}
};

RosRobot::RosRobot(int argc, char** argv) {
    ros::init(argc, argv, "RosRobot"); // Name of this node.
    ros::NodeHandle nodeHandle;
    nodeHandle.advertise<nxt_msgs::JointCommand>("/joint_command", 10);
    nodeHandle.subscribe("/joystick", 10, &RosRobot::joystickCB, this);
    nodeHandle.subscribe("/color_sensor", 10, &RosRobot::colorCB, this);
    nodeHandle.subscribe("/range_sensor", 10, &RosRobot::rangeCB, this);
    nodeHandle.subscribe("/touch_sensor", 10, &RosRobot::touchCB1, this);
    nodeHandle.subscribe("/touch_sensor2", 10, &RosRobot::touchCB2, this);
    nodeHandle.subscribe("/joint_state", 10, &RosRobot::motorCB, this);
}

void RosRobot::colorCB(const nxt_msgs::Color& col)
{
    double r = col.r;
    double g = col.g;
    double b = col.b;
    double intensity = col.intensity;

    ROS_INFO( "Color r:%lf g:%lf b:%lf, i:%lf\n", r, g, b, intensity);
}

void RosRobot::rangeCB(const nxt_msgs::Range& rng)
{
    double range = rng.range;
    double range_min = rng.range_min;
    double range_max = rng.range_max;
    double spread_angle = rng.spread_angle;

    ROS_INFO( "Range range:%lf min:%lf max:%lf spread:%lf\n", range, range_min, range_max, spread_angle);
}

void RosRobot::touchCB1(const nxt_msgs::Contact& cnt)
{
    short contact = cnt.contact;
    if (contact == 1) {
        effortMotor1 = ZERO_EFFORT;
        effortMotor2 = ZERO_EFFORT;

        nxt_msgs::JointCommand jointMotor1;
        jointMotor1.name = "motor_joint";
        jointMotor1.effort = Efforts[effortMotor1];
        motorPublisher.publish(jointMotor1);

        nxt_msgs::JointCommand jointMotor2;
        jointMotor2.name = "motor_joint2";
        jointMotor2.effort = Efforts[effortMotor2];
        motorPublisher.publish(jointMotor2);
    }
}

void RosRobot::touchCB2(const nxt_msgs::Contact& cnt)
{
    short contact = cnt.contact;
    if (contact == 1) {
        effortMotor1 = ZERO_EFFORT;
        effortMotor2 = ZERO_EFFORT;

        nxt_msgs::JointCommand jointMotor1;
        jointMotor1.name = "motor_joint";
        jointMotor1.effort = Efforts[effortMotor1];
        motorPublisher.publish(jointMotor1);

        nxt_msgs::JointCommand jointMotor2;
        jointMotor2.name = "motor_joint2";
        jointMotor2.effort = Efforts[effortMotor2];
        motorPublisher.publish(jointMotor2);
    }
}

void RosRobot::motorCB(const sensor_msgs::JointState& mot)
{
}


void RosRobot::joystickCB(const joystick::Joystick& joy)
{
    JoystickEvents event = static_cast<JoystickEvents>(joy.event);
    switch (event) {
        case JOYSTICK_BUTTON0_DOWN:
        case JOYSTICK_BUTTON2_DOWN:
        case JOYSTICK_BUTTON3_DOWN:
            effortMotor1 = ZERO_EFFORT;
            effortMotor2 = ZERO_EFFORT;
            break;
        case JOYSTICK_BUTTON1_DOWN:
            effortMotor1 = (effortMotor1 < effortMotor2) ? effortMotor1 : effortMotor2;
            effortMotor2 = effortMotor1;
            break;
        case JOYSTICK_AXIS_DOWN:
            effortMotor1 -= (effortMotor1 == MIN_EFFORT) ? 0 : 1;
            effortMotor2 -= (effortMotor2 == MIN_EFFORT) ? 0 : 1;
            break;
        case JOYSTICK_AXIS_UP:
            // Increase
            effortMotor1 += (effortMotor1 == MAX_EFFORT) ? 0 : 1;
            effortMotor2 += (effortMotor2 == MAX_EFFORT) ? 0 : 1;
            break;
        case JOYSTICK_AXIS_LEFT:
            if (effortMotor2 >= ZERO_EFFORT)
                effortMotor2 -= (effortMotor2 == MIN_EFFORT) ? 0 : 1;
            else
                effortMotor2 += (effortMotor2 == MAX_EFFORT) ? 0 : 1;
            break;
        case JOYSTICK_AXIS_RIGHT:
            if (effortMotor2 >= ZERO_EFFORT)
                effortMotor2 += (effortMotor2 == MAX_EFFORT) ? 0 : 1;
            else
                effortMotor2 -= (effortMotor2 == MIN_EFFORT) ? 0 : 1;
            break;
        default:
            break;
    }

    ROS_INFO("effortMotor1 = %f, effortMotor2 = %f\n", Efforts[effortMotor1], Efforts[effortMotor2]);
    nxt_msgs::JointCommand jointMotor1;
    jointMotor1.name = "motor_joint";
    jointMotor1.effort = Efforts[effortMotor1];
    motorPublisher.publish(jointMotor1);

    nxt_msgs::JointCommand jointMotor2;
    jointMotor2.name = "motor_joint2";
    jointMotor2.effort = Efforts[effortMotor2];
    motorPublisher.publish(jointMotor2);
}

int main(int argc, char** argv) {
    RosRobot rosRobot(argc, argv);
    rosRobot.run();
}
