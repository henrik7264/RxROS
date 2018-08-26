#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <nxt_msgs/JointCommand.h>
#include <joystick/Joystick.h>
#include <nxt_msgs/Contact.h>
#include <nxt_msgs/Color.h>
#include <nxt_msgs/Range.h>
#include <sensor_msgs/JointState.h>


#define JS_EVENT_BUTTON 0x01    /* button pressed/released */
#define JS_EVENT_AXIS   0x02    /* joystick moved */

#define MAX_EFFORT 12
#define ZERO_EFFORT 6
#define MIN_EFFORT 0
static float Efforts[] = {-1.0F, -0.9F, -0.8F, -0.7F, -0.6F, -0.5F, 0.0F, 0.5F, 0.6F, 0.7F, 0.8F, 0.9F, 1.0F};

class RosRobot {
private:
    ros::Publisher motorPublisher;
    ros::Subscriber joystickSubscriber;
    ros::Subscriber colorSubscriber;
    ros::Subscriber rangeSubscriber;
    ros::Subscriber touchSubscriber1;
    ros::Subscriber touchSubscriber2;
    ros::Subscriber motorSubscriber;

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
    motorPublisher = nodeHandle.advertise<nxt_msgs::JointCommand>("/joint_command", 10);
    joystickSubscriber = nodeHandle.subscribe("/joystick", 10, &RosRobot::joystickCB, this);
    colorSubscriber = nodeHandle.subscribe("/color_sensor", 10, &RosRobot::colorCB, this);
    rangeSubscriber = nodeHandle.subscribe("/range_sensor", 10, &RosRobot::rangeCB, this);
    touchSubscriber1 = nodeHandle.subscribe("/touch_sensor", 10, &RosRobot::touchCB1, this);
    touchSubscriber2 = nodeHandle.subscribe("/touch_sensor2", 10, &RosRobot::touchCB2, this);
    motorSubscriber = nodeHandle.subscribe("/joint_state", 10, &RosRobot::motorCB, this);
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
    short type = joy.type;
    short number = joy.number;
    short value = joy.value;

    if (type == JS_EVENT_BUTTON) {
        if (number == 0) { // full stop
            effortMotor1 = ZERO_EFFORT;
            effortMotor2 = ZERO_EFFORT;
        }
        else if (number == 1) { // drive straight ahead
            effortMotor1 = (effortMotor1 < effortMotor2) ? effortMotor1 : effortMotor2;
            effortMotor2 = effortMotor1;
        }
        else {
            effortMotor1 = ZERO_EFFORT;
            effortMotor2 = ZERO_EFFORT;
        }
    } else if (type == JS_EVENT_AXIS) {
        if (number == 0) { // left - right
            if (value == 32767) { // right
                if (effortMotor2 >= ZERO_EFFORT)
                    effortMotor2 += (effortMotor2 == MAX_EFFORT) ? 0 : 1;
                else
                    effortMotor2 -= (effortMotor2 == MIN_EFFORT) ? 0 : 1;
            }
            else if (value == -32767) { // left
                if (effortMotor2 >= ZERO_EFFORT)
                    effortMotor2 -= (effortMotor2 == MIN_EFFORT) ? 0 : 1;
                else
                    effortMotor2 += (effortMotor2 == MAX_EFFORT) ? 0 : 1;
            }
        }
        else { // up-down, no = 1;
            if (value == 32767) { // down
                effortMotor1 -= (effortMotor1 == MIN_EFFORT) ? 0 : 1;
                effortMotor2 -= (effortMotor2 == MIN_EFFORT) ? 0 : 1;
            }
            else if (value == -32767) { // up
                effortMotor1 += (effortMotor1 == MAX_EFFORT) ? 0 : 1;
                effortMotor2 += (effortMotor2 == MAX_EFFORT) ? 0 : 1;
            }
        }
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
