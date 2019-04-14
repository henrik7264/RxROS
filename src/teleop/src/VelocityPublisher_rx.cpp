//
// Created by hl on 2/7/19.
//

#include <rxros.h>
#include <teleop_msgs/Joystick.h>
#include <teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"
using namespace rxcpp::operators;
using namespace rxros::operators;


int main(int argc, char** argv) {
    rxros::init(argc, argv, "velocity_publisher"); // Name of this node.

    const auto frequencyInHz = rxros::parameter::get("/velocity_publisher/frequency", 10.0); // hz
    const auto minVelLinear = rxros::parameter::get("/velocity_publisher/min_vel_linear", 0.04); // m/s
    const auto maxVelLinear = rxros::parameter::get("/velocity_publisher/max_vel_linear", 0.10); // m/s
    const auto minVelAngular = rxros::parameter::get("/velocity_publisher/min_vel_angular", 0.64); // rad/s
    const auto maxVelAngular = rxros::parameter::get("/velocity_publisher/max_vel_angular", 1.60); // rad/s
    const auto deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    const auto deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;

    rxros::logging().info() << "frequency: " << frequencyInHz;
    rxros::logging().info() << "min_vel_linear: " << minVelLinear << " m/s";
    rxros::logging().info() << "max_vel_linear: " << maxVelLinear << " m/s";
    rxros::logging().info() << "min_vel_angular: " << minVelAngular << " rad/s";
    rxros::logging().info() << "max_vel_angular: " << maxVelAngular << " rad/s";

    auto adaptVelocity = [=] (auto newVel, auto minVel, auto maxVel, auto isIncrVel) {
        if (newVel > maxVel)
            return maxVel;
        else if (newVel < -maxVel)
            return -maxVel;
        else if (newVel > -minVel && newVel < minVel)
            return (isIncrVel) ? minVel : -minVel;
        else
            return newVel;};

    auto teleop2VelTuple = [=](const auto& prevVelTuple, const int event) {
        const auto prevVelLinear = std::get<0>(prevVelTuple);  // use previous linear and angular velocity
        const auto prevVelAngular = std::get<1>(prevVelTuple); // to calculate the new linear and angular velocity.
        if (event == JS_EVENT_BUTTON0_DOWN || event == JS_EVENT_BUTTON1_DOWN || event == KB_EVENT_SPACE)
            return std::make_tuple(0.0, 0.0); // Stop the robot
        else if (event == JS_EVENT_AXIS_UP || event == KB_EVENT_UP)
            return std::make_tuple(adaptVelocity((prevVelLinear + deltaVelLinear), minVelLinear, maxVelLinear, true), prevVelAngular); // move forward
        else if (event == JS_EVENT_AXIS_DOWN || event == KB_EVENT_DOWN)
            return std::make_tuple(adaptVelocity((prevVelLinear - deltaVelLinear), minVelLinear, maxVelLinear, false), prevVelAngular); // move backward
        else if (event == JS_EVENT_AXIS_LEFT || event == KB_EVENT_LEFT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular + deltaVelAngular), minVelAngular, maxVelAngular, true)); // move left
        else if (event == JS_EVENT_AXIS_RIGHT || event == KB_EVENT_RIGHT)
            return std::make_tuple(prevVelLinear, adaptVelocity((prevVelAngular - deltaVelAngular), minVelAngular, maxVelAngular, false));}; // move right

    auto velTuple2TwistMsg = [](auto velTuple) {
        geometry_msgs::Twist vel;
        vel.linear.x = std::get<0>(velTuple);
        vel.angular.z = std::get<1>(velTuple);
        return vel;};

    auto joyObsrv = rxros::observable::from_topic<teleop_msgs::Joystick>("/joystick") // create an observable stream from "/joystick" topic
        | map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto keyObsrv = rxros::observable::from_topic<teleop_msgs::Keyboard>("/keyboard") // create an observable stream from "/keyboard" topic
        | map([](teleop_msgs::Keyboard key) { return key.event; });
    joyObsrv.merge(keyObsrv)                              // merge the joystick and keyboard messages into an observable teleop stream.
    | scan(std::make_tuple(0.0, 0.0), teleop2VelTuple)    // turn the teleop stream into a linear and angular velocity stream.
    | map(velTuple2TwistMsg)                              // turn the linear and angular velocity stream into a Twist stream.
    | sample_with_frequency(frequencyInHz)                // take latest Twist msg and populate it with the specified frequency.
    | publish_to_topic<geometry_msgs::Twist>("/cmd_vel"); // publish the Twist messages to the topic "/cmd_vel"

    rxros::logging().info() << "Spinning velocity_publisher ...";
    rxros::spin();
}
