//
// Created by hl on 2/7/19.
//

#include "rxros.h"
#include <rxcpp/operators/rx-map.hpp>
#include <rxcpp/operators/rx-scan.hpp>
#include <rxcpp/operators/rx-time_interval.hpp>
#include <teleop_msgs/Joystick.h>
#include <teleop_msgs/Keyboard.h>
#include <geometry_msgs/Twist.h>
#include "JoystickPublisher.h"
#include "KeyboardPublisher.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "velocity_publisher"); // Name of this node.

    const auto publishEveryMs = rxros::Parameter::get("/velocity_publisher/publish_every_ms", 100);
    const auto minVelLinear = rxros::Parameter::get("/velocity_publisher/min_vel_linear", 0.04); // m/s
    const auto maxVelLinear = rxros::Parameter::get("/velocity_publisher/max_vel_linear", 0.10); // m/s
    const auto minVelAngular = rxros::Parameter::get("/velocity_publisher/min_vel_angular", 0.64); // rad/s
    const auto maxVelAngular = rxros::Parameter::get("/velocity_publisher/max_vel_angular", 1.60); // rad/s
    const auto deltaVelLinear = (maxVelLinear - minVelLinear) / 10.0;
    const auto deltaVelAngular = (maxVelAngular - minVelAngular) / 10.0;

    rxros::Logging().info() << "publish_every_ms: " << publishEveryMs;
    rxros::Logging().info() << "min_vel_linear: " << minVelLinear << " m/s";
    rxros::Logging().info() << "max_vel_linear: " << maxVelLinear << " m/s";
    rxros::Logging().info() << "min_vel_angular: " << minVelAngular << " rad/s";
    rxros::Logging().info() << "max_vel_angular: " << maxVelAngular << " rad/s";

    auto joyObsrv = rxros::Observable<teleop_msgs::Joystick>::fromTopic("/joystick").map([](teleop_msgs::Joystick joy) { return joy.event; });
    auto keyObsrv = rxros::Observable<teleop_msgs::Keyboard>::fromTopic("/keyboard").map([](teleop_msgs::Keyboard key) { return key.event; });
    auto velObsrv = joyObsrv.merge(keyObsrv) |
        scan(std::make_tuple(0.0, 0.0), [=](const auto prevVelTuple, const int event) {
            auto currVelLinear = std::get<0>(prevVelTuple);
            auto currVelAngular = std::get<1>(prevVelTuple);
            switch (event) {
                case JS_EVENT_BUTTON0_DOWN:
                case JS_EVENT_BUTTON1_DOWN:
                case KB_EVENT_SPACE:
                    currVelLinear = 0.0;
                    currVelAngular = 0.0;
                    break;
                case JS_EVENT_AXIS_UP:
                case KB_EVENT_UP:
                    currVelLinear += deltaVelLinear;
                    if (currVelLinear > maxVelLinear) {
                        currVelLinear = maxVelLinear;
                    }
                    else if (currVelLinear > -minVelLinear && currVelLinear < minVelLinear) {
                        currVelLinear = minVelLinear;
                    }
                    break;
                case JS_EVENT_AXIS_DOWN:
                case KB_EVENT_DOWN:
                    currVelLinear -= deltaVelLinear;
                    if (currVelLinear < -maxVelLinear) {
                        currVelLinear = -maxVelLinear;
                    }
                    else if (currVelLinear > -minVelLinear && currVelLinear < minVelLinear) {
                        currVelLinear = -minVelLinear;
                    }
                    break;
                case JS_EVENT_AXIS_LEFT:
                case KB_EVENT_LEFT:
                    currVelAngular -= deltaVelAngular;
                    if (currVelAngular < -maxVelAngular) {
                        currVelAngular = -maxVelAngular;
                    }
                    else if (currVelAngular > -minVelAngular && currVelAngular < minVelAngular) {
                        currVelAngular = -minVelAngular;
                    }
                    break;
                case JS_EVENT_AXIS_RIGHT:
                case KB_EVENT_RIGHT:
                    currVelAngular += deltaVelAngular;
                    if (currVelAngular > maxVelAngular) {
                        currVelAngular = maxVelAngular;
                    }
                    else if (currVelAngular > -minVelAngular && currVelAngular < minVelAngular) {
                        currVelAngular = minVelAngular;
                    }
                    break;
                default:
                    break;
            }
            return std::make_tuple(currVelLinear, currVelAngular);
        });

    ros::NodeHandle nodeHandle;
    auto cmdVelPublisher(nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10));
    auto scheduler = rxcpp::observable<>::interval(std::chrono::milliseconds(publishEveryMs));
    scheduler.subscribe_on(rxcpp::serialize_new_thread()).subscribe(
        [=](int n) { // on_next
            geometry_msgs::Twist vel;
            vel.linear.x = 0.0; // currVelLinear;
            vel.angular.z = 0.0; // currVelAngular;
            cmdVelPublisher.publish(vel);
        });

    rxros::Logging().debug() << "Spinning ...";
    ros::spin();
}
