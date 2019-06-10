//
// Created by hl on 2/13/19.
//

#include <algorithm>
#include <rxros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <brickpi3_msgs/JointCommand.h>
using namespace rxcpp::operators;
using namespace rxros::operators;


int main(int argc, char** argv)
{
    rxros::init(argc, argv, "brickpi3_base_controller"); // Name of this node.

    const auto l_wheel_joint = rxros::parameter::get("/brickpi3/r_wheel_joint", "l_wheel_joint");
    const auto r_wheel_joint = rxros::parameter::get("/brickpi3/r_wheel_joint", "r_wheel_joint");
    const auto wheel_radius = rxros::parameter::get("/brickpi3/wheel_radius", 0.028); // m
    const auto wheel_basis = rxros::parameter::get("/brickpi3/wheel_basis", 0.0625); // m

    rxros::logging().info() << "brickpi3_base_controller:";
    rxros::logging().info() << "l_wheel_joint: " << l_wheel_joint;
    rxros::logging().info() << "r_wheel_joint: " << r_wheel_joint;
    rxros::logging().info() << "wheel_radius: " << wheel_radius;
    rxros::logging().info() << "wheel_basis: " << wheel_basis;

    auto adjust_factor = [] (const auto& curr_factor, const auto& curr_vel, const auto& desi_vel) {
        if (curr_vel != 0.0) // wheel is turning
            return curr_factor + ((desi_vel / curr_vel) > 1.0) ? 0.01 : -0.01;
        else if (desi_vel != 0.0) // wheel is not turning should it?
            return curr_factor + 0.05;
        else
            return curr_factor;};

    auto desired_lWheel_vel = [=](const auto& cmd_vel) {
        return cmd_vel.linear.x - cmd_vel.angular.z * wheel_basis;}; // m/s

    auto desired_rWheel_vel = [=](const auto& cmd_vel) {
        return cmd_vel.linear.x + cmd_vel.angular.z * wheel_basis;}; // m/s

    auto curr_lWheel_vel = [=](const auto& joint_states) {
        return joint_states.velocity[0] * wheel_radius;}; // first element is expected to be a left wheel

    auto curr_rWheel_vel = [=](const auto& joint_states) {
        return joint_states.velocity[1] * wheel_radius}; // second element is expected to be a right wheel

    auto update_effort = [=](const auto& prevTuple, const auto& tuple) {  // tuple: joint_states, cmd_vel
        const auto last_factor_lWheel = std::get<2>(prevTuple);
        const auto last_factor_rWheel = std::get<3>(prevTuple);
        const auto joint_states = std::get<0>(tuple);
        const auto cmd_vel = std::get<1>(tuple);
        const auto curr_factor_lWheel = adjust_factor(last_factor_lWheel, curr_lWheel_vel(joint_states), desired_lWheel_vel(cmd_vel));
        const auto curr_factor_rWheel = adjust_factor(last_factor_rWheel, curr_rWheel_vel(joint_states), desired_rWheel_vel(cmd_vel));
        return std::make_tuple(
            desired_lWheel_vel(cmd_vel) * curr_factor_lWheel, // effort for left wheel
            desired_rWheel_vel(cmd_vel) * curr_factor_rWheel, // effort for right whell
            curr_factor_lWheel,
            curr_factor_rWheel);};

    auto joint_states_observable = rxros::observable::from_topic<sensor_msgs::JointState>("/joint_states");
    auto cmd_vel_observable = rxros::observable::from_topic<geometry_msgs::Twist>("/cmd_vel");
    auto effort_observable = joint_states_observable.with_latest_from([](const auto& js, const auto& cv){return std::make_tuple(js, cv);}, cmd_vel_observable)
        | scan(std::make_tuple(0.0, 0.0, 0.0, 0.0), update_effort) // tuple: effort left wheel, effort right wheel, factor left wheel, factor right wheel.


    | publish_to_topic<brickpi3_msgs::JointCommand>("/joint_command");

    rxros::spin();
}
