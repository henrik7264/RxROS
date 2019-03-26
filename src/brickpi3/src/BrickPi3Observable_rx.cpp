//
// Created by hl on 2/10/19.
//

#include <string>
#include <rxros.h>
#include "BrickPi3Device.h"
#include "BrickPi3Observable_rx.h"


auto brickpi3::Observable::colorSensor(const std::string& name, const std::string& port, double frequency)
{
    return rxcpp::observable<>::create<sensor_color_t>(
        [=](rxcpp::subscriber<sensor_color_t> subscriber) {
            const uint8_t id = port2id(port);
            bool errReported = false;
            BrickPi3 brickPi3;
            brickPi3.detect();
            brickPi3.set_sensor_type(id, SENSOR_TYPE_NXT_COLOR_FULL);

            ros::Rate rate(frequency);
            while (rxros::ok()) {
                sensor_color_t color;
                int rc = brickPi3.get_sensor(id, &color);
                if (rc == 0) {
                    rxros::Logging().debug() << "Color sensor:" << color.color << ", red: " << color.reflected_red << ", green: " << color.reflected_green << ", blue: " << color.reflected_blue << ", ambient: " << color.ambient;
                    subscriber.on_next(color);
                } else {
                    errReported = true;
                    brickPi3.reset_all();
                    subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Color sensor '" +  name + "' rc: " + std::to_string(rc)));
                    break;
                }
                rate.sleep();
            }
            if (!errReported) {
                brickPi3.reset_all();
                subscriber.on_completed();
            }});
};

auto brickpi3::Observable::ultrasonicSensor(const std::string& name, const std::string& port, double frequency)
{
    return rxcpp::observable<>::create<sensor_ultrasonic_t>(
        [=](rxcpp::subscriber<sensor_ultrasonic_t> subscriber) {
            const uint8_t id = port2id(port);
            bool errReported = false;
            BrickPi3 brickPi3;
            brickPi3.detect();
            brickPi3.set_sensor_type(id, SENSOR_TYPE_NXT_ULTRASONIC);

            ros::Rate rate(frequency);
            while (rxros::ok()) {
                sensor_ultrasonic_t ultrasonic;
                int rc = brickPi3.get_sensor(id, &ultrasonic);
                if (rc == 0) {
                    rxros::Logging().debug() << "Ultrasonic sensor: cm: " << ultrasonic.cm << ", inches: " << ultrasonic.inch;
                    subscriber.on_next(ultrasonic);
                } else {
                    errReported = true;
                    brickPi3.reset_all();
                    subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Ultrasonic sensor '" +  name + "' rc: " + std::to_string(rc)));
                    break;
                }
                rate.sleep();
            }
            if (!errReported) {
                brickPi3.reset_all();
                subscriber.on_completed();
            }});
};

auto brickpi3::Observable::motorActuator(const std::string& name, const std::string& port, double frequency)
{
    return rxcpp::observable<>::create<actuator_motor_t>(
        [=](rxcpp::subscriber<actuator_motor_t> subscriber) {
            const uint8_t id = port2id(port);
            bool errReported = false;
            BrickPi3 brickPi3;
            brickPi3.detect();
            brickPi3.reset_motor_encoder(id);

            ros::Rate rate(frequency);
            while (rxros::ok()) {
                actuator_motor_t motor{};
                int rc = brickPi3.get_motor_status(id, motor.motorState, motor.motorPower, motor.motorPosition, motor.motorDPS);
                if (rc == 0) {
                    rxros::Logging().debug() << "Motor name '" << name << "', State: " << motor.motorState << ", Power: " << motor.motorPower << ", Position: " << motor.motorPosition << ", DPS: " << motor.motorDPS;
                    subscriber.on_next(motor);
                } else {
                    errReported = true;
                    brickPi3.reset_motor_encoder(id);
                    subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Motor '" +  name + "' rc: " + std::to_string(rc)));
                    break;
                }
                rate.sleep();
            }
            if (!errReported) {
                brickPi3.reset_motor_encoder(id);
                subscriber.on_completed();
            }});
};
