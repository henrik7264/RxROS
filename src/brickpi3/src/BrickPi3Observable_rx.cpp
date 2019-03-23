//
// Created by hl on 2/10/19.
//

#include <string>
#include <rxros.h>
#include "BrickPi3Device.h"
#include "BrickPi3Observable_rx.h"


auto brickpi3::Observable::colorSensor(const std::string& name, const std::string& port, const double frequency)
{
    return rxcpp::observable<>::create<sensor_color_t>([=](rxcpp::subscriber<sensor_color_t> subscriber) {
        const uint8_t id = port2id(port);
        BrickPi3 brickPi3;
        brickPi3.detect();
        brickPi3.set_sensor_type(id, SENSOR_TYPE_NXT_COLOR_FULL);

        ros::Rate rate(frequency);
        while (rxros::ok()) {
            sensor_color_t sensorColor;
            int rc = brickPi3.get_sensor(id, &sensorColor);
            if (rc == 0) {
                rxros::Logging().debug() << "Color sensor:" << sensorColor.color << ", red: " << sensorColor.reflected_red << ", green: " << sensorColor.reflected_green << ", blue: " << sensorColor.reflected_blue << ", ambient: " << sensorColor.ambient;
                subscriber.on_next(sensorColor);
            } else {
                brickPi3.reset_all();
                subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Color sensor '" +  name + "' rc: " + std::to_string(rc)));
            }
            rate.sleep();
        }
        if (!rxros::ok()) {
            brickPi3.reset_all();
            subscriber.on_completed();
        }
    });
};

auto brickpi3::Observable::ultrasonicSensor(const std::string& name, const std::string& port, const double frequency)
{
    return rxcpp::observable<>::create<sensor_ultrasonic_t>([=](rxcpp::subscriber<sensor_ultrasonic_t> subscriber) {
        const uint8_t id = port2id(port);
        BrickPi3 brickPi3;
        brickPi3.detect();
        brickPi3.set_sensor_type(id, SENSOR_TYPE_NXT_ULTRASONIC);

        ros::Rate rate(frequency);
        while (rxros::ok()) {
            sensor_ultrasonic_t sensorUltrasonic;
            int rc = brickPi3.get_sensor(id, &sensorUltrasonic);
            if (rc == 0) {
                rxros::Logging().debug() << "Ultrasonic sensor: CM: " << sensorUltrasonic.cm << ", Inches: " << sensorUltrasonic.inch);
                subscriber.on_next(sensorUltrasonic);
            } else {
                brickPi3.reset_all();
                subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Ultrasonic sensor '" +  name + "' rc: " + std::to_string(rc)));
            }
            rate.sleep();
        }
        if (!rxros::ok()) {
            brickPi3.reset_all();
            subscriber.on_completed();
        }
    });
};

auto brickpi3::Observable::motorActuator(const std::string& name, const std::string& port, const double frequency)
{
    return rxcpp::observable<>::create<actuator_motor_t>([=](rxcpp::subscriber<actuator_motor_t> subscriber) {
        const uint8_t id = port2id(port);
        BrickPi3 brickPi3;
        brickPi3.detect();
        brickPi3.reset_motor_encoder(id);

        ros::Rate rate(frequency);
        while (rxros::ok()) {
            actuator_motor_t actuatorMotor;
            int rc = brickPi3.get_motor_status(id, actuatorMotor.motorState, actuatorMotor.motorPower, actuatorMotor.motorPosition, actuatorMotor.motorDPS);
            if (rc == 0) {
                rxros::Logging().debug() << "Motor name '" << name << "', State: " << actuatorMotor.motorState << ", Power: " << actuatorMotor.motorPower << ", Position: " << actuatorMotor.motorPosition << ", DPS: " << actuatorMotor.motorDPS;
                subscriber.on_next(actuatorMotor);
            } else {
                brickPi3.reset_motor_encoder(id);
                subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Motor '" +  name + "' rc: " + std::to_string(rc)));
            }
            rate.sleep();
        }
        if (!rxros::ok()) {
            brickPi3.reset_motor_encoder(id);
            subscriber.on_completed();
        }
    });
};
