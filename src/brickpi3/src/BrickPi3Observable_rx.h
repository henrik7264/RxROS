//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3OBSERVABLE_RX_H
#define BRICKPI3_BRICKPI3OBSERVABLE_RX_H

#include <string>
#include <rxros.h>
#include "BrickPi3Device.h"
#include "BrickPi3Observable_rx.h"

struct actuator_motor_t
{
    uint8_t motorState;// Variable for reading motor motorState
    int8_t motorPower;// Variable for reading motor powers
    int32_t motorPosition;  // Variable for reading motor encoder positions
    int16_t motorDPS; // Variable for reading motor speeds (Degrees Per Second)
};

namespace brickpi3
{
    class Observable
    {
    private:
        Observable() = default;

    public:
        ~Observable() = default;

        static auto colorSensor(const std::string& name, const std::string& port, double frequency);
        static auto ultrasonicSensor(const std::string& name, const std::string& port, double frequency);
        static auto motor(const std::string &name, const std::string &port, double frequency);
    };// end class Observable
} // end namespace brickpi3


auto brickpi3::Observable::colorSensor(const std::string& name, const std::string& port, double frequency)
{
    auto observable = rxcpp::observable<>::create<sensor_color_t>(
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
    return observable.subscribe_on(synchronize_new_thread());
}

auto brickpi3::Observable::ultrasonicSensor(const std::string& name, const std::string& port, double frequency)
{
    auto observable = rxcpp::observable<>::create<sensor_ultrasonic_t>(
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
    return observable.subscribe_on(synchronize_new_thread());
}

auto brickpi3::Observable::motor(const std::string &name, const std::string &port, double frequency)
{
    auto observable = rxcpp::observable<>::create<actuator_motor_t>(
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
                    brickPi3.reset_all();
                    subscriber.on_error(rxros::Exception::systemError(errno, "BrickPi3 failed to read Motor '" +  name + "' rc: " + std::to_string(rc)));
                    break;
                }
                rate.sleep();
            }
            if (!errReported) {
                brickPi3.reset_all();
                subscriber.on_completed();
            }});
    return observable.subscribe_on(synchronize_new_thread());
}


#endif //BRICKPI3_BRICKPI3OBSERVABLE_RX_H
