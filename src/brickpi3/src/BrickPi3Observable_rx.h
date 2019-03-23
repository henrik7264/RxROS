//
// Created by hl on 2/10/19.
//

#ifndef BRICKPI3_BRICKPI3OBSERVABLE_RX_H
#define BRICKPI3_BRICKPI3OBSERVABLE_RX_H

#include <string>

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
        virtual ~Observable() = default;

        static auto colorSensor(const std::string& name, const std::string& port, double frequency);
        static auto ultrasonicSensor(const std::string& name, const std::string& port, double frequency);
        static auto motorActuator(const std::string& name, const std::string& port, double frequency);
    };// end class Observable
}; // end namespace brickpi3

#endif //BRICKPI3_BRICKPI3OBSERVABLE_RX_H
