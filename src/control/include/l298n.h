#pragma once
#include <ros/ros.h>
#include <pigpio.h>

struct PinValues
{
    uint8_t in1; // determine spinning direction
    uint8_t in2;
    uint8_t in3;
    uint8_t in4;
    uint8_t enA; // PWM duty cycle
    uint8_t enB;
};

class l298n
{
public:
    l298n(PinValues pinValues);
    ~l298n();
    void SendCommandToWheels(double* cmd);
    void GetVelocityFromWheels(double* vel); // TODO

private:
    void Forward(uint8_t dutyCycle);
    void Backward(uint8_t dutyCycle);
    void Stop();
    void ReadEncoderSpeedM1(); // TODO
    void ReadEncoderSpeedM2(); // TODO
    void SetPinValues(PinValues pinValues);
    PinValues pins;
};
