#pragma once
#include <ros/ros.h>
#include <pigpio.h>

struct PinValues
{
    int in1; // determine spinning direction
    int in2;
    int in3;
    int in4;
    int enA; // PWM duty cycle
    int enB;
};

class l298n
{
public:
    /** 
     * Options:
     * Second bit set = use right motor
     * First bit set = use out1 output
     * First and second bit set = use out2 output
     * Third bit = use pwm signals to the l298n
     * Default is use out1 and out2 output and no pwm
    */
    l298n(PinValues pinValues, unsigned char options);
    ~l298n();
    void Forward(uint8_t dutyCycle);
    void Backward(uint8_t dutyCycle);
    void Stop();

    void TrySetBothDutyCycle(uint8_t dutyCycle);
    void TrySetLeftDutyCycle(uint8_t dutyCycle);
    void TrySetRightDutyCycle(uint8_t dutyCycle);

private:
    void SetPinValues(PinValues pinValues);
    void SetOptions(unsigned char options);
    
    bool useOut1;
    bool useOut2;
    bool usePwm;
    PinValues pins;
};
