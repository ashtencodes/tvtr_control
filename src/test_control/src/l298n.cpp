
#include <stdio.h>

#include <pigpio.h>
#include <test_control/l298n.h>


l298n::l298n(PinValues pins, unsigned char options)
{
    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
        return;
    }

    SetOptions(options);
    SetPinValues(pins);
    TrySetBothDutyCycle(255);
}

l298n::~l298n() 
{
    gpioTerminate();
}

void l298n::SetPinValues(PinValues pins)
{
    this->pins.in1 = pins.in1;
    this->pins.in2 = pins.in2;
    this->pins.enA = pins.enA;
    this->pins.in3 = pins.in3;
    this->pins.in4 = pins.in4;
    this->pins.enB = pins.enB;
    if (useOut1)
    {
        gpioSetMode(pins.in1, PI_OUTPUT);
        gpioSetMode(pins.in2, PI_OUTPUT);
        gpioSetMode(pins.enA, PI_OUTPUT);
    }
    if (useOut2)
    {
        gpioSetMode(pins.in3, PI_OUTPUT);
        gpioSetMode(pins.in4, PI_OUTPUT);
        gpioSetMode(pins.enB, PI_OUTPUT);
    }
}

void l298n::SetOptions(unsigned char options)
{
    usePwm = false;
    useOut1 = false;
    useOut2 = false;
    if ((options >> 2) & 1 == 1) 
    {
        this->usePwm = true;
    }
    if ((options >> 1) & 1 == 1) 
    {
        this->useOut1 = true;
    }
    if ((options & 1) == 1) 
    {
        this->useOut2 = true;
    }
}

void l298n::Forward(uint8_t dutyCycle)
{
    if (useOut1)
    {
        gpioWrite(pins.in1, 1);
        gpioWrite(pins.in2, 0);
    }
    if (useOut2)
    {
        gpioWrite(pins.in3, 1);
        gpioWrite(pins.in4, 0);
    }
    TrySetBothDutyCycle(dutyCycle);
}

void l298n::Backward(uint8_t dutyCycle)
{
    if (useOut1)
    {
        gpioWrite(23, 0);
        gpioWrite(18, 1);
    }
    if (useOut2)
    {
        gpioWrite(pins.in3, 0);
        gpioWrite(pins.in4, 1);
    }
    TrySetBothDutyCycle(dutyCycle);
}

void l298n::Stop()
{
    if (useOut1)
    {
        gpioWrite(pins.in1, 0);
        gpioWrite(pins.in2, 0);
        TrySetLeftDutyCycle(0);
    }
    if (useOut2)
    {
        gpioWrite(pins.in3, 0);
        gpioWrite(pins.in4, 0);
        TrySetRightDutyCycle(0);
    }
}

void l298n::TrySetBothDutyCycle(uint8_t dutyCycle)
{
    if (usePwm)
    {
        TrySetLeftDutyCycle(dutyCycle);
        TrySetRightDutyCycle(dutyCycle);
    }
}

void l298n::TrySetLeftDutyCycle(uint8_t dutyCycle)
{
    if (useOut1)
        gpioPWM(pins.enA, dutyCycle);
}

void l298n::TrySetRightDutyCycle(uint8_t dutyCycle)
{
    if (useOut2)
        gpioPWM(pins.enB, dutyCycle);
}