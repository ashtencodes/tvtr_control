
#include <stdio.h>

#include <pigpio.h>
#include <gnc_control/l298n.h>


l298n::l298n(PinValues pins)
{
   if (gpioInitialise() < 0)
   {
      fprintf(stderr, "pigpio initialisation failed\n");
      return;
   }
    SetPinValues(pins);
    gpioPWM(pins.enA, 255);
    gpioPWM(pins.enB, 255);
}

void l298n::SetPinValues(PinValues pins)
{
    gpioSetMode(pins.in1, PI_OUTPUT);
    gpioSetMode(pins.in2, PI_OUTPUT);
    gpioSetMode(pins.in3, PI_OUTPUT);
    gpioSetMode(pins.in4, PI_OUTPUT);
    gpioSetMode(pins.enA, PI_OUTPUT);
    gpioSetMode(pins.enB, PI_OUTPUT);
}

void l298n::Forward(uint8_t dutyCycle)
{
    gpioWrite(pins.in1, 1);
    gpioWrite(pins.in2, 0);
    gpioWrite(pins.in3, 1);
    gpioWrite(pins.in4, 0);
    gpioPWM(pins.enA, dutyCycle);
    gpioPWM(pins.enB, dutyCycle);
}

void l298n::Backward(uint8_t dutyCycle)
{
    gpioWrite(pins.in1, 0);
    gpioWrite(pins.in2, 1);
    gpioWrite(pins.in3, 0);
    gpioWrite(pins.in4, 1);
    gpioPWM(pins.enA, dutyCycle);
    gpioPWM(pins.enB, dutyCycle);
}

void l298n::Stop()
{
    gpioWrite(pins.in1, 0);
    gpioWrite(pins.in2, 0);
    gpioWrite(pins.in3, 0);
    gpioWrite(pins.in4, 0);
    gpioPWM(pins.enA, 0);
    gpioPWM(pins.enB, 0);
}