/* MIT License
Copyright (c) [2022] [VIP Team RoSE]
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. */

#ifndef CR_CONTROL_ROBOCLAW_H
#define CR_CONTROL_ROBOCLAW_H

#include <ros/ros.h>
#include <stdint.h>
#include <termios.h>

struct RoboclawSettings 
{
    std::string serialPortAddress;
    int addresses[8]; // 0x80 - 0x87
    int timeoutMs                    = 12;
    int retries                     = 3;
    int maxBufferSize               = 100;
    int baudRate                    = 115200;
    uint8_t m1Forward               = 0;
    uint8_t m2Forward               = 4;
    uint8_t m1Backward              = 1;
    uint8_t m2Backward              = 5;
    uint8_t m1ReadEncoderSpeed      = 18;
    uint8_t m2ReadEncoderSpeed      = 19;
    uint8_t m1Position = 119;
    uint8_t m2Position = 120;
    uint8_t maxEffortValue = 126;
    double loopFrequency            = 10;
    bool debugMode                  = false;
};

class Roboclaw 
{
 public:
    explicit Roboclaw(RoboclawSettings*);
    ~Roboclaw();
    void SetupEncoders();
    void CloseEncoders();
    void SendCommandToWheels(double* cmd);
    void GetVelocityFromWheels(double* vel);

    void ForwardM1(uint8_t address, uint8_t value);
    void ForwardM2(uint8_t address, uint8_t value);
    void BackwardM1(uint8_t address, uint8_t value);
    void BackwardM2(uint8_t address, uint8_t value);
    void ReadEncoderSpeedM1(uint8_t address);
    void ReadEncoderSpeedM2(uint8_t address);
    void SetPositionM1(uint8_t address, uint32_t value);
    void SetPositionM2(uint8_t address, int32_t value);

 private:
    void GetBaudRate();
    int ClearIOBuffers();
    int WriteToEncoders(uint8_t* data, int nBytes);
    int WaitReadStatus(int nBytes, int timeout_ms);
    int ReadFromEncoders(int nBytes);
    int SendCommands(uint8_t* data, int writeBytes, int readBytes);
    double ConvertPulsesToRadians(double vel);
    uint8_t ScaleCommand(double cmd);
    uint32_t ValidateChecksum(uint8_t* packet, int nBytes);
    uint32_t RecombineBuffer(uint8_t* buf);


    termios tty;
    RoboclawSettings* settings;
    int serialPort;
    int zeroCmdVelCount;
    unsigned int baudRate;  // instead of uint32_t for compatability
    uint8_t buf[100];
    char* errorBufPtr;
    char errorBuf[256];     // used by strerror_r, thread safe
};

#endif  // CR_CONTROL_ROBOCLAW_H
