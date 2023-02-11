#pragma once

class l298n
{
public:
    explicit roboclaw(settings* es_protobot);
    ~roboclaw();
    void SetupEncoders();
    void CloseEncoders();
    void SendCommandToWheels(double* cmd);
    void GetVelocityFromWheels(double* vel);

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

    void ForwardM1(uint8_t address, uint8_t value);
    void ForwardM2(uint8_t address, uint8_t value);
    void BackwardM1(uint8_t address, uint8_t value);
    void BackwardM2(uint8_t address, uint8_t value);
    void ReadEncoderSpeedM1(uint8_t address);
    void ReadEncoderSpeedM2(uint8_t address);

    termios tty;
    settings* es;
    int serialPort;
    int zeroCmdVelCount;
    unsigned int baudRate;  // instead of uint32_t for compatability
    uint8_t buf[100];
    char* errorBufPtr;
    char errorBuf[256];     // used by strerror_r, thread safe
}