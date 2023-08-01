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

// Linux headers
#include <fcntl.h> // File controls like O_RDWR
#include <errno.h> // Error integer and strerror()
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // read(), write(), close()

// Standard library headers
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#include <cstdlib>
#include <cstring>
#include <bitset>

// GNC headers
#include <cr_control/roboclaw.h>

Roboclaw::Roboclaw(RoboclawSettings* settings) {
    for (int i = 0; i < 256; i++)
        errorBuf[i] = 0x00;
    zeroCmdVelCount = 0;
    this->settings = settings;
    GetBaudRate();
    SetupEncoders();
}

Roboclaw::~Roboclaw() {
    ROS_INFO("Deconstructing: closing serial port connection");
    ForwardM2(0x80, 0);
    ForwardM1(0x80, 0);
    CloseEncoders();
}

/* Setup Roboclaw motor encoders. Open serial port for reading and writing, 
    configure for 1 stop bit, 8 bits per byte, no parity checking (8N1 uart config)
   as per the roboclaw user manual. Send and receive raw bytes only. 
   Set baud rate of sending and receiving end. */

void Roboclaw::SetupEncoders() {
    // enable read & write, disable controlling terminal
    serialPort = open(settings->serialPortAddress.c_str(), O_RDWR | O_NOCTTY);

    if (serialPort < 0) {
        errorBufPtr = strerror_r(errno, errorBuf, sizeof(errorBuf));
        ROS_ERROR("Could not open %s: Error %i from open: %s",
                 settings->serialPortAddress.c_str(), errno, errorBufPtr);
        exit(EXIT_FAILURE);
    }

    fcntl(serialPort, F_SETFL, 0);  // set to blocking mode (for reads)

    if (tcgetattr(serialPort, &tty) != 0) {
        errorBufPtr = strerror_r(errno, errorBuf, sizeof(errorBuf));
        ROS_ERROR("Error %i from tcgetattr: %s", errno, errorBufPtr);
        exit(EXIT_FAILURE);
    }

    // set necessary bits

    tty.c_cflag &= ~PARENB;         // don't use parity bit
    tty.c_cflag &= ~CSTOPB;         // use 1 stop bit
    tty.c_cflag |= CS8;             // 8 bit characters
    tty.c_cflag |= CREAD | CLOCAL;  // ignore modem controls


    //cfmakeraw(&tty);
    tty.c_cflag &= ~CRTSCTS;        // no hardware flowcontrol
    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ECHOE;
    tty.c_lflag &= ~ECHONL;
    tty.c_lflag &= ~ISIG;

    // setup for non canonical mode
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    tty.c_oflag &= ~OPOST;
    tty.c_oflag &= ~ONLCR;

    // set timeout for reading, VMIN is the minimum bytes to keep from blocking
    tty.c_cc[VTIME] = 0;
    tty.c_cc[VMIN]  = 1;


    // set baud rates
    cfsetispeed(&tty, baudRate);
    cfsetospeed(&tty, baudRate);

    // save flag settings

    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        strerror_r(errno, errorBuf, sizeof(errorBuf));
        ROS_ERROR("Error %i from tcsetattr: %s", errno, errorBuf);
    }

    ClearIOBuffers();
}

uint32_t Roboclaw::RecombineBuffer(uint8_t* buf) {
    return (buf[3] << 24 | buf[2] << 16 | buf[1] << 8 | buf[0]);
}

void Roboclaw::GetBaudRate() {
    switch (settings->baudRate) {
        case 9600:
            ROS_INFO("Setting baud rate to 9600");
            baudRate = B9600;
            break;
        case 19200:
            ROS_INFO("Setting baud rate to 19200");
            baudRate = B19200;
            break;
        case 38400:
            ROS_INFO("Setting baud rate to 38400");
            baudRate = B38400;
            break;
        case 57600:
            ROS_INFO("Setting baud rate to 57600");
            baudRate = B57600;
            break;
        case 115200:
            ROS_INFO("Setting baud rate to 115200");
            baudRate = B115200;
            break;
        case 230400:
            ROS_INFO("Setting baud rate to 230400");
            baudRate = B230400;
            break;
        case 460800:
            ROS_INFO("Setting baud rate to 460800");
            baudRate = B460800;
            break;
        default:
            ROS_ERROR("Invalid Baud Rate Selection, setting to 115200");
            baudRate = B115200;
            break;
    }
}

/* Send and execute commands to encoders. Returns -1 or es.retries on failure, 1 on success. Commands will be sent up to max es.retries.
   Successful writes to encoders are then polled to check if data is available to be read back. Once data is available, the data
   is read back, then the user's I/O buffers are cleared. */

int Roboclaw::SendCommands(uint8_t* data, int writeBytes, int readBytes) {
    int r, writeFlag, readFlag, flushFlag, waitStatus;

    // clear read buffer
    for (int i = 0; i < settings->maxBufferSize; i++)
        buf[i] = 0;

    for (r=0; r < settings->retries; ++r) {
        for ( ; r < settings->retries; ++r) {
            writeFlag = WriteToEncoders(data, writeBytes);
            if (writeFlag == -1) return -1;

            waitStatus = WaitReadStatus(readBytes, settings->timeoutMs);
            
            // data is available to be read back
            if (waitStatus == 1) break;
            if (waitStatus == -1) return -1;

            flushFlag = ClearIOBuffers();
            if (flushFlag == -1) return -1;
        }

        if (r >= settings->retries) return -1;

        // read data from uart
        readFlag = ReadFromEncoders(readBytes);
        if (readFlag == readBytes) return 1;
        if (readFlag == -1) 
        {
            ROS_INFO_STREAM("Error reading packet. Read " << readFlag << "bytes");
            return -1;
        }

        if (readFlag != readBytes)
            ROS_INFO_STREAM("Didn't get all bytes");

        flushFlag = ClearIOBuffers();
        if (flushFlag == -1) return -1;
    }

    return r;  // max retries exceeded
}

int Roboclaw::ClearIOBuffers() {
    return tcflush(serialPort, TCIOFLUSH);
}


/* Close serial port file descriptor. */

void Roboclaw::CloseEncoders() {
    close(serialPort);
}

/* Write commands to encoders, check flag status. Returns number of bytes successfully sent to encoders (writeFlag > 0).
   Returns -1 on failure. */

int Roboclaw::WriteToEncoders(uint8_t* data, int nBytes) {
    int writeFlag = write(serialPort, data, nBytes);

    if (writeFlag != nBytes) return -1;

    return writeFlag;
}

/* Check to see if roboclaw encoders sent back data (ACK byte, encoder data, etc.). Poll serial port to check that
   data is available to be read back, within the specified timeout period. Returns 1 on success. Otherwise,
   return 0 or -1 signifying an error. */

int Roboclaw::WaitReadStatus(int nBytes, int timeout_ms) {
    struct timeval tv;
    fd_set input;
    int ret;

    FD_ZERO(&input);
    FD_SET(serialPort, &input);

    tv.tv_sec = 0;
    tv.tv_usec = timeout_ms*1000;

    
    if (tty.c_cc[VMIN] != nBytes) {
        tty.c_cc[VMIN] = nBytes;

        if (tcsetattr(serialPort, TCSANOW, &tty) < 0)
            return -1;  // save settings
    }

    ret = select(serialPort + 1, &input, NULL, NULL, &tv);
    // ret = FD_ISSET(serialPort, &input);

    if (ret < 0)
        return -1;
    else if (ret == 0)
        return 0;

    return 1; // data is avaiable to read
}

/* Reads available data from roboclaw buffer. returns the number of bytes specified by the user upon success (readFlag > 0).
   Returns -1 upon failure. */

int Roboclaw::ReadFromEncoders(int nBytes) {
    for (int i = 0; i < settings->maxBufferSize; i++)
        buf[i] = 0;

    int readFlag = read(serialPort, buf, nBytes);

    if (readFlag != nBytes) 
    {
        return -1;
    }

    return readFlag;
}

/* Get checksum calculation. the Checksum is calculated using Cyclic Redundancy Check (CRC), which uses the encoder address,
   commands and data sent to the encoders. Valid commands will be executed by the encoder, invalid commands will be discarded. */

uint16_t Roboclaw::CalculateChecksum(uint8_t* packet, int nBytes) {
    int crc = 0;

    for (int byte = 0; byte < nBytes; byte++) {
        crc = crc ^ ((uint16_t)packet[byte] << 8);

        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc = crc << 1;
        }
    }

    return crc;
}

/* Move M1 motors. specify the motor address and desired speed value (0-127), then get checksum. Set commands
   and checksum in an array, send commands to be executed. */

void Roboclaw::ForwardM1(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, settings->m1Forward, value};
    uint8_t data[5];

    uint16_t crc = CalculateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

/* Move M1 motors backwards */

void Roboclaw::BackwardM1(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, settings->m1Backward, value};
    uint8_t data[5];

    uint16_t crc = CalculateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

/* Move M2 motors. specify the motor address and desired speed value (0-127), then get checksum. Set commands
   and checksum in an array, send commands to be executed. */

void Roboclaw::ForwardM2(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, settings->m2Forward, value};
    uint8_t data[5];

    uint16_t crc = CalculateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

/* Move M2 motors backwards */

void Roboclaw::BackwardM2(uint8_t address, uint8_t value) {
    uint8_t get_crc[3] = {address, settings->m2Backward, value};
    uint8_t data[5];

    uint16_t crc = CalculateChecksum(get_crc, 3);

    for (int i = 0; i < 3; i++)
        data[i] = get_crc[i];

    data[3] = crc >> 8;
    data[4] = crc;

    SendCommands(data, 5, 1);
}

void Roboclaw::DriveSpeedM1(uint8_t address, int32_t speed)
{
    uint8_t data[8];
    data[0] = address;
    data[1] = settings->m1DriveSpeed;
    Copy_bytes_from_int32(&data[2], speed);

    uint8_t get_crc[6];
    for (int i = 0; i < 6; i++)
        get_crc[i] = data[i];
    uint16_t crc = CalculateChecksum(get_crc, 6);
    data[6] = crc >> 8;
    data[7] = crc & 0xFF;

    SendCommands(data, 8, 1);
}

void Roboclaw::DriveSpeedM2(uint8_t address, int32_t speed)
{
    uint8_t data[8];
    data[0] = address;
    data[1] = settings->m2DriveSpeed;
    Copy_bytes_from_int32(&data[2], speed);

    uint8_t get_crc[6];
    for (int i = 0; i < 6; i++)
        get_crc[i] = data[i];
    uint16_t crc = CalculateChecksum(get_crc, 6);
    data[6] = crc >> 8;
    data[7] = crc & 0xFF;

    SendCommands(data, 8, 1);
}

void Roboclaw::DriveSpeedAccelM1(uint8_t address, int32_t acceleration, int32_t speed)
{
    uint8_t data[12];
    data[0] = address;
    data[1] = settings->m1DriveSpeedAccel;
    Copy_bytes_from_int32(&data[2], acceleration);
    Copy_bytes_from_int32(&data[6], speed);

    uint8_t get_crc[10];
    for (int i = 0; i < 10; i++)
        get_crc[i] = data[i];
    uint16_t crc = CalculateChecksum(get_crc, 10);
    data[10] = crc >> 8;
    data[11] = crc & 0xFF;

    SendCommands(data, 12, 1);
}

void Roboclaw::DriveSpeedAccelM2(uint8_t address, int32_t acceleration, int32_t speed)
{
    uint8_t data[12];
    data[0] = address;
    data[1] = settings->m2DriveSpeedAccel;
    Copy_bytes_from_int32(&data[2], acceleration);
    Copy_bytes_from_int32(&data[6], speed);

    int nBytes = 10;
    uint8_t get_crc[nBytes];
    for (int i = 0; i < nBytes; i++)
        get_crc[i] = data[i];
    uint16_t crc = CalculateChecksum(get_crc, nBytes);
    data[10] = crc >> 8;
    data[11] = crc & 0xFF;

    SendCommands(data, 12, 1);
}

RoboclawPidQppsSettings Roboclaw::ReadPidQppsSettingsM1(uint8_t address)
{
    uint8_t data[2] = {address, settings->m1ReadPidQppsSettings};

    SendCommands(data, 2, 18);

    // read data from buffer
    RoboclawPidQppsSettings pidQppsSettings;
    uint32_t P_temp;
    uint32_t I_temp;
    uint32_t D_temp;
    uint16_t crc;
    Copy_uint32_from_bytes(P_temp, &buf[0]);
    Copy_uint32_from_bytes(I_temp, &buf[4]);
    Copy_uint32_from_bytes(D_temp, &buf[8]);
    Copy_uint32_from_bytes(pidQppsSettings.Qpps, &buf[12]);
    Copy_uint16_from_bytes(crc, &buf[16]);
    pidQppsSettings.P = static_cast<float>(P_temp);
    pidQppsSettings.I = static_cast<float>(I_temp);
    pidQppsSettings.D = static_cast<float>(D_temp);

    return pidQppsSettings;
}

RoboclawPidQppsSettings Roboclaw::ReadPidQppsSettingsM2(uint8_t address)
{
    uint8_t data[2] = {address, settings->m2ReadPidQppsSettings};

    SendCommands(data, 2, 18);

    // read data from buffer
    RoboclawPidQppsSettings pidQppsSettings;
    uint32_t P_temp;
    uint32_t I_temp;
    uint32_t D_temp;
    uint16_t crc;
    Copy_uint32_from_bytes(P_temp, &buf[0]);
    Copy_uint32_from_bytes(I_temp, &buf[4]);
    Copy_uint32_from_bytes(D_temp, &buf[8]);
    Copy_uint32_from_bytes(pidQppsSettings.Qpps, &buf[12]);
    Copy_uint16_from_bytes(crc, &buf[16]);
    pidQppsSettings.P = static_cast<float>(P_temp);
    pidQppsSettings.I = static_cast<float>(I_temp);
    pidQppsSettings.D = static_cast<float>(D_temp);

    return pidQppsSettings;
}

float Roboclaw::ReadMainBatteryVoltage(uint8_t address)
{
    uint8_t data[2] = {address, settings->readMainBatteryVoltage};

    SendCommands(data, 2, 4);


    uint16_t voltage_temp; // read as tenths of a volt
    Copy_uint16_from_bytes(voltage_temp, &buf[0]);
    float voltage = static_cast<float>(voltage_temp) / 10.0f; // divide 10 to get correct voltage units

    return voltage; }

RoboclawMotorCurrents Roboclaw::ReadMotorCurrents(uint8_t address)
{
    uint8_t data[2] = {address, settings->readMotorCurrents};

    SendCommands(data, 2, 6);

    RoboclawMotorCurrents motorCurrents;
    int16_t m1CurrentTemp;
    int16_t m2CurrentTemp;
    Copy_int16_from_bytes(m1CurrentTemp, &buf[0]);
    Copy_int16_from_bytes(m2CurrentTemp, &buf[2]);
    // given in 10mA increments so divide by 100 to get amps
    motorCurrents.m1Current = static_cast<float>(m1CurrentTemp) / 100.0f;
    motorCurrents.m2Current = static_cast<float>(m2CurrentTemp) / 100.0f;

    return motorCurrents;
}

int32_t Roboclaw::ReadEncoderPositionM1(uint8_t address)
{
    //uint8_t get_crc[2] = {address, settings->m1ReadEncoderPosition};
    uint8_t data[2] = {address, settings->m1ReadEncoderPosition};
    
    SendCommands(data, 2, 7);
    int32_t encoderVal = 0;
    encoderVal |= (buf[0] << 24);
    encoderVal |= (buf[1] << 16);
    encoderVal |= (buf[2] << 8);
    encoderVal |= (buf[3] << 0);

    return static_cast<float>(encoderVal);
}

int32_t Roboclaw::ReadEncoderPositionM2(uint8_t address)
{
    //uint8_t get_crc[2] = {address, settings->m1ReadEncoderPosition};
    
    const int statusByte = 4;

    uint8_t data[2] = {address, settings->m2ReadEncoderPosition};
    
    SendCommands(data, 2, 7);
    int32_t encoderVal = 0;
    encoderVal |= (buf[0] << 24);
    encoderVal |= (buf[1] << 16);
    encoderVal |= (buf[2] << 8);
    encoderVal |= (buf[3] << 0);

    return encoderVal;
}

int32_t Roboclaw::ReadEncoderSpeedM1(uint8_t address) {
    uint8_t get_crc[2] = {address, settings->m1ReadEncoderSpeed};
    uint8_t data[7];

    for (int i = 0; i < 2; i++)
        data[i] = get_crc[i];

    SendCommands(data, 2, 7);

    int32_t speed;
    uint8_t direction; // 0 = forwards, 1 = backwards
    Copy_int32_from_bytes(speed, &buf[0]);

    return speed;
}

int32_t Roboclaw::ReadEncoderSpeedM2(uint8_t address) {
    uint8_t get_crc[2] = {address, settings->m2ReadEncoderSpeed};
    uint8_t data[7];

    for (int i = 0; i < 2; i++)
        data[i] = get_crc[i];

    SendCommands(data, 2, 7);

    int32_t speed;
    uint8_t direction; // 0 = forwards, 1 = backwards
    Copy_int32_from_bytes(speed, &buf[0]);

    return speed;
}

void Roboclaw::SetPositionM1(uint8_t address, uint32_t value)
{
    uint8_t tempVal[4];
    tempVal[0] = (uint8_t)((value >> 24) & 0xFF);
    tempVal[1] = (uint8_t)((value >> 16) & 0xFF);
    tempVal[2] = (uint8_t)((value >> 8) & 0xFF);
    tempVal[3] = (uint8_t)(value & 0xFF);
    uint8_t get_crc[7] = {address, settings->m1Position, tempVal[0], tempVal[1], tempVal[2], tempVal[3], 0};
    uint8_t data[9];

    data[0] = address;
    data[1] = settings->m2Position;
    data[2] = (uint8_t)((value >> 24) & 0xFF);
    data[3] = (uint8_t)((value >> 16) & 0xFF);
    data[4] = (uint8_t)((value >> 8) & 0xFF);
    data[5] = (uint8_t)(value & 0xFF);
    data[6] = 0; // buffer option

    uint16_t crc = CalculateChecksum(get_crc, 7);

    data[7] = crc >> 8;
    data[8] = crc;

    SendCommands(data, 9, 1);
}

void Roboclaw::SetPositionM2(uint8_t address, int32_t value)
{
    uint8_t tempVal[4];

    tempVal[0] = ((value >> 24) & 0xFF);
    tempVal[1] = ((value >> 16) & 0xFF);
    tempVal[2] = ((value >> 8) & 0xFF);
    tempVal[3] = (value & 0xFF);

    uint8_t get_crc[7] = {address, settings->m2Position, tempVal[0], tempVal[1], tempVal[2], tempVal[3], 0};
    uint8_t data[9];

    data[0] = address;
    data[1] = settings->m2Position;
    data[2] = ((value >> 24) & 0xFF);
    data[3] = ((value >> 16) & 0xFF);
    data[4] = ((value >> 8) & 0xFF);
    data[5] = (value & 0xFF);
    data[6] = 0; // buffer option

    uint16_t crc = CalculateChecksum(get_crc, 7);

    data[7] = crc >> 8;
    data[8] = crc;

    SendCommands(data, 9, 1);
}

void Roboclaw::SendCommandToWheels(double* cmd) {
    uint8_t cmd_send[6] = {0};

    // convert cmd_vel to a usable command between 0-127

    for (int i = 0; i <= 5; i++)
        cmd_send[i] = ScaleCommand(cmd[i]);

    // prevent zero velocity spamming from ros_control
    if (zeroCmdVelCount <= settings->retries) {
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] >= 0)  // right_front
            ForwardM1(0x80, cmd_send[0]);
        else
            BackwardM1(0x80, cmd_send[0]);

        if (cmd[1] >= 0)  // right_middle
            ForwardM1(0x81, cmd_send[1]);
        else
            BackwardM1(0x81, cmd_send[1]);

        if (cmd[2] >= 0)  // right_back
            ForwardM1(0x82, cmd_send[2]);
        else
            BackwardM1(0x82, cmd_send[2]);

        if (cmd[3] >= 0)  // left_front
            ForwardM2(0x80, cmd_send[3]);
        else
            BackwardM2(0x80, cmd_send[3]);

        if (cmd[4] >= 0)  // left_middle
            ForwardM2(0x81, cmd_send[4]);
        else
            BackwardM2(0x81, cmd_send[4]);

        if (cmd[5] >= 0)  // left_back
            ForwardM2(0x82, cmd_send[5]);
        else
            BackwardM2(0x82, cmd_send[5]);
    }

    // if any of the cmd_vel are zero, increment counter

    if (cmd[0] == 0 || cmd[1] == 0 || cmd[2] == 0 ||
        cmd[3] == 0 || cmd[4] == 0 || cmd[5] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = cmd[4] = cmd[5] = 0;
    }
}

void Roboclaw::GetVelocityFromWheels(double* vel) {
    // return positive or negative value from encoders, depending on direction
    ReadEncoderSpeedM1(0x80);  // right_front
    vel[0] = ConvertPulsesToRadians(
        static_cast<double> (RecombineBuffer(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x80);  // right_middle
    vel[1] = ConvertPulsesToRadians(
        static_cast<double> (RecombineBuffer(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM1(0x81);  // right_back
    vel[2] = ConvertPulsesToRadians(
        static_cast<double> (RecombineBuffer(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x82);  // left_front
    vel[3] = ConvertPulsesToRadians(
        static_cast<double> (RecombineBuffer(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM1(0x82);  // left_middle
    vel[4] = ConvertPulsesToRadians(
        static_cast<double> (RecombineBuffer(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];

    ReadEncoderSpeedM2(0x81);  // left_back
    vel[5] = ConvertPulsesToRadians(
        static_cast<double> (RecombineBuffer(buf)));
    if (buf[4] == 1) vel[0] = -vel[0];
}

/**
 *  Scale command between 0-127 to be sent to encoders
 */

uint8_t Roboclaw::ScaleCommand(double cmd) {
    double res = (fabs(cmd) / 16.667) * settings->maxEffortValue;

    if (res >= settings->maxEffortValue)
        return settings->maxEffortValue;

    return (uint8_t) res;  // setup for teleop_twist for now, max teleop_twist is 16.667
}

/* will need to fix conversion in later push */

double Roboclaw::ConvertPulsesToRadians(double vel) {
    return (vel/119.3697);  // convert from QPPS to rad/s, (750 p/rev)(1/(2*pi))
}

// leftmost byte is most significant byte
void Roboclaw::Copy_uint16_from_bytes(uint16_t& to, uint8_t* from)
{
    to = 0;
    to |= from[0] << 8;
    to |= from[1];
}

// leftmost byte is most significant byte
void Roboclaw::Copy_uint32_from_bytes(uint32_t& to, uint8_t* from)
{
    to = 0;
    to |= from[0] << 24;
    to |= from[1] << 16;
    to |= from[2] << 8;
    to |= from[3];
}

// leftmost byte is most significant byte
void Roboclaw::Copy_int16_from_bytes(int16_t& to, uint8_t* from)
{
    to = 0;
    to |= from[0] << 8;
    to |= from[1];
}

// leftmost byte is most significant byte
void Roboclaw::Copy_int32_from_bytes(int32_t& to, uint8_t* from)
{
    to = 0;
    to |= from[0] << 24;
    to |= from[1] << 16;
    to |= from[2] << 8;
    to |= from[3];
}

void Roboclaw::Copy_bytes_from_int32(uint8_t* to, int32_t from)
{
    to[0] = (from >> 24) & 0xFF;
    to[1] = (from >> 16) & 0xFF;
    to[2] = (from >> 8) & 0xFF;
    to[3] = (from) & 0xFF;
}