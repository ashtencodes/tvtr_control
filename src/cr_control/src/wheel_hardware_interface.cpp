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

#include <cr_control/roboclaw.h>
#include <cr_control/wheel_hardware_interface.h>
#include <vector>
#include <string>
#include <math.h>
#include <cstdlib>
// ros messages
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <cr_control/wheel_data.h>
// for debugging
#include <bitset>

void GetYamlParameters(ros::NodeHandle*, WheelHwinSettings*, RoboclawSettings*);
bool validateSettingsAndLogErrors(WheelHwinSettings*);

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("Loading wheel_hardware_interface_node");
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO("Getting yaml parameters for wheel hardware interface settings");
    WheelHwinSettings wheelSettings;
    RoboclawSettings roboclawSettings;
    GetYamlParameters(&nh, &wheelSettings, &roboclawSettings);

    ROS_INFO("Initializing wheel hardware interface");
    WheelHardwareInterface wheelHwin(&nh, &wheelSettings);

    ROS_INFO("Initializing controller manager");
    controller_manager::ControllerManager cm(&wheelHwin);

    ROS_INFO("Initializing roboclaw interface");
    Roboclaw roboclaw(&roboclawSettings);
    ros::Rate rate(wheelSettings.rosLoopRate);

    while (ros::ok()) 
    {
        wheelHwin.readFromWheels(&roboclaw);
        cm.update(wheelHwin.get_time(), wheelHwin.get_period());
        wheelHwin.writeToWheels(&roboclaw);
        rate.sleep();
    }

    return 0;
}

void GetYamlParameters(ros::NodeHandle* nh, WheelHwinSettings *wheelSettings, RoboclawSettings *roboclawSettings) {

    for (int i = 0; i < 2; i++) {
        wheelSettings->rightWheelRoboclawAddresses[i] = 0;
        wheelSettings->leftWheelRoboclawAddresses[i] = 0;
    }

    nh->getParam("/roboclaw_settings/serial_port", roboclawSettings->serialPortAddress);
    nh->getParam("/roboclaw_settings/send_command_retries", roboclawSettings->retries);
    nh->getParam("/roboclaw_settings/encoder_timeout_ms", roboclawSettings->timeoutMs);
    nh->getParam("/roboclaw_settings/loop_frequency", roboclawSettings->loopFrequency);
    nh->getParam("/roboclaw_settings/baud_rate", roboclawSettings->baudRate);

    std::vector<std::string> rightWheelNames, leftWheelNames; // list of wheel joint names
    std::vector<int> leftWheelAddresses, rightWheelAddresses; // each wheel's corresponding roboclaw addresses
    nh->getParam("/wheel_hwin_settings/debug_mode", wheelSettings->debugMode);
    nh->getParam("/roboclaw_settings/send_command_retries", wheelSettings->maxRetries);
    nh->getParam("/wheel_hwin_settings/ros_loop_rate", wheelSettings->rosLoopRate);
    nh->getParam("/wheel_hwin_settings/left_addr", leftWheelAddresses);
    nh->getParam("/wheel_hwin_settings/right_addr", rightWheelAddresses);
    nh->getParam("/wheel_hwin_settings/left_wheel", leftWheelNames);
    nh->getParam("/wheel_hwin_settings/right_wheel", rightWheelNames);

    for (int i = 0; i < 2; i++) {
        // copy ros_params settings into wheel settings struct
        std::copy(leftWheelNames.begin(), leftWheelNames.end(), wheelSettings->leftWheelNames);
        std::copy(rightWheelNames.begin(), rightWheelNames.end(), wheelSettings->rightWheelNames);
        std::copy(leftWheelAddresses.begin(), leftWheelAddresses.end(), wheelSettings->leftWheelRoboclawAddresses);
        std::copy(rightWheelAddresses.begin(), rightWheelAddresses.end(), wheelSettings->rightWheelRoboclawAddresses);

    }

    nh->getParam("/wheel_hwin_settings/motor_data/encoderTicks_per_radian", wheelSettings->encoderTicksPerRadian);
    nh->getParam("/wheel_hwin_settings/motor_data/radians_per_encoderTick", wheelSettings->radiansPerEncoderTick);

    bool errorFlag = validateSettingsAndLogErrors(wheelSettings);

    if (errorFlag)
        exit(EXIT_FAILURE);
}

bool validateSettingsAndLogErrors(WheelHwinSettings *wheelSettings)
{
    bool errorFlag = false;
    for (int i = 0; i < 2; i++)
    {
        // error checking
        if (wheelSettings->rightWheelNames[i] == "") {
            ROS_ERROR("Right joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->rightWheelRoboclawAddresses[i] < 128 ||
             wheelSettings->rightWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Right address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->leftWheelNames[i] == "") {
            ROS_ERROR("Left Joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            errorFlag = true;
        }

        if (wheelSettings->leftWheelRoboclawAddresses[i] < 128 || 
            wheelSettings->leftWheelRoboclawAddresses[i] > 135) {
            ROS_ERROR("Left address [%d] : Incorrect address "
                      "specified in YAML file", i);
            errorFlag = true;
        }
    }

    return errorFlag;
}

WheelHardwareInterface::WheelHardwareInterface(ros::NodeHandle* nh, WheelHwinSettings* wheelSettings)
{
    // data publishing setup
    this->nodeHandle = nh;
    roverDataPub = nh->advertise<cr_control::wheel_data>("Wheel/data", 1000);
    test = nh->advertise<std_msgs::Float64>("WheelVelocity", 1000);
    // wheelPosPub = nh->advertise<std_msgs::Int32>("/Wheels/position", 1000);
    // wheelVoltagePub = nh->advertise<std_msgs::Float64>("/Wheels/voltage", 1000);
    // wheelAmpPub = nh->advertise<std_msgs::Float64>("/Wheels/amps", 1000);

    this->wheelSettings = wheelSettings;
    ROS_INFO("Registering ros_control joint interfaces");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 4; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

WheelHardwareInterface::~WheelHardwareInterface() { }

void WheelHardwareInterface::writeToWheels(Roboclaw *rb)
{
    if (wheelSettings->debugMode) {
        ROS_INFO_STREAM("READING JOINT STATES FROM ROS");
        printDebugInfo("SENDING CMD_VEL TO", cmd);
    }

    // need to divide cmd by 10 because diff drive controller 
    // multiplies topic input by 10 for some reason
    for (int i = 0; i < 4; i++)
        cmd[i] /= 10;
    // sendCommandToWheels(rb);
    driveWithSpeed(rb);
}

void WheelHardwareInterface::readFromWheels(Roboclaw *rb)
{
    if (wheelSettings->debugMode) {
        rb->GetVelocityFromWheels(vel);

        ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
        printDebugInfo("VEL FROM", vel);
    }

    // cr_control::wheel_data wheel_data_msg;

    // msg.voltage = rb->ReadMainBatteryVoltage(129);
    // RoboclawMotorCurrents motorCurrents = rb->ReadMotorCurrents(129);
    // msg.m1Amps = motorCurrents.m1Current;
    // msg.m2Amps = motorCurrents.m2Current;

    // msg.m1EncoderCount = 0;
    // msg.m2EncoderCount = 0;
    cr_control::wheel_data msg;
    RoboclawMotorCurrents motorCurrentsFront = rb->ReadMotorCurrents(128);
    RoboclawMotorCurrents motorCurrentsBack = rb->ReadMotorCurrents(129);
    msg.m1FrontAmps = motorCurrentsFront.m1Current;
    msg.m2FrontAmps = motorCurrentsFront.m2Current;
    msg.m1BackAmps = motorCurrentsBack.m1Current;
    msg.m2BackAmps = motorCurrentsBack.m2Current;

    msg.voltageFront = rb->ReadMainBatteryVoltage(128);
    msg.voltageBack = rb->ReadMainBatteryVoltage(129);

    msg.m1FrontVelocity = encoderCountToRadians(rb->ReadEncoderSpeedM1(128));
    msg.m2FrontVelocity = encoderCountToRadians(rb->ReadEncoderSpeedM2(128));
    msg.m1BackVelocity = encoderCountToRadians(rb->ReadEncoderSpeedM1(129));
    msg.m2BackVelocity = encoderCountToRadians(rb->ReadEncoderSpeedM2(129));

    std_msgs::Float64 velocityMsg;
    velocityMsg.data = encoderCountToRadians(rb->ReadEncoderSpeedM1(128));
    test.publish(velocityMsg);

    roverDataPub.publish(msg);
    // std_msgs::Float64 voltage_msg;
    // voltage_msg.data = rb->ReadMainBatteryVoltage(129);
    // wheelVoltagePub.publish(voltage_msg);

    // std_msgs::Float64 m1Amp_msg;
    // RoboclawMotorCurrents currents = rb->ReadMotorCurrents(129);
    // m1Amp_msg.data = currents.m1Current;
    // wheelAmpPub.publish(m1Amp_msg);
    //    uint32_t m1Pos = rb->ReadEncoderPositionM1(129);
    //    std_msgs::Int32 msg;
    //    msg.data = m1Pos;
    //    wheelPosPub.publish(msg);
    // ROS_INFO_STREAM("Encoder value: " << std::bitset<32>(m1Pos));
}

void WheelHardwareInterface::sendCommandToWheels(Roboclaw* rb)
{
    // convert cmd_vel to a usable command between 0-127
    scaleCommands();

    

    // prevent zero velocity spamming from ros_control
    if (zeroCmdVelCount <= wheelSettings->maxRetries) {
        // if positive, move motors forward. if negative, move backwards
        if (cmd[0] >= 0)  // right_front
            rb->ForwardM2(0x80, cmdToSend[0]);
        else
            rb->BackwardM2(0x80, cmdToSend[0]);

        if (cmd[1] >= 0)  // right_back
            rb->ForwardM2(0x81, cmdToSend[1]);
        else
            rb->BackwardM2(0x81, cmdToSend[1]);

        if (cmd[2] >= 0)  // left_front
            rb->ForwardM1(0x80, cmdToSend[2]);
        else
            rb->BackwardM1(0x80, cmdToSend[2]);

        if (cmd[3] >= 0)  // left_back
            rb->ForwardM1(0x81, cmdToSend[3]);
        else
            rb->BackwardM1(0x81, cmdToSend[3]);

    }
    // if any of the cmd_vel are zero, increment counter

    if (cmd[0] == 0 || cmd[1] == 0 || cmd[2] == 0 ||
        cmd[3] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0;
    }
}

void WheelHardwareInterface::driveWithSpeed(Roboclaw *rb)
{
    int32_t cmd_to_send[4];
    for (int i = 0; i < 4; i++)
    {
        cmd_to_send[i] = radiansToEncoderCount(cmd[i]);
    }

    // if (cmd[0] != 0)
        rb->DriveSpeedM2(128, cmd_to_send[0]);
    // if (cmd[0] != 0)
        rb->DriveSpeedM2(129, cmd_to_send[1]);
    // if (cmd[0] != 0)
        rb->DriveSpeedM1(128, cmd_to_send[2]);
    // if (cmd[0] != 0)
        rb->DriveSpeedM1(129, cmd_to_send[3]);

    if (cmd[0] == 0 || cmd[1] == 0 || cmd[2] == 0 ||
        cmd[3] == 0) {
        zeroCmdVelCount++;
    } else {
        zeroCmdVelCount = 0;  // reset counter
        cmd[0] = cmd[1] = cmd[2] = cmd[3] = 0;
    }
}

void WheelHardwareInterface::scaleCommands()
{
    // TODO scaled the commands from 0-126 to send to roboclaw
    for (int i = 0; i < 4; i++)
    {
        double res = (fabs(cmd[i]) / 16.667) * wheelSettings->maxEffortValue;

        if (res >= wheelSettings->maxEffortValue)
            cmdToSend[i] = wheelSettings->maxEffortValue;
        else
            cmdToSend[i] = (uint8_t) res;
    }
}

void WheelHardwareInterface::getVelocityFromEncoders()
{
    // TODO use convertPulsesToRadians for distance conversion
    // velocity = distance / time
}

float WheelHardwareInterface::encoderCountToRadians(int32_t encoderCount)
{
    return ((float)encoderCount) * wheelSettings->radiansPerEncoderTick;
}

int32_t WheelHardwareInterface::radiansToEncoderCount(float radians)
{
    return radians * wheelSettings->encoderTicksPerRadian;
}

ros::Time WheelHardwareInterface::get_time()
{
    return ros::Time::now();
}

ros::Duration WheelHardwareInterface::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

void WheelHardwareInterface::printDebugInfo(std::string name, double* data) {
    ROS_INFO_STREAM(name << " RIGHT_FRONT_WHEEL_JOINT "  << data[0]);
    ROS_INFO_STREAM(name << " RIGHT_BACK_WHEEL_JOINT " << data[1]);
    ROS_INFO_STREAM(name << " LEFT_FRONT_WHEEL_JOINT "   << data[2]);
    ROS_INFO_STREAM(name << " LEFT_BACK_WHEEL_JOINT "   << data[3]);
}


void WheelHardwareInterface::registerStateHandlers()
{
    ROS_INFO("Registering Joint State Interface");
    hardware_interface::JointStateHandle wheelRightFront(
        wheelSettings->rightWheelNames[0], &pos[0], &vel[0], &eff[0]);
    jointStateInterface.registerHandle(wheelRightFront);

    hardware_interface::JointStateHandle wheelRightBack(
        wheelSettings->rightWheelNames[1], &pos[1], &vel[1], &eff[1]);
    jointStateInterface.registerHandle(wheelRightBack);

    hardware_interface::JointStateHandle wheelLeftFront(
        wheelSettings->leftWheelNames[0], &pos[2], &vel[2], &eff[2]);
    jointStateInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointStateHandle wheelLeftBack(
        wheelSettings->leftWheelNames[1], &pos[3], &vel[3], &eff[3]);
    jointStateInterface.registerHandle(wheelLeftBack);

    registerInterface(&jointStateInterface);
}
void WheelHardwareInterface::registerJointVelocityHandlers()
{
    ROS_INFO("Registering Velocity Joint Interface");
    hardware_interface::JointHandle wheelRightFront(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[0]), &cmd[0]);
    velocityJointInterface.registerHandle(wheelRightFront);

    hardware_interface::JointHandle wheelRightBack(
        jointStateInterface.getHandle(wheelSettings->rightWheelNames[1]), &cmd[1]);
    velocityJointInterface.registerHandle(wheelRightBack);

    hardware_interface::JointHandle wheelLeftFront(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[0]), &cmd[2]);
    velocityJointInterface.registerHandle(wheelLeftFront);

    hardware_interface::JointHandle wheelLeftBack(
        jointStateInterface.getHandle(wheelSettings->leftWheelNames[1]), &cmd[3]);
    velocityJointInterface.registerHandle(wheelLeftBack);

    registerInterface(&velocityJointInterface);
}