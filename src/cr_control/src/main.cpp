#include <cr_control/roboclaw.h>
#include <cr_control/wheel_hardware_interface.h>
#include <vector>

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
    MPU6050 mpu6050[2] = {MPU6050(0x68, 0), MPU6050(0x68, 1)};
    WheelHardwareInterface wheelHwin(&nh, &wheelSettings, &mpu6050[0]);

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