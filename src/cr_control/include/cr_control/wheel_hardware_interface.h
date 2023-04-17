#ifndef CR_CONTROL_WHEEL_HARDWARE_INTERFACE_H
#define CR_CONTROL_WHEEL_HARDWARE_INTERFACE_H

// essential header files
#include <hardware_interface/joint_state_interface.h>  // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <controller_manager/controller_manager.h> // to have a controller in our class
#include <hardware_interface/robot_hw.h> // useful functions
#include <ros/ros.h>
#include <string>
#include <cr_control/roboclaw.h>

struct WheelHwinSettings
{
    std::string leftWheelNames[2];
    std::string rightWheelNames[2];
    int leftWheelRoboclawAddresses[2]; // Corresponding wheel to roboclaw address
    int rightWheelRoboclawAddresses[2];
    uint8_t maxEffortValue = 126;
    int rosLoopRate = 10;
    int maxRetries = 3;
    bool debugMode = false;
};

//might need serial to send commands to the hardware
class WheelHardwareInterface : public hardware_interface::RobotHW
{
public:
    WheelHardwareInterface(ros::NodeHandle *, WheelHwinSettings*);
    ~WheelHardwareInterface();
    void writeToWheels(Roboclaw*);
    void readFromWheels(Roboclaw*);

    ros::Time get_time();
    ros::Duration get_period();

private:
    WheelHwinSettings *wheelSettings;

    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jointStateInterface;
    hardware_interface::VelocityJointInterface velocityJointInterface;
    
    void registerStateHandlers();
    void registerJointVelocityHandlers();
    void printDebugInfo(std::string name, double* data);

    void sendCommandToWheels(Roboclaw *);
    void scaleCommands();
    void getVelocityFromEncoders(); // TODO important for arm
    double convertPulsesToRadians(double vel); 

    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
    int zeroCmdVelCount;
    // For reading commands sent from the controller
    double cmd[4];
    uint8_t cmdToSend[4];
    // for sending data relating to the joints
    double pos[4];
    double vel[4];
    double eff[4];
    static constexpr double BILLION = 1000000000.0; // nanoseconds to seconds
};

#endif
