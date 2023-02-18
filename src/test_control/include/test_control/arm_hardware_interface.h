// essential header files
#include <hardware_interface/joint_state_interface.h>  // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <controller_manager/controller_manager.h> // to have a controller in our class
#include <hardware_interface/robot_hw.h> // useful functions
#include <ros/ros.h>
#include <string>

#include <test_control/l298n.h>


//might need serial to send commands to the hardware
class ArmHardwareInterface : public hardware_interface::RobotHW
{
public:
    ArmHardwareInterface(ros::NodeHandle *nh);
    ~ArmHardwareInterface();
    void writeToArm(l298n* _l298n);
    void readFromArm(l298n* _l298n);

    ros::Time get_time();
    ros::Duration get_period();

private:
    // for conversion from nanoseconds to seconds
    static constexpr double BILLION = 1000000000.0;
    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jntStateInterface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;

    void registerStateHandlers();
    void registerJointVelocityHandlers();
    void printDebugInfo(std::string name, double* data);
    void ScaleCommands();

    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
    // For reading commands sent from the controller
    double cmd[6];
    // for sending data relating to the joints
    double pos[6];
    double vel[6];
    double eff[6];
    std::string armJoints[6] = {"left_joint","right_joint","shoulderToLittle","armToBot","armToEnd","endeffector"};
    uint8_t linearActuatorIndex = 0;
    // arm_settings *es;
};