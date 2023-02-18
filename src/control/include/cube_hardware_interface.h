// essential header files
#include <hardware_interface/joint_state_interface.h>  // for reading the state of the joints (position, velocity, effort)
#include <hardware_interface/joint_command_interface.h> // for sending commands to the joints
#include <controller_manager/controller_manager.h> // to have a controller in our class
#include <hardware_interface/robot_hw.h> // useful functions
#include <ros/ros.h>
#include <string>
#include <gnc_control/roboclaw.h>
#include <gnc_control/settings.h>


//might need serial to send commands to the hardware
class RobotHardwareInterface : public hardware_interface::RobotHW
{
public:
    RobotHardwareInterface(ros::NodeHandle *nh, settings *es_ptr);
    ~RobotHardwareInterface();
    void writeToWheels(roboclaw* rb, settings* es);
    void readFromWheels(roboclaw* rb, settings* es);

    ros::Time get_time();
    ros::Duration get_period();

private:
    // for conversion from nanoseconds to seconds
    static constexpr double BILLION = 1000000000.0;
    // For reading the state of the wheels, and sending desired velocity (commands) to wheels
    hardware_interface::JointStateInterface jntStateInterface;
    hardware_interface::VelocityJointInterface jnt_vel_interface;
    XmlRpc::XmlRpcValue right_joint_list, left_joint_list;
    XmlRpc::XmlRpcValue right_joint_addr_list, left_joint_addr_list;

    void registerStateHandlers();
    void registerJointVelocityHandlers();
    void printDebugInfo(std::string name, double* data);
    void setYamlParameters(ros::NodeHandle* nh);

    ros::Duration elapsed_time;
    struct timespec last_time;
    struct timespec current_time;
    // For reading commands sent from the controller
    double cmd[6];
    // for sending data relating to the joints
    double pos[6];
    double vel[6];
    double eff[6];
    settings *es;
};