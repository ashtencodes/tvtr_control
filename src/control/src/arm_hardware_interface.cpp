#include <tvtr_control/robot_hardware_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO_STREAM("Loading arm_control_hw_node");
    settings esMain;
    RobotHardwareInterface robot(&nh, &esMain);
    controller_manager::ControllerManager cm(&robot);
    roboclaw rb(&esMain);
    ros::Rate rate(esMain.loopFrequency);

    ROS_INFO("Initializing roboclaw motor encoders");
    rb.SetupEncoders();

    while (ros::ok()) {
        robot.readFromWheels(&rb, &esMain);
        cm.update(robot.get_time(), robot.get_period());
        robot.writeToWheels(&rb, &esMain);
        rate.sleep();
    }

    ROS_INFO("Shutting down roboclaw motor encoders");
    rb.CloseEncoders();

    return 0;
}

RobotHardwareInterface::RobotHardwareInterface(ros::NodeHandle* nh, settings* esPtr)
{
    es = esPtr;
    ROS_INFO("Setting Yaml parameters for serial port");
    setYamlParameters(nh);
    ROS_INFO("Registering ros_control handlers");
    registerStateHandlers();
    registerJointVelocityHandlers();

    clock_gettime(CLOCK_MONOTONIC, &last_time);

    for (int i = 0; i < 6; i++)
        cmd[i] = vel[i] = pos[i] = eff[i] = 0;
}

RobotHardwareInterface::~RobotHardwareInterface() { }

void RobotHardwareInterface::registerStateHandlers()
{
    hardware_interface::JointStateHandle stateHandleA(
        es->rightJoints[0], &pos[0], &vel[0], &eff[0]);
    jntStateInterface.registerHandle(stateHandleA);

    hardware_interface::JointStateHandle stateHandleB(
        es->rightJoints[1], &pos[1], &vel[1], &eff[1]);
    jntStateInterface.registerHandle(stateHandleB);

    hardware_interface::JointStateHandle stateHandleC(
        es->rightJoints[2], &pos[2], &vel[2], &eff[2]);
    jntStateInterface.registerHandle(stateHandleC);

    hardware_interface::JointStateHandle stateHandleD(
        es->leftJoints[0], &pos[3], &vel[3], &eff[3]);
    jntStateInterface.registerHandle(stateHandleD);

    hardware_interface::JointStateHandle stateHandleE(
        es->leftJoints[1], &pos[4], &vel[4], &eff[4]);
    jntStateInterface.registerHandle(stateHandleE);

    hardware_interface::JointStateHandle stateHandleF(
        es->leftJoints[2], &pos[5], &vel[5], &eff[5]);
    jntStateInterface.registerHandle(stateHandleF);

    registerInterface(&jntStateInterface);
}
void RobotHardwareInterface::registerJointVelocityHandlers()
{
        hardware_interface::JointHandle vel_handle_a(
        jntStateInterface.getHandle(es->rightJoints[0]), &cmd[0]);
    jnt_vel_interface.registerHandle(vel_handle_a);

    hardware_interface::JointHandle vel_handle_b(
        jntStateInterface.getHandle(es->rightJoints[1]), &cmd[1]);
    jnt_vel_interface.registerHandle(vel_handle_b);

    hardware_interface::JointHandle vel_handle_c(
        jntStateInterface.getHandle(es->rightJoints[2]), &cmd[2]);
    jnt_vel_interface.registerHandle(vel_handle_c);

    hardware_interface::JointHandle vel_handle_d(
        jntStateInterface.getHandle(es->leftJoints[0]), &cmd[3]);
    jnt_vel_interface.registerHandle(vel_handle_d);

    hardware_interface::JointHandle vel_handle_e(
        jntStateInterface.getHandle(es->leftJoints[1]), &cmd[4]);
    jnt_vel_interface.registerHandle(vel_handle_e);

    hardware_interface::JointHandle vel_handle_f(
        jntStateInterface.getHandle(es->leftJoints[2]), &cmd[5]);
    jnt_vel_interface.registerHandle(vel_handle_f);

    registerInterface(&jnt_vel_interface);
}

void RobotHardwareInterface::writeToWheels(roboclaw *rb, settings *es)
{
    if (es->debug_mode) {
        ROS_INFO_STREAM("READING JOINT STATES FROM ROS");
        printDebugInfo("SENDING CMD_VEL TO", cmd);
    }

    rb->SendCommandToWheels(cmd);
}

void RobotHardwareInterface::readFromWheels(roboclaw *rb, settings *es)
{
    if (es->debug_mode) {
        rb->GetVelocityFromWheels(vel);

        ROS_INFO_STREAM("READING JOINT STATES FROM MOTOR ENCODERS");
        printDebugInfo("VEL FROM", vel);
    }
}

ros::Time RobotHardwareInterface::get_time()
{
    return ros::Time::now();
}

ros::Duration RobotHardwareInterface::get_period()
{
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    elapsed_time =
            ros::Duration(current_time.tv_sec - last_time.tv_sec
            + (current_time.tv_nsec - last_time.tv_nsec) / BILLION);
    last_time = current_time;

    return elapsed_time;
}

void RobotHardwareInterface::printDebugInfo(std::string name, double* data) {
    ROS_INFO_STREAM(name << " RIGHT_FRONT_WHEEL_JOINT "  << data[0]);
    ROS_INFO_STREAM(name << " RIGHT_MIDDLE_WHEEL_JOINT " << data[1]);
    ROS_INFO_STREAM(name << " RIGHT_BACK_WHEEL_JOINT "   << data[2]);
    ROS_INFO_STREAM(name << " LEFT_FRONT_WHEEL_JOINT "   << data[3]);
    ROS_INFO_STREAM(name << " LEFT_MIDDLE_WHEEL_JOINT "  << data[4]);
    ROS_INFO_STREAM(name << " LEFT_BACK_WHEEL_JOINT "    << data[5]);
}

void RobotHardwareInterface::setYamlParameters(ros::NodeHandle* nh) {
    int exitFlag = false;
    for (int i = 0; i < 2; i++) {
        es->rightAddr[i] = 0;
        es->leftAddr[i] = 0;
    }

    nh->getParam("/wheel_encoders/debug_mode", es->debug_mode);
    nh->getParam("/wheel_encoders/serial_port", es->serialPortAddr);
    nh->getParam("/wheel_encoders/send_command_retries", es->retries);
    nh->getParam("/wheel_encoders/encoder_timeout_ms", es->timeout_ms);
    nh->getParam("/wheel_encoders/loop_frequency", es->loopFrequency);
    nh->getParam("/wheel_encoders/baud_rate", es->baud_rate);
    nh->getParam("/wheel_encoders/right_wheel", right_joint_list);
    nh->getParam("/wheel_encoders/left_wheel", left_joint_list);
    nh->getParam("/wheel_encoders/right_addr", right_joint_addr_list);
    nh->getParam("/wheel_encoders/left_addr", left_joint_addr_list);

    for (int i = 0; i <= 2; i++) {
        es->rightJoints[i] = static_cast<std::string> (right_joint_list[i]);
        es->leftJoints[i] = static_cast<std::string> (left_joint_list[i]);
        es->rightAddr[i] = static_cast<int> (right_joint_addr_list[i]);
        es->leftAddr[i] = static_cast<int> (left_joint_addr_list[i]);

        if (es->rightJoints[i] == "") {
            ROS_ERROR("Right joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            exitFlag = true;
        }

        if (es->rightAddr[i] < 128 || es->rightAddr[i] > 135) {
            ROS_ERROR("Right address [%d] : Incorrect address "
                      "specified in YAML file", i);
            exitFlag = true;
        }

        if (es->leftJoints[i] == "") {
            ROS_ERROR("Left Joint [%d] : Incorrect number of "
                      "joints specified in YAML file", i);
            exitFlag = true;
        }

        if (es->leftAddr[i] < 128 || es->leftAddr[i] > 135) {
            ROS_ERROR("Left address [%d] : Incorrect address "
                      "specified in YAML file", i);
            exitFlag = true;
        }
    }

    if (exitFlag)
        exit(EXIT_FAILURE);
}