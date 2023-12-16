/************************************************************************
*
* @brief Wrapper around Unitree's UDP communication protocol
* This class is meant to 
*     (i)  be compiled as a Python module using pybind11
*     (ii) be the parent class of ROS interfaces to publish the UDP readings
* 
*
* @author Alonso Marco
* Contact: amarco@berkeley.edu
************************************************************************/

#include <python_interface_go1/real_robot_interface_go1_highlevel.hpp>

void RealRobotInterfaceGo1HighLevel::_collect_observations() {
    this->udp.Recv();
    this->udp.GetRecv(this->state);

    // // DBG:
    // for(int i=0; i<4; ++i){
    //     std::cout << "\nRealRobotInterfaceGo1HighLevel::_collect_observations:\n";
    //     std::cout << "this->state.footForce[i]: " << this->state.footForce[i] << "\n";
    //     std::cout << "this->state.footForceEst[i]: " << this->state.footForceEst[i] << "\n";
    // }

    return;
}

void RealRobotInterfaceGo1HighLevel::get_observations(void) {

    this->_collect_observations();
    this->get_joint_pos_curr();
    this->get_joint_vel_curr();
    this->get_body_linear_velocity();
    this->get_body_angular_velocity();
    this->get_body_orientation();

    return;
}

void RealRobotInterfaceGo1HighLevel::get_joint_pos_curr(void){

    for (int jj = 0; jj < this->Njoints; jj++){ // amarco: std::array<MotorState, 20> motorState; we only need the first 12 elements, corresponding to the joints
        this->joint_pos_curr[jj] = this->state.motorState[jj].q;
    }

    return;
}

void RealRobotInterfaceGo1HighLevel::get_joint_vel_curr(void){

    for (int jj = 0; jj < this->Njoints; jj++)
        this->joint_vel_curr[jj] = this->state.motorState[jj].dq;

    return;
}


void RealRobotInterfaceGo1HighLevel::get_body_linear_velocity(void){

    // Numerical integration:
    for (int jj = 0; jj < this->body_linear_velocity.size(); jj++)
        this->body_linear_velocity[jj] += this->deltaT*this->state.imu.accelerometer[jj];

    return;
}

void RealRobotInterfaceGo1HighLevel::get_body_angular_velocity(void){

    // Numerical integration:
    for (int jj = 0; jj < this->body_angular_velocity.size(); jj++)
        this->body_angular_velocity[jj] = this->state.imu.gyroscope[jj];

    return;
}


void RealRobotInterfaceGo1HighLevel::get_body_orientation(void){

    // Numerical integration:
    for (int jj = 0; jj < this->body_orientation.size(); jj++)
        this->body_orientation[jj] = this->state.imu.rpy[jj];

    return;
}

void RealRobotInterfaceGo1HighLevel::update_mode_behavior(uint8_t mode_behavior){

    // See include/comm.h for more info

    if(mode_behavior != 0 && mode_behavior != 1 && mode_behavior != 2){
        std::cout << "Requested mode is unaccepted; only modes {0: idle; 1: force stand; 2: walking with target velocity} admitted\n";
        std::cout << "Not sending desired mode to the robot\n";
        return;
    }
    else{
        this->cmd.mode = mode_behavior;        
    }

    this->udp.SetSend(this->cmd);
    this->udp.Send();

    return;
}

void RealRobotInterfaceGo1HighLevel::update_gait_type(uint8_t gait_type){

    // See include/comm.h for more info

    if(gait_type != 0 && gait_type != 1){
        std::cout << "Requested gait is unaccepted; only modes {0: idle; 1: trot} admitted\n";
        std::cout << "Not sending desired gait to the robot\n";
        return;
    }
    else{
        this->cmd.gaitType = gait_type;        
    }

    this->udp.SetSend(this->cmd);
    this->udp.Send();

    return;
}

void RealRobotInterfaceGo1HighLevel::send_linear_and_angular_velocity_commands(float forwardSpeed, float sideSpeed, float yawSpeed){

    if(forwardSpeed < -1.0 || forwardSpeed > 1.0){
        std::cout << "Requested forwardSpeed is incorrect; must be within range [0,1]\n";
        std::cout << "Not sending desired velocity to the robot\n";
        return;
    }

    // if(sideSpeed <= 0.0 || sideSpeed >= 1.0){
    //     std::cout << "Requested sideSpeed is incorrect; must in range [0.0,1.0]\n";
    //     return;
    // }


    // this->cmd.levelFlag = 0x00;
    this->cmd.levelFlag = HIGHLEVEL; // Do we need to do this every time?
    this->cmd.velocity[0] = forwardSpeed;
    this->cmd.velocity[1] = sideSpeed;
    this->cmd.yawSpeed = yawSpeed;
    this->udp.SetSend(this->cmd);
    this->udp.Send();

    return;
}


void RealRobotInterfaceGo1HighLevel::send_desired_body_height(float bodyHeight){

    if(bodyHeight < -0.1 || bodyHeight > +0.1){
        std::cout << "Requested body height is incorrect. Accepted values are between -0.1m and +0.1m. The requested body height is relative to current height and is restricted for safety.\n";
        std::cout << "Not sending height to the robot\n";
        return;
    }

    // this->cmd.levelFlag = 0x00;
    this->cmd.levelFlag = HIGHLEVEL; // Do we need to do this every time?
    this->cmd.bodyHeight = bodyHeight;
    this->udp.SetSend(this->cmd);
    this->udp.Send();

    return;
}



void RealRobotInterfaceGo1HighLevel::_activate_highlevel_mode_inside_robot(void){

    /*
    Inform the robot that we are going to control it using HIGHLEVEL mode
    */

    // this->cmd.levelFlag = 0xff; 
    this->cmd.levelFlag = HIGHLEVEL;
    this->cmd.mode = 0;
    this->cmd.gaitType = 0;
    this->udp.SetSend(this->cmd);
    this->udp.Send();

    return;

}

void RealRobotInterfaceGo1HighLevel::set_deltaT(double deltaT){
    this->deltaT = deltaT;
    std::cout << "Setting deltaT: " << deltaT << " seconds\n";
    std::cout << " * NOTE: deltaT will only we used for state estimation using IMUs\n";
    return;
}

double RealRobotInterfaceGo1HighLevel::get_deltaT(void){
    return this->deltaT;
}

void RealRobotInterfaceGo1HighLevel::print_joint_info(std::string name, const Eigen::Ref<Vector12d>& joint_vec){
    std::cout << name << ": " << joint_vec.transpose().format(this->clean_format) << "\n";
    return;
}

void RealRobotInterfaceGo1HighLevel::set_IP_and_port_udp(const char* targetIP, uint16_t targetPort){

    std::cout << "New target IP: " << std::string(targetIP) << "\n";
    std::cout << "New target port: " << std::to_string(targetPort) << "\n";

    this->udp.SetIpPort(targetIP,targetPort);

    return;
}

void RealRobotInterfaceGo1HighLevel::read_IPs_and_ports(void){

    std::cout << "\nCurrent IPs and ports:\n";
    std::cout << " * targetIP: " << std::string(this->udp.targetIP) << "\n";
    std::cout << " * targetPort: " << std::to_string(this->udp.targetPort) << "\n";
    std::cout << " * localIP: " << std::string(this->udp.localIP) << "\n";
    std::cout << " * localPort: " << std::to_string(this->udp.localPort) << "\n";

    return;
}



