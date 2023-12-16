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

#ifndef _REAL_ROBOT_INTERFACE_GO1_HIGHLEVEL_H_
#define _REAL_ROBOT_INTERFACE_GO1_HIGHLEVEL_H_


#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default
// https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
// (recommended by Eigen only for sizes smaller than roughly 32; larger sue dynamic allocation)

class RealRobotInterfaceGo1HighLevel
{
public:
    // RealRobotInterfaceGo1HighLevel(uint16_t localPort, std::string targetIP, uint16_t targetPort) : safe(LeggedType::Go1) {
    // RealRobotInterfaceGo1HighLevel() : safe(LeggedType::Go1), udp(HIGHLEVEL){
    // RealRobotInterfaceGo1HighLevel() :  safe(LeggedType::Go1), udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState)){
    RealRobotInterfaceGo1HighLevel(uint16_t localPort, const char* targetIP, uint16_t targetPort) : safe(LeggedType::Go1), udp(localPort, targetIP, targetPort, sizeof(HighCmd), sizeof(HighState)){

        // this->udp = UDP(localPort,targetIP.c_str(),targetPort,sizeof(HighCmd),sizeof(HighState));


        this->cmd.velocity[0] = 0.0;
        this->cmd.velocity[1] = 0.0;
        this->cmd.yawSpeed = 0.0;
        this->cmd.postion[0] = 0.0;
        this->cmd.postion[1] = 0.0;

        this->udp.InitCmdData(this->cmd);

        std::cout << "this->cmd.velocity[0] = " << this->cmd.velocity[0] << "\n";
        std::cout << "this->cmd.velocity[1] = " << this->cmd.velocity[1] << "\n";
        std::cout << "this->cmd.yawSpeed = " << this->cmd.yawSpeed << "\n";
        std::cout << "this->cmd.postion[0] = " << this->cmd.postion[0] << "\n";
        std::cout << "this->cmd.postion[1] = " << this->cmd.postion[1] << "\n";


        this->clean_format = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");

        // amarco: Can this be avoided?
        this->joint_pos_curr.setZero();
        this->joint_vel_curr.setZero();
        this->body_linear_velocity.setZero();
        this->body_angular_velocity.setZero();
        this->body_orientation.setZero();
        
        std::cout << "\nPrinting relevant safety parameters:\n";
        std::cout << " * safe.WattLimit: " << safe.WattLimit << "\n";
        std::cout << " * safe.Wcount: " << safe.Wcount << "\n";
        std::cout << " * safe.Hip_max: " << safe.Hip_max << "\n";
        std::cout << " * safe.Hip_min: " << safe.Hip_min << "\n";
        std::cout << " * safe.Thigh_max: " << safe.Thigh_max << "\n";
        std::cout << " * safe.Thigh_min: " << safe.Thigh_min << "\n";
        std::cout << " * safe.Calf_max: " << safe.Calf_max << "\n";
        std::cout << " * safe.Calf_min;: " << safe.Calf_min << "\n";

        std::cout << "\nOther parameters:\n";
        std::cout << " * deltaT: " << deltaT << "\n";
        std::cout << " * Njoints: " << Njoints << "\n";




        this->_activate_highlevel_mode_inside_robot();

        this->read_IPs_and_ports();

    }

    ~RealRobotInterfaceGo1HighLevel(){
        std::cout << "Destroying RealRobotInterfaceGo1HighLevel class ...\n";
    }

    size_t Njoints = 12;


    void get_observations(void);
    void get_joint_pos_curr(void);
    void get_joint_vel_curr(void);
    void get_body_linear_velocity(void);
    void get_body_angular_velocity(void);
    void get_body_orientation(void);

    void update_mode_behavior(uint8_t mode_behavior);
    void update_gait_type(uint8_t gait_type);

    void send_linear_and_angular_velocity_commands(float forwardSpeed, float sideSpeed, float yawSpeed);
    void send_desired_body_height(float bodyHeight);

    void set_deltaT(double deltaT);
    double get_deltaT(void);
    void print_joint_info(std::string name, const Eigen::Ref<Vector12d>& joint_vec);

    // Communication:
    void set_IP_and_port_udp(const char* targetIP, uint16_t targetPort);
    void read_IPs_and_ports(void);

    // Communication:
    UDP udp;
    Safety safe;
    HighState state;
    HighCmd cmd;

    // amarco:
    Vector12d joint_pos_curr;
    Vector12d joint_vel_curr;

    // IMU estimation:
    Eigen::Vector3d body_linear_velocity;
    Eigen::Vector3d body_angular_velocity;
    Eigen::Vector3d body_orientation;
    double deltaT = 0.002;

    // Flags:

protected:
    void _activate_highlevel_mode_inside_robot(void);
    void _collect_observations(void);
    Eigen::IOFormat clean_format;
};


#endif  // _REAL_ROBOT_INTERFACE_GO1_HIGHLEVEL_H_
