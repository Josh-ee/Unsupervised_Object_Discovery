/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _REAL_ROBOT_INTERFACE_GO1_H_
#define _REAL_ROBOT_INTERFACE_GO1_H_


#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <array>
#include <math.h>
#include <Eigen/Core>


// // #include <python_interface_go1/pybind11/operators.h>
// // #include <python_interface_go1/pybind11/numpy.h>
// #include <python_interface_go1/pybind11/include/pybind11/pybind11.h>
// #include <python_interface_go1/pybind11/include/pybind11/eigen.h>
// #include <python_interface_go1/pybind11/include/pybind11/stl.h>


#include <iostream>

using namespace UNITREE_LEGGED_SDK;

typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default
// https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html
// (recommended by Eigen only for sizes smaller than roughly 32; larger sue dynamic allocation)

class RealRobotInterfaceGo1
{
public:
    RealRobotInterfaceGo1() : safe(LeggedType::Go1), udp(LOWLEVEL){
        
        // InitEnvironment();
        // InitializeAllFieldsToZero(); // amarco

        // amarco: TODO: double check that this is ok
        udp.InitCmdData(this->cmd);

        // // [DBG]: Print field before editing:
        // std::cout << "cmd.levelFlag: " << std::hex << int(cmd.levelFlag) << "\n";
        // this->cmd.levelFlag = LOWLEVEL;
        // std::cout << "cmd.levelFlag: " << std::hex << int(cmd.levelFlag) << "\n";
        // for(int i = 0; i<this->Njoints; i++){

        //     // [DBG]: Print field before editing:
        //     std::cout << "cmd.motorCmd[i" << i << "].mode = " << std::hex << int(this->cmd.motorCmd[i].mode) << "\n";
        //     std::cout << "cmd.motorCmd[i" << i << "].q = " << this->cmd.motorCmd[i].q << "\n";
        //     std::cout << "cmd.motorCmd[i" << i << "].dq = " << this->cmd.motorCmd[i].dq << "\n";


        //     this->cmd.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        //     this->cmd.motorCmd[i].q = PosStopF;        // 禁止位置环 (Prohibit position loop)
        //     this->cmd.motorCmd[i].Kp = 0;
        //     this->cmd.motorCmd[i].dq = VelStopF;        // 禁止速度环 (Prohibit speed loop)
        //     this->cmd.motorCmd[i].Kd = 0;
        //     this->cmd.motorCmd[i].tau = 0;

        //     std::cout << "cmd.motorCmd[i" << i << "].mode = " << std::hex << int(this->cmd.motorCmd[i].mode) << "\n";
        // }
        // The cmd.levelFlag is set to 0xff and cmd.motorCmd[i].mode to 0x0A



        // TODO: Maybe read this from a Yaml file here directly
        // But because this class will be always handled from Python, we can do the yaml reading on the python side, easier
        // set_PD_gains(P_gains,D_gains);

        this->clean_format = Eigen::IOFormat(4, 0, ", ", "\n", "[", "]");

        // amarco: Can this be avoided?
        this->joint_pos_curr.setZero();
        this->joint_vel_curr.setZero();
        this->P_gains_.setZero();
        this->D_gains_.setZero();
        this->body_linear_velocity.setZero();
        this->body_angular_velocity.setZero();
        this->body_orientation.setZero();
        

        // joint_pos_init_read.setZero();
        // joint_pos_init_target.setZero();


        // joint_pos_init_target << 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;

        std::cout << "Print relevant safety parameters:\n";
        std::cout << " * safe.WattLimit: " << safe.WattLimit << "\n";
        std::cout << " * safe.Wcount: " << safe.Wcount << "\n";
        std::cout << " * safe.Hip_max: " << safe.Hip_max << "\n";
        std::cout << " * safe.Hip_min: " << safe.Hip_min << "\n";
        std::cout << " * safe.Thigh_max: " << safe.Thigh_max << "\n";
        std::cout << " * safe.Thigh_min: " << safe.Thigh_min << "\n";
        std::cout << " * safe.Calf_max: " << safe.Calf_max << "\n";
        std::cout << " * safe.Calf_min;: " << safe.Calf_min << "\n";

        std::cout << "Other parameters:\n";
        std::cout << "deltaT: " << deltaT << "\n";
        std::cout << "Njoints: " << Njoints << "\n";

    }

    ~RealRobotInterfaceGo1(){
        std::cout << "Destroying RealRobotInterfaceGo1 class ...\n";
    }

    size_t Njoints = 12;

    void CollectObservations();
    void update_all_observations();
    void SendCommand(   const Eigen::Ref<Vector12d>& joint_pos_des,
                        const Eigen::Ref<Vector12d>& joint_vel_des,
                        const Eigen::Ref<Vector12d>& joint_torque_des);

    void send_desired_position(const Eigen::Ref<Vector12d>& joint_pos_des);
    void change_operation_mode_inside_robot(void);
    
    // void set_PD_gains(const std::array<float, 12> & P_gains, const std::array<float, 12> & D_gains);
    void set_PD_gains(const Eigen::Ref<Vector12d> & P_gains, const Eigen::Ref<Vector12d> & D_gains);

    void update_joint_pos_curr();
    void update_joint_vel_curr();
    void update_body_linear_velocity();
    void update_body_angular_velocity();
    void update_body_orientation();

    void get_joint_pos_curr(Eigen::Ref<Vector12d> joint_pos_curr);

    void set_deltaT(double deltaT);
    double get_deltaT(void);
    void print_joint_info(std::string name, const Eigen::Ref<Vector12d>& joint_vec);

    // void main();
    // void stand_up(int Nsteps);


    // Communication:
    UDP udp;
    Safety safe;
    LowState state;
    LowCmd cmd;

    // amarco:
    Vector12d joint_pos_curr;
    Vector12d joint_vel_curr;

    Vector12d P_gains_;
    Vector12d D_gains_;

    // IMU estimation:
    Eigen::Vector3d body_linear_velocity;
    Eigen::Vector3d body_angular_velocity;
    Eigen::Vector3d body_orientation;
    double deltaT = 0.002;

    // Vector12d joint_pos_init_read;
    // Vector12d joint_pos_init_target;

    // Flags:
    bool is_running_control_loop = false;
    bool use_position_protect = false;
    bool use_gravity_compensation = true;

protected:
    // void InitializeAllFieldsToZero();
    void ensure_safety();
    Eigen::IOFormat clean_format;

};


/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Gym Environment ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */



class GymEnvironmentRealGo1 : public RealRobotInterfaceGo1 {

public:

    GymEnvironmentRealGo1(): RealRobotInterfaceGo1(){

        double action_std = 0.01;
        actionStd_.setConstant(action_std);
        actionMean_.setZero();
        observation_env_.setZero(Nobs);

        // joint_pos_init_target << 0.0, -1.5245, 0.3435, 1.0, 0.0, 0.0, 1.0, 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;
        // joint_pos_init_target << 0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944;

    }

    ~GymEnvironmentRealGo1(){
        std::cout << "Destroying GymEnvironmentRealGo1 class ...\n";
    }

    void init();
    void update_env_observation();
    void step(const Eigen::Ref<Vector12d> & action);
    void set_action_std(double action_std);
    void set_action_mean(const Eigen::Ref<Vector12d>& action_mean);

    // typedef Eigen::Matrix<double, 12, 1> Vector12d; // Column vector by default
    Eigen::VectorXd get_env_observation();

    // void isTerminalState(); // Not sure we need this...
    // void curriculumUpdate(); // Not sure we need this...
    // void reset(); // We can't really 'reset' the real robot to the initial position
    // void observe(); // Not sure we need this...

private:
    
    void action2joint_pos_des(const Eigen::Ref<Vector12d>& action, Eigen::Ref<Vector12d> joint_pos_des);

    Vector12d actionMean_, actionStd_;

    size_t Nobs = 34;
    Eigen::VectorXd observation_env_;

    // Vector12d joint_pos_init_read;
    // Vector12d joint_pos_init_target;

};


#endif  // _REAL_ROBOT_INTERFACE_GO1_H_
