/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "utils/data_logging.hpp"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

// // amarco:
// #include <fstream>
// #include <chrono>
// #include <ctime>

using namespace std;
using namespace UNITREE_LEGGED_SDK;


class Custom
{
public:
    Custom(uint8_t level): safe(LeggedType::Go1), udp(level) {
        udp.InitCmdData(cmd);
        // Initialize();
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    // void Initialize();

    Safety safe;
    UDP udp;
    LowCmd cmd = {0};
    LowState state = {0};
    // float qInit[3]={0};
    float qInit[12]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    float qDes[12]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    float sin_mid_q[12] = {0.0, 1.2, -2.0, 0.0, 1.2, -2.0, 0.0, 1.2, -2.0, 0.0, 1.2, -2.0};
    // float Kp[12] = {5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0};
    // float Kd[12] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
    double Kp = 5.0;
    double Kd = 1.0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    // float dt = 0.01;     // amarco

    // amarco: data to write:
    std::array< std::array<std::array<float, 4000>, 13> , 5 > data_fields;
    std::array<std::string, 13> data_joint_names = {"time_stamp","FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"};
    std::array<std::string, 5> name_data_fields = {"q_des","q_curr","dq_curr","u_des","u_est"};
    // data_q_des: Desired position for all the joints, [1000,12]
    // data_q_curr: Actual position for all the joints, [1000,12]
    // data_dq_curr: Actual velocity for all the joints, [1000,12]
    // data_u_des: Desired torque for all the joints, [1000,12]
    // data_u_est: Estimated output torque for all the joints, [1000,12]

    float time_elapsed;
    std::chrono::high_resolution_clock::time_point time_start;

    int ind_data = 0;

    
    int Nsteps_read_initial_pos = 1000;
    int Nsteps_go_home = Nsteps_read_initial_pos + 2000;


    // template<std::size_t SIZE>
    // void mulArray(std::array<int, SIZE>& arr, const int multiplier);


    // // template<std::size_t SIZE2> // template<std::size_t SIZE_DATA_FIELD_NAMES>
    // template<std::size_t SIZE_TIME, std::size_t SIZE_JOINT_NAMES, std::size_t SIZE_DATA_FIELD_NAMES, std::size_t SIZE1>
    // // template<std::size_t SIZE1> // template<std::size_t SIZE_JOINT_NAMES>
    // void trial_fun(std::array< std::array<std::array<float, SIZE_TIME>, SIZE_JOINT_NAMES> , SIZE_DATA_FIELD_NAMES> & data_fields,
    //                 std::array<std::string, SIZE1> & data_joint_names);

};



void Custom::UDPRecv()
{  
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);

    for(int ii=0; ii < 12; ii++){
        std::cout << "state.MotorState[" << ii << "].q = " << state.MotorState[ii].q << "\n";
        std::cout << "state.MotorState[" << ii << "].dq = " << state.MotorState[ii].dq << "\n";
        std::cout << "state.MotorState[" << ii << "].ddq = " << state.MotorState[ii].ddq << "\n";
        std::cout << "state.MotorState[" << ii << "].q_raw = " << state.MotorState[ii].q_raw << "\n";
        std::cout << "state.MotorState[" << ii << "].dq_raw = " << state.MotorState[ii].dq_raw << "\n";
        std::cout << "state.MotorState[" << ii << "].ddq_raw = " << state.MotorState[ii].ddq_raw << "\n";
        std::cout << "state.MotorState[" << ii << "].tauEst = " << state.MotorState[ii].tauEst << "\n";
    }



}




int main(void)
{



    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();





    Custom custom(LOWLEVEL);



    // InitEnvironment();
    LoopFunc loop_control("control_loop", custom.dt,    boost::bind(&Custom::RobotControl, &custom));
    LoopFunc loop_udpSend("udp_send",     custom.dt, 3, boost::bind(&Custom::UDPSend,      &custom));
    LoopFunc loop_udpRecv("udp_recv",     custom.dt, 3, boost::bind(&Custom::UDPRecv,      &custom));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    // while(1){
    //     sleep(10);
    // };


    float time_sleep = 100.0;
    std::cout << "Sleeping for " << std::to_string(int(time_sleep)) << " seconds ...\n";
    sleep(time_sleep);



    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();

    sleep(0.1);


    return 0; 
}
