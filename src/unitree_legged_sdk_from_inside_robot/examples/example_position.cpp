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
    float qInit[3]={0};
    float qDes[3]={0};
    float sin_mid_q[3] = {0.0, 1.2, -2.0};
    float Kp[3] = {0};  
    float Kd[3] = {0};
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    // float dt = 0.01;     // amarco

    // amarco: data to write:
    std::array< std::array<std::array<float, 6000>, 13> , 5 > data_fields;
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

    int Nsteps_go_home = 1000;
    int Nstart_motion_after = Nsteps_go_home + 500;


    // template<std::size_t SIZE>
    // void mulArray(std::array<int, SIZE>& arr, const int multiplier);


    // // template<std::size_t SIZE2> // template<std::size_t SIZE_DATA_FIELD_NAMES>
    // template<std::size_t SIZE_TIME, std::size_t SIZE_JOINT_NAMES, std::size_t SIZE_DATA_FIELD_NAMES, std::size_t SIZE1>
    // // template<std::size_t SIZE1> // template<std::size_t SIZE_JOINT_NAMES>
    // void trial_fun(std::array< std::array<std::array<float, SIZE_TIME>, SIZE_JOINT_NAMES> , SIZE_DATA_FIELD_NAMES> & data_fields,
    //                 std::array<std::string, SIZE1> & data_joint_names);

};

// template<std::size_t SIZE>
// void Custom::mulArray(std::array<int, SIZE>& arr, const int multiplier) {
//     for(auto& e : arr) {
//         e *= multiplier;
//     }

//     for (auto x : arr) { std::cout << x << " "; }
//     std::cout << std::endl;
// }


// // template<std::size_t SIZE2> // template<std::size_t SIZE_DATA_FIELD_NAMES>
// template<std::size_t SIZE_TIME, std::size_t SIZE_JOINT_NAMES, std::size_t SIZE_DATA_FIELD_NAMES, std::size_t SIZE1>
// // template<std::size_t SIZE1> // template<std::size_t SIZE_JOINT_NAMES>
// void Custom::trial_fun(std::array< std::array<std::array<float, SIZE_TIME>, SIZE_JOINT_NAMES> , SIZE_DATA_FIELD_NAMES> & data_fields,
//                         std::array<std::string, SIZE1> & data_joint_names){


//     std::cout << "hola!!!!!!!!!!!!!" << "\n";

// }



// void Custom::Initialize(){

    // // LowState:
    // state.levelFlag = 0;
    // state.commVersion = 0;
    // state.robotID = 0;
    // state.SN = 0; 
    // state.bandWidth = 0;

    // state.imu.quaternion.fill(0.0);
    // state.imu.gyroscope.fill(0.0);
    // state.imu.accelerometer.fill(0.0);
    // state.imu.rpy.fill(0.0);
    // state.imu.temperature = 0;

    // for (int ii = 0; ii < state.motorState.size(); ii++) {

    //     state.motorState[ii].mode = 0;
    //     state.motorState[ii].q = 0.0;
    //     state.motorState[ii].dq = 0.0;
    //     state.motorState[ii].ddq = 0.0;
    //     state.motorState[ii].tauEst = 0.0;
    //     state.motorState[ii].q_raw = 0.0;
    //     state.motorState[ii].dq_raw = 0.0;
    //     state.motorState[ii].ddq_raw = 0.0;
    //     state.motorState[ii].temperature = 0;
    //     state.motorState[ii].reserve.fill(0);
    // }

    // state.footForce.fill(0);
    // state.footForceEst.fill(0);

    // state.bms.version_h = 0;
    // state.bms.version_l = 0;
    // state.bms.bms_status = 0;
    // state.bms.SOC = 0;
    // state.bms.current = 0;
    // state.bms.cycle = 0;

    // state.tick = 0;

    // state.wirelessRemote.fill(0);        // wireless commands
    // state.reserve = 0;
    // state.crc = 0;


    // // LowCmd
    // cmd.levelFlag = 0;
    // cmd.commVersion = 0;
    // cmd.robotID = 0;
    // cmd.SN = 0;
    // cmd.bandWidth = 0;
    // cmd.bms.off = 0;
    // // cmd.bms.reserve;

    // // MotorCmd motorCmd[20];
    // for (int ii = 0; ii < cmd.motorCmd.size(); ii++) {

    //     // cmd.motorCmd[ii].mode = 0;
    //     cmd.motorCmd[ii].q = 0.0;
    //     cmd.motorCmd[ii].dq = 0.0;
    //     cmd.motorCmd[ii].tau = 0.0;
    //     cmd.motorCmd[ii].Kp = 0.0;
    //     cmd.motorCmd[ii].Kd = 0.0;
    //     // cmd.motorCmd[ii].reserve.fill(0);
    // }

// }

void Custom::UDPRecv()
{  
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void Custom::RobotControl() 
{
    motiontime++;
    udp.GetRecv(state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);
    // printf("%d  %f\n", motiontime, state.imu.quaternion[2]);

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;

    double sign_right_side = 1.0;
    if(motiontime == 1)
        time_start = std::chrono::high_resolution_clock::now();

    // if( motiontime >= 100){
    if( motiontime >= 0){
        // std::cout << "here [custom] 1: " << "\n";
        // first, get record initial position
        // if( motiontime >= 100 && motiontime < 500){
        if( motiontime >= 0 && motiontime < 10){
            qInit[0] = state.motorState[FR_0].q;
            qInit[1] = state.motorState[FR_1].q;
            qInit[2] = state.motorState[FR_2].q;

            // std::cout << "here [custom] 2: " << "\n";
        }
        // second, move to the origin point of a sine movement with Kp Kd
        // if( motiontime >= 500 && motiontime < 1500){
        if( motiontime >= 10 && motiontime < Nsteps_go_home){
            rate_count++;
            double rate = rate_count/200.0;                       // needs count to 200
            Kp[0] = 5.0; Kp[1] = 5.0; Kp[2] = 5.0; 
            Kd[0] = 1.0; Kd[1] = 1.0; Kd[2] = 1.0;
            
            qDes[0] = jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
            qDes[1] = jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
            qDes[2] = jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);

            // std::cout << "here [custom] 3: " << "\n";
        }
        double sin_joint0, sin_joint1, sin_joint2;
        // last, do sine wave
        if( motiontime >= Nstart_motion_after){
            sin_count++;
            sin_joint0 = 0.3 * sin(1.8*M_PI*sin_count/1000.0);
            sin_joint1 = 0.4 * sin(1.8*M_PI*sin_count/1000.0);
            sin_joint2 = 0.6 * sin(1.8*M_PI*sin_count/1000.0);
            qDes[0] = sin_mid_q[0] + sin_joint0;
            qDes[1] = sin_mid_q[1] + sin_joint1;
            qDes[2] = sin_mid_q[2] + sin_joint2;
            // qDes[2] = sin_mid_q[2];

            sign_right_side = -1.0;

            // std::cout << "here [custom] 4: " << "\n";
        }

        cmd.motorCmd[FR_0].q = sign_right_side*qDes[0];
        cmd.motorCmd[FR_0].dq = 0;
        cmd.motorCmd[FR_0].Kp = Kp[0];
        cmd.motorCmd[FR_0].Kd = Kd[0];
        cmd.motorCmd[FR_0].tau = -0.65f;

        cmd.motorCmd[FR_1].q = qDes[1];
        cmd.motorCmd[FR_1].dq = 0;
        cmd.motorCmd[FR_1].Kp = Kp[1];
        cmd.motorCmd[FR_1].Kd = Kd[1];
        cmd.motorCmd[FR_1].tau = 0.0f;

        cmd.motorCmd[FR_2].q =  qDes[2];
        cmd.motorCmd[FR_2].dq = 0;
        cmd.motorCmd[FR_2].Kp = Kp[2];
        cmd.motorCmd[FR_2].Kd = Kd[2];
        cmd.motorCmd[FR_2].tau = 0.0f;

        cmd.motorCmd[FL_0].q = qDes[0];
        cmd.motorCmd[FL_0].dq = 0;
        cmd.motorCmd[FL_0].Kp = Kp[0];
        cmd.motorCmd[FL_0].Kd = Kd[0];
        cmd.motorCmd[FL_0].tau = -0.65f;

        cmd.motorCmd[FL_1].q = qDes[1];
        cmd.motorCmd[FL_1].dq = 0;
        cmd.motorCmd[FL_1].Kp = Kp[1];
        cmd.motorCmd[FL_1].Kd = Kd[1];
        cmd.motorCmd[FL_1].tau = 0.0f;

        cmd.motorCmd[FL_2].q =  qDes[2];
        cmd.motorCmd[FL_2].dq = 0;
        cmd.motorCmd[FL_2].Kp = Kp[2];
        cmd.motorCmd[FL_2].Kd = Kd[2];
        cmd.motorCmd[FL_2].tau = 0.0f;

        cmd.motorCmd[RR_0].q = sign_right_side*qDes[0];
        cmd.motorCmd[RR_0].dq = 0;
        cmd.motorCmd[RR_0].Kp = Kp[0];
        cmd.motorCmd[RR_0].Kd = Kd[0];
        cmd.motorCmd[RR_0].tau = -0.65f;

        cmd.motorCmd[RR_1].q = qDes[1];
        cmd.motorCmd[RR_1].dq = 0;
        cmd.motorCmd[RR_1].Kp = Kp[1];
        cmd.motorCmd[RR_1].Kd = Kd[1];
        cmd.motorCmd[RR_1].tau = 0.0f;

        cmd.motorCmd[RR_2].q =  qDes[2];
        cmd.motorCmd[RR_2].dq = 0;
        cmd.motorCmd[RR_2].Kp = Kp[2];
        cmd.motorCmd[RR_2].Kd = Kd[2];
        cmd.motorCmd[RR_2].tau = 0.0f;

        cmd.motorCmd[RL_0].q = qDes[0];
        cmd.motorCmd[RL_0].dq = 0;
        cmd.motorCmd[RL_0].Kp = Kp[0];
        cmd.motorCmd[RL_0].Kd = Kd[0];
        cmd.motorCmd[RL_0].tau = -0.65f;

        cmd.motorCmd[RL_1].q = qDes[1];
        cmd.motorCmd[RL_1].dq = 0;
        cmd.motorCmd[RL_1].Kp = Kp[1];
        cmd.motorCmd[RL_1].Kd = Kd[1];
        cmd.motorCmd[RL_1].tau = 0.0f;

        cmd.motorCmd[RL_2].q =  qDes[2];
        cmd.motorCmd[RL_2].dq = 0;
        cmd.motorCmd[RL_2].Kp = Kp[2];
        cmd.motorCmd[RL_2].Kd = Kd[2];
        cmd.motorCmd[RL_2].tau = 0.0f;



    }

    if(motiontime > 10){
        safe.PositionLimit(cmd);
        int res1 = safe.PowerProtect(cmd, state, 1);
        // You can uncomment it for position protection
        // int res2 = safe.PositionProtect(cmd, state, 0.087);
        if(res1 < 0) exit(-1);

        // std::cout << "here [custom] 5: " << "\n";
    }

    udp.SetSend(cmd);


    // std::cout << "After sending...\n";
    // std::cout << "cmd.motorCmd[FR_0].q: " << cmd.motorCmd[FR_0].q << "\n";
    // std::cout << "cmd.motorCmd[FR_0].dq: " << cmd.motorCmd[FR_0].dq << "\n";
    // std::cout << "cmd.motorCmd[FR_0].Kp: " << cmd.motorCmd[FR_0].Kp << "\n";
    // std::cout << "cmd.motorCmd[FR_0].Kd: " << cmd.motorCmd[FR_0].Kd << "\n";
    // std::cout << "cmd.motorCmd[FR_0].tau: " << cmd.motorCmd[FR_0].tau << "\n";
    // std::cout << "cmd.motorCmd[FR_1].q: " << cmd.motorCmd[FR_1].q << "\n";
    // std::cout << "cmd.motorCmd[FR_1].dq: " << cmd.motorCmd[FR_1].dq << "\n";
    // std::cout << "cmd.motorCmd[FR_1].Kp: " << cmd.motorCmd[FR_1].Kp << "\n";
    // std::cout << "cmd.motorCmd[FR_1].Kd: " << cmd.motorCmd[FR_1].Kd << "\n";
    // std::cout << "cmd.motorCmd[FR_1].tau: " << cmd.motorCmd[FR_1].tau << "\n";
    // std::cout << "cmd.motorCmd[FR_2].q: " << cmd.motorCmd[FR_2].q << "\n";
    // std::cout << "cmd.motorCmd[FR_2].dq: " << cmd.motorCmd[FR_2].dq << "\n";
    // std::cout << "cmd.motorCmd[FR_2].Kp: " << cmd.motorCmd[FR_2].Kp << "\n";
    // std::cout << "cmd.motorCmd[FR_2].Kd: " << cmd.motorCmd[FR_2].Kd << "\n";
    // std::cout << "cmd.motorCmd[FR_2].tau: " << cmd.motorCmd[FR_2].tau << "\n";

    if(ind_data < data_fields[0][0].size()){

        // std::cout << "ind_data: " << std::to_string(ind_data) << "\n";

        std::chrono::duration<float, std::nano> time_dur = std::chrono::high_resolution_clock::now() - time_start;
        time_elapsed = time_dur.count() / 1000000000.0; // NOTE: time_dur.count() returns a double

        for (std::size_t jj = 0 ; jj < data_fields[0].size() ; jj++){

            // std::cout << "jj: " << std::to_string(jj) << "\n";

            // {"time_stamp",q_des","q_curr","dq_curr","u_des","u_est"}
            if (jj == 0){
                data_fields[0][jj][ind_data] = time_elapsed;
                data_fields[1][jj][ind_data] = time_elapsed;
                data_fields[2][jj][ind_data] = time_elapsed;
                data_fields[3][jj][ind_data] = time_elapsed;
                data_fields[4][jj][ind_data] = time_elapsed;
            }
            else{
                data_fields[0][jj][ind_data] = cmd.motorCmd[jj-1].q;
                data_fields[1][jj][ind_data] = state.motorState[jj-1].q;
                data_fields[2][jj][ind_data] = state.motorState[jj-1].dq;
                data_fields[3][jj][ind_data] = cmd.motorCmd[jj-1].tau;
                data_fields[4][jj][ind_data] = state.motorState[jj-1].tauEst;

            }
            // data_q_des[ind_data][jj] = cmd.motorCmd[jj].q;
            // data_q_curr[ind_data][jj] = state.motorState[jj].q;
            
            // data_dq_curr[ind_data][jj] = state.motorState[jj].dq;
            
            // data_u_des[ind_data][jj] = cmd.motorCmd[jj].tau;
            // data_u_est[ind_data][jj] = state.motorState[jj].tauEst;
        }
    }


    ind_data++;

    // Record here the sent data and the received data with a timestamp
    // Fill in a vector and write it into a file
    // std::vector files


    // consider protobuf: https://developers.google.com/protocol-buffers/docs/reference/overview
    // what about ROS? I can leverage rviz and so on...

}


// template<std::size_t SIZE>
// void mulArray(std::array<int, SIZE>& arr, const int multiplier) {
//     for(auto& e : arr) {
//         e *= multiplier;
//     }

//     for (auto x : arr) { std::cout << x << " "; }
//     std::cout << std::endl;
// }


// template <std::size_t SIZE1, std::size_t SIZE2>
// void mulMat(std::array<std::array<int, SIZE1>, SIZE2>& mat, const int multiplier) {

//     for( auto &row : mat)
//         for(auto &col : row)
//             col *= multiplier;

//     for( auto &row : mat){
//         for(auto &col : row){
//             std::cout << col << ", ";
//         }
//         std::cout << "\n";
//     }

//     std::cout << std::endl;
// }


int main(void)
{

    // std::array<int, 2> arr2 = {1, 2};
    // mulArray(arr2,2);

    // std::array<std::array<int, 3>, 2> mat2 = { { { {1, 2, 3} }, { { 4, 5, 6} } } };; // [2,3]
    // mulMat(mat2,2);

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();


    std::string rootpath("../examples");
    // std::string path2folder;
    // path2folder = get_folder_name_with_time_of_day(rootpath);

    std::string filename_base("/data_robot");
    Write2File write2file(rootpath,filename_base);


    Custom custom(LOWLEVEL);

    // custom.mulArray(arr2,2);


    
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


    float time_sleep = 14.0;
    std::cout << "Sleeping for " << std::to_string(int(time_sleep)) << " seconds ...\n";
    sleep(time_sleep);
    // std::cout << "Here finally!!! " <<  "!!!\n";

    // write2file.dump(custom);

    write2file.dump(custom.data_fields,custom.data_joint_names,custom.name_data_fields);

    // custom.trial_fun(custom.data_fields,custom.data_joint_names);


    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();

    sleep(0.1);


    return 0; 
}
