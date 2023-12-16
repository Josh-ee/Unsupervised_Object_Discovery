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
    // Custom(): udp(8117, "192.168.12.1", 8118, sizeof(LowCmd), sizeof(LowState)), safe(LeggedType::Go1){
    //     udp.InitCmdData(cmd);
    // } // ubuntu laptop -> robot computer

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

    float qStandUp[12]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // float q_target[12] = {0.0, 1.2, -2.0, 0.0, 1.2, -2.0, 0.0, 1.2, -2.0, 0.0, 1.2, -2.0};
    float q_target[12] = {0.015, 0.75, -1.45, 0.015, 0.75, -1.45, 0.015, 0.75, -1.45, -0.015, 0.75, -1.45};
    // float q_target[12] = {0.0, 0.75, -1.45, 0.0, 0.75, -1.45, 0.0, 0.75, -1.45, -0.0, 0.75, -1.45};
    // TODO: There's a disimliraity in the convention of the URDF VS the real robot; look at the signs...
    // float Kp[12] = {5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0,5.0};
    // float Kd[12] = {1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0};
    double Kp = 40.0;
    double Kd = 2.0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002;     // 0.001~0.01
    // float dt = 0.01;     // amarco

    
    int Ndur_read_init_pos = 1000;
    int Ndur_stand_up = 1000;
    int Ndur_wait_up = 1500;
    int Ndur_go2floor = 1000;
    int Ndur_wait_on_floor = 1000;


    // amarco: data to write:
    std::array< std::array<std::array<float, 6500>, 13> , 10 > data_fields;
    std::array<std::string, 13> data_joint_names = {"time_stamp","FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"};
    // std::array<std::string, 9> name_data_fields = {"q_des","q_curr","dq_curr","u_des","u_est","ddq_curr","q_raw_curr","dq_raw_curr","ddq_raw_curr"};
    std::array<std::string, 10> name_data_fields = {"q_curr","dq_curr","ddq_curr","q_raw_curr","dq_raw_curr","ddq_raw_curr","u_est","q_des","dq_des","u_des"};

    // data_q_des: Desired position for all the joints, [1000,12]
    // data_q_curr: Actual position for all the joints, [1000,12]
    // data_dq_curr: Actual velocity for all the joints, [1000,12]
    // data_u_des: Desired torque for all the joints, [1000,12]
    // data_u_est: Estimated output torque for all the joints, [1000,12]

    float time_elapsed;
    std::chrono::high_resolution_clock::time_point time_start;

    int ind_data = 0;




    // int Nsteps_read_initial_pos = 1000;

    // int Nsteps_go_home = Nsteps_read_initial_pos + Nduration_stand_up;

};

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
    // udp.GetRecv((char*)&state);
    // printf("%d  %f\n", motiontime, state.motorState[FR_2].q);
    // printf("%d  %f\n", motiontime, state.imu.quaternion[2]);

    if(motiontime == 1)
        time_start = std::chrono::high_resolution_clock::now();

    // Read initial position:
    if( motiontime >= 0 && motiontime < Ndur_read_init_pos){
        for(int ii=0; ii < 12; ii++){
            qInit[ii] = state.motorState[ii].q;
        }

        for(int ii=0; ii < 12; ii++){
            cmd.motorCmd[ii].q = 0.0;
            cmd.motorCmd[ii].dq = 0;
            cmd.motorCmd[ii].Kp = 0.0; // Set to zero, otherwise it will try to go to the desired position, which is zero, i.e., cmd.motorCmd[ii].q = 0.0;
            cmd.motorCmd[ii].Kd = 0.0; // Set to zero, otherwise it will try to go to the desired position, which is zero, i.e., cmd.motorCmd[ii].q = 0.0;
            cmd.motorCmd[ii].tau = 0.0f;
        }

    }

    if( motiontime == Ndur_read_init_pos){

        for(int ii=0; ii < 12; ii++){
            std::cout << "qInit[" << ii << "] = " << qInit[ii] << "\n";
        }

    }


    // Stand up:
    if( motiontime >= Ndur_read_init_pos && motiontime < (Ndur_read_init_pos + Ndur_stand_up)){
        double rate = (double)rate_count/(double)Ndur_stand_up;
        rate_count++;
        
        for(int ii=0; ii < 12; ii++){
            qDes[ii] = jointLinearInterpolation(qInit[ii], q_target[ii], rate);
        }

        for(int ii=0; ii < 12; ii++){
            cmd.motorCmd[ii].q = qDes[ii];
            cmd.motorCmd[ii].dq = 0;
            cmd.motorCmd[ii].Kp = Kp;
            cmd.motorCmd[ii].Kd = Kd;
            cmd.motorCmd[ii].tau = 0.0f;
        }


    }


    // Wait while standing up and read current position:
    if( motiontime >= (Ndur_read_init_pos + Ndur_stand_up) && motiontime < (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up)){

        for(int ii=0; ii < 12; ii++){
            qStandUp[ii] = state.motorState[ii].q;
        }

        for(int ii=0; ii < 12; ii++){
            cmd.motorCmd[ii].q = qDes[ii];
            cmd.motorCmd[ii].dq = 0;
            cmd.motorCmd[ii].Kp = Kp;
            cmd.motorCmd[ii].Kd = Kd;
            cmd.motorCmd[ii].tau = 0.0f;
        }

        rate_count = 0;


    }

    if( motiontime == (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up)){

        for(int ii=0; ii < 12; ii++){
            std::cout << "qStandUp[" << ii << "] = " << qStandUp[ii] << "\n";
        }

    }

    // Go down:
    if( motiontime >= (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up) && motiontime < (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up + Ndur_go2floor)){

        double rate = (double)rate_count/(double)Ndur_go2floor;
        rate_count++;
        
        for(int ii=0; ii < 12; ii++){
            qDes[ii] = jointLinearInterpolation(qStandUp[ii], qInit[ii], rate);
        }

        for(int ii=0; ii < 12; ii++){
            cmd.motorCmd[ii].q = qDes[ii];
            cmd.motorCmd[ii].dq = 0;
            cmd.motorCmd[ii].Kp = Kp;
            cmd.motorCmd[ii].Kd = Kd;
            cmd.motorCmd[ii].tau = 0.0f;
        }

    }

    // Wait while down:
    if( motiontime >= (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up + Ndur_go2floor) && motiontime < (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up + Ndur_go2floor + Ndur_wait_on_floor)){

        for(int ii=0; ii < 12; ii++){
            cmd.motorCmd[ii].q = qDes[ii];
            cmd.motorCmd[ii].dq = 0;
            cmd.motorCmd[ii].Kp = Kp;
            cmd.motorCmd[ii].Kd = Kd;
            cmd.motorCmd[ii].tau = 0.0f;
        }

        rate_count = 0;


    }

    if(motiontime >= (Ndur_read_init_pos + Ndur_stand_up + Ndur_wait_up + Ndur_go2floor)){

        for(int ii=0; ii < 12; ii++){
            cmd.motorCmd[ii].q = qDes[ii];
            cmd.motorCmd[ii].dq = 0;
            cmd.motorCmd[ii].Kp = 0.0;
            cmd.motorCmd[ii].Kd = 0.0;
            cmd.motorCmd[ii].tau = 0.0f;
        }
    }

    // gravity compensation
    cmd.motorCmd[FR_0].tau = -0.65f;
    cmd.motorCmd[FL_0].tau = +0.65f;
    cmd.motorCmd[RR_0].tau = -0.65f;
    cmd.motorCmd[RL_0].tau = +0.65f;


    safe.PositionLimit(cmd);
    int res1 = safe.PowerProtect(cmd, state, 1);
    if(res1 < 0) exit(-1);

    udp.SetSend(cmd);
    // udp.SetSend((char*)&cmd);


    if(ind_data < data_fields[0][0].size()){

        std::chrono::duration<float, std::nano> time_dur = std::chrono::high_resolution_clock::now() - time_start;
        time_elapsed = time_dur.count() / 1000000000.0; // NOTE: time_dur.count() returns a double

        for (std::size_t jj = 0 ; jj < data_fields[0].size() ; jj++){

            if (jj == 0){
                data_fields[0][jj][ind_data] = time_elapsed;
                data_fields[1][jj][ind_data] = time_elapsed;
                data_fields[2][jj][ind_data] = time_elapsed;
                data_fields[3][jj][ind_data] = time_elapsed;
                data_fields[4][jj][ind_data] = time_elapsed;
                data_fields[5][jj][ind_data] = time_elapsed;
                data_fields[6][jj][ind_data] = time_elapsed;
                data_fields[7][jj][ind_data] = time_elapsed;
                data_fields[8][jj][ind_data] = time_elapsed;
                data_fields[9][jj][ind_data] = time_elapsed;

            }
            else{

                data_fields[0][jj][ind_data] = state.motorState[jj-1].q;
                data_fields[1][jj][ind_data] = state.motorState[jj-1].dq;
                data_fields[2][jj][ind_data] = state.motorState[jj-1].ddq;
                data_fields[3][jj][ind_data] = state.motorState[jj-1].q_raw;
                data_fields[4][jj][ind_data] = state.motorState[jj-1].dq_raw;
                data_fields[5][jj][ind_data] = state.motorState[jj-1].ddq_raw;
                data_fields[6][jj][ind_data] = state.motorState[jj-1].tauEst;
                data_fields[7][jj][ind_data] = cmd.motorCmd[jj-1].q;
                data_fields[8][jj][ind_data] = cmd.motorCmd[jj-1].dq;
                data_fields[9][jj][ind_data] = cmd.motorCmd[jj-1].tau;

            }

        }


        ind_data++;
    
    }

    /* NOTES:
        1) Consider protobuf: https://developers.google.com/protocol-buffers/docs/reference/overview
        2) Consider ROS tools
    */

}



int main(void)
{

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
    // Custom custom;
    
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


    float time_sleep = 13.0;
    std::cout << "Sleeping for " << std::to_string(int(time_sleep)) << " seconds ...\n";
    sleep(time_sleep);
    std::cout << "Here finally!!! " <<  "!!!\n";

    write2file.dump(custom.data_fields,custom.data_joint_names,custom.name_data_fields);

    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();

    sleep(0.1);


    return 0; 
}
