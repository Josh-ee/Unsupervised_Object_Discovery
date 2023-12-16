/************************************************************************
*
* @author Alonso Marco
* Contact: amarco@berkeley.edu
************************************************************************/

#include <ros/ros.h>
#include <string>
#include <pthread.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

float qInit[12] = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

double jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos*(1-rate) + targetPos*rate;
    return p;
}

void lowCmdCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
    
    for(int ii=0; ii < 12; ii++)
        qInit[ii] = msg->motorState[ii].q;

    std::cout << "Receiving qInit ... || qInit[0] = " << qInit[0] << "\n";

    return;
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "stand_up_ros_mode");

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    ros::NodeHandle nh;
    ros::Rate loop_rate(500);

    long motiontime = 0;
    int rate_count = 0;
    float q_target[12] = {0.015, 0.75, -1.45, 0.015, 0.75, -1.45, 0.015, 0.75, -1.45, -0.015, 0.75, -1.45};
    // float qInit[12] = {-0.426913,1.07988,-2.6027,0.498489,1.09138,-2.63725,-0.512356,1.05699,-2.65154,0.557954,1.07958,-2.65481};
    float qDes[12]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
    
    // float Kp[3] = {0};  
    // float Kd[3] = {0};
    // UNITREE_LEGGED_SDK::LowCmd SendLowLCM = {0};
    // UNITREE_LEGGED_SDK::LowState RecvLowLCM = {0};
    unitree_legged_msgs::LowCmd SendLowCmdROS;
    unitree_legged_msgs::LowState RecvLowStateROS;

    // amarco:
    int Ndur_read_init_pos = 1000; // Hardcoded for now
    int Ndur_stand_up = 2000;

    // Subscribe to the robot state: we need this to read the initial position before we start moving:
    ros::Subscriber sub2low_state = nh.subscribe("low_state_from_robot", 100, lowCmdCallback);

    // Publish the desired positions:
    ros::Publisher pub2low_cmd = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd_to_robot", 100);


    SendLowCmdROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowCmdROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowCmdROS.motorCmd[i].q = PosStopF;        // 禁止位置环
        SendLowCmdROS.motorCmd[i].Kp = 0;
        SendLowCmdROS.motorCmd[i].dq = VelStopF;        // 禁止速度环
        SendLowCmdROS.motorCmd[i].Kd = 0;
        SendLowCmdROS.motorCmd[i].tau = 0;
    }

    // Read initial position, do not publish anything yet:
    std::cout << "Reading initial position for " << Ndur_read_init_pos << " seconds ...\n";
    while (motiontime < Ndur_read_init_pos){

        motiontime++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Initial position:\n";
    for(int ii=0; ii < 12; ii++)
        std::cout << "qInit[" << ii << "] = " << qInit[ii] << "\n";

    std::cout << "Starting movement to target ...\n";
    while (ros::ok()){

        // Stand up:
        if( motiontime >= Ndur_read_init_pos && motiontime < (Ndur_read_init_pos + Ndur_stand_up)){

            double rate = (double)rate_count/(double)Ndur_stand_up;
            rate_count++;
            
            for(int ii=0; ii < 12; ii++)
                qDes[ii] = jointLinearInterpolation(qInit[ii], q_target[ii], rate);

            motiontime++;

        }

        // Always publish whatever is inside qDes:
        for(int ii=0; ii < 12; ii++){

            SendLowCmdROS.motorCmd[ii].q = qDes[ii];
            SendLowCmdROS.motorCmd[ii].dq = 0;
            SendLowCmdROS.motorCmd[ii].Kp = 0.0;
            SendLowCmdROS.motorCmd[ii].Kd = 0.0;
            SendLowCmdROS.motorCmd[ii].tau = 0.0f;

        }

        SendLowCmdROS.motorCmd[FR_0].tau = -0.65f;
        SendLowCmdROS.motorCmd[FL_0].tau = +0.65f;
        SendLowCmdROS.motorCmd[RR_0].tau = -0.65f;
        SendLowCmdROS.motorCmd[RL_0].tau = +0.65f;

        pub2low_cmd.publish(SendLowCmdROS);

        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}


