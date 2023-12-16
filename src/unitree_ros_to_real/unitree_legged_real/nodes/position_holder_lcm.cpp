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

// Global variables:
float position2hold[12]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
ros::Subscriber sub_low;
ros::Publisher pub_low;
unitree_legged_msgs::LowState RecvLowROS;
unitree_legged_msgs::LowCmd SendLowROS;

void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    
    // UNITREE_LEGGED_SDK::LowCmd low_cmd_subs = {0};
    // low_cmd_subs = ToLcm(msg,low_cmd_subs);

    // NOTE: no need to convert to LCM types because we're just receiving and copying to position2hold
    // Read desired position, broadcasted to the network
    // Write the desired position in the global position2hold:
    for(int ii=0; ii < 12; ii++){
        // position2hold[ii] = low_cmd_subs.motorCmd[ii].q;
        position2hold[ii] = msg->motorCmd[ii].q;
    }

    std::cout << "Receiving position2hold ... || position2hold[0] = " << position2hold[0] << "\n";

    return;
}


// template<typename TLCM>
void* update_loop(void* param)
{
    UNITREE_LEGGED_SDK::LCM *data = (UNITREE_LEGGED_SDK::LCM *)param;
    while(ros::ok){
        data->Recv();
        usleep(2000);
    }
}

void read_position(unitree_legged_msgs::LowState & RecvLowROS, float * qInit){

    for(int ii=0; ii < 12; ii++){
        qInit[ii] = RecvLowROS.motorState[ii].q;
    }
}

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "position_ros_mode");

    UNITREE_LEGGED_SDK::LCM roslcm(LOWLEVEL);

    std::cout << "WARNING: Control level is set to LOW-level." << std::endl
              << "Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    /*

    We should publish the current position always, so that stand_up can read the initial position

    */

    ros::NodeHandle nh;
    ros::Rate loop_rate(500); // amarco: Frequency in Hz

    long motiontime = 0;
    UNITREE_LEGGED_SDK::LowCmd SendLowLCM = {0};
    UNITREE_LEGGED_SDK::LowState RecvLowLCM = {0};
    // unitree_legged_msgs::LowState RecvLowROS;

    // amarco:
    int Ndur_read_init_pos = 1000;
    double Kp = 40.0;
    double Kd = 2.0;

    // Initialize global variables:
    sub_low = nh.subscribe("low_cmd_to_robot", 100, lowCmdCallback); // Receive commands from the network
    pub_low = nh.advertise<unitree_legged_msgs::LowState>("low_state_from_robot", 100);

    roslcm.SubscribeState();

    pthread_t tid;
    pthread_create(&tid, NULL, update_loop, &roslcm);

    std::cout << "Receiving LCM low-level state data from the robot and publishing it ...\n";
    std::cout << "Subscribing to low-level commands from the network and sending them to the robot via LCM ...\n";

    SendLowROS.levelFlag = LOWLEVEL;
    for(int i = 0; i<12; i++){
        SendLowROS.motorCmd[i].mode = 0x0A;   // motor switch to servo (PMSM) mode
        SendLowROS.motorCmd[i].q = PosStopF;        // 禁止位置环
        SendLowROS.motorCmd[i].Kp = 0;
        SendLowROS.motorCmd[i].dq = VelStopF;        // 禁止速度环
        SendLowROS.motorCmd[i].Kd = 0;
        SendLowROS.motorCmd[i].tau = 0;
    }

    // Read initial position, do not send anything:
    std::cout << "Reading initial position for " << Ndur_read_init_pos << " seconds ...\n";
    while (motiontime < Ndur_read_init_pos){

        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        read_position(RecvLowROS,position2hold); // Just read, do not send anything

        motiontime++;
        loop_rate.sleep();

        // No need to spinOnce through the callbacks because we're not interested yet in listening commands from the network
        // We need to read the robot's current joint position and store it in position2hold
    }

    std::cout << "Initial position:\n";
    for(int ii=0; ii < 12; ii++){
        std::cout << "(initial) position2hold[" << ii << "] = " << position2hold[ii];
    }

    // Send to the robot whatever is inside position2hold using the pre-defined PD gains
    // position2hold is only modified by the lowCmdCallback, which is listening to incoming messages from the network
    std::cout << "reading position2hold from the subscriber and sending it to the robot indefinitely ...\n";
    while (ros::ok()){

        roslcm.Get(RecvLowLCM);
        RecvLowROS = ToRos(RecvLowLCM);

        for(int ii=0; ii < 12; ii++){
            SendLowROS.motorCmd[ii].q = position2hold[ii];
            SendLowROS.motorCmd[ii].dq = 0;
            SendLowROS.motorCmd[ii].Kp = Kp;
            SendLowROS.motorCmd[ii].Kd = Kd;
            SendLowROS.motorCmd[ii].tau = 0.0f;
        }

        SendLowROS.motorCmd[FR_0].tau = -0.65f;
        SendLowROS.motorCmd[FL_0].tau = +0.65f;
        SendLowROS.motorCmd[RR_0].tau = -0.65f;
        SendLowROS.motorCmd[RL_0].tau = +0.65f;

        SendLowLCM = ToLcm(SendLowROS, SendLowLCM);
        roslcm.Send(SendLowLCM);

        // Publish the current state:
        pub_low.publish(RecvLowROS);

        std::cout << "position2hold[0] = " << position2hold[0] << "\n";

        ros::spinOnce(); // Go through the callbacks and fill position2hold with the commands collected from the network in lowCmdCallback()
        loop_rate.sleep();
    }


    return 0;
}

