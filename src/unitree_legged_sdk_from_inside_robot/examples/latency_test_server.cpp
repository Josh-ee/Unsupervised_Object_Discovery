/* amarco

    Ping-pong test implementation, to measure the round-time between the computer and the robot

    Borrowed ideas from: https://github.com/farnyser/cpp-ping-pong/blob/master/src/udp.cpp

    Tools like https://github.com/Mellanox/sockperf do not meet our goals, 
    because we want an idea of the latency using Unitree's libraries

 */

// Notice: This exemple should running on another PC, and make sure the Ethernet is stable.

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"

// amarco:
#include <chrono>
#include <iostream>

using namespace UNITREE_LEGGED_SDK;

class Custom
{

public:

    // uint16_t localPort, const char* targetIP, uint16_t targetPort, int sendLength, int recvLength
    // Custom(): udp(8117, "192.168.64.3", 8118, sizeof(StampedSequence), sizeof(StampedSequence)), time_elapsed_vec(10000){}
    // Custom(): udp(8117, "192.168.64.3", 8118, sizeof(HighCmd), sizeof(HighState)), time_elapsed_vec(10000){
    //     // udp.InitCmdData(sseq_server);
    //     return;
    // }
    // Custom(): udp(8117, "192.168.64.3", 8118, sizeof(HighCmd), sizeof(HighCmd)), time_elapsed_vec(100000){} // mac laptop -> mac laptop
    // Custom(): udp(8117, "127.0.0.1", 8118, sizeof(HighCmd), sizeof(HighCmd)), time_elapsed_vec(100000){} // ubuntu laptop -> ubuntu laptop
    Custom(): udp(8117, "192.168.12.1", 8118, sizeof(HighCmd), sizeof(HighCmd)), time_elapsed_vec(100000){} // ubuntu laptop -> robot
    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udp;
    float dt = 0.01;
    // float dt = 0.001;

    // amarco:
    // StampedSequence sseq_server = {0};
    // StampedSequence sseq_client = {0};
    bool send_new_number = false;
    std::vector<double> time_elapsed_vec;
    long ind_global = 0;
    // auto clock = std::chrono::high_resolution_clock{};
    // using Clock = std::chrono::high_resolution_clock;
    // using TimePoint = std::chrono::time_point<Clock>;
    std::chrono::high_resolution_clock::time_point time_start;

    // HighCmd sseq_server = {0};
    // HighState sseq_client = {0};

    HighCmd sseq_server = {0};
    HighCmd sseq_client = {100}; // Just some dummy value different than zero


};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{  
    udp.Send();
}








// void Custom::Calc() 
// {
    
//     if(send_new_number == true && ind_global < 10000){

//         std::chrono::duration<double, std::nano> time_dur = std::chrono::high_resolution_clock::now() - time_start;
//         double time_dur_sec = double(time_dur.count()) / 1000000000.0;

//         std::cout << "time_dur: " << time_dur_sec << " seconds\n";

//         sseq_server.footRaiseHeight = time_dur_sec;

//         // std::cout << "time_dur: " << time_dur/1000000000. << " seconds\n";
//         // sseq_server.time_stamp = double(aux) / 1000000000.;


//         // sseq_server.time_stamp = clock.now(); // Record current timestamp
//         std::cout << "sseq_server.mode: " << unsigned(sseq_server.mode) << " (before increasing sequence)\n";
//         sseq_server.mode += 1; // Increase sequence
//         std::cout << "sseq_server.mode: " << unsigned(sseq_server.mode) << " (after increasing sequence)\n";

//         // Update statistical variables:
//         time_elapsed_vec[ind_global] = sseq_server.footRaiseHeight - sseq_client.footRaiseHeight;


//         // std::cout << "time_start: " << time_start.count() << "\n";
//         std::cout << "sseq_server.mode: " << unsigned(sseq_server.mode) << "\n";
//         std::cout << "elapsed_time: " << time_elapsed_vec[ind_global] << " seconds\n";

//         std::cout << "Roundtrip time: " << time_elapsed_vec[ind_global] << " seconds\n";



//         std::cout << "sseq_server.footRaiseHeight: " << sseq_server.footRaiseHeight << " seconds\n";
//         std::cout << "sseq_client.footRaiseHeight: " << sseq_client.footRaiseHeight << " seconds\n\n";

//         // Others:
//         ind_global += 1;
//         send_new_number = false;

//     }


    
//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (before sending)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " (before sending)\n";

//     // Keep sending whatever is inside sseq
//     udp.SetSend((char*)&sseq_server);
//     // udp.SetSend(sseq_server);

//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (after sending)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " (after sending)\n\n";



//     // amarco: the field sseq_server.footRaiseHeight gets overwritten with some trash value, and that's actually what is being ultimately sent...
//     // use the HighCmd, the char conversion is doint something strange...


//     // std::cout << "sseq_server.sequence_nr: "  << sseq_server.sequence_nr << "(after)\n\n";



//     // sseq_client.sequence_nr = -9;
//     // std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << "(before)\n";


//     std::cout << "sseq_client.mode: "  << unsigned(sseq_client.mode) << " (before receiving)\n";
//     std::cout << "sseq_client.footRaiseHeight: "  << sseq_client.footRaiseHeight << " (before receiving)\n";

//     // Keep reading whatever comes back
//     udp.GetRecv((char*)&sseq_client);
//     // udp.GetRecv(sseq_client);

//     std::cout << "sseq_client.mode: "  << unsigned(sseq_client.mode) << " (after receiving)\n";
//     std::cout << "sseq_client.footRaiseHeight: "  << sseq_client.footRaiseHeight << " (after receiving)\n\n";

//     // TODO: This function sets the fileds to zero...?

//     // std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << "(after)\n\n";
    

//     if(sseq_client.mode == sseq_server.mode){
//         send_new_number = true;

//         // TODO: Use "sseq_client.sequence_nr == sseq_server.sequence_nr" as the condition for the big if-else above
//     }
    
// }
















// void Custom::Calc() 
// {
//     // udp.GetRecv((char*)&bbb);
//     // printf("%f\n", bbb.yaw);

//     // printf("Server\n");
    
//     if(send_new_number == true && ind_global < 10000){
//     // if(sseq_client.mode == sseq_server.mode && ind_global < 10000){

        

//         // Update server data:


//         // Provide the time stamp in nanoseconds already:


//         std::chrono::duration<double, std::nano> time_dur = std::chrono::high_resolution_clock::now() - time_start;


//         std::cout << "\nUpdating!!!\n";

//         // auto aux = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - time_start);
//         std::cout << "time_dur: " << time_dur.count() << " nanoseconds\n";

//         double time_dur_sec = double(time_dur.count()) / 1000000000.0;

//         std::cout << "time_dur: " << time_dur_sec << " seconds\n";

//         sseq_server.footRaiseHeight = time_dur_sec;

//         // std::cout << "time_dur: " << time_dur/1000000000. << " seconds\n";
//         // sseq_server.time_stamp = double(aux) / 1000000000.;


//         // sseq_server.time_stamp = clock.now(); // Record current timestamp
//         std::cout << "sseq_server.mode: " << unsigned(sseq_server.mode) << " (before increasing sequence)\n";
//         sseq_server.mode += 1; // Increase sequence
//         std::cout << "sseq_server.mode: " << unsigned(sseq_server.mode) << " (after increasing sequence)\n";

//         // Update statistical variables:
//         time_elapsed_vec[ind_global] = sseq_server.footRaiseHeight - sseq_client.footRaiseHeight;


//         // std::cout << "time_start: " << time_start.count() << "\n";
//         std::cout << "sseq_server.mode: " << unsigned(sseq_server.mode) << "\n";
//         std::cout << "elapsed_time: " << time_elapsed_vec[ind_global] << " seconds\n";



//         std::cout << "sseq_server.footRaiseHeight: " << sseq_server.footRaiseHeight << " seconds\n";
//         std::cout << "sseq_client.footRaiseHeight: " << sseq_client.footRaiseHeight << " seconds\n\n";

//         // Others:
//         ind_global += 1;
//         send_new_number = false;

//     }


    
//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (before sending)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " (before sending)\n";

//     // Keep sending whatever is inside sseq
//     udp.SetSend((char*)&sseq_server);
//     // udp.SetSend(sseq_server);

//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (after sending)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " (after sending)\n\n";



//     // amarco: the field sseq_server.footRaiseHeight gets overwritten with some trash value, and that's actually what is being ultimately sent...
//     // use the HighCmd, the char conversion is doint something strange...


//     // std::cout << "sseq_server.sequence_nr: "  << sseq_server.sequence_nr << "(after)\n\n";



//     // sseq_client.sequence_nr = -9;
//     // std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << "(before)\n";


//     std::cout << "sseq_client.mode: "  << unsigned(sseq_client.mode) << " (before receiving)\n";
//     std::cout << "sseq_client.footRaiseHeight: "  << sseq_client.footRaiseHeight << " (before receiving)\n";

//     // Keep reading whatever comes back
//     udp.GetRecv((char*)&sseq_client);
//     // udp.GetRecv(sseq_client);

//     std::cout << "sseq_client.mode: "  << unsigned(sseq_client.mode) << " (after receiving)\n";
//     std::cout << "sseq_client.footRaiseHeight: "  << sseq_client.footRaiseHeight << " (after receiving)\n\n";

//     // TODO: This function sets the fileds to zero...?

//     // std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << "(after)\n\n";
    

//     if(sseq_client.mode == sseq_server.mode){
//         send_new_number = true;

//         // TODO: Use "sseq_client.sequence_nr == sseq_server.sequence_nr" as the condition for the big if-else above
//     }
    
// }

void Custom::Calc() 
{
    
    if(send_new_number == true && ind_global < time_elapsed_vec.size()){

        std::chrono::duration<double, std::nano> time_dur = std::chrono::high_resolution_clock::now() - time_start;
        sseq_server.footRaiseHeight = time_dur.count() / 1000000000.0; // NOTE: time_dur.count() returns a double

        // sseq_server.mode += 1; // Increase sequence
        sseq_server.mode = (sseq_server.mode + 1) % 256; // Increasing sequence; we use 'remainder' to avoid overflow

        // Update statistical variables:
        time_elapsed_vec[ind_global] = sseq_server.footRaiseHeight - sseq_client.footRaiseHeight;

        std::cout << "elapsed_time: " << time_elapsed_vec[ind_global] << " seconds\n";

        // Others:
        // ind_global = (ind_global + 1) % 10 ;
        ind_global += 1;
        send_new_number = false;
    }

    
    // Keep sending whatever is inside sseq
    udp.SetSend((char*)&sseq_server);
    udp.Send();

    // Keep reading whatever comes back
    udp.GetRecv((char*)&sseq_client);
    udp.Recv();

    if(sseq_client.mode == sseq_server.mode)
        send_new_number = true;
    
}

int main(void)
{
    Custom custom;
    // InitEnvironment();

    custom.time_start = std::chrono::high_resolution_clock::now();

    // Send a sequence of numbers and see when the numbers are getting back by comparing timestamps
    // Inside the robot, we should have another script which only purpose is to return the sequence back in order


    // Keep sending a message on the loop until it's received back. Store the time stamp and repeat the operation with
    // another number

    // Dump the c++ vector into Python. Several options:
    // 1) Copy-paste the data from the terminal; it's just a quick test. Or print after data collection as a `np.array([3.4, 6.87, ..., 5.90])`
    // 2) Start from here: https://stackoverflow.com/questions/6157409/stdvector-to-boostpythonlist
    // 3) Use pybind11



    // amarco: these loops do not guarantee real-time, they're just
    LoopFunc loop_calc("calc_loop",   custom.dt,    boost::bind(&Custom::Calc,    &custom)); // std::string name, float period, const Callback& _cb
    // LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom)); // std::string name, float period, int bindCPU, const Callback& _cb
    // LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom)); // std::string name, float period, int bindCPU, const Callback& _cb

    // loop_udpSend.start();
    // loop_udpRecv.start();
    loop_calc.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
