/* amarco */

// Notice: This exemple should running on another PC, and make sure the Ethernet is stable.

#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include "multi_pc_type.h"

// amarco:
#include <iostream>
#include <unitree_legged_sdk/json.hpp>
#include <fstream>
using json = nlohmann::json;

using namespace UNITREE_LEGGED_SDK;

class Custom
{

public:
    // Custom(): udp(8118, "192.168.64.3", 8117, sizeof(StampedSequence), sizeof(StampedSequence)){}
    // Custom(): udp(8118, "192.168.64.3", 8117, sizeof(HighCmd), sizeof(HighState)){
    //     // udp.InitCmdData(sseq_client);
    //     return;
    // }
    // Custom(): udp(8118, "192.168.64.3", 8117, sizeof(HighCmd), sizeof(HighCmd)){} // mac laptop -> mac laptop
    // Custom(): udp(8118, "127.0.0.1", 8117, sizeof(HighCmd), sizeof(HighCmd)){} // ubuntu laptop -> ubuntu laptop
    Custom(): udp(8118, "192.168.12.235", 8117, sizeof(HighCmd), sizeof(HighCmd)){} // robot -> ubuntu laptop
    void UDPRecv();
    void UDPSend();
    void Calc();

    UDP udp;
    float dt = 0.01;
    // float dt = 0.001;

    bool verbo = true;

    // // amarco:
    // StampedSequence sseq_server = {0};
    // StampedSequence sseq_client = {0};

    // // amarco:
    // HighCmd sseq_client = {0};
    // HighState sseq_server = {0};


    // // amarco:
    // HighState sseq_client = {0};
    // HighCmd sseq_server = {0};


    // // amarco:
    // HighCmd sseq_client = {0};
    // HighCmd sseq_server = {0};


    // amarco:
    HighCmd sseq_package = {0};
    HighCmd sseq_package_local_copy = {0};



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

//     // printf("Client\n");

//     std::cout << "sseq_server.sequence_nr: "  << sseq_server.sequence_nr << " (before receiving)\n";
//     std::cout << "sseq_server.time_stamp: "  << sseq_server.time_stamp << " (before receiving)\n";


//     // Read whatever is being received from the server:
//     udp.GetRecv((char*)&sseq_server);

//     std::cout << "sseq_server.sequence_nr: "  << sseq_server.sequence_nr << " (after receiving)\n";
//     std::cout << "sseq_server.time_stamp: "  << sseq_server.time_stamp << " (after receiving)\n";



//     // Copy the data and send it back:
//     sseq_client.sequence_nr = sseq_server.sequence_nr;
//     sseq_client.time_stamp = sseq_server.time_stamp;



//     std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << " (before sending)\n";
//     std::cout << "sseq_client.time_stamp: "  << sseq_client.time_stamp << " (before sending)\n";



//     udp.SetSend((char*)&sseq_client);


//     std::cout << "sseq_client.sequence_nr: "  << sseq_client.sequence_nr << " (after sending)\n";
//     std::cout << "sseq_client.time_stamp: "  << sseq_client.time_stamp << " (after sending)\n";


// }


// void Custom::Calc() 
// {

//     // printf("Client\n");

//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (before receiving)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " seconds (before receiving)\n";


//     // Read whatever is being received from the server:
//     udp.GetRecv((char*)&sseq_server);
//     // udp.GetRecv(sseq_server);

//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (after receiving)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " seconds (after receiving)\n";



//     // // Copy the data and send it back:
//     // sseq_client.sequence_nr = sseq_server.sequence_nr;
//     // sseq_client.time_stamp = sseq_server.time_stamp;


//     // Copy the data and send it back:
//     sseq_client.mode = sseq_server.mode;
//     sseq_client.footRaiseHeight = sseq_server.footRaiseHeight;



//     std::cout << "sseq_client.mode: "  << unsigned(sseq_client.mode) << " (before sending)\n";
//     std::cout << "sseq_client.footRaiseHeight: "  << sseq_client.footRaiseHeight << " seconds (before sending)\n";

//     // udp.SetSend(sseq_client);
//     udp.SetSend((char*)&sseq_client);

//     std::cout << "sseq_client.mode: "  << unsigned(sseq_client.mode) << " (after sending)\n";
//     std::cout << "sseq_client.footRaiseHeight: "  << sseq_client.footRaiseHeight << " seconds (after sending)\n";



// }


// void Custom::Calc() 
// {

//     // Read whatever is being received from the server:
//     udp.GetRecv((char*)&sseq_server);

//     // Copy the data and send it back:
//     sseq_client.mode = sseq_server.mode;
//     sseq_client.footRaiseHeight = sseq_server.footRaiseHeight;
//     udp.SetSend((char*)&sseq_client);

//     std::cout << "Bouncing data back: \n";
//     std::cout << "sseq_server.mode: "  << unsigned(sseq_server.mode) << " (after receiving)\n";
//     std::cout << "sseq_server.footRaiseHeight: "  << sseq_server.footRaiseHeight << " seconds (after receiving)\n\n";


// }


void Custom::Calc() 
{
    // Read whatever is being received from the server:
    udp.GetRecv((char*)&sseq_package);
    udp.Recv();

    // If it differs from the local copy, send it back:
    if(sseq_package.mode != sseq_package_local_copy.mode){
    
        udp.SetSend((char*)&sseq_package);
        udp.Send();
        sseq_package_local_copy.mode = sseq_package.mode;
    
        if(verbo == true){
            std::cout << "Bouncing data back (ONLY WHEN DIFFERS FROM LOCAL COPY): \n";
            std::cout << "sseq_package.mode: "  << unsigned(sseq_package.mode) << " (after receiving)\n";
            std::cout << "sseq_package.footRaiseHeight: "  << sseq_package.footRaiseHeight << " seconds (after receiving)\n\n";
        }

    }


    // // Always send it back:
    // udp.SetSend((char*)&sseq_package);
    // udp.Send();
    // if(verbo == true){
    //     std::cout << "Bouncing data back ALWAYS: \n";
    //     std::cout << "sseq_package.mode: "  << unsigned(sseq_package.mode) << " (after receiving)\n";
    //     std::cout << "sseq_package.footRaiseHeight: "  << sseq_package.footRaiseHeight << " seconds (after receiving)\n\n";
    // }

}


int main(void)
{
    Custom custom;
    // InitEnvironment();

    // https://github.com/nlohmann/json
    // std::ifstream f("/Users/alonrot/work/code_projects_WIP/unitree_legged_sdk_from_inside_robot/examples/cfg_latency.json", std::ifstream::binary);
    // json data = json::parse(f);
    // json data;
    // f >> data;
    // std::cout << "data: " << data << "\n";

    // std::cout << "data[pi]: " << data["pi"] << "\n";


    // amarco: these loops do not guarantee real-time, they're just
    LoopFunc loop_calc("calc_loop",   custom.dt,    boost::bind(&Custom::Calc,    &custom));
    // LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    // LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));

    // loop_udpSend.start();
    // loop_udpRecv.start();
    loop_calc.start();

    while(1){
        sleep(10);
    };

    return 0; 
}
