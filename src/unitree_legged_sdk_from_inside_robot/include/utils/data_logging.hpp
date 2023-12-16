// #include "unitree_legged_sdk/unitree_legged_sdk.h"
// #include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>

// amarco:
#include <fstream>
#include <chrono>
#include <ctime>


class Write2File
{
public:
    Write2File(std::string rootpath, std::string filename_base): rootpath_(rootpath), filename_base_(filename_base){

    	path2folder_ = get_folder_name_with_time_of_day(rootpath_);

    }

    std::string get_folder_name_with_time_of_day(std::string rootpath, bool create_folder = true);

    template<std::size_t SIZE_TIME, std::size_t SIZE_JOINT_NAMES, std::size_t SIZE_DATA_FIELD_NAMES>
    void dump(std::array< std::array<std::array<float, SIZE_TIME>, SIZE_JOINT_NAMES> , SIZE_DATA_FIELD_NAMES> & data_fields,
                    std::array<std::string, SIZE_JOINT_NAMES> & data_joint_names,
                    std::array<std::string, SIZE_DATA_FIELD_NAMES> & name_data_fields);

    // void dump(Custom &custom);


    std::string rootpath_;
    std::string filename_base_;
    std::string path2folder_;
    std::array<std::string, 10> file_path_named;
    std::array<std::ofstream, 10> files_vec;

};

// void Write2File::dump(Custom &custom){
template<std::size_t SIZE_TIME, std::size_t SIZE_JOINT_NAMES, std::size_t SIZE_DATA_FIELD_NAMES>
void Write2File::dump(	std::array< std::array<std::array<float, SIZE_TIME>, SIZE_JOINT_NAMES> , SIZE_DATA_FIELD_NAMES> & data_fields,
            std::array<std::string, SIZE_JOINT_NAMES> & data_joint_names,
            std::array<std::string, SIZE_DATA_FIELD_NAMES> & name_data_fields){


    // char buffer[8096]; // larger = faster (within limits)
    // file.rdbuf()->pubsetbuf(buffer, sizeof(buffer));

    // std::cout << "here1" << "\n";
    // std::cout << "name_data_fields.size(): " << name_data_fields.size() << "\n";


    for (std::size_t ff = 0 ; ff < name_data_fields.size() ; ff++){
        file_path_named[ff] = path2folder_ + filename_base_ + "_" + name_data_fields[ff] + ".csv";
        std::cout << "Dumping data to file: " << file_path_named[ff] << "!!!\n";
        files_vec[ff].open(file_path_named[ff], std::ofstream::out | std::ofstream::trunc);
    }

    // std::cout << "here2\n";


    for (std::size_t ff = 0 ; ff < name_data_fields.size() ; ff++) {
    
        // names:
        for (std::size_t jj = 0 ; jj < data_joint_names.size() ; jj++) {
            if (jj < data_joint_names.size()-1)
                files_vec[ff] << data_joint_names[jj] << ",";
            else
                files_vec[ff] << data_joint_names[jj];
        }
        files_vec[ff] << "\n";

        // std::cout << "here2.1\n";

        // values:
        size_t Nrows = data_fields[0][0].size();
        size_t Ncols = data_fields[0].size();
        // std::cout << "here2.2\n";
        // std::cout << "Nrows: " << std::to_string(Nrows) << "\n";
        // std::cout << "Ncols: " << std::to_string(Ncols) << "\n";
        for (std::size_t ii = 0 ; ii < Nrows ; ii++) {

            // std::cout << "here2.3\n";

            for (std::size_t jj = 0 ; jj < Ncols ; jj++) {

                // std::cout << "here2.4\n";
                
                if (jj < Ncols-1)
                    files_vec[ff] << data_fields[ff][jj][ii] << ',';
                else
                    files_vec[ff] << data_fields[ff][jj][ii] << "\n";
            
            }
        }

        files_vec[ff].close();

    }

    std::cout << "Done!!! " << "\n";

    // std::cout << "here3\n";
}


std::string Write2File::get_folder_name_with_time_of_day(std::string rootpath, bool create_folder){
    
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%Y_%m_%d_%H_%M_%S",timeinfo);
    std::string str_time(buffer);

    std:string path2folder(rootpath + "/" + str_time);

    if(create_folder == true){
        std::cout << "Creating folder: "+path2folder << "\n";
        std::string filepath_command("mkdir -p "+path2folder); // Create command
        char* filepath_command_char = &*filepath_command.begin(); // Convert to char*
        std::cout << "Creating folder via system call...\n";
        int sys_res = system(filepath_command_char); // System call
        std::cout << "System call result: " << sys_res << "\n";
        std::cout << "Done!" << "\n";
    }
    else{
        std::cout << "Returning path: "+path2folder << "\n";
        std::cout << "No folder will be created!" << "\n";
    }

    return path2folder;
}