/************************************************************************
*
* @author Alonso Marco
* Contact: amarco@berkeley.edu
************************************************************************/

/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Position Holder ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */


#include <position_holder_control_loop.hpp>
#include <chrono>
#include <thread>

void PositionHolderControlLoop::go2target_linear_interpolation( const Eigen::Ref<Vector12d>& joint_pos_init, 
                                                    const Eigen::Ref<Vector12d>& joint_pos_final,
                                                    Eigen::Ref<Vector12d> joint_pos_interp,
                                                    double rate){
    
    // double rate = std::min(std::max(rate, 0.0), 1.0);
    joint_pos_interp = joint_pos_init*(1.-rate) + joint_pos_final*rate;

    return;
}


void PositionHolderControlLoop::lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    
    // UNITREE_LEGGED_SDK::LowCmd low_cmd_subs = {0};
    // low_cmd_subs = ToLcm(msg,low_cmd_subs);

    // NOTE: no need to convert to LCM types because we're just receiving and copying to joint_pos_des_hold
    // Read desired position, broadcasted to the network
    // Write the desired position in the global joint_pos_des_hold:
    for(int ii=0; ii < this->Njoints; ii++){

        // joint_pos_des_hold[ii] = low_cmd_subs.motorCmd[ii].q;
        this->joint_pos_des_hold[ii] = msg->motorCmd[ii].q;
    }

    std::cout << "Receiving joint_pos_des_hold ... || joint_pos_des_hold[0] = " << this->joint_pos_des_hold[0] << "\n";

    return;
}

void PositionHolderControlLoop::read_initial_position(int Nsteps_timeout, int time_sleep_ms, Eigen::Ref<Vector12d> joint_pos_init, bool verbosity){

    // Change the operation mode inside the robot
    // Assumption: this->state.motorCmd[ii].q is equal to zero unless readings from the robot have been received ...

    int ii = 0;
    bool is_desired_position_all_zeros;
    double time_sleep_total_seconds = double(time_sleep_ms*Nsteps_timeout)/1000.0;
    std::cout << "Sending to the robot: 'levelFlag = LOWLEVEL (0xFF)' and 'mode = 0x0A'\n";
    std::cout << "Looping until current joint position is received ...\n";
    std::cout << "Looping for ~" << time_sleep_total_seconds << " seconds, at rate: " << double(1000.0/double(time_sleep_ms)) << " Hz\n";
    while (ii < Nsteps_timeout){

        if(verbosity == true)
            std::cout << "Sending low-level mode, trial: " << ii+1 << "/" << Nsteps_timeout << "\n";

        this->change_operation_mode_inside_robot();
        this->CollectObservations();
        this->update_all_observations();

        // When a new position is received, state.motorCmd[ii].q becomes non-zero
        is_desired_position_all_zeros = true;
        for(int ii=0; ii < this->Njoints; ii++){

            // Fill in initial position:
            joint_pos_init[ii] = this->state.motorState[ii].q;

            // Make sure that the initial position is no all zeros
            if(this->state.motorState[ii].q != 0.0)
                is_desired_position_all_zeros = false; // If only one is different, change the flag
        }

        if(verbosity == true){
            std::cout << "Current initial position:\n";
            this->print_joint_info("read position",joint_pos_init);
        }

        ii++;
        std::this_thread::sleep_for(std::chrono::milliseconds(time_sleep_ms));
    }

    if(is_desired_position_all_zeros == true){
        std::cout << "Mode change uneffective; timed out; exiting program ...\n";
        exit(-1);
    }

    std::cout << "Initial position:\n";
    this->print_joint_info("position2hold",this->joint_pos_des_hold);

    return;

}


