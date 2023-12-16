/**********************************************************************************************
 * Robot Interface inspired in
     * Unitree's example_position.cpp
     * Google/UC Berkeley repo https://github.com/erwincoumans/motion_imitation
 * We provide a set of functions to interface with the real robot using Python
**********************************************************************************************/

#include <python_interface_go1/real_robot_interface_go1.hpp>

// #include <pybind11/pybind11.h>
// #include <pybind11/eigen.h> // This header includes #include <Eigen/Core>
// #include <pybind11/stl.h>

// void RealRobotInterfaceGo1::InitializeAllFieldsToZero(){

//     // LowState:
//     state.levelFlag = 0;
//     state.commVersion = 0;
//     state.robotID = 0;
//     state.SN = 0; 
//     state.bandWidth = 0;

//     state.imu.quaternion.fill(0.0);
//     state.imu.gyroscope.fill(0.0);
//     state.imu.accelerometer.fill(0.0);
//     state.imu.rpy.fill(0.0);
//     state.imu.temperature = 0;

//     for (int ii = 0; ii < state.motorState.size(); ii++) {

//         state.motorState[ii].mode = 0;
//         state.motorState[ii].q = 0.0;
//         state.motorState[ii].dq = 0.0;
//         state.motorState[ii].ddq = 0.0;
//         state.motorState[ii].tauEst = 0.0;
//         state.motorState[ii].q_raw = 0.0;
//         state.motorState[ii].dq_raw = 0.0;
//         state.motorState[ii].ddq_raw = 0.0;
//         state.motorState[ii].temperature = 0;
//         state.motorState[ii].reserve.fill(0);
//     }

//     state.footForce.fill(0);
//     state.footForceEst.fill(0);

//     state.bms.version_h = 0;
//     state.bms.version_l = 0;
//     state.bms.bms_status = 0;
//     state.bms.SOC = 0;
//     state.bms.current = 0;
//     state.bms.cycle = 0;

//     state.tick = 0;

//     state.wirelessRemote.fill(0);        // wireless commands
//     state.reserve = 0;
//     state.crc = 0;


//     // LowCmd
//     cmd.levelFlag = 0;
//     cmd.commVersion = 0;
//     cmd.robotID = 0;
//     cmd.SN = 0;
//     cmd.bandWidth = 0;
//     cmd.bms.off = 0;
//     // cmd.bms.reserve;

//     // MotorCmd motorCmd[20];
//     for (int ii = 0; ii < cmd.motorCmd.size(); ii++) {

//         cmd.motorCmd[ii].mode = 0;
//         cmd.motorCmd[ii].q = 0.0;
//         cmd.motorCmd[ii].dq = 0.0;
//         cmd.motorCmd[ii].tau = 0.0;
//         cmd.motorCmd[ii].Kp = 0.0;
//         cmd.motorCmd[ii].Kd = 0.0;
//         cmd.motorCmd[ii].reserve.fill(0);
//     }

// }

void RealRobotInterfaceGo1::CollectObservations() {
    udp.Recv();
    udp.GetRecv(state);
    return;
}

void RealRobotInterfaceGo1::update_all_observations() {

    this->update_joint_pos_curr();
    this->update_joint_vel_curr();
    this->update_body_linear_velocity();
    this->update_body_angular_velocity();
    this->update_body_orientation();

    return;
}

void RealRobotInterfaceGo1::update_joint_pos_curr(){

    for (int jj = 0; jj < Njoints; jj++){ // amarco: std::array<MotorState, 20> motorState; we only need the first 12 elements, corresponding to the joints
        joint_pos_curr[jj] = state.motorState[jj].q;
    }


    return;
}

void RealRobotInterfaceGo1::update_joint_vel_curr(){

    for (int jj = 0; jj < Njoints; jj++)
        joint_vel_curr[jj] = state.motorState[jj].dq;

    return;
}


void RealRobotInterfaceGo1::update_body_linear_velocity(){

    // Numerical integration:
    for (int jj = 0; jj < body_linear_velocity.size(); jj++)
        body_linear_velocity[jj] += deltaT*state.imu.accelerometer[jj];

    return;
}

void RealRobotInterfaceGo1::update_body_angular_velocity(){

    // Numerical integration:
    for (int jj = 0; jj < body_angular_velocity.size(); jj++)
        body_angular_velocity[jj] = state.imu.gyroscope[jj];

    return;
}


void RealRobotInterfaceGo1::update_body_orientation(){

    // Numerical integration:
    for (int jj = 0; jj < body_orientation.size(); jj++)
        body_orientation[jj] = state.imu.rpy[jj];

    return;
}

void RealRobotInterfaceGo1::get_joint_pos_curr(Eigen::Ref<Vector12d> joint_pos_curr){
    joint_pos_curr = joint_pos_curr;

    // joint_pos_curr[0] = 100.0; // dbg, this works

    return;
}

void RealRobotInterfaceGo1::ensure_safety(){

    // Saturate positions:
    safe.PositionLimit(cmd);

    int trigger_pow = safe.PowerProtect(cmd,state,1);
    if (trigger_pow < 0){
        std::cout << "Power protection triggered exit (!!)\n";
        exit(-1);
    }

    if (this->use_position_protect == true){
        int trigger_pos = safe.PositionProtect(cmd,state,0.087);
        if (trigger_pos < 0){
            std::cout << "Position protection triggered exit (!!)\n";
            exit(-1);
        }
    }

    return;
}

void RealRobotInterfaceGo1::SendCommand(const Eigen::Ref<Vector12d>& joint_pos_des,
                                    const Eigen::Ref<Vector12d>& joint_vel_des,
                                    const Eigen::Ref<Vector12d>& joint_torque_des){

    // cmd.levelFlag = 0xff; 
    cmd.levelFlag = LOWLEVEL;
    for (int ii = 0; ii < this->Njoints; ii++) { // amarco: std::array<MotorCmd, 20> motorCmd; we only need the first 12 dimensions, corresponding to the joints
        cmd.motorCmd[ii].mode = 0x0A; // motor switch to servo (PMSM) mode
        cmd.motorCmd[ii].q = joint_pos_des[ii];
        cmd.motorCmd[ii].dq = joint_vel_des[ii];
        cmd.motorCmd[ii].tau = joint_torque_des[ii];
        cmd.motorCmd[ii].Kp = P_gains_[ii];
        cmd.motorCmd[ii].Kd = D_gains_[ii];
    }

    this->ensure_safety(); // Will modify the contents of cmd to ensure that joint/torque limits are not exceeded

    udp.SetSend(cmd);
    udp.Send();

    return;
}


void RealRobotInterfaceGo1::send_desired_position(const Eigen::Ref<Vector12d>& joint_pos_des){
    /*

    Send only desired position, while setting the desired velocity to zero and torque to appropriate values

    */

    // TODO: Fix this: differentiate the desired position to get the velocity?
    Vector12d joint_vel_des; // How do we get this? What do we set it to?
    joint_vel_des.setZero(); // By setting to zero, we basically bypass the influence of the D gain
    
    Vector12d joint_torque_des; // u_total = u_des + u_PID; in order to set u_PID=0, we need to set the PD gains to zero
    joint_torque_des.setZero();

    // Gravity compensation (as in example_position.cpp):
    if(this->use_gravity_compensation == true){
        joint_torque_des[FR_0] = -0.65f;
        joint_torque_des[FL_0] = +0.65f;
        joint_torque_des[RR_0] = -0.65f;
        joint_torque_des[RL_0] = +0.65f;
    }

    this->SendCommand(joint_pos_des,joint_vel_des,joint_torque_des);

    return;
}


void RealRobotInterfaceGo1::change_operation_mode_inside_robot(void){

    /*
    Inform the robot that we are going to control it using lowlevel mode
    */

    // cmd.levelFlag = 0xff; 
    cmd.levelFlag = LOWLEVEL;
    for (int ii = 0; ii < this->Njoints; ii++) { // amarco: std::array<MotorCmd, 20> motorCmd; we only need the first 12 dimensions, corresponding to the joints
        cmd.motorCmd[ii].mode = 0x0A; // motor switch to servo (PMSM) mode
        cmd.motorCmd[ii].q = 0.0;
        cmd.motorCmd[ii].dq = 0.0;
        cmd.motorCmd[ii].tau = 0.0;
        cmd.motorCmd[ii].Kp = 0.0;
        cmd.motorCmd[ii].Kd = 0.0;
    }

    this->ensure_safety(); // Will modify the contents of cmd to ensure that joint/torque limits are not exceeded

    udp.SetSend(cmd);
    udp.Send();

    return;

}

void RealRobotInterfaceGo1::set_PD_gains(const Eigen::Ref<Vector12d>& P_gains, const Eigen::Ref<Vector12d>& D_gains){

    // std::copy(std::begin(P_gains), std::end(P_gains), std::begin(this->P_gains_));
    // std::copy(std::begin(D_gains), std::end(D_gains), std::begin(this->D_gains_));

    this->P_gains_ = P_gains;
    this->D_gains_ = D_gains;

    std::cout << "Setting PD gains for position control:\n";

    std::cout << "P_gains: " << this->P_gains_.transpose().format(this->clean_format) << "\n";
    std::cout << "D_gains: " << this->D_gains_.transpose().format(this->clean_format) << "\n";

    return;
}


void RealRobotInterfaceGo1::set_deltaT(double deltaT){
    this->deltaT = deltaT;
    std::cout << "Setting deltaT: " << deltaT << " seconds\n";
    std::cout << "deltaT will only we used for state estimation using IMUs\n";
    return;
}

double RealRobotInterfaceGo1::get_deltaT(void){
    return this->deltaT;
}

void RealRobotInterfaceGo1::print_joint_info(std::string name, const Eigen::Ref<Vector12d>& joint_vec){

    std::cout << name << ": " << joint_vec.transpose().format(this->clean_format) << "\n";

    return;

}


/* --------------------------------------------------------------------------------------------------------------- */
/* ------------------------------------------ Gym Environment ---------------------------------------------------- */
/* --------------------------------------------------------------------------------------------------------------- */


void GymEnvironmentRealGo1::step(const Eigen::Ref<Vector12d>& action){


    // This updates joint_pos_des_
    Vector12d joint_pos_des;
    this->action2joint_pos_des(action,joint_pos_des); // We have 12 actuators; action is 12-dimensional and it's transformed here into desired positions

    this->send_desired_position(joint_pos_des);

    // Update:
    this->update_env_observation();

    return;
}

void GymEnvironmentRealGo1::set_action_std(double action_std){

    this->actionStd_.setConstant(action_std);

    return;
}

void GymEnvironmentRealGo1::set_action_mean(const Eigen::Ref<Vector12d>& action_mean){

    this->actionMean_ = action_mean;

    return;
}

void GymEnvironmentRealGo1::action2joint_pos_des(const Eigen::Ref<Vector12d>& action, Eigen::Ref<Vector12d> joint_pos_des){

    joint_pos_des = action;

    // action scaling
    joint_pos_des.cwiseProduct(actionStd_); // amarco: element-wise product; https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    joint_pos_des += actionMean_;

    return;

}


void GymEnvironmentRealGo1::update_env_observation(){

    this->CollectObservations();
    this->update_all_observations();

    this->observation_env_ << 3.0, // body height // TODO: change this using IK ...
                        body_orientation, // body orientation [3]
                        joint_pos_curr, // euler angle（unit: rad) [12]
                        body_linear_velocity, // m/s [3]
                        body_angular_velocity, // rad/s (numerically-integrated raw data, will drift over time) [3]
                        joint_vel_curr; // rad/s [12]

    return;
}


Eigen::VectorXd GymEnvironmentRealGo1::get_env_observation(){
    return observation_env_;
}

void GymEnvironmentRealGo1::init() { 


    // Have here a selector of different initialization behaviors:
    // int Nsteps = 200;
    // this->stand_up(Nsteps);

    return; 
}


