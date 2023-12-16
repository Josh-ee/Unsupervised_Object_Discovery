



void RealRobotInterfaceGo1::main(){


    LoopFunc loop_control("control_loop", deltaT, boost::bind(&RealRobotInterfaceGo1::ControlLoop, &*this));

    loop_control.start();

    sleep(0.5);


    bool is_joinable = loop_control._thread.joinable();
    std::thread::id id_thread = loop_control._thread.get_id();
    std::cout << "is_joinable: " << is_joinable << '\n';
    std::cout << "id_thread: " << id_thread << '\n';
    // prints: id_thread: thread::id of a non-executing thread, TODO: why? https://stackoverflow.com/questions/36357909/printing-stdthis-threadget-id-gives-threadid-of-a-non-executing-thread

    is_running_control_loop = true;

    // sleep(0.5);

    float time_sleep = 1.0;
    std::cout << "Sleeping for " << std::to_string(int(time_sleep)) << " seconds ...\n";
    sleep(time_sleep);


    std::cout << "Detaching thread !!!\n";

    loop_control._thread.detach();
    // amarco: throws terminate called after throwing an instance of 'std::system_error'
    // I don't think it really starts... 
    // terminate called after throwing an instance of 'std::system_error'
    //   what():  No such process
    // Aborted (core dumped)
    // https://stackoverflow.com/questions/65811630/how-can-i-check-if-thread-is-done-when-using-threaddetach


    // So, ros::spin() does NOT return, it blocks: https://answers.ros.org/question/58060/how-do-i-get-out-of-rosspin-in-another-thread/
    // That means that we unfortunately need to go through ROS already for the position holder; otherwise, we'll re-invent the wheel

    std::cout << "Detaching completed !!!\n";

    // loop_control.shutdown();

    is_running_control_loop = false;

    std::cout << "Exiting c++ function\n";

    return;
}





void RealRobotInterfaceGo1::stand_up(int Nsteps){
    /*


    Assumption 1) The robot is lying on the floor before calling this, and in low-level mode
    Assumption 2) This function assumes that ControlLoop() is running inside main()

    1) Read the current position
    2) Soft transition to the next one
    3) Create a class that does this smoothonly, something like TransitionBetweenJointPositions
    4) TODO: Do we need to block the variable joint_pos_des_hold so that it's not read while being written?

    */

    if(!is_running_control_loop){
        std::cout << "Control loop is not running, this function returns without dong anything!\n";
        return;
    }


    int rate_count = 0;


    // Read current position:
    // NOTE: The current position is being updated inside ControlLoop(), which is
    Vector12d joint_pos_init;
    joint_pos_init.setZero();
    Vector12d joint_pos_interp;
    joint_pos_interp.setZero();


    // TODO: Adjust the loop limits
    for (int ii = 0; ii < Nsteps; ii++) {

        if( ii >= 0 && ii < 10){

            joint_pos_init = joint_pos_curr;


            // TODO: Update already? If the gains are stiff, it will likely change the robot's position a bit
            joint_pos_des_hold = joint_pos_curr;

        }

        if( ii >= 10 && ii < 400){

            rate_count++;
            double rate = rate_count/200.0;
            rate = std::min(rate, 1.0);

            this->go2target_linear_interpolation(joint_pos_init,joint_pos_init_target,joint_pos_interp,rate);

            joint_pos_des_hold = joint_pos_interp;
        
        }

    }

    return;
}
