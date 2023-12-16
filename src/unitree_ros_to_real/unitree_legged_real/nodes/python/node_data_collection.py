#!/usr/bin/env python
import os
import numpy as np
import pdb
import time
from datetime import datetime
import pickle
import math
import rospy

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
import std_msgs.msg
import ood_gpssm_msgs.msg

msg_data_collection = ood_gpssm_msgs.msg.DataCollection()
msg_data_collection.start = False
msg_data_collection.stop = False
msg_go1_state = ood_gpssm_msgs.msg.Go1State()
msg_high_cmd = unitree_legged_msgs.msg.HighCmd()

def callback_data_collection(msg_in):
    
    global msg_data_collection
    msg_data_collection = msg_in

    if msg_data_collection.start:
        rospy.loginfo("Received message to START data collection")
    
    if msg_data_collection.stop:
        rospy.loginfo("Received message to STOP data collection")

def callback_cmd_high(msg_in):
    global msg_high_cmd
    msg_high_cmd = msg_in

def callback_go1_state(msg_in):
    global msg_go1_state
    msg_go1_state = msg_in

def reset_data_structure(Nsteps_total):

    data2save = dict(   robot_pos=np.zeros((Nsteps_total,3)),\
                        robot_vel=np.zeros((Nsteps_total,3)),\
                        robot_orientation=np.zeros((Nsteps_total,3)),\
                        robot_angular_velocity=np.zeros((Nsteps_total,3)),\
                        vel_forward_des=np.zeros((Nsteps_total,1)),\
                        vel_yaw_des=np.zeros((Nsteps_total,1)),\
                        time_stamp=np.zeros((Nsteps_total,1)))

    return data2save


if __name__ == "__main__":


    # path2save_root = "/home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # ubuntu VM
    # path2save_root = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/data_experiments_go1" # robot's laptop

    # Assuming you are in <path/to/catkin_ws>/src/unitree_ros_to_real/unitree_legged_real/nodes/python
    path2save_root = "./data_experiments_go1"

    input("Enter experiments folder name!!!!")
    # folder_name_experiments = "experiments_2023_03_16"
    # folder_name_experiments = "experiments_2023_03_24"
    # folder_name_experiments = "experiments_2023_03_25"
    folder_name_experiments = "experiments_2023_03_29"
    path2save = "{0:s}/{1:s}".format(path2save_root,folder_name_experiments)


    # rostopic pub --once /experiments_gpssm_ood/data_collection_triggers ood_gpssm_msgs/DataCollection '{start: True}'
    # rostopic pub --once /experiments_gpssm_ood/data_collection_triggers ood_gpssm_msgs/DataCollection '{stop: True}'
    
    np.random.seed(1)

    rospy.init_node("run_go2experiment", anonymous=False)
    rate_main_loop = 500 # Hz
    ros_loop = rospy.Rate(rate_main_loop) # Hz

    time_max_data_collection = 600 # sec
    Nsteps_total = int(rate_main_loop*time_max_data_collection)

    rospy.loginfo("Data will be collected at ~{0:d} Hz ...".format(rate_main_loop))
    rospy.loginfo("Data will be saved at {0:s}".format(path2save))

    topic_data_collection_triggers = "/experiments_gpssm_ood/data_collection_triggers"
    rospy.Subscriber(topic_data_collection_triggers, ood_gpssm_msgs.msg.DataCollection, callback_data_collection)

    topic_high_cmd = "/high_cmd_to_robot"
    rospy.Subscriber(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, callback_cmd_high)

    topic_robot_state = "/experiments_gpssm_ood/robot_state"
    rospy.Subscriber(topic_robot_state, ood_gpssm_msgs.msg.Go1State, callback_go1_state)


    data2save = reset_data_structure(Nsteps_total)


    collect_data_on = False
    save_data = False

    rospy.loginfo("Ready to collect data; looping ...")
    while True:


        """
        =========================
        <<<< Data collection >>>>
        =========================
        """

        if msg_data_collection.start and collect_data_on: # msg_data_collection.start is global, written inside its callback
            rospy.loginfo("Data collection was already requested and data is currently being collected...")
        elif msg_data_collection.start == True:
            rospy.loginfo("Starting data collection!")
            data2save = reset_data_structure(Nsteps_total)
            msg_data_collection.start = False
            collect_data_on = True
            time_start = time.time()
            ii = 0

        if msg_data_collection.stop and not collect_data_on: # msg_data_collection.stop is global, written inside its callback
            rospy.loginfo("Someone is trying to stop data collection, but data collection hasn't started yet...")
            msg_data_collection.stop = False

        if collect_data_on and (ii >= Nsteps_total or msg_data_collection.stop): # Check termination criterion
            
            if ii > Nsteps_total: rospy.loginfo("Data collection terminated! Reason: timed out")
            if msg_data_collection.stop: rospy.loginfo("Data collection terminated! Reason: external trigger")
            collect_data_on = False
            save_data = True
            msg_data_collection.stop = False

        elif collect_data_on: # Collect and store the data

            if (ii + 1) % rate_main_loop == 0: rospy.loginfo("Collecting data; will time out after {0:d} seconds if not stopped externally\n * elapsed time: {1:2.1f} seconds".format(time_max_data_collection,(ii+1)*(1./rate_main_loop),time_max_data_collection))

            data2save["robot_pos"][ii,:] = np.array([msg_go1_state.position.x,msg_go1_state.position.y,msg_go1_state.position.z])
            data2save["robot_vel"][ii,:] = np.array([msg_go1_state.twist.linear.x,msg_go1_state.twist.linear.y,msg_go1_state.twist.linear.z])
            data2save["robot_orientation"][ii,:] = np.array([msg_go1_state.orientation.x,msg_go1_state.orientation.y,msg_go1_state.orientation.z])
            data2save["robot_angular_velocity"][ii,:] = np.array([msg_go1_state.twist.angular.x,msg_go1_state.twist.angular.y,msg_go1_state.twist.angular.z])
            data2save["time_stamp"][ii,0] = time.time() - time_start
            data2save["vel_forward_des"][ii,0] = msg_high_cmd.velocity[0]
            data2save["vel_yaw_des"][ii,0] = msg_high_cmd.yawSpeed
            ii += 1


        # Save data:
        if save_data:

            name_file_data = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
            name_file = "{0:s}_experiments_go1trajs.pickle".format(name_file_data)
            path2save_full = "{0:s}/{1:s}".format(path2save,name_file)
            rospy.loginfo("Saving data at {0:s} ...".format(path2save_full))
            file = open(path2save_full, 'wb')
            pickle.dump(data2save,file)
            file.close()
            rospy.loginfo("Done saving data!")
            rospy.loginfo("Ready to collect data; looping ...")
            save_data = False

        ros_loop.sleep()



