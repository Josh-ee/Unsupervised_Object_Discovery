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
import ood_gpssm_msgs.msg
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib

from utils.generate_vel_profile import get_velocity_profile_given_waypoints, generate_random_set_of_waypoints, generate_waypoints_in_circle

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)


HIGHLEVEL = 0x00

msg_go1_state = ood_gpssm_msgs.msg.Go1State()


def callback_go1_state(msg_in):
    # print("in callback")
    global msg_go1_state
    msg_go1_state = msg_in


def go2next_waypoint(waypoint_new,msg_high_cmd,pub2high_cmd,ros_loop,Nsteps_timeout,deltaT):
    """
    yaw_des: given in Vicon coordinates, zero angle is at X axis; angle grows positive in anti-clockwise direction (from top view)
    heading_des: zero heading is at the Y axis; angle grows positive in anti-clockwise direction (from top view)

    We want the robot to be facing towards the tangent indicated by yaw_des. Hence, the desired heading is:
    heading_des = yaw_des - pi/2

    """

    global msg_go1_state

    # rospy.loginfo("About to rotate the robot to the initial heading, yaw_des = {0:f} [rad] | yaw_cur = {1:f} [rad] | Will time out after {2:d} steps".format(yaw_des,msg_go1_state.orientation.z,Nsteps_timeout))
    rospy.loginfo("Going to waypoint {0:s} | Press return to initiate the movement ...".format(str(waypoint_new)))
    # input()

    pos_xy_cur = np.zeros(2)
    
    tt = 0
    Kp_heading = 0.6
    tol_error = 0.15
    error_pos = np.inf
    rospy.loginfo("Moving ...")
    yawSpeed_des_prev = 0.0
    yawSpeed_des_send = 0.0


    # DBG:
    deltaT = 1.0


    while tt < Nsteps_timeout and abs(error_pos) > tol_error:

        # Update position:
        pos_xy_cur[0] = msg_go1_state.position.x
        pos_xy_cur[1] = msg_go1_state.position.y

        # Update desired yaw:
        direction = waypoint_new - pos_xy_cur
        yaw_des = np.arctan2(direction[1], direction[0]) # [-pi,pi]
        heading_des = yaw_des - math.pi/2.

        # Are we there yet?
        error_pos = np.sqrt(np.sum(direction**2))
        if tt % 120 == 0: rospy.loginfo("error_pos: {0:2.2f}".format(error_pos))

        # Read current yaw angle:
        yaw_curr = msg_go1_state.orientation.z # w.r.t Vicon frame

        error_yaw = (heading_des - yaw_curr)
        yawSpeed_control = Kp_heading * error_yaw
        if abs(error_yaw) > math.pi:
            error_yaw_tilde = -np.sign(error_yaw)*(2.*math.pi - abs(error_yaw)) # Take shortest arc and flip the sign
            yawSpeed_control = Kp_heading * error_yaw_tilde

        # We control the yaw acceleration and integrate to get the desired velocity, which is what Unitree's interface requires
        yawSpeed_des_send = yawSpeed_control * deltaT + yawSpeed_des_prev

        # Update
        # yawSpeed_des_prev = yawSpeed_des_send
        yawSpeed_des_prev = 0.0

        # Control commands:
        msg_high_cmd.yawSpeed = yawSpeed_des_send
        msg_high_cmd.velocity[0] = 0.1 + 0.3*error_pos / np.sqrt(3**2 + 4**2)
        msg_high_cmd.velocity[1] = 0.0

        msg_high_cmd.levelFlag = HIGHLEVEL
        msg_high_cmd.mode = 2
        msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
        msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....



        pub2high_cmd.publish(msg_high_cmd)

        ros_loop.sleep()

        tt += 1

    rospy.loginfo("Done!")
    if tt >= Nsteps_timeout: rospy.loginfo("Required tolerance not reached; timed out")
    if abs(error_pos) < tol_error: rospy.loginfo("Reached desired angle within required tolerance: {0:f} < {1:f} [rad]".format(abs(error_pos),tol_error))
    
    return


if __name__ == "__main__":

    """
    
    Generate control commands and publish them. The same control commands are published in two different formats:
    1) unitree_legged_msgs.msg.HighCmd() -> The cpp robot interface is subscribed to this one
    2) ood_gpssm_msgs.msg.Go1Control() -> The data collection node is subscribed tot his one


    """

    np.random.seed(9)

    rate_freq_send_commands = 120 # Hz
    # save_data_trajs_dict = dict(save=True,path2data="/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/trajs_generated/trajs.pickle")
    save_data_trajs_dict = None
    deltaT = 1./rate_freq_send_commands

 
    Nwaypoints = 20
    xlim = [-2.0,1.2]
    ylim = [0.0,5.5]
    rate_freq_send_commands_for_trajs = rate_freq_send_commands # Hz
    time_tot = Nwaypoints*10.0 # sec

    # _, _, pos_waypoints = generate_random_set_of_waypoints(Nwaypoints,xlim,ylim,rate_freq_send_commands_for_trajs,time_tot,block_plot=False,plotting=True)

    pos_waypoints = generate_waypoints_in_circle(Nwaypoints,xlim,ylim,rate_freq_send_commands,time_tot,block_plot=False,plotting=True)


    print("time_tot: {0:f}".format(time_tot))
    print("pos_waypoints:\n{0:s}".format(str(pos_waypoints)))

    rospy.init_node("node_walk_open_loop", anonymous=False)
    ros_loop = rospy.Rate(rate_freq_send_commands) # Hz

    # Subscribe to RobotState:
    topic_robot_state = "/experiments_gpssm_ood/robot_state"
    rospy.Subscriber(topic_robot_state, ood_gpssm_msgs.msg.Go1State, callback_go1_state)


    # Publish control command:
    topic_high_cmd = "/high_cmd_to_robot"
    pub2high_cmd = rospy.Publisher(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, queue_size=10)


    # Data collection triggers:
    topic_data_collection_triggers = "/experiments_gpssm_ood/data_collection_triggers"
    msg_data_collection = ood_gpssm_msgs.msg.DataCollection()
    pub_data_collection_triggers = rospy.Publisher(topic_data_collection_triggers, ood_gpssm_msgs.msg.DataCollection, queue_size=1)


    # Message containing walking mode:
    msg_high_cmd = unitree_legged_msgs.msg.HighCmd()
    msg_high_cmd.levelFlag = HIGHLEVEL
    msg_high_cmd.mode = 2
    msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed in body frame
    msg_high_cmd.velocity[1] = 0.0 # [-1,1] # (unit: m/s), sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
    msg_high_cmd.yawSpeed = 0.0

    collect_data = True

    if collect_data: rospy.loginfo("Data will be automatically collected and saved ...")
    rospy.loginfo("Ready to send the velocity profile to the robot; press return to continue ...")
    input()

    # Activate data collection here:
    if collect_data: 
        time_pause = 2
        rospy.loginfo("Starting data collection! Pausing for {0:d} sec, in order for the message to propagate ...".format(time_pause))
        msg_data_collection.start = True
        msg_data_collection.stop = False
        pub_data_collection_triggers.publish(msg_data_collection)
        time.sleep(time_pause) # Wait a bit for the message to propagate

    rospy.loginfo("Starting loop now!")
    ww = 0
    curr_state = np.zeros(3)
    Nwaypoints = pos_waypoints.shape[0]
    timeout = 5.0 # sec
    Nsteps_timeout = rate_freq_send_commands*timeout
    Nloops = 5
    # while True:
    for _ in range(Nloops*Nwaypoints):

        curr_state[0] = msg_go1_state.position.x
        curr_state[1] = msg_go1_state.position.y
        curr_state[2] = msg_go1_state.orientation.z
        
        go2next_waypoint(pos_waypoints[ww,:],msg_high_cmd,pub2high_cmd,ros_loop,Nsteps_timeout,deltaT)

        ww = (ww+1) % Nwaypoints



    # Reset to mode 0:
    rospy.loginfo("Trajectory completed!")
    rospy.loginfo("Back to standing still....")
    msg_high_cmd.mode = 0 # TODO: Shouldn't this be 0?
    msg_high_cmd.gaitType = 0 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed, sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
    tt = 0
    while tt < 100:
        pub2high_cmd.publish(msg_high_cmd)
        ros_loop.sleep()
        tt += 1


    # Activate data collection here:
    if collect_data: 
        rospy.loginfo("Stopping data collection!")
        msg_data_collection.stop = True
        msg_data_collection.start = False
        pub_data_collection_triggers.publish(msg_data_collection)

    rospy.loginfo("Exiting; node finished!")

