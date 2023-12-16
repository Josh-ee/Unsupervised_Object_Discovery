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
import geometry_msgs.msg

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib

from utils.generate_vel_profile import get_velocity_profile_given_waypoints, generate_random_set_of_waypoints

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


cmd_vel_keyboard = geometry_msgs.msg.Twist()



def callback_vel_keyboard(msg_in):
    print("in callback keyboard")
    global cmd_vel_keyboard
    cmd_vel_keyboard = msg_in

def callback_go1_state(msg_in):
    # print("in callback")
    global msg_go1_state
    msg_go1_state = msg_in


def go_home_heading(msg_high_cmd,pub2high_cmd,ros_loop,yaw_des,Nsteps_timeout,deltaT):
    """
    yaw_des: given in Vicon coordinates, zero angle is at X axis; angle grows positive in anti-clockwise direction (from top view)
    heading_des: zero heading is at the Y axis; angle grows positive in anti-clockwise direction (from top view)

    We want the robot to be facing towards the tangent indicated by yaw_des. Hence, the desired heading is:
    heading_des = yaw_des - pi/2

    """

    global msg_go1_state

    rospy.loginfo("About to rotate the robot to the initial heading, yaw_des = {0:f} [rad] | yaw_cur = {1:f} [rad] | Will time out after {2:d} steps".format(yaw_des,msg_go1_state.orientation.z,Nsteps_timeout))
    rospy.loginfo("Press return to initiate the movement ...")
    input()

    heading_des = yaw_des - math.pi/2.

    tt = 0
    Kp_heading = 0.05
    tol_error = 0.2
    error = np.inf
    rospy.loginfo("Rotating the robot in place using a P controller ...")
    yawSpeed_des_prev = 0.0
    yawSpeed_des_send = 0.0
    while tt < Nsteps_timeout and abs(error) > tol_error:

        # Read current yaw angle:
        yaw_curr = msg_go1_state.orientation.z # w.r.t Vicon frame

        error = (heading_des - yaw_curr)
        yawSpeed_control = Kp_heading * error
        if abs(error) > math.pi:
            error_tilde = -np.sign(error)*(2.*math.pi - abs(error)) # Take shortest arc and flip the sign
            yawSpeed_control = Kp_heading * error_tilde

        # We control the yaw acceleration and integrate to get the desired velocity, which is what Unitree's interface requires
        yawSpeed_des_send = yawSpeed_control * deltaT + yawSpeed_des_prev

        # Update
        yawSpeed_des_prev = yawSpeed_des_send

        # P controller: rotate in place
        msg_high_cmd.yawSpeed = yawSpeed_des_send
        msg_high_cmd.velocity[0] = 0.0
        msg_high_cmd.velocity[1] = 0.0

        pub2high_cmd.publish(msg_high_cmd)

        ros_loop.sleep()

        tt += 1

    rospy.loginfo("Done!")
    if tt >= Nsteps_timeout: rospy.loginfo("Required tolerance not reached; timed out")
    if abs(error) < tol_error: rospy.loginfo("Reached desired angle within required tolerance: {0:f} < {1:f} [rad]".format(abs(error),tol_error))
    
def get_velocity_in_robot_frame(vel_world,theta_rot):

    theta_in = -theta_rot

    rot_mat = np.array([[np.cos(theta_in), -np.sin(theta_in)],[np.sin(theta_in), np.cos(theta_in)]])

    vel_body = rot_mat @ vel_world

    print(rot_mat)
    print(theta_rot)

    return vel_body


def pos_controller(des_state,curr_state,prev_vel,deltaT):
    """
    
    des_state: [x,y,th]
    curr_state: [x,y,th] (vicon readings)

    prev_vel: [vel_for,vel_lat,vel_yaw] -> velocity commands sent to the robot at the previous time step

    des_vel_for: scalar
    des_vec_yaw: scalar
    """

    Kp_heading = 0.05
    Kp_pos_for = 0.2
    Kp_pos_lat = 0.2


    """
    Position controller
    """
    des_xy_pos = des_state[0:2]
    cur_xy_pos = curr_state[0:2]
    error_pos = des_xy_pos - cur_xy_pos

    vel_in_world_frame = np.array([[Kp_pos_for * error_pos[1]],[Kp_pos_lat * error_pos[0]]])

    # theta_rot = des_state[2] - np.pi/2.
    theta_rot = curr_state[2]
    print("vel_in_world_frame:",vel_in_world_frame)
    vel_body = get_velocity_in_robot_frame(vel_in_world_frame,theta_rot)

    print("vel_body:",vel_body)

    # pdb.set_trace()

    forwad_vel_control = vel_body[0]
    lateral_vel_control = vel_body[1]


    # Hack
    prev_vel[0] = 0.0
    prev_vel[1] = 0.0
    prev_vel[2] = 0.0
    deltaT = 1.0


    # We control the forward acceleration:
    forward_vel_send = forwad_vel_control * deltaT + prev_vel[0]
    lateral_vel_send = lateral_vel_control * deltaT + prev_vel[1]

    # print("forward_vel_send:",forward_vel_send)
    # print("lateral_vel_send:",lateral_vel_send)



    """
    Heading controller
    """

    # See go_home_heading() for an explanation:
    des_heading = des_state[2]
    # des_heading = np.arctan2(err_xy[1], err_xy[0]) # [-pi,pi]
    # print("des_heading:",des_heading)
    des_yaw_pos = des_heading - np.pi/2.

    # Yaw controller:
    error_yaw = (des_yaw_pos - curr_state[2])
    if abs(error_yaw) > math.pi:
        error_yaw_tilde = -np.sign(error_yaw)*(2.*math.pi - abs(error_yaw)) # Take shortest arc and flip the sign
    else:
        error_yaw_tilde = error_yaw
    yawSpeed_control = Kp_heading * error_yaw_tilde

    # We control the yaw acceleration and integrate to get the desired velocity, which is what Unitree's interface requires
    yawSpeed_des_send = yawSpeed_control * deltaT + prev_vel[2]


    # Flip the sign; for some reason, Unitree's command works the other way around
    # lateral_vel_send = -lateral_vel_send


    # Update
    prev_vel[0] = forward_vel_send
    prev_vel[1] = lateral_vel_send 
    prev_vel[2] = yawSpeed_des_send

    return forward_vel_send, lateral_vel_send, yawSpeed_des_send, prev_vel

if __name__ == "__main__":

    """
    
    Generate control commands and publish them. The same control commands are published in two different formats:
    1) unitree_legged_msgs.msg.HighCmd() -> The cpp robot interface is subscribed to this one
    2) ood_gpssm_msgs.msg.Go1Control() -> The data collection node is subscribed tot his one


    """

    np.random.seed(1)

    rate_freq_send_commands = 120 # Hz
    # save_data_trajs_dict = dict(save=True,path2data="/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/trajs_generated/trajs.pickle")
    save_data_trajs_dict = None
    deltaT = 1./rate_freq_send_commands

    # time_tot = 15.0 # sec
    # pos_waypoints = np.array(   [[0.0,0.0],
    #                             [1.0,1.0],
    #                             [-1.0,2.0],
    #                             [0.0,3.5]])

    # time_tot = 15.0 # sec
    # pos_waypoints = np.array(   [[0.0,0.0],
    #                             [0.0,3.0]])

    # # S-traj-1
    # time_tot = 15.0 # sec
    # pos_waypoints = np.array(   [[0.0,0.0],
    #                             [1.0,1.0],
    #                             [-1.0,2.0],
    #                             [0.0,3.0]])

    # # S-traj-2
    # time_tot = 15.0 # sec
    # pos_waypoints = np.array(   [[0.0,0.0],
    #                             [-1.0,1.0],
    #                             [1.0,2.0],
    #                             [0.0,3.0]])

    # # Straight-traj-left_corner
    # time_tot = 7.5 # sec
    # pos_waypoints = np.array(   [[0.0,0.0],
    #                             [-1.5,3.0]])

    # # Straight-traj-right_corner
    # time_tot = 7.5 # sec
    # pos_waypoints = np.array(   [[0.0,0.0],
    #                             [1.0,3.0]])

    # state_tot, vel_tot = get_velocity_profile_given_waypoints(pos_waypoints,deltaT,time_tot,block_plot=False,plotting=True) # state_tot: [Nsteps_tot,2] || vel_tot: [Nsteps_tot,2]

    Nwaypoints = 10
    xlim = [-1.0,1.0]
    ylim = [0.0,3.0]
    rate_freq_send_commands_for_trajs = rate_freq_send_commands # Hz
    time_tot = Nwaypoints*7.0 # sec

    state_tot, vel_tot, pos_waypoints = generate_random_set_of_waypoints(Nwaypoints,xlim,ylim,rate_freq_send_commands_for_trajs,time_tot,block_plot=False,plotting=True)
    # state_tot: [Nsteps_tot,2] || vel_tot: [Nsteps_tot,2]


    rospy.loginfo("Press return to continue ...")
    # input()

    if np.any(abs(vel_tot[:,0]) > 1.0):
        rospy.logerr("Trajectory not accepted; limit of 1 m/s reached. Try a larger time horizon. This program will terminate. Press return to terminate.")
        input()
        raise ValueError("Invalid trajectory")

    rospy.init_node("node_walk_open_loop", anonymous=False)
    ros_loop = rospy.Rate(rate_freq_send_commands) # Hz

    # Subscribe to RobotState:
    topic_robot_state = "/experiments_gpssm_ood/robot_state"
    rospy.Subscriber(topic_robot_state, ood_gpssm_msgs.msg.Go1State, callback_go1_state)


    # Subscribe to Velocity commands from keyboard:
    topic_vel_keyboard = "/cmd_vel_keyboard"
    rospy.Subscriber(topic_vel_keyboard, geometry_msgs.msg.Twist, callback_vel_keyboard)


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

    # Go to initial heading:
    yaw_des = state_tot[0,2]
    timeout_go_home_heading = 15.0 # sec

    # # dbg:
    # yaw_des = -11.*math.pi/12 + math.pi/2.
    go_home_heading(msg_high_cmd,pub2high_cmd,ros_loop,yaw_des,Nsteps_timeout=int(timeout_go_home_heading*rate_freq_send_commands),deltaT=deltaT)

    # pdb.set_trace()


    Nsteps = vel_tot.shape[0]
    rospy.loginfo("Velocity profile will be published at {0:d} Hz for {1:2.2f} seconds ({2:d} steps)".format(rate_freq_send_commands,float(time_tot),Nsteps))
    if collect_data: rospy.loginfo("Data will be automatically collected and saved ...")
    rospy.loginfo("Ready to send the velocity profile to the robot; press return to continue ...")
    # input()

    # Activate data collection here:
    if collect_data: 
        time_pause = 2
        rospy.loginfo("Starting data collection! Pausing for {0:d} sec, in order for the message to propagate ...".format(time_pause))
        msg_data_collection.start = True
        msg_data_collection.stop = False
        pub_data_collection_triggers.publish(msg_data_collection)
        time.sleep(time_pause) # Wait a bit for the message to propagate

    rospy.loginfo("Starting loop now!")
    tt = 0
    curr_state = np.zeros(3)
    des_state = np.zeros(3)
    prev_vel = np.zeros(3)
    # Nsteps = 10000000
    while tt < Nsteps:

        # vel_des = vel_tot[tt,0]
        # vel_cur = np.sqrt(msg_go1_state.twist.linear.x**2 + msg_go1_state.twist.linear.y**2)
        # vel_send = vel_des + 1.0*(vel_des-vel_cur)

        # # msg_high_cmd.velocity[0] = vel_send
        # msg_high_cmd.velocity[0] = vel_tot[tt,0] # desired linear velocity || vel_tot: [Nsteps_tot,2]
        # msg_high_cmd.yawSpeed = vel_tot[tt,1] # desired angular velocity || vel_tot: [Nsteps_tot,2]


        curr_state[0] = msg_go1_state.position.x
        curr_state[1] = msg_go1_state.position.y
        curr_state[2] = msg_go1_state.orientation.z
        
        # des_state[:] = state_tot[tt,:]

        # # dbg:
        # des_state[0] = 0.0
        # des_state[1] = 3.0
        # des_state[2] = math.pi/2.

        # # Position control:
        # forward_vel_send, lateral_vel_send, yawSpeed_des_send, prev_vel = pos_controller(des_state,curr_state,prev_vel,deltaT)
        # # des_vel_vec, des_vec_yaw = pos_controller(des_state,curr_state)
        # msg_high_cmd.velocity[0] = forward_vel_send # desired linear forward velocity
        # msg_high_cmd.velocity[1] = lateral_vel_send # desired linear lateral velocity
        # msg_high_cmd.yawSpeed = yawSpeed_des_send # desired angular velocity

        # # DBG:
        # msg_high_cmd.velocity[0] = 0.0 # desired linear forward velocity
        # msg_high_cmd.velocity[1] = 0.0 # desired linear lateral velocity
        # msg_high_cmd.yawSpeed = 0.0 # desired angular velocity


        # # # DBG:
        # msg_high_cmd.velocity[0] = cmd_vel_keyboard.linear.y
        # msg_high_cmd.velocity[1] = cmd_vel_keyboard.linear.x
        # msg_high_cmd.yawSpeed = cmd_vel_keyboard.angular.z


        msg_high_cmd.velocity[0] = vel_tot[tt,0]
        msg_high_cmd.velocity[1] = 0.0
        msg_high_cmd.yawSpeed = vel_tot[tt,1]


        # print("cmd_vel_keyboard.linear.y:",cmd_vel_keyboard.linear.y)

        msg_high_cmd.levelFlag = HIGHLEVEL
        msg_high_cmd.mode = 2
        msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
        msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....

        pub2high_cmd.publish(msg_high_cmd)

        ros_loop.sleep()

        tt += 1

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

