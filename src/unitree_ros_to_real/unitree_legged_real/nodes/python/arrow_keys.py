#!/usr/bin/env python
import os
import numpy as np
import pdb
import time
import rospy

from pynput import keyboard  
import threading

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')

HIGHLEVEL = 0x00



def on_press(key):
    global msg_high_cmd
    # This function will be called whenever a key is pressed
    if hasattr(key, 'char'):  # Check if the key has a printable representation
        if key.char == 'w':
            msg_high_cmd.velocity[0] = 0.5  # Forward
        elif key.char == 's':
            msg_high_cmd.velocity[0] = -0.5  # Backward
        elif key.char == 'a':
            msg_high_cmd.yawSpeed = 0.7  # Turn left
        elif key.char == 'd':
            msg_high_cmd.yawSpeed = -0.7  # Turn right
        elif key.char == 'q':
            msg_high_cmd.velocity[1] = 0.2  # strafe left
        elif key.char == 'e':
            msg_high_cmd.velocity[1] = -0.2  # strafe right
        elif key.char == 'c':
            msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....

def on_release(key):
    global msg_high_cmd
    # This function will be called whenever a key is released
    msg_high_cmd.velocity[0] = 0.0 # Stop forward/backward motion
    msg_high_cmd.velocity[1] = 0.0 # Stop left/right strafe
    msg_high_cmd.yawSpeed = 0.0  # Stop turning
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....




def walk_with_fixed_vel(msg_high_cmd,pub2high_cmd,ros_loop,Nsteps_timeout):
    """
    yaw_des: given in Vicon coordinates, zero angle is at X axis; angle grows positive in anti-clockwise direction (from top view)
    heading_des: zero heading is at the Y axis; angle grows positive in anti-clockwise direction (from top view)

    We want the robot to be facing towards the tangent indicated by yaw_des. Hence, the desired heading is:
    heading_des = yaw_des - pi/2

    """

    # rospy.loginfo("About to rotate the robot to the initial heading, yaw_des = {0:f} [rad] | yaw_cur = {1:f} [rad] | Will time out after {2:d} steps".format(yaw_des,msg_go1_state.orientation.z,Nsteps_timeout))
    # rospy.loginfo("Going to waypoint {0:s} | Press return to initiate the movement ...".format(str(waypoint_new)))
    # input()

    tt = 0
    rospy.loginfo("Moving ...")

    while tt < Nsteps_timeout:

        msg_high_cmd.levelFlag = HIGHLEVEL
        msg_high_cmd.mode = 2
        msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
        msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....

        pub2high_cmd.publish(msg_high_cmd)

        ros_loop.sleep()

        tt += 1

    rospy.loginfo("Done!")    
    return


if __name__ == "__main__":

    """
    
    Send a high-level velocity command for the robot to walk straight during a short period of time
    """

    rate_freq_send_commands = 120 # Hz
 
    time_tot = 15.0 # Stops the connection after 15 seconds

    print("time_tot: {0:f}".format(time_tot))

    rospy.init_node("node_vel_control", anonymous=False)
    ros_loop = rospy.Rate(rate_freq_send_commands) # Hz

    # Publish control command:
    topic_high_cmd = "/high_cmd_to_robot"
    pub2high_cmd = rospy.Publisher(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, queue_size=10)

    # Message containing walking mode:
    msg_high_cmd = unitree_legged_msgs.msg.HighCmd()
    msg_high_cmd.levelFlag = HIGHLEVEL
    msg_high_cmd.mode = 2
    msg_high_cmd.gaitType = 1 # 0.idle  1.trot  2.trot running  3.climb stair
    msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed in body frame
    msg_high_cmd.velocity[1] = 0.0 # [-1,1] # (unit: m/s), sideSpeed in body frame
    msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
    msg_high_cmd.yawSpeed = 0.0

    # print("Sleeping for 1 second ...")
    # time.sleep(1.0)
        
    rospy.loginfo("Ready to start the movement. The robot will walk forward at a low pace for {0:2.1f} seconds".format(time_tot))
    rospy.loginfo("Make sure there are no obstacles in front of the robot")
    rospy.loginfo("Ready to start the movement. Press enter to continue ...")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    listener_thread = threading.Thread(target=listener.join)
    listener_thread.start()

    try:
        input()
        Nsteps_timeout = int(time_tot * rate_freq_send_commands)
        walk_with_fixed_vel(msg_high_cmd, pub2high_cmd, ros_loop, Nsteps_timeout)
    finally:
        listener.stop()
        listener_thread.join()


    # input()

    # Nsteps_timeout = int(time_tot * rate_freq_send_commands)
    # walk_with_fixed_vel(msg_high_cmd,pub2high_cmd,ros_loop,Nsteps_timeout)

    # # Reset to mode 0:
    # rospy.loginfo("Movement completed!")
    # rospy.loginfo("Back to standing still....")
    # msg_high_cmd.mode = 0 # TODO: Shouldn't this be 0?
    # msg_high_cmd.gaitType = 0 # 0.idle  1.trot  2.trot running  3.climb stair
    # msg_high_cmd.velocity[0] = 0.0 # [-1,1] # (unit: m/s), forwardSpeed, sideSpeed in body frame
    # msg_high_cmd.bodyHeight = 0.0 # # (unit: m) -> WARNING: This is NOT an absolute position w.r.t the ground, but rather w.r.t the current height....
    # tt = 0
    # while tt < 120:
    #     pub2high_cmd.publish(msg_high_cmd)
    #     ros_loop.sleep()
    #     tt += 1

    rospy.loginfo("Exiting; node finished!")
