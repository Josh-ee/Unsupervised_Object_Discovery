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
import nav_msgs.msg
import geometry_msgs.msg

msg_go1_state = ood_gpssm_msgs.msg.Go1State()
msg_high_cmd = unitree_legged_msgs.msg.HighCmd()

msg_state_predictions = ood_gpssm_msgs.msg.Go1StatePredictions()

msg_predictions_one = nav_msgs.msg.Path()



def predict_with_model_fake(state_in,control_in,Nrollouts):
    """
    state_in: [1,3]
    control_in: [Nhor-1,2]

    state_out: [Nrollouts,Nhor,3] (NOTE: the first state is equal to state_in)
    """

    t_steps = np.arange(control_in.shape[0]+1) / control_in.shape[0]
    phase_vec = np.ones((len(t_steps),1))*np.array([[0.0, np.pi/4.0, np.pi/2.0]])
    freq_plus_phase = np.pi*2*np.reshape(t_steps,(-1,1)) + phase_vec
    
    state_out_base_states = np.sin(freq_plus_phase)

    state_out_base_rollouts = np.reshape(state_out_base_states,(1,control_in.shape[0]+1,3)) + 0.01*np.random.randn(Nrollouts,control_in.shape[0]+1,3)
    # state_out_base_rollouts: [Nrollouts,Nhor,dim_out]

    msg_predictions_one.header.frame_id = "base_link"
    msg_predictions_one.header.stamp = rospy.Time.now()
    msg_predictions_one.header.seq = state_out_base_rollouts.shape[1]+1

    # Following in one rollout for now:
    rr = 0
    msg_predictions_one.poses = []
    for tt in range(state_out_base_rollouts.shape[1]):
        msg_new = geometry_msgs.msg.PoseStamped()
        msg_new.header.frame_id = "roll{0:d}_tt{1:d}".format(rr,tt)
        msg_new.header.stamp = rospy.Time.now()
        msg_new.header.seq = tt + 1
        msg_new.pose.position.x = state_out_base_rollouts[rr,tt,0]
        msg_new.pose.position.y = state_out_base_rollouts[rr,tt,1]
        msg_new.pose.position.z = 0.28
        msg_predictions_one.poses += [msg_new]
    # pdb.set_trace()

    return state_out_base_rollouts, msg_predictions_one


def OoD_detection(observations_hist,x_hindcast):
    return 100*np.random.rand(1)

def callback_go1_state(msg_in):
    # print("in calback")
    global msg_go1_state
    msg_go1_state = msg_in

def callback_cmd_high(msg_in):
    global msg_high_cmd
    msg_high_cmd = msg_in



if __name__ == "__main__":

    """
    
    Listen to robot state and use GPSSM to predict future states. Compare predictions with observations online using the ELBO loss
    """

    np.random.seed(1)

    rate_freq_send_commands = 120 # Hz
    deltaT = 1./rate_freq_send_commands

    rospy.init_node("node_ood_detection", anonymous=False)
    ros_loop = rospy.Rate(rate_freq_send_commands) # Hz

    # Subscribe to RobotState:
    topic_robot_state = "/experiments_gpssm_ood/robot_state"
    rospy.Subscriber(topic_robot_state, ood_gpssm_msgs.msg.Go1State, callback_go1_state)

    # Subscribe to Control:
    topic_high_cmd = "/high_cmd_to_robot"
    rospy.Subscriber(topic_high_cmd, unitree_legged_msgs.msg.HighCmd, callback_cmd_high)


    # Publish control command:
    topic_robot_state_predictions = "/experiments_gpssm_ood/robot_state_predictions"
    msg_state_predictions = ood_gpssm_msgs.msg.Go1StatePredictions()
    pub_state_predictions = rospy.Publisher(topic_robot_state_predictions, ood_gpssm_msgs.msg.Go1StatePredictions, queue_size=10)


    topic_robot_state_predictions_nav = "/experiments_gpssm_ood/robot_state_predictions_nav"
    pub_state_predictions_nav = rospy.Publisher(topic_robot_state_predictions_nav, nav_msgs.msg.Path, queue_size=1)



    rospy.loginfo("Ready to start; press return to continue ...")
    input()

    rospy.loginfo("Starting loop now!")
    dim_x = 3
    dim_u = 2
    Nhor = 10
    Nrollouts = 15
    observations_hist = np.zeros((Nhor,dim_x)) # History of observations (FIFO sequence)
    control_input_hist = np.zeros((Nhor-1,dim_u)) # History of control inputs (FIFO sequence)
    state_in = np.zeros(dim_x+dim_u)
    tt = 0
    first_time = True
    msg_state_predictions.Nstates = dim_x
    msg_state_predictions.Nhorizon = Nhor
    msg_state_predictions.Nrollouts = Nrollouts
    while True:

        # Displace historical observations one backwards in time and leave room for the current one:
        observations_hist[0:-1,:] = observations_hist[1::,:]
        observations_hist[-1,0] = msg_go1_state.position.x
        observations_hist[-1,1] = msg_go1_state.position.y
        observations_hist[-1,2] = msg_go1_state.orientation.z

        # Displace historical control sequence one backwards in time and leave room for the current one:
        control_input_hist[0:-1,:] = control_input_hist[1::,:]
        control_input_hist[-1,0] = msg_high_cmd.velocity[0]
        control_input_hist[-1,1] = msg_high_cmd.yawSpeed

        # Use model to predict past states up to the current one x(t) by starting at x(t-Nhor) and using the control sequence u(t-Nhor),u(t-Nhor+1),...,u(t-1)
        x_oldest = observations_hist[0,:] # The oldest observation is the first element in the history (FIFO sequence)
        u_seq = control_input_hist # We pass the entire sequence
        x_hindcast, msg_predictions_one = predict_with_model_fake(state_in=x_oldest,control_in=u_seq,Nrollouts=Nrollouts) # [Nrollouts,Nhor,3]; the element x_hindcast[:,0,:] is assigned to x_oldest
        loss_val_OoD = OoD_detection(observations_hist,x_hindcast)

        msg_state_predictions.predictions = np.reshape(x_hindcast,(-1))
        pub_state_predictions.publish(msg_state_predictions)
        
        # pdb.set_trace()
        # print("msg_predictions_one.poses[0].pose.position.x: {0:f}".format(msg_predictions_one.poses[0].pose.position.x))
        print("msg_predictions_one.poses[1].pose.position.x: {0:f}".format(msg_predictions_one.poses[1].pose.position.x))
        # print("msg_predictions_one.poses[2].pose.position.x: {0:f}".format(msg_predictions_one.poses[2].pose.position.x))

        pub_state_predictions_nav.publish(msg_predictions_one)

        # # Selector index: it loops through the batch dimension in predictions_batch_hist:
        # tt_new = (tt + 1) % Nhor

        ros_loop.sleep()


