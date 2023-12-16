#!/usr/bin/env python
import os
import numpy as np
import pdb
import time
from datetime import datetime
import pickle
import math

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib import cm
import matplotlib

from utils.generate_vel_profile import generate_random_set_of_waypoints, generate_waypoints_in_circle

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)


# export PYTHONPATH=/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python

def test_random_set_waypoints_data_collection():

    Nwaypoints = 20
    xlim = [-1.5,1.5]
    ylim = [0.0,4.0]
    rate_freq_send_commands = 120 # Hz
    time_tot = Nwaypoints*5.0 # sec

    generate_random_set_of_waypoints(Nwaypoints,xlim,ylim,rate_freq_send_commands,time_tot,block_plot=True,plotting=True)


def test_waypoints_in_circle_ood_detection():

    Nwaypoints = 20
    xlim = [-2.0,1.2]
    ylim = [0.0,5.5]
    rate_freq_send_commands = 120 # Hz
    time_tot = Nwaypoints*5.0 # sec

    generate_waypoints_in_circle(Nwaypoints,xlim,ylim,rate_freq_send_commands,time_tot,block_plot=True,plotting=True)




if __name__ == "__main__":

    """
    
    export PYTHONPATH=$PYTHONPATH:/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python
    Run as  python test/test_generate_trajectories.py
    """
    np.random.seed(1)


    # test_random_set_waypoints_data_collection()

    test_waypoints_in_circle_ood_detection()

