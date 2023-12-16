import os
import numpy as np
import time
import pdb
from real_robot_interface_go1 import RealRobotInterfaceGo1  # pytype: disable=import-error | amarco: # From motion_imitation/motion_imitation/robots/a1_robot.py

from unitree_legged_sdk_python_tools.utils.visualization_raisim  import VisualizeRaisim

def init():

    print("Communication level is set to LOW-level")
    print("WARNING: Make sure the robot is hung up")
    input("Press Enter to continue...")

    deltaT = 0.002; # seconds

    Njoints = 12
    Pgains = 5.*np.ones(Njoints)
    Dgains = 1.*np.ones(Njoints)

    print("Initializing Go1 interface ...")
    interface_real_go1 = RealRobotInterfaceGo1()
    interface_real_go1.set_PD_gains(Pgains,Dgains)
    interface_real_go1.set_deltaT(deltaT)

    return interface_real_go1


def read_current_position(interface_real_go1):

    Nsteps = 100
    for ii in range(Nsteps):

        # Update observations:
        interface_real_go1.collect_observations()
        interface_real_go1.update_all_observations()

        time.sleep(interface_real_go1.get_deltaT())

    joint_pos_curr = np.zeros(12)
    interface_real_go1.get_joint_pos_curr(joint_pos_curr)

    return np.copy(joint_pos_curr)


def interpolation_linear(joint_pos_init,joint_pos_target,ind,Nsteps):

    alpha = min(ind / Nsteps,1)
    joint_pos_curr = joint_pos_init*(1.-alpha) + joint_pos_target*alpha

    return joint_pos_curr


def main():

    # Initialization:
    interface_real_go1 = init()

    # Start raisim server:
    use_raisim_visualization = True
    if use_raisim_visualization: visualize_raisim = VisualizeRaisim()

    # Read for a while and record the last position. This assumes that the robot is standing still or hung up
    joint_pos_init = read_current_position(interface_real_go1)

    # Target position we want to reach:
    joint_pos_target = np.array([0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944])

    time_wait = 1.0
    print("Waiting {0:f} before starting the loop ...".format(time_wait))
    time.sleep(time_wait)

    # Transition smoothly to a desired position:
    joint_pos_curr = np.zeros(12)
    Nsteps_total = 7000
    Nsteps_transition = 3000
    for ii in range(Nsteps_total):

        joint_pos_des = interpolation_linear(joint_pos_init,joint_pos_target,ind=ii,Nsteps=Nsteps_transition)

        # Send desired positions:
        interface_real_go1.send_desired_position(joint_pos_des)

        # Update observations:
        interface_real_go1.collect_observations()
        interface_real_go1.update_all_observations()

        interface_real_go1.get_joint_pos_curr(joint_pos_curr)
        print(joint_pos_curr)

        if use_raisim_visualization: visualize_raisim.update_visualization(joint_pos_curr,joint_pos_des)

        time.sleep(interface_real_go1.get_deltaT())

    if use_raisim_visualization: visualize_raisim.server.killServer()


if __name__ == "__main__":

    main()


