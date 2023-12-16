import os
import numpy as np
import time
import pdb
from real_robot_interface_go1 import GymEnvironmentRealGo1  # pytype: disable=import-error | amarco: # From motion_imitation/motion_imitation/robots/a1_robot.py

from unitree_legged_sdk_python_tools.utils.visualization_raisim  import VisualizeRaisim

def init():

    print("Communication level is set to LOW-level")
    print("WARNING: Make sure the robot is hung up")
    input("Press Enter to continue...")

    Njoints = 12
    Pgains = 5.*np.ones(Njoints)
    Dgains = 1.*np.ones(Njoints)

    print("Initializing Go1 Gym Env interface ...")
    gym_env_go1 = GymEnvironmentRealGo1()
    gym_env_go1.set_PD_gains(Pgains,Dgains)
    gym_env_go1.set_deltaT(0.002) # seconds
    gym_env_go1.set_action_std(1.0)
    gym_env_go1.set_action_mean(np.zeros(12))

    return gym_env_go1


def read_current_position(gym_env_go1):

    Nsteps = 100
    print("Reading current position for {0:d} steps ...".format(Nsteps))
    for ii in range(Nsteps):

        # Update observations:
        gym_env_go1.update_env_observation()

        time.sleep(gym_env_go1.get_deltaT())

    joint_pos_curr = np.zeros(12)
    gym_env_go1.get_joint_pos_curr(joint_pos_curr)

    return np.copy(joint_pos_curr)


def interpolation_linear(joint_pos_init,joint_pos_target,ind,Nsteps):

    alpha = min(ind / Nsteps,1)
    joint_pos_curr = joint_pos_init*(1.-alpha) + joint_pos_target*alpha

    return joint_pos_curr


def main():

    # Initialization:
    gym_env_go1 = init()

    # Start raisim server:
    use_raisim_visualization = True
    if use_raisim_visualization: visualize_raisim = VisualizeRaisim()

    # Read for a while and record the last position. This assumes that the robot is standing still or hung up
    joint_pos_init = read_current_position(gym_env_go1)

    print("Current position:",joint_pos_init)

    # Target position we want to reach:
    joint_pos_target = np.array([0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944])

    # Transition smoothly to a desired position:
    joint_pos_curr = np.zeros(12)
    Nsteps_total = 7000
    Nsteps_movement = 3000
    print("Target position:",joint_pos_target)
    print("Initializing movement ...")
    print("    Nsteps_total:",Nsteps_total)
    print("    Nsteps_movement:",Nsteps_movement)
    for ii in range(Nsteps_total):

        joint_pos_des = interpolation_linear(joint_pos_init,joint_pos_target,ind=ii,Nsteps=Nsteps_movement)

        # Send desired positions:
        gym_env_go1.step(joint_pos_des)

        gym_env_go1.get_joint_pos_curr(joint_pos_curr)

        if use_raisim_visualization: visualize_raisim.update_visualization(joint_pos_curr,joint_pos_des)

        time.sleep(gym_env_go1.get_deltaT())

    print("Done!")
    if use_raisim_visualization: visualize_raisim.server.killServer()


if __name__ == "__main__":

    main()


