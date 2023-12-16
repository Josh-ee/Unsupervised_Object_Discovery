import os
import numpy as np
import time
import pdb
from real_robot_interface_go1_highlevel import RealRobotInterfaceGo1HighLevel  # pytype: disable=import-error | amarco: # From motion_imitation/motion_imitation/robots/a1_robot.py

# from unitree_legged_sdk_python_tools.utils.visualization_raisim  import VisualizeRaisim

def init():

    print("Communication level is set to HIGG-level")
    print("WARNING: Make sure the robot is standing")
    input("Press Enter to continue...")

    deltaT = 0.002; # seconds

    print("Initializing Go1 interface ...")
    interface_real_go1 = RealRobotInterfaceGo1HighLevel()
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


def main():

    pdb.set_trace()

    # Initialization:
    interface_real_go1 = init()

    # # Start raisim server:
    # use_raisim_visualization = True
    # if use_raisim_visualization: visualize_raisim = VisualizeRaisim()

    # Read for a while and record the last position. This assumes that the robot is standing still or hung up
    joint_pos_curr = read_current_position(interface_real_go1)
    print("joint_pos_curr: "+str(joint_pos_curr))

    time_wait = 1.0
    print("Waiting {0:f} before starting the loop ...".format(time_wait))
    time.sleep(time_wait)

    # Transition smoothly to a desired position:
    Nsteps_total = 7000
    interface_real_go1.send_desired_body_height(bodyHeight=0.1)
    for ii in range(Nsteps_total):

        # Send desired velocity commands:
        interface_real_go1.send_linear_and_angular_velocity_commands(forwardSpeed=0.2,sideSpeed=0.0,yawSpeed=0.0)

        # Update observations:
        interface_real_go1.get_observations()

        interface_real_go1.get_joint_pos_curr(joint_pos_curr)
        print("joint_pos_curr: "+str(joint_pos_curr))

        if use_raisim_visualization: visualize_raisim.update_visualization(joint_pos_curr,joint_pos_des)

        time.sleep(interface_real_go1.get_deltaT())

    if use_raisim_visualization: visualize_raisim.server.killServer()


if __name__ == "__main__":

    main()


