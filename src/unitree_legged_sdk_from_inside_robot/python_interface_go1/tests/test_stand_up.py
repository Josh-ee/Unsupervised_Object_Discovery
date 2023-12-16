import os
import numpy as np
import time
import pdb
from real_robot_interface_go1 import RealRobotInterfaceGo1  # pytype: disable=import-error | amarco: # From motion_imitation/motion_imitation/robots/a1_robot.py


def main():

    Njoints = 12
    Pgains = 5.*np.ones(Njoints)
    Dgains = 1.*np.ones(Njoints)

    # pdb.set_trace()

    interface_real_go1 = RealRobotInterfaceGo1()
    interface_real_go1.set_PD_gains(Pgains,Dgains)

    Nsteps = 5
    joint_pos_des = 0.1*np.ones(Njoints,dtype=np.float64)
    joint_vel_des = 0.1*np.ones(Njoints,dtype=np.float64)
    joint_torque_des = 0.1*np.ones(Njoints,dtype=np.float64)
    for ii in range(Nsteps):

        interface_real_go1.send_command(joint_pos_des,joint_vel_des,joint_torque_des)


        interface_real_go1.collect_observations()


        interface_real_go1.update_joint_pos_curr()

        interface_real_go1.update_joint_vel_curr()


        time.sleep(0.1)


    interface_real_go1.main()

    print("In python, just exited main, now sleeping foreever")


    time.sleep(100000)


if __name__ == "__main__":

    main()


