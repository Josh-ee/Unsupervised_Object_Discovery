import os
import numpy as np
import time
import pdb

import unitree_legged_msgs.msg # Located at /home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/devel/lib/python3/dist-packages (this path is added automatically to the PYTHONPATH after doing 'source devel/setup.bash')
from unitree_legged_sdk_python_tools.utils.visualization_raisim  import VisualizeRaisim
import rospy

# Data logging:
import time

from datetime import datetime

from .go_to import GoToJointPosition

class 


class RaisimGymEnvGo1:

    def __init__(self,path2load):
        """
        Parse here the config file used in raisim, located in path2load
        """

        self.rot_mat_body_orientation = np.zeros((3,3))
        self.body_lin_velocity_local = np.zeros(3)
        self.body_lin_velocity = np.zeros(3)


        self.body_lin_accel_curr = self.go_to.msg_low_state.imu.accelerometer
        self.body_lin_accel_prev = self.go_to.msg_low_state.imu.accelerometer

        self.deltaT = 0.01 # TODO: pass this as input variable

        self.grav_vec = np.zeros((3,1))
        self.grav_vec[2,0] = -1

        self.joint_pos_cur = np.zeros(12)
        self.joint_vel_cur = np.zeros(12)

        self.joint_position_init = np.array([0.0136, 0.7304, -1.4505,  # FR [hip lateral, hip, knee]
                                              -0.0118, 0.7317, -1.4437, # FL [hip lateral, hip, knee]
                                              0.0105, 0.6590, -1.3903,  # RR [hip lateral, hip, knee]
                                              -0.0102, 0.6563, -1.3944]); # RL [hip lateral, hip, knee]
        self.action_clipped_vec = np.zeros(4)

        self.action_lim = 10.0


    def update_rotation_matrix_from_quaternion(self):

        # // Transform to rotation matrix:
        # raisim::quatToRotMat(this->base_orientation_quat_curr, this->body_rotation_matrix);

        # inline void quatToRotMat(const Vec<4> &q, Mat<3, 3> &R) {
        #   R[0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        #   R[1] = 2 * q[0] * q[3] + 2 * q[1] * q[2];
        #   R[2] = 2 * q[1] * q[3] - 2 * q[0] * q[2];

        #   R[3] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
        #   R[4] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
        #   R[5] = 2 * q[0] * q[1] + 2 * q[2] * q[3];

        #   R[6] = 2 * q[0] * q[2] + 2 * q[1] * q[3];
        #   R[7] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
        #   R[8] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        # }

        R = self.rot_mat_body_orientation
        q = self.body_orientation_quaternion

        R[0,0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        R[0,1] = 2 * q[0] * q[3] + 2 * q[1] * q[2];
        R[0,2] = 2 * q[1] * q[3] - 2 * q[0] * q[2];

        R[1,0] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
        R[1,1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
        R[1,2] = 2 * q[0] * q[1] + 2 * q[2] * q[3];

        R[2,0] = 2 * q[0] * q[2] + 2 * q[1] * q[3];
        R[2,1] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
        R[2,2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

        self.rot_mat_body_orientation = R # This is not necessary
        

    def update_body_linear_and_angular_velocity(self):

        self.update_rotation_matrix_from_quaternion()

        # Get body angular velocity:
        self.body_ang_velocity = self.rot_mat_body_orientation.T @ self.go_to.msg_low_state.imu.gyroscope

        # Get body linear velocity
        self.body_lin_accel_curr = self.go_to.msg_low_state.imu.accelerometer
        self.body_lin_velocity_local = self.body_lin_velocity_local + 0.5*(self.body_lin_accel_curr + self.body_lin_accel_prev)*self.deltaT
        self.body_lin_accel_prev = self.body_lin_accel_curr

        self.body_lin_velocity = self.rot_mat_body_orientation.T @ self.body_lin_velocity_local


    def update_projected_gravity(self):
        self.update_rotation_matrix_from_quaternion()
        self.gravity_vec_projected = self.rot_mat_body_orientation.T @ self.grav_vec

    def update_joint_pos_and_vel(self):

        for ii in range(12):
            self.joint_pos_cur[ii] = self.go_to.msg_low_state.MotorState[ii].q
            self.joint_vel_cur[ii] = self.go_to.msg_low_state.MotorState[ii].dq


    # def update_actions(self,actions):
    #     self.action_clipped_vec = np.cip(actions,-self.action_lim,self.action_lim)



    def observe(self):

        # obDouble_ <<   this->bodyLinearVel_*scale_lin_vel, // [3,]
        #                             this->bodyAngularVel_*scale_ang_vel, // [3,]
        #                             this->gravity_projected, // [3,]
        #                             (gc_.tail(12)-this->body_pos_init)*scale_dof_pos, // [12,]
        #                             gv_.tail(12)*scale_dof_vel, // [12,]
        #                             this->action_clipped; // [this->actionDim_,]

        self.update_body_linear_and_angular_velocity()
        self.update_projected_gravity()
        self.update_joint_pos_and_vel()
        # self.update_actions()


        observation = np.concatenate([  self.body_lin_velocity,
                                        self.body_ang_velocity,
                                        self.gravity_vec_projected,
                                        self.joint_pos_cur-self.joint_position_init,
                                        self.joint_vel_cur,
                                        self.action_clipped_vec
                                        ],axis=0)
        return observation


    def parse_actions_into_joint_pos_des(self,action_vec):
        """
        
        Do whatever we're doing in Environment.hpp
        
        """
        pass


    def step(self,actions):

        # Clip the actions:
        # self.update_actions()
        self.action_clipped_vec = np.cip(actions,-self.action_lim,self.action_lim)

        joint_pos_des = self.parse_actions_into_joint_pos_des(self.action_clipped_vec)

        # Pass them through the policy
        for _ in range(4):
            # Apply joint_pos_des

        # Collect observations
        pass


    def reset(self):
        pass


    def _load_policy(self,path2load):
        pass



    def read_initial_position(self):

        self.go_to = GoToJointPosition()
        self.go_to.reset_reading_of_initial_position()


    def read_initial_orientation_from_imu_for_calibration():
        pass



class GoToJointPosition():

    def __init__(self):

        self.Njoints = 12

        # Current positions (read from robot):
        self.joint_pos_curr = np.zeros(self.Njoints)
        
        # Desired position (commanded, sent to robot):
        self.joint_pos_des_init = np.zeros(self.Njoints)
        self.joint_pos_des_target = np.zeros(self.Njoints)

        # Keep track of th entire state:
        self.msg_low_state = unitree_legged_msgs.msg.LowState()

        # The callback will be called for each message published on the topic; we can't set the frequency
        self.topic_low_state = "low_state_from_robot"
        rospy.Subscriber(self.topic_low_state, unitree_legged_msgs.msg.LowState, self.callback_robot_state)

    def callback_robot_state(self,msg):

        self.msg_low_state = msg

        for ii in range(self.Njoints):
            self.joint_pos_curr[ii] = msg.motorState[ii].q

    def update_target(self,joint_pos_des_target):
        self.joint_pos_des_target = joint_pos_des_target

    def interpolation_linear(self,ind,Nsteps):

        alpha = min(ind / Nsteps,1)
        joint_pos_des_curr = self.joint_pos_des_init*(1.-alpha) + self.joint_pos_des_target*alpha

        return joint_pos_des_curr

    def reset_reading_of_initial_position(self):
        """

        NOTE: We do not need to call spinOnce (in fact, there's no such a function in rospy) because
        rospy runs all the suscribers "in the background" once initialized.
        Se, we just set the flag to True for a bit and back to False, hoping that 
        self.joint_pos_curr will have been updated by then
        """

        time_sleep = 3.0
        print("Reading current robot position from the topic {0:s} ...".format(self.topic_low_state))
        print("Sleeping for {0:f} seconds ...".format(time_sleep))
        time.sleep(time_sleep)

        self.joint_pos_des_init = np.copy(self.joint_pos_curr)
        print("Initial robot position: ",self.joint_pos_des_init)


def main(joint_pos_des_target):
    """

    1. Read the current robot's position from topic "low_state_from_robot"
    2. Create a smooth transition to a desired target and publish it

    This program assumes that position_holder is running in the network: "rosrun unitree_legged_real position_holder"

    """

    rospy.init_node("go_to", anonymous=False)
    rate_freq = 500 # Hz
    rate = rospy.Rate(rate_freq) # Hz

    go2_joint_position = GoToJointPosition() # This starts a subscriber to the current robot state

    go2_joint_position.update_target(joint_pos_des_target)

    # Read initial position from the network:
    go2_joint_position.reset_reading_of_initial_position()

    # Start raisim server:
    use_raisim_visualization = True
    if use_raisim_visualization: visualize_raisim = VisualizeRaisim()

    pub2low_cmd = rospy.Publisher("low_cmd_to_robot", unitree_legged_msgs.msg.LowCmd, queue_size=10)
    msg_low_cmd = unitree_legged_msgs.msg.LowCmd()

    # Send lowlevel flags:
    msg_low_cmd.levelFlag = 0xFF
    for ii in range(go2_joint_position.Njoints):
        msg_low_cmd.motorCmd[ii].mode = 0x0A


    Nsteps_total = 4000
    Nsteps_transition = 3000
    print("Moving to target within {0:f} seconds; the program will exit after {0:f} seconds ...".format(1/rate_freq*Nsteps_transition,1/rate_freq*Nsteps_total))
    input("Press any key to continue ...")
    time.sleep(1.0)
    # ii = 0
    # while(not rospy.is_shutdown() and ii < Nsteps_total):

    # Log data out:
    data_fields = ["q_curr","dq_curr","ddq_curr","q_raw_curr","dq_raw_curr","ddq_raw_curr","u_est","q_des","dq_des","u_des"];
    data_joint_names = ["time_stamp","FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"];

    Nsteps_record = Nsteps_total
    data = np.zeros((len(data_fields),Nsteps_record,len(data_joint_names)))

    time_start = time.time()

    for ii in range(Nsteps_total):

        time_curr = time.time() - time_start

        joint_pos_des_curr = go2_joint_position.interpolation_linear(ind=ii,Nsteps=Nsteps_transition)

        for jj in range(go2_joint_position.Njoints):
            msg_low_cmd.motorCmd[jj].q = joint_pos_des_curr[jj]

        pub2low_cmd.publish(msg_low_cmd)

        # print("joint_pos_des_curr:",joint_pos_des_curr)

        if use_raisim_visualization: visualize_raisim.update_visualization(go2_joint_position.joint_pos_curr,joint_pos_des_curr)


        # Log data:
        data[:,ii,0] = time_curr # Time

        for jj in range(1,len(data_joint_names)):

            data[0,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].q
            data[1,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].dq
            data[2,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].ddq

            data[3,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].q_raw
            data[4,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].dq_raw
            data[5,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].ddq_raw

            data[6,ii,jj] = go2_joint_position.msg_low_state.motorState[jj].tauEst

            data[7,ii,jj] = msg_low_cmd.motorCmd[jj].q
            data[8,ii,jj] = msg_low_cmd.motorCmd[jj].dq
            data[9,ii,jj] = msg_low_cmd.motorCmd[jj].tau


        rate.sleep()

    visualize_raisim.server.killServer()

    # pdb.set_trace()

    # Save data:
    print("Saving data ...")
    name_file = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
    path2save = "./data_from_go_to_{0:s}.npy".format(name_file)
    np.save(path2save,data)
    print("Done!")

    return;


if __name__ == "__main__":


    # Target position we want to reach: Stand-up position
    # joint_pos_des_target = np.array([0.0136, 0.7304, -1.4505, -0.0118, 0.7317, -1.4437, 0.0105, 0.6590, -1.3903, -0.0102, 0.6563, -1.3944])

    # Target position we want to reach: Lying-down position
    joint_pos_des_target = np.array([-0.303926,1.15218,-2.69135,0.346799,1.17985,-2.73951,-0.348858,1.15957,-2.75885,0.348737,1.20456,-2.79926])

    main(joint_pos_des_target)


