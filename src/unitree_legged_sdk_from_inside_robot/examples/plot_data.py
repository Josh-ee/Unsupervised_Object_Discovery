import numpy as np
import pdb
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib
import csv
import pdb

from unitree_legged_sdk_python_tools.utils.data_parsing import read_cvs_file

def main():

	path2data = "./"
	# folder_name = "2022_11_22_16_43_76"; # from laptop
	# folder_name = "2022_11_22_16_46_34"; # inside the robot


	# folder_name = "2022_01_13_18_30_55"

	# folder_name = "2022_01_13_19_28_46"

	# data = data[:,0:5500,:] # 2022_01_13_19_24_16

	# data, file_names, joints_names = read_cvs_file(path2data,folder_name)
	
	# print(file_names)

	# path2data = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/"



	# Log data out:
	file_names = ["q_curr","dq_curr","ddq_curr","q_raw_curr","dq_raw_curr","ddq_raw_curr","u_est","q_des","dq_des","u_des"];
	joints_names = ["time_stamp","FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"];



	folder_name = "data_from_go_to_2022_12_06_18_40_07.npy" # Go up
	# folder_name = "data_from_go_to_2022_12_06_18_41_55.npy" # Go down

	data = np.load("{0:s}/{1:s}".format(path2data,folder_name))

	# pdb.set_trace()

	ind_is_zero = data[0,:,0] == 0.0 # Check when the timestamp is zero and remove those values (the program finished before the data logging did...)
	data = data[:,~ind_is_zero,:]


	Njoints = 3
	k_cut = 1 # Cut the first time steps
	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Position - Front right Leg")
	ind_plot = 0


	ind_leg = 6
	joints_range = list(range(ind_leg,Njoints+ind_leg))

	for jj in joints_range:
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[ind_plot].plot(time_stamp,data[7,k_cut::,jj+1],label=file_names[7]) # desired
		# hdl_splots[jj].plot(time_stamp[5::],data[0,5::,jj+1],label=file_names[0]) # current (delay compensated)
		hdl_splots[ind_plot].plot(time_stamp,data[0,k_cut::,jj+1],label=file_names[0]) # current
		# hdl_splots[jj].plot(time_stamp,data[2,:,jj+1],label=file_names[2]) # raw
		hdl_splots[ind_plot].set_title(joints_names[jj+1])
		hdl_splots[ind_plot].set_ylabel("angle [rad]")
		hdl_splots[ind_plot].legend()
		ind_plot += 1
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Velocity - Front right Leg")
	# k_cut = 0
	cc = 0
	for jj in joints_range:
		# time_stamp_time_step = np.diff(data[0,:,0])
		# time_stamp_time_step = time_stamp_time_step[k_cut::]
		# pdb.set_trace()

		# joint_pos_vec = data[0,:,jj+1]
		# joint_vel_vec_num_diff = np.diff(joint_pos_vec) / time_stamp_time_step

		joint_vel_vec = data[1,0:-1,jj+1]
		joint_vel_des = data[8,0:-1,jj+1]

		time_stamp = data[0,0:-1,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[cc].plot(time_stamp,joint_vel_vec,label=file_names[1])
		hdl_splots[cc].plot(time_stamp,joint_vel_des,label=file_names[8])
		# hdl_splots[cc].plot(time_stamp,joint_vel_vec_num_diff,label="dq_curr (num diff)")
		hdl_splots[cc].set_title(joints_names[jj+1])
		hdl_splots[cc].set_ylabel("angular velocity [rad/s]")
		hdl_splots[cc].legend()
		cc += 1
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Velocity - Front right Leg")
	# k_cut = 0
	cc = 0
	for jj in joints_range:
		# time_stamp_time_step = np.diff(data[0,:,0])
		# time_stamp_time_step = time_stamp_time_step[k_cut::]
		# pdb.set_trace()

		# joint_pos_vec = data[0,:,jj+1]
		# joint_vel_vec_num_diff = np.diff(joint_pos_vec) / time_stamp_time_step

		joint_vel_vec = data[2,:,jj+1]

		time_stamp = data[0,:,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[cc].plot(time_stamp,joint_vel_vec,label=file_names[2])
		# hdl_splots[cc].plot(time_stamp,joint_vel_vec_num_diff,label="dq_curr (num diff)")
		hdl_splots[cc].set_title(joints_names[jj+1])
		hdl_splots[cc].set_ylabel("angular acceleration [rad/s2]")
		hdl_splots[cc].legend()
		cc += 1
	hdl_splots[Njoints-1].set_xlabel("time [sec]")


	hdl_fig, hdl_splots = plt.subplots(Njoints,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("Torque - Front right Leg")
	cc = 0
	for jj in joints_range:
		time_stamp = data[0,k_cut::,0] # It's the same for all, q_des, q_curr, etc.
		hdl_splots[cc].plot(time_stamp,data[6,k_cut::,jj+1],label=file_names[6]) # u des
		hdl_splots[cc].plot(time_stamp,data[9,k_cut::,jj+1],label=file_names[9]) # u est
		hdl_splots[cc].set_title(joints_names[jj+1])
		hdl_splots[cc].set_ylabel("torque [Nm]")
		hdl_splots[cc].legend()
		cc += 1
	hdl_splots[Njoints-1].set_xlabel("time [sec]")

	hdl_fig, hdl_splots = plt.subplots(1,1,figsize=(13,9),sharex=True)
	hdl_fig.suptitle("time step distribution")
	time_step_vec = np.diff(data[0,:,0])
	hdl_splots.hist(time_step_vec,bins=200,density=True,color="b",alpha=0.5,fill=True)



	plt.show(block=True)




if __name__ == "__main__":

	main()