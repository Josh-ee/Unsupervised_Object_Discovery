import numpy as np
import pdb
import matplotlib.pyplot as plt
from matplotlib import cm
import matplotlib
import csv
import pdb

def read_cvs_file(path2data,folder_name):

	# See examples/plot_data.py for usage

	# The convention for the ordering of the joints follows include/quadruped.h
	# joints_names = ["FR_0","FR_1","FR_2","FL_0","FL_1","FL_2","RR_0","RR_1","RR_2","RL_0","RL_1","RL_2"]
	joints_names = ["Front Right - Hip Lateral","Front Right - Hip Forward","Front Right - Knee",
					"Front Left - Hip Lateral","Front Left - Hip Forward","Front Left - Knee",
					"Rear Right - Hip Lateral","Rear Right - Hip Forward","Rear Right - Knee",
					"Rear Right - Hip Lateral","Rear Right - Hip Forward","Rear Right - Knee"]

	# file_names = ["q_curr","q_des","dq_curr","u_des","u_est"]
	# file_ind = [0,1,2,3,4]
	file_names = ["q_curr","dq_curr","ddq_curr","q_raw_curr","dq_raw_curr","ddq_raw_curr","u_est","q_des","dq_des","u_des"]

	# pdb.set_trace()

	Ndata = 10000 + 1
	data = np.zeros((len(file_names),Ndata,13))
	for ff in range(data.shape[0]):

		file_path_full = "{0:s}/{1:s}/data_robot_{2:s}.csv".format(path2data,folder_name,file_names[ff])
		with open(file_path_full, newline='') as csvfile:
			spamreader = csv.reader(csvfile, delimiter=',', quotechar='|')
			ii = 0
			for row in spamreader:
				jj = 0
				if ii == 0:
					ii += 1
					continue
				for col in row:
					if jj == 12:
						if col[-1] == ";": 
							# print("double check that this is correct .... col = col[0:-1] !!!!!!!!!!!!!!!!!!!")
							# pdb.set_trace()
							col = col[0:-1]
					data[ff,ii,jj] = np.float64(col)
					jj += 1
				ii += 1

	return data, file_names, joints_names



	