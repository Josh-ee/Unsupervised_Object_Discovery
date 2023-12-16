import numpy as np
import pdb
import pickle

from utils.data_parsing import plot_all, join_data

# path2load_root = "/home/ubuntu/mounted_home/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # ubuntu VM
# path2load_root = "/home/amarco/catkin_real_robot_ws/src/unitree_ros_to_real/unitree_legged_real/nodes/python/data_experiments_go1" # robot's laptop
path2load_root = "/Users/alonrot/work/code_projects_WIP/catkin_real_robot_ws/src/unitree_ros_to_real_forked/unitree_legged_real/nodes/python/data_experiments_go1" # mac




def main_experiments_2023_03_13():

	folder_name_experiments = "experiments_2023_03_13"
	path2load = "{0:s}/{1:s}".format(path2load_root,folder_name_experiments)

	data = dict()
	data.update(Shape_traj_1=dict(name_file_list=[],time_tot=15.0,pos_waypoints=np.array([[0.0,0.0],[1.0,1.0],[-1.0,2.0],[0.0,3.0]])))
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_05_20_experiments_go1trajs.pickle"] # S-traj-1, 1
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_06_04_experiments_go1trajs.pickle"] # S-traj-1, 2
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_09_39_experiments_go1trajs.pickle"] # S-traj-1, 3
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_10_44_experiments_go1trajs.pickle"] # S-traj-1, 4
	data["Shape_traj_1"]["name_file_list"] += ["2023_03_13_14_11_35_experiments_go1trajs.pickle"] # S-traj-1, 5

	data.update(Shape_traj_2=dict(name_file_list=[],time_tot=15.0,pos_waypoints=np.array([[0.0,0.0],[-1.0,1.0],[1.0,2.0],[0.0,3.0]])))
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_16_30_experiments_go1trajs.pickle"] # S-traj-2, 1
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_18_45_experiments_go1trajs.pickle"] # S-traj-2, 2
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_20_10_experiments_go1trajs.pickle"] # S-traj-2, 3
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_21_39_experiments_go1trajs.pickle"] # S-traj-2, 4
	data["Shape_traj_2"]["name_file_list"] += ["2023_03_13_14_23_12_experiments_go1trajs.pickle"] # S-traj-2, 5

	data.update(Straight_traj_left=dict(name_file_list=[],time_tot=7.5,pos_waypoints=np.array([[0.0,0.0],[-1.5,3.0]])))
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_33_37_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 1
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_34_49_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 2
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_35_58_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 3
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_37_16_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 4
	data["Straight_traj_left"]["name_file_list"] += ["2023_03_13_14_41_19_experiments_go1trajs.pickle"] # Straight-traj-left_corner, 5

	data.update(Straight_traj_right=dict(name_file_list=[],time_tot=7.5,pos_waypoints=np.array([[0.0,0.0],[1.0,3.0]])))
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_45_19_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 1
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_46_19_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 2
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_52_59_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 3
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_54_03_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 4
	data["Straight_traj_right"]["name_file_list"]  += ["2023_03_13_14_54_52_experiments_go1trajs.pickle"] # Straight-traj-right_corner, 5

	# name_file = "2023_03_13_13_01_09_experiments_go1trajs.pickle" # good, all signals coming, fixed time step
	# name_file = "2023_03_13_13_16_04_experiments_go1trajs.pickle" # good, all signals coming, fixed time step, vel profile at 120 Hz
	# name_file = "2023_03_13_13_30_35_experiments_go1trajs.pickle" # good, all signals coming, variable time step, vel profile at 120 Hz
	# name_file = "2023_03_13_13_33_40_experiments_go1trajs.pickle" # straight line
	# name_file = "2023_03_13_13_50_03_experiments_go1trajs.pickle" # straight line, corrected vel
	# name_file = "2023_03_13_13_52_41_experiments_go1trajs.pickle" # straight line, corrected vel, higher Kp





	plot_all(data,path2load,subsample_every_nr_steps=10,ind_beg=1500,Ncut_end=4990)
	state_and_control_curr, state_next_traj = join_data(data,path2load,save_data_trajs_dict=False,subsample_every_nr_steps=10,ind_beg=1500,Ncut_end=4990,name_file2save="joined_go1trajs_trimmed.pickle")
	# pdb.set_trace()






	# Copy data to remote server:
	# scp -P 4444 -r data_experiments_go1/experiments_2023_03_13/joined_go1trajs_trimmed.pickle amarco@hybridrobotics.hopto.org:/home/amarco/code_projects/ood_project/ood/experiments/data_quadruped_experiments_03_13_2023/


	# Copy data to ood project directory:
	# cp data_experiments_go1/experiments_2023_03_13/joined_go1trajs_trimmed.pickle ~/work/code_projects_WIP/ood_project/ood/experiments/data_quadruped_experiments_03_13_2023/




def main_experiments_2023_03_25():



	folder_name_experiments = "experiments_2023_03_25"
	path2load = "{0:s}/{1:s}".format(path2load_root,folder_name_experiments)
	data = dict()


	# pos_waypoints = np.array(	[[ 0., 0.],
	# 							[-0.16595599  ,2.16097348],
	# 							[-0.99977125  ,0.90699772],
	# 							[-0.20646505  ,1.6164502 ],
	# 							[-0.5910955   ,2.63435231],
	# 							[-0.1653904   ,1.67606949],
	# 							[-0.71922612  ,0.59430447],
	# 							[ 0.60148914  ,2.90478473],
	# 							[-0.37315164  ,2.07696785],
	# 							[ 0.7527783   ,2.68381999]])

	# data.update(batch_1=dict(name_file_list=[],time_tot=70.0,pos_waypoints=pos_waypoints))
	# data["batch_1"]["name_file_list"] += ["2023_03_25_22_50_42_experiments_go1trajs.pickle"]


	pos_waypoints = np.array(	[[ 0.          ,0.        ],
								 [-0.07618133  ,2.39427316],
								 [-1.28372978  ,3.40599031],
								 [-0.95120864  ,1.46755001],
								 [ 0.17396841  ,2.91028152],
								 [-1.52897019  ,2.82467967],
								 [-1.35446047  ,4.31934331],
								 [ 0.98891352  ,2.71830261],
								 [ 0.9629652   ,0.43805012],
								 [-0.23163868  ,0.35907577],
								 [-1.5549401   ,3.2820992 ],
								 [-1.208958    ,0.58820126],
								 [-1.22892828  ,1.92404457],
								 [-0.3627438   ,1.10958774],
								 [ 0.24142354  ,2.6568841 ],
								 [-1.43195491  ,3.85413791],
								 [ 1.37592878  ,2.75004599],
								 [-0.01499555  ,2.3515028 ],
								 [-0.47138458  ,4.27107552],
								 [-0.12538539  ,5.24558225]])

	data.update(batch_2=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_2"]["name_file_list"] += ["2023_03_25_23_53_12_experiments_go1trajs.pickle"]


	pos_waypoints = np.array(	[[ 0.          ,0.        ],
								 [-0.07220734  ,3.89481302],
								 [-0.98183341  ,2.80955183],
								 [ 1.12531434  ,4.92961199],
								 [-1.56045141  ,1.13983583],
								 [-1.81986479  ,2.42445414],
								 [ 0.27200417  ,1.53168005],
								 [ 0.36689216  ,3.2497455 ],
								 [-1.91606341  ,3.07369748],
								 [-1.09261644  ,2.28305658],
								 [-1.00766221  ,3.81225855],
								 [-0.45841199  ,0.86277256],
								 [-0.09372844  ,4.2917312 ],
								 [-0.92772764  ,1.22076836],
								 [-0.6421006   ,5.15011007],
								 [ 1.41598398  ,3.69811022],
								 [-0.67702086  ,0.50719355],
								 [ 0.28693816  ,3.06812419],
								 [-0.73452333  ,1.23779978],
								 [-0.57718029  ,2.57917137]])


	data.update(batch_3=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_3"]["name_file_list"] += ["2023_03_26_00_03_29_experiments_go1trajs.pickle"]





	pos_waypoints = np.array(	[[ 0.         , 0.        ],
								[-1.2230239   ,4.78902768],
								[-0.29056084  ,3.36459125],
								[ 0.6806775   ,2.85129893],
								[-0.96119824  ,1.03246676],
								[-1.71740556  ,4.06142163],
								[-0.45541772  ,0.87070427],
								[ 1.07977961  ,1.50747554],
								[-0.55017743  ,1.62843963],
								[ 0.20075768  ,3.18910796],
								[ 0.09975219  ,1.46200515],
								[-1.00359942  ,1.39473513],
								[-1.42035499  ,5.30161791],
								[ 1.3607935   ,1.03628061],
								[-1.91492703  ,1.12505551],
								[ 0.44945265  ,4.28733022],
								[-1.91973418  ,3.17714572],
								[ 0.23928312  ,5.42093422],
								[-1.09315841  ,4.41373287],
								[ 1.0466908   ,5.07512288]])


	data.update(batch_4=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_4"]["name_file_list"] += ["2023_03_26_00_17_36_experiments_go1trajs.pickle"]



	pos_waypoints = np.array(	[[ 0.          ,0.        ],
								 [ 1.12501053  ,1.82588893],
								 [ 0.87430193  ,0.22933144],
								 [-1.62320162  ,3.27278635],
								 [-0.14563923  ,2.30344086],
								 [-0.82607253  ,3.42385688],
								 [ 0.25874284  ,5.44623349],
								 [ 0.86950369  ,2.27260514],
								 [ 1.06693679  ,4.53067688],
								 [-1.80933922  ,3.9525048 ],
								 [ 0.80759697  ,4.05023654],
								 [ 0.48196113  ,2.9751527 ],
								 [-1.56311539  ,5.26706013],
								 [-0.58860295  ,1.19323138],
								 [ 0.51046546  ,5.46814091],
								 [-1.10535081  ,3.69220188],
								 [ 0.0965207   ,3.9453268 ],
								 [ 1.28072337  ,1.93495373],
								 [-1.11228066  ,2.2135988 ],
								 [ 0.61278751  ,3.98238812]])

	data.update(batch_5=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_5"]["name_file_list"] += ["2023_03_26_00_28_37_experiments_go1trajs.pickle"]


	pos_waypoints = np.array(	[[ 0.         , 0.        ],
								[-1.96680271  ,2.76031026],
								[-0.41352546  ,0.73606241],
								[-1.54524453  ,1.20207272],
								[-1.46631569  ,4.83207497],
								[ 1.0430849   ,0.21311607],
								[ 0.23714365  ,3.15017899],
								[-0.76327306  ,3.81941273],
								[ 0.63958549  ,2.56117661],
								[ 1.00967811  ,4.42056874],
								[ 1.14674959  ,0.93299942],
								[-0.33252892  ,5.20034579],
								[ 1.10670724  ,1.02047928],
								[-1.22202984  ,4.80370464],
								[ 0.90008729  ,4.17052893],
								[-0.88991375  ,2.54944093],
								[ 0.78530655  ,1.03047256],
								[ 0.2005571   ,3.86941046],
								[ 0.96064025  ,0.2170015 ],
								[ 0.07077348  ,3.50929692]])


	data.update(batch_6=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_6"]["name_file_list"] += ["2023_03_26_00_43_22_experiments_go1trajs.pickle"]




	pos_waypoints = np.array(	[[ 0.          ,0.        ],
								 [-1.96680271  ,2.76031026],
								 [-0.41352546  ,0.73606241],
								 [-1.54524453  ,1.20207272],
								 [-1.46631569  ,4.83207497],
								 [ 1.0430849   ,0.21311607],
								 [ 0.23714365  ,3.15017899],
								 [-0.76327306  ,3.81941273],
								 [ 0.63958549  ,2.56117661],
								 [ 1.00967811  ,4.42056874],
								 [ 1.14674959  ,0.93299942],
								 [-0.33252892  ,5.20034579],
								 [ 1.10670724  ,1.02047928],
								 [-1.22202984  ,4.80370464],
								 [ 0.90008729  ,4.17052893],
								 [-0.88991375  ,2.54944093],
								 [ 0.78530655  ,1.03047256],
								 [ 0.2005571   ,3.86941046],
								 [ 0.96064025  ,0.2170015 ],
								 [ 0.07077348  ,3.50929692]])


	data.update(batch_7=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_7"]["name_file_list"] += ["2023_03_26_00_50_50_experiments_go1trajs.pickle"]


	# data.pop("batch_4")
	# data.pop("batch_5")
	# data.pop("batch_6")
	# data.pop("batch_7")


	# Data comes at 500 Hz
	# We subsample at 10 Hz
	plot_all(data,path2load,subsample_every_nr_steps=50,ind_beg=1250,Ncut_end=None)
	# state_and_control_curr, state_next_traj = join_data(data,path2load,save_data_trajs_dict=True,subsample_every_nr_steps=50,ind_beg=1250,Ncut_end=None,name_file2save="joined_go1trajs_trimmed_2023_03_25.pickle")




def main_experiments_2023_03_29():



	folder_name_experiments = "experiments_2023_03_29"
	path2load = "{0:s}/{1:s}".format(path2load_root,folder_name_experiments)
	data = dict()


	pos_waypoints = np.array([[-1.92658477  ,2.53647451],
								[-1.71352549  ,2.11832212],
								[-1.38167788  ,1.78647451],
								[-0.96352549  ,1.57341523],
								[-0.5         ,1.5       ],
								[-0.03647451  ,1.57341523],
								[ 0.38167788  ,1.78647451],
								[ 0.71352549  ,2.11832212],
								[ 0.92658477  ,2.53647451],
								[ 1.          ,3.        ],
								[ 0.92658477  ,3.46352549],
								[ 0.71352549  ,3.88167788],
								[ 0.38167788  ,4.21352549],
								[-0.03647451  ,4.42658477],
								[-0.5         ,4.5       ],
								[-0.96352549  ,4.42658477],
								[-1.38167788  ,4.21352549],
								[-1.71352549  ,3.88167788],
								[-1.92658477  ,3.46352549],
								[-2.          ,3.        ]])


	# data.update(batch_1=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	# data["batch_1"]["name_file_list"] += ["2023_03_29_15_54_24_experiments_go1trajs.pickle"]


	# data.update(batch_2=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	# data["batch_2"]["name_file_list"] += ["2023_03_29_15_54_24_experiments_go1trajs.pickle"]

	# Rope tied 1
	data.update(batch_1=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_1"]["name_file_list"] += ["2023_03_29_16_41_01_experiments_go1trajs.pickle"]

	# Rope tied 2
	data.update(batch_2=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_2"]["name_file_list"] += ["2023_03_29_16_53_12_experiments_go1trajs.pickle"]

	# No rope
	data.update(batch_3=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_3"]["name_file_list"] += ["2023_03_29_17_05_51_experiments_go1trajs.pickle"]

	# No rope
	data.update(batch_4=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_4"]["name_file_list"] += ["2023_03_29_17_19_00_experiments_go1trajs.pickle"]

	# Rocky terrain
	data.update(batch_5=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_5"]["name_file_list"] += ["2023_03_29_17_58_20_experiments_go1trajs.pickle"]

	# Rocky terrain
	data.update(batch_6=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_6"]["name_file_list"] += ["2023_03_29_18_18_12_experiments_go1trajs.pickle"]

	# Poking
	data.update(batch_7=dict(name_file_list=[],time_tot=200.0,pos_waypoints=pos_waypoints))
	data["batch_7"]["name_file_list"] += ["2023_03_29_18_47_21_experiments_go1trajs.pickle"]


	data.pop("batch_1")
	data.pop("batch_2")
	# data.pop("batch_3")
	# data.pop("batch_4")
	data.pop("batch_5")
	data.pop("batch_6")
	data.pop("batch_7")



	# Data comes at 500 Hz
	# We subsample at 10 Hz
	plot_all(data,path2load,subsample_every_nr_steps=50,ind_beg=2000,Ncut_end=None)
	state_and_control_curr, state_next_traj = join_data(data,path2load,save_data_trajs_dict=False,subsample_every_nr_steps=50,ind_beg=2000,Ncut_end=None,name_file2save="joined_go1trajs_trimmed_2023_03_29_circle_poking.pickle")



if __name__ == "__main__":

	# main_experiments_2023_03_13()

	main_experiments_2023_03_29()


