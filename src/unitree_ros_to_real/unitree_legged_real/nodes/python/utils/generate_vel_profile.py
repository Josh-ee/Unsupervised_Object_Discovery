import os
import numpy as np
import pdb

import pickle
import math
from .min_jerk_gen import min_jerk

import matplotlib.pyplot as plt
import matplotlib

markersize_x0 = 10
markersize_trajs = 0.4
fontsize_labels = 25
matplotlib.rc('xtick', labelsize=fontsize_labels)
matplotlib.rc('ytick', labelsize=fontsize_labels)
matplotlib.rc('text', usetex=False)
# matplotlib.rc('font',**{'family':'serif','serif':['Computer Modern Roman']})
plt.rc('legend',fontsize=fontsize_labels+2)



def generate_waypoints_in_circle(Nwaypoints,xlim,ylim,rate_freq_send_commands,time_tot,block_plot,plotting):


    deltaT = 1./rate_freq_send_commands

    # Divide circle in Nwaypoints parts:
    theta_vec = np.linspace(0.0,2.*math.pi,Nwaypoints+1)

    # Remove the last one because 0 is same as 2pi
    theta_vec = theta_vec[0:-1]
    theta_vec = np.reshape(theta_vec,(-1,1))

    radius = 1.5

    # Waypoints:
    x_pos = radius*np.cos(theta_vec)
    y_pos = radius*np.sin(theta_vec)

    # Shift the circle so that the lowest point is near (0,0)
    y_pos = y_pos + abs(np.amin(y_pos)) + 1.5

    x_pos = x_pos + -0.5

    pos_waypoints = np.concatenate([x_pos,y_pos],axis=1)


    # We want the first point to be the one at ind_first:
    ind_first = Nwaypoints//2 +1
    # pdb.set_trace()
    pos_waypoints_aux = np.concatenate([pos_waypoints,pos_waypoints],axis=0)
    pos_waypoints_new = pos_waypoints_aux[ind_first:ind_first+Nwaypoints]

    # pdb.set_trace()


    if plotting:
        hdl_fig_data, hdl_splots_data = plt.subplots(1,1,figsize=(6,6),sharex=True)

        hdl_splots_data.plot(pos_waypoints_new[:,0],pos_waypoints_new[:,1],marker="o",markersize=8,color="crimson",alpha=0.5,linestyle="None")
        hdl_splots_data.plot(pos_waypoints_new[0,0],pos_waypoints_new[0,1],marker="o",markersize=8,color="navy",alpha=0.5,linestyle="None")
        hdl_splots_data.set_aspect("equal","box")
        hdl_splots_data.set_xlim(xlim)
        hdl_splots_data.set_ylim(ylim)

        plt.show(block=block_plot)


        plt.pause(0.1)

    return pos_waypoints_new





def generate_random_set_of_waypoints(Nwaypoints,xlim,ylim,rate_freq_send_commands,time_tot,block_plot,plotting):


    deltaT = 1./rate_freq_send_commands

    # Generate waypoints a bit away from each other:
    pos_waypoints = np.zeros((Nwaypoints,2))
    # Steer away 
    waypoint_next = np.zeros(2)
    rad = 1.0
    too_close = True
    for ii in range(1,Nwaypoints):

        while too_close:
            waypoint_next[0] = xlim[0] + np.diff(xlim)*np.random.rand(1)
            waypoint_next[1] = ylim[0] + np.diff(ylim)*np.random.rand(1)
            too_close = np.sum((waypoint_next-pos_waypoints[ii-1,...])**2) <= rad

        pos_waypoints[ii,...] = waypoint_next
        too_close = True

    state_tot, vel_tot = get_velocity_profile_given_waypoints(pos_waypoints,deltaT,time_tot,block_plot=block_plot,plotting=plotting) # state_tot: [Nsteps_tot,2] || vel_tot: [Nsteps_tot,2]

    return state_tot, vel_tot, pos_waypoints




def get_velocity_profile_given_waypoints(pos_waypoints,deltaT,time_tot,block_plot=True,plotting=True):
    """
    
    pos_waypoints: [Nwaypoints,2] -> We pass [x,y] positions; the heading will be inferred using arctan2()
    
    :return:
    Xtrain: [(Nsteps-3)*Ntrajs, dim_x+dim_u]
    Ytrain: [(Nsteps-3)*Ntrajs, dim_x]
    """

    # Number of steps:
    Nsteps = int(time_tot/deltaT)
    Nsteps_tot = Nsteps + 2 # We add 2 because numerical differentiation will suppress 2 points
    print(" * Generating trajectory ...")
    pos_profile,_ = min_jerk(pos=pos_waypoints, dur=Nsteps_tot, vel=None, acc=None, psg=None) # [Nsteps_tot, D]

    # Velocity profiles and heading with numerical differentiation:
    vel_profile_batch = np.zeros((Nsteps_tot-1,2))
    th_profile_batch = np.zeros((Nsteps_tot-1,1))
    th_vel_profile_batch = np.zeros((Nsteps_tot-2,1))
    vel_profile_batch[:,0] = np.diff(pos_profile[:,0]) / deltaT
    vel_profile_batch[:,1] = np.diff(pos_profile[:,1]) / deltaT
    vel_mod_profile_batch = np.sqrt(vel_profile_batch[:,0:1]**2 + vel_profile_batch[:,1:2]**2)
    th_profile_batch[:,0] = np.arctan2(vel_profile_batch[:,1], vel_profile_batch[:,0])
    th_vel_profile_batch[:,0] = np.diff(th_profile_batch[:,0]) / deltaT

    state_tot = np.concatenate((pos_profile[0:Nsteps,:],th_profile_batch[0:Nsteps,:]),axis=1) # [Nsteps_tot,3], with Nsteps_tot=Nsteps-2 due to the integration issues
    vel_tot = np.concatenate((vel_mod_profile_batch[0:Nsteps,:],th_vel_profile_batch),axis=1) # [Nsteps_tot,2], with Nsteps_tot=Nsteps-2 due to the integration issues


    if plotting:

        # Velocity profiles:
        hdl_fig_data, hdl_splots_data = plt.subplots(3,1,figsize=(12,8),sharex=True)
        time_vec = np.arange(0,Nsteps)*deltaT

        hdl_splots_data[0].plot(time_vec,vel_tot[:,0],lw=1,alpha=0.3,color="navy")
        hdl_splots_data[0].set_title("Forward velocity",fontsize=fontsize_labels)
        hdl_splots_data[0].set_ylabel(r"$v$ [m/s]",fontsize=fontsize_labels)

        hdl_splots_data[1].plot(time_vec,vel_tot[:,1],lw=1,alpha=0.3,color="navy")
        hdl_splots_data[1].set_ylabel(r"$\dot{\theta}$ [rad/s]",fontsize=fontsize_labels)
        hdl_splots_data[1].set_title("Angular velocity",fontsize=fontsize_labels)

        hdl_splots_data[2].plot(time_vec,state_tot[:,2],lw=1,alpha=0.3,color="navy")
        hdl_splots_data[2].set_ylabel(r"$\theta$ [rad]",fontsize=fontsize_labels)
        hdl_splots_data[2].set_title("Heading",fontsize=fontsize_labels)

        hdl_splots_data[-1].set_xlabel(r"$t$ [sec]",fontsize=fontsize_labels)
        hdl_splots_data[-1].set_xlim([time_vec[0],time_vec[-1]])
        hdl_splots_data[-1].set_xticks([time_vec[0],time_vec[-1]])


        # Trajectories:
        hdl_fig_data, hdl_splots_data = plt.subplots(1,1,figsize=(12,8),sharex=False)
        hdl_splots_data.plot(state_tot[:,0],state_tot[:,1],alpha=0.3,color="navy")
        hdl_splots_data.plot(state_tot[-1,0],state_tot[-1,1],marker="x",color="black",markersize=5)
        hdl_splots_data.plot(state_tot[0,0],state_tot[0,1],marker=".",color="green",markersize=3)
        hdl_splots_data.set_xlabel(r"$x$ [m]",fontsize=fontsize_labels)
        hdl_splots_data.set_ylabel(r"$y$ [m]",fontsize=fontsize_labels)

        # Plot all waypoints:
        for pp in range(1,pos_waypoints.shape[0]-1):
            hdl_splots_data.plot(pos_waypoints[pp,0],pos_waypoints[pp,1],marker=".",color="navy",markersize=7)

        hdl_splots_data.set_aspect('equal', 'box')

        plt.show(block=block_plot)
        plt.pause(1.)

    return state_tot, vel_tot




def get_multiple_velocity_profiles_from_random_waypoints(deltaT,which_trajectory,save_data_trajs_dict=None,block_plot=True,plotting=True):
    """
    
    :return:
    Xtrain: [(Nsteps-3)*Ntrajs, dim_x+dim_u]
    Ytrain: [(Nsteps-3)*Ntrajs, dim_x]
    """

    # Trajectory duration:
    time_tot = 5.0 # sec

    # Number of steps:
    Nsteps = int(time_tot/deltaT) + 2 # We add 2 because numerical differentiation will suppress 2 points

    # Generate randomly the middle points of vel_lin_way_points, vel_ang_way_points, and also z0
    Ntrajs = 10
    Nwaypoints = 4
    x_lim = [0.0,5.0]; y_lim = [-5.0,5.0]; 
    # th_lim_low = [-math.pi/2., math.pi/2.]; # Why do we not need the heading? Because it's inferred from [x(t),y(t)] as th = arctan(yd(t) / xd(t)), where xd(t) = d/dt x(t)
    # s_lim = np.reshape(np.array([x_lim;y_lim;th_lim]),(2,-1))
    s_lim = np.vstack((x_lim,y_lim))
    pos_waypoints = s_lim[:,0] + (s_lim[:,1]-s_lim[:,0])*np.random.rand(Ntrajs,Nwaypoints,2)

    # Set the initial position to zero without loss of generality:
    pos_waypoints[:,0,:] = np.zeros(2)

    # Force the final position to be at a random value for x in [4,5]:
    pos_waypoints[:,-1,0] = 4. + (5.-4.)*np.random.rand(Ntrajs)

    # # Initial conditions:
    # z0_x_lim = 0.1; z0_y_lim = 0.1; z0_th_lim = math.pi;
    # z0_lim = np.reshape(np.array([z0_x_lim,z0_y_lim,z0_th_lim]),(1,-1))
    # z0_vec = -z0_lim + 2.*z0_lim*np.random.rand(Ntrajs,3)

    # Sort out the x positions in increasing order to prevent weird turns:
    pos_waypoints[:,:,0] = np.sort(pos_waypoints[:,:,0],axis=1)

    pos_profile_batch = np.zeros((Ntrajs,Nsteps,2))
    for ii in range(Ntrajs):
        if ii % 10 == 0: print(" Generating trajectory {0:d} / {1:d} ...".format(ii+1,Ntrajs))
        pos_profile_batch[ii,...],_ = min_jerk(pos=pos_waypoints[ii,...], dur=Nsteps, vel=None, acc=None, psg=None) # [Nsteps, D]


    # Velocity profiles and heading with numerical differentiation:
    vel_profile_batch = np.zeros((Ntrajs,Nsteps-1,2))
    th_profile_batch = np.zeros((Ntrajs,Nsteps-1,1))
    th_vel_profile_batch = np.zeros((Ntrajs,Nsteps-2,1))
    vel_profile_batch[...,0] = np.diff(pos_profile_batch[...,0],axis=1) / deltaT
    vel_profile_batch[...,1] = np.diff(pos_profile_batch[...,1],axis=1) / deltaT
    vel_mod_profile_batch = np.sqrt(vel_profile_batch[...,0:1]**2 + vel_profile_batch[...,1:2]**2)
    th_profile_batch[...,0] = np.arctan2(vel_profile_batch[...,1], vel_profile_batch[...,0])
    th_vel_profile_batch[...,0] = np.diff(th_profile_batch[...,0],axis=1) / deltaT

    # Subselect those very smooth ones:
    vx_is_positive = np.all(vel_profile_batch[...,0] >= -0.5,axis=1)
    th_is_within_range = np.all(abs(th_profile_batch[...,0]) <= 0.98*math.pi/2.,axis=1)
    ind_smooth = np.arange(0,Ntrajs)[vx_is_positive & th_is_within_range]

    print("Smooth trajectories: {0:d} / {1:d}".format(len(ind_smooth),Ntrajs))
    assert len(ind_smooth) > 0

    Nsteps_tot = th_vel_profile_batch.shape[1]
    state_tot = np.concatenate((pos_profile_batch[ind_smooth,0:Nsteps_tot,:],th_profile_batch[ind_smooth,0:Nsteps_tot,:]),axis=2) # [Ntrajs,Nsteps_tot,3], with Nsteps_tot=Nsteps-2 due to the integration issues
    vel_tot = np.concatenate((vel_mod_profile_batch[ind_smooth,0:Nsteps_tot,:],th_vel_profile_batch[ind_smooth,:,:]),axis=2) # [Ntrajs,Nsteps_tot,2], with Nsteps_tot=Nsteps-2 due to the integration issues

    # # Get a round number of trajectories:
    # # pdb.set_trace()
    # # if state_tot.shape[0] > 300 and vel_tot.shape[0] > 300:
    # #   state_tot = state_tot[0:300,...]
    # #   vel_tot = vel_tot[0:300,...]

    # state_and_control_tot = np.concatenate((state_tot,vel_tot),axis=2)

    # # The data needs to be reshaped in this particular way; it's incorrect to do this: Xtot = np.reshape(Ntrajs*(Nsteps-3),dim_x+dim_u); Xtrain_np = Xtot[0:-1,:]; Ytrain_np = Xtot[1::,:]
    # Xtrain_np = np.reshape(state_and_control_tot[:,0:-1,:],(state_and_control_tot.shape[0]*(state_and_control_tot.shape[1]-1),state_and_control_tot.shape[2]),order="C") # order="C" -> last axis index changing fastest
    # Ytrain_np = np.reshape(state_tot[:,1::,:],(state_tot.shape[0]*(state_tot.shape[1]-1),state_tot.shape[2]),order="C") # order="C" -> last axis index changing fastest

    # Xtrain = tf.convert_to_tensor(value=Xtrain_np,dtype=np.float32)
    # Ytrain = tf.convert_to_tensor(value=Ytrain_np,dtype=np.float32)

    if plotting:

        # Velocity profiles:
        hdl_fig_data, hdl_splots_data = plt.subplots(4,1,figsize=(12,8),sharex=True)
        time_vec = np.arange(0,Nsteps-1)*deltaT
        for ii in ind_smooth:
            hdl_splots_data[0].plot(time_vec,vel_profile_batch[ii,:,0],lw=1,alpha=0.3,color="navy")
            hdl_splots_data[1].plot(time_vec,vel_profile_batch[ii,:,1],lw=1,alpha=0.3,color="navy")
            hdl_splots_data[2].plot(time_vec,vel_mod_profile_batch[ii,:],lw=1,alpha=0.3,color="navy")
            hdl_splots_data[3].plot(time_vec[0:-1],th_vel_profile_batch[ii,:,0],lw=1,alpha=0.3,color="navy")

        # Select first trajectory:
        hdl_splots_data[2].plot(time_vec[0:-1],vel_tot[which_trajectory,:,0],alpha=0.9,color="navy",lw=3.0)
        hdl_splots_data[3].plot(time_vec[0:-1],vel_tot[which_trajectory,:,1],alpha=0.9,color="navy",lw=3.0)

        hdl_splots_data[0].set_ylabel(r"$v_x$ [m/s]",fontsize=fontsize_labels)
        hdl_splots_data[1].set_ylabel(r"$v_y$ [m/s]",fontsize=fontsize_labels)
        hdl_splots_data[2].set_ylabel(r"$v$ [m/s]",fontsize=fontsize_labels)
        hdl_splots_data[3].set_ylabel(r"$\dot{\theta}$ [rad/s]",fontsize=fontsize_labels)
        hdl_splots_data[-1].set_xlabel(r"$t$ [sec]",fontsize=fontsize_labels)
        hdl_splots_data[-1].set_xlim([time_vec[0],time_vec[-1]])
        hdl_splots_data[-1].set_xticks([time_vec[0],time_vec[-1]])


        # Trajectories:
        
        hdl_fig_data, hdl_splots_data = plt.subplots(1,1,figsize=(12,8),sharex=False)
        for ii in ind_smooth:
            hdl_splots_data.plot(pos_profile_batch[ii,:,0],pos_profile_batch[ii,:,1],alpha=0.3,color="navy")
            hdl_splots_data.plot(pos_profile_batch[ii,-1,0],pos_profile_batch[ii,-1,1],marker="x",color="black",markersize=5)
            hdl_splots_data.plot(pos_profile_batch[ii,0,0],pos_profile_batch[ii,0,1],marker=".",color="green",markersize=3)

        # Select first trajectory:
        hdl_splots_data.plot(state_tot[which_trajectory,:,0],state_tot[which_trajectory,:,1],alpha=0.9,color="navy",lw=3.0)
        hdl_splots_data.set_xlabel(r"$x$ [m]",fontsize=fontsize_labels)
        hdl_splots_data.set_ylabel(r"$y$ [m]",fontsize=fontsize_labels)



    Nsteps = state_tot.shape[1]-1
    if save_data_trajs_dict is not None:
        if save_data_trajs_dict["save"]:
            data2save = dict(state_tot=state_tot,vel_tot=vel_tot,Nsteps=Nsteps,Ntrajs=Ntrajs,deltaT=deltaT)
            file = open(save_data_trajs_dict["path2data"], 'wb')
            pickle.dump(data2save,file)
            file.close()
    elif plotting:
        plt.show(block=block_plot)

    return state_tot, vel_tot, Nsteps, Ntrajs

