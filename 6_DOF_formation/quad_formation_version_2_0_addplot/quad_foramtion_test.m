close all;
clear;

addpath('utils');

%%%%% main loop %%%%%

%% pre-calculated trajectories
trajhandle = @traj_line; 

%% controller selection
controlhandle = @controller_flock_pid;

% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
num_agents = 5; % Number of agents
% [t, state] = simulation_3d_att_plus(trajhandle, controlhandle, num_agents); 
[t, state] = simulation_3d_att_add_plot(trajhandle, controlhandle, num_agents); 
