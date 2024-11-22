close all;
clear;

addpath('utils');

%%%%% caution! %%%%%
% 保证traj文件中的t_max 大于 simulation_3d_basic 的tmax
%%%%% main loop %%%%%

%% pre-calculated trajectories
% trajhandle = @traj_line; 
trajhandle = @traj_circle; 
% trajhandle = @traj_circle_lift; 


%% controller selection
controlhandle = @controller_flock_pid;

% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]
num_agents = 5; % Number of agents

[t, state] = simulation_3d_basic(trajhandle, controlhandle, num_agents); 


