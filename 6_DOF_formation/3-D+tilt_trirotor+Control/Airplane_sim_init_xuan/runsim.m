close all;

clear;


addpath('utils');
addpath('traj');
addpath('controller');
addpath('test_tools');
addpath('test_airplane');

%% pre-calculated trajectories

% trajhandle = @traj_line; % 可以运行的轨迹，与时间设定形式有关，配合simulation_3D ----- [F, M, att_des_save_A] = controlhandle(0, current_state_A, desired_state_A, params);  % s: [13 * 1]; 
trajhandle = @traj_helix;
% trajhandle = @traj_circle; 
% trajhandle = @traj_dubin; % 分析不同，设定单机轨迹，并拓展多机编队；
% trajhandle = @traj_helix_adjust; % 还需调整

%% Trajectory generation with waypoints
%% You need to implement this
% trajhandle = @traj_generator;
% waypoints = [0    0   0;
%              1    1   1;
%              2    0   2;
%              3    -1  1;
%              4    0   0]';
% trajhandle([],[],waypoints);

%% controller selection
% controlhandle = @controller_adrc_NL; % 需要调试
% controlhandle = @controller_adrc;
controlhandle = @controller_pid;


% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

% [t, state] = simulation_3d_att_plus_1_1_over(trajhandle, controlhandle); % 姿态跟踪完成版1_1，扩展接口实现R
[t, state] = simulation_3d_ttr_test(trajhandle, controlhandle); % 添加姿态跟踪效果

% viewer
% planeplot_ttr_animation;