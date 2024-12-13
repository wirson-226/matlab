%%% 整体逻辑 %%%
% 路径规划--参考轨迹--拿出想要的跟踪状态--比如该案例：时刻相关的位置与偏航角--利用视线导引或者其他方法（后续可改）计算期望状态--速度，加速度，角度，角速度，角加速度
% 其中，一般六自由度划分横航向xoy与纵航向yoz，四旋翼中可称为高度控制与姿态控制，固定翼利用速度以及滚转与偏航角（考虑风速是航迹角）控制xoy运动（视线导引），旋翼利用滚转与俯仰角控制xoy运动
% 此外，参考轨迹拿出来的状态是那一时刻参考轨迹上的或者说虚拟轨迹上的状态，并非我们当前智能体的控制输入状态比如说加速度，我们的控制输入状态是当前智能体的控制期望不是跟踪期望，我们是利用加速度控制期望用以跟踪速度或者最终的位置层。
% 并不一定对于跟踪轨迹的物理参考意义，要看利用的参考状态要看控制最上层的输出，
% todo --- 修改俯仰正方向，现在为向下 -y 为正 

close all;

clear;


addpath('utils');
addpath('traj');
addpath('controller');
addpath('test_tools');
addpath('test_airplane');

%% pre-calculated trajectories

trajhandle = @traj_line; % 可以运行的轨迹，与时间设定形式有关，配合simulation_3D ----- [F, M, att_des_save_A] = controlhandle(0, current_state_A, desired_state_A, params);  % s: [13 * 1]; 
% trajhandle = @traj_helix;
% trajhandle = @traj_circle; 
% trajhandle = @traj_dubin; % 分析不同，设定单机轨迹，并拓展多机编队；
% trajhandle = @traj_helix_adjust; % 还需调整
% trajhandle = @traj_helix_ttr;

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
% controlhandle = @controller_pid_NL; % 可用
% controlhandle = @controller_pid;
controlhandle = @controller_pid_ttr_test;

% Run simulation with given trajectory generator and controller
% state - n x 13, with each row having format [x, y, z, xdot, ydot, zdot, qw, qx, qy, qz, p, q, r]

% [t, state] = simulation_3d_att_plus_1_1_over(trajhandle, controlhandle); % 姿态跟踪完成版1_1，扩展接口实现R
[t, state] = simulation_3d_ttr_test(trajhandle, controlhandle); % 添加姿态跟踪效果

% viewer
% planeplot_ttr_animation;