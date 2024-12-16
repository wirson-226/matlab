function [F, M, des_from_ctrl,command] = att_controller(t, state, des_state, params)
%   CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%%   Current state
%   phi = state.rot(1);
%   theta = state.rot(2);
%   psi = state.rot(3);
%   p = state.omega(1);
%   q = state.omega(2);
%   r = state.omega(3);


%   des_state: The desired states are:
%   des_state.pos = [x; y; z], yaw_cmd = des_state.yaw

%   Using these current and desired states, you have to compute the desired
%   controls：des_from_ctrl = [vn_cmd, ve_cmd, vd_cmd, roll_cmd, pitch_cmd, yaw_cmd, p_cmd, q_cmd, r_cmd, Mx_cmd, My_cmd, Mz_cmd];




% Position control -- velocity_des from position error using PIDControl(class)
% PID gains from params..... also the limits of every virables
pos_err = des_state.pos - state.pos; % pn_err = pos_err（1）,pe_err = pos_err（2),ps_err = pos_err（3), 北东天方向
vn_cmd = PIDControl(); % n north
ve_cmd = PIDControl(); % e east
vs_cmd = PIDControl(); % s sky


% velocity control -- attitudes cmd with acceleration_des from velocity error using PIDControl(class)
vel_err_n = vn_cmd - state.vel(1);
vel_err_e = ve_cmd - state.vel(2);
vel_err_s = vs_cmd - state.vel(3);
acc_des = PIDControl(); %  n e d

phi_cmd = (acc_des(1) * sin(des_state.yaw) - acc_des(2) * cos(des_state.yaw)) / params.gravity;
theta_cmd = (acc_des(1) * cos(des_state.yaw) + acc_des(2) * sin(des_state.yaw)) / params.gravity;
psi_cmd = des_state.yaw;

% Compute desired thrust
thrust_cmd = params.mass * (params.gravity + acc_des(3));

% attitude control -- p_cmd, q_cmd, r_cmd from accitudes error using PIDControl(class)
phi_err = phi_cmd - state.rot(1); % roll
theta_err = theta_cmd - state.rot(2); % pitch
psi_err = psi_cmd - state.rot(3); % yaw

p_cmd = PIDControl();
q_cmd = PIDControl();
r_cmd = PIDControl();


% omega control





%% 期望状态输出 1 * 12 --- vel-att-omege-M
des_from_ctrl = [vn_cmd, ve_cmd, vd_cmd, roll_cmd, pitch_cmd, yaw_cmd, p_cmd, q_cmd, r_cmd, Mx_cmd, My_cmd, Mz_cmd];




%% 旋翼悬停 测试用 参考Media--TTR_arm_a  --- mode 1 hovering
% 悬停 hovering 成功
% 偏航plan A

% arm_a = 0;
% arm_b = -arm_a;
% tc = (1/3) / cos(params.arm_c);
% ta = 2 * (params.l1/params.l2) * tc * cos(params.arm_c);
% tb = ta;
% 
% command.throttle = [ta,tb,tc];
% command.elevon = [0,0]; 
% command.arm = [arm_a,arm_b];


%% 满油平飞 测试用                       ---  mode 2 cruise

% elevon_a = deg2rad(0);
% elevon_b = deg2rad(0);
% 
% command.throttle = [0.2,0.2,0];
% command.arm = [pi/2,pi/2];
% command.elevon = [elevon_a, elevon_b];

%% 过渡 hovering to cruise 定高加速 测试用 --- mode 3 hov2cru (transision)
% 过渡 transition 成功
arm_a = pi/6;
arm_b = arm_a;
tc = (1/3) / cos(params.arm_c);
ta = (2 * (params.l1/params.l2) * tc * cos(params.arm_c))/cos(arm_a);
tb = ta;

command.throttle = [ta,tb,tc];
command.elevon = [0,0]; 
command.arm = [arm_a,arm_b];



%% 过渡 cruise to hovering 定高加速 测试用 --- mode 4 cru2hov (transision)

% arm_a = pi/6;
% arm_b = arm_a;
% tc = (1/3) / cos(params.arm_c);
% ta = (2 * (params.l1/params.l2) * tc * cos(params.arm_c))/cos(arm_a);
% tb = ta;
% 
% command.throttle = [ta,tb,tc];
% command.elevon = [0,0]; 
% command.arm = [arm_a,arm_b];



end
