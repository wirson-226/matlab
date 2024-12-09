function [F, M, ATT_des] = controller_pid(t, state, des_state, params)
%   CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%   ATT_des = [phi_des; theta_des; psi_des];
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
%   PD mian core




% =================== Your code goes here ===================

% % Controller gains
%大角度--todo--待调试
% Kp_pos = [20; 20; 50];  % Position proportional gain
% Kd_pos = [5; 5; 10];  % Position derivative gain
% 
% Kp_att = [0.005; 0.005; 0.005]; % Attitude proportional gain
% Kd_att = [0.002; 0.002; 0.002];    % Attitude derivative gain
% Kp_att = [50; 50; 50]; % Attitude proportional gain
% Kd_att = [2; 2; 2];    % Attitude derivative gain


% Controller gains
%小角度--可用
Kp_pos = [0.1; 0.1; 4];  % Position proportional gain
Kd_pos = [2; 2; 2];  % Position derivative gain

Kp_att = [0.005; 0.005; 0.005]; % Attitude proportional gain
Kd_att = [0.002; 0.002; 0.002];    % Attitude derivative gain





% Position control
pos_err = des_state.pos - state.pos;
vel_err = des_state.vel - state.vel;

% Compute desired acceleration with saturation
% 这并不是实际的轨迹加速度，而是虚拟控制加速度，一种控制量增益的结果，并无物理意义；下面att_des是同理
acc_des = Kp_pos .* pos_err + Kd_pos .* vel_err;    



% Limit the maximum acceleration
acc_max = 10; % m/s^2
acc_des = max(min(acc_des, acc_max), -acc_max);

% Compute desired thrust
u1 = params.mass * (params.gravity + acc_des(3));

% =================== Desired Angles Calculation ===================


des_stateyaw = atan2(des_state.pos(2) - state.pos(2), des_state.pos(1) - state.pos(1)); %  视线导引 期望偏航角
% des_stateyaw = 0; % 取消偏航
% des_stateyaw = des_state.yaw; % 跟踪轨迹既定偏航
phi_des = (acc_des(1) * sin(des_stateyaw) - acc_des(2) * cos(des_stateyaw)) / params.gravity;
theta_des = (acc_des(1) * cos(des_stateyaw) + acc_des(2) * sin(des_stateyaw)) / params.gravity;
psi_des = des_stateyaw;


% Limit the desired angles
phi_max = pi/4; % ±45 degrees
theta_max = pi/4; % ±45 degrees
psi_max = pi; % ±** degrees

phi_des = max(min(phi_des, phi_max), -phi_max);
theta_des = max(min(theta_des, theta_max), -theta_max);
psi_des = max(min(psi_des, psi_max), -psi_max);



% Desired angular velocities (assumed to be zero for hover)
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

% Current state
phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

% Attitude control using PD control laws
u2 = [Kp_att(1) * (phi_des - phi) + Kd_att(1) * (p_des - p);
      Kp_att(2) * (theta_des - theta) + Kd_att(2) * (q_des - q);
      Kp_att(3) * (psi_des - psi) + Kd_att(3) * (r_des - r)];

% Output force and moments
F = u1;
M = u2;
ATT_des = [phi_des; theta_des; psi_des]; % [3 * 1]

% =================== Your code ends here ===================

end
