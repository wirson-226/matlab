function [F, M, ATT_des] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
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
%   这段代码中的加速度朝向顺序是[x, y, z]，分别对应东、北、天方向（ENU坐标系） 机头朝向 Y
%   无人机模型的机头朝向是沿着机体坐标系的Y轴正方向，这符合"右前上"(RFU)坐标系中Y轴指向前方的定义。
%   Using these current and desired states, you have to compute the desired
%   controls
%   PD mian core


% =================== Your code goes here ===================

% Controller gains
Kp_pos = [5; 5; 5];  % Position proportional gain
Kd_pos = [3; 3; 3];  % Position derivative gain

Kp_att = [200; 200; 200]; % Attitude proportional gain
Kd_att = [50; 50; 50];    % Attitude derivative gain

% Position control
pos_err = des_state.pos - state.pos;
vel_err = des_state.vel - state.vel;

acc_des = des_state.acc + Kp_pos .* pos_err + Kd_pos .* vel_err;

% Desired accelerations
r1_des_ddot = acc_des(1);
r2_des_ddot = acc_des(2);
r3_des_ddot = acc_des(3);

% Desired thrust
u1 = params.mass * (params.gravity + r3_des_ddot);

% Desired roll and pitch angles
phi_des = (r1_des_ddot * sin(des_state.yaw) - r2_des_ddot * cos(des_state.yaw)) / params.gravity;
theta_des = (r1_des_ddot * cos(des_state.yaw) + r2_des_ddot * sin(des_state.yaw)) / params.gravity;
psi_des = des_state.yaw;

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
