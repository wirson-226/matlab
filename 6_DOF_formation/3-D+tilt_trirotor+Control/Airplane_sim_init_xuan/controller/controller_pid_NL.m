function [F, M, ATT_des] = controller_pid(t, state, des_state, params)
%   CONTROLLER_PID  Controller for the quadrotor with improved desired angle calculations
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

% =================== Gains ===================
Kp_pos = [0.1; 0.1; 2];  % Position proportional gain
Kd_pos = [2; 2; 2];  % Position derivative gain

Kp_att = [0.005; 0.005; 0.05]; % Attitude proportional gain
Kd_att = [0.002; 0.002; 0.02]; % Attitude derivative gain

% =================== Position Control ===================
pos_err = des_state.pos - state.pos;
vel_err = des_state.vel - state.vel;

% Compute desired acceleration with saturation
acc_des = Kp_pos .* pos_err + Kd_pos .* vel_err;

% Limit the maximum acceleration
acc_max = 10; % m/s^2
acc_des = max(min(acc_des, acc_max), -acc_max);

% Compute desired thrust
u1 = params.mass * (params.gravity + acc_des(3));

% =================== Desired Angles Calculation ===================
% Compute desired angles using non-linear relations
acc_total = sqrt(acc_des(1)^2 + acc_des(2)^2 + (acc_des(3) + params.gravity)^2);
phi_des = asin((acc_des(2) * cos(des_state.yaw) - acc_des(1) * sin(des_state.yaw)) / acc_total);
theta_des = atan2((acc_des(1) * cos(des_state.yaw) + acc_des(2) * sin(des_state.yaw)), (acc_des(3) + params.gravity));
psi_des = des_state.yaw;

% Limit the desired angles
phi_max = pi/4; % ±45 degrees
theta_max = pi/4; % ±45 degrees
phi_des = max(min(phi_des, phi_max), -phi_max);
theta_des = max(min(theta_des, theta_max), -theta_max);

% =================== Attitude Control ===================
p_des = 0;
q_des = 0;
r_des = des_state.yawdot;

phi = state.rot(1);
theta = state.rot(2);
psi = state.rot(3);
p = state.omega(1);
q = state.omega(2);
r = state.omega(3);

u2 = [Kp_att(1) * (phi_des - phi) + Kd_att(1) * (p_des - p);
      Kp_att(2) * (theta_des - theta) + Kd_att(2) * (q_des - q);
      Kp_att(3) * (psi_des - psi) + Kd_att(3) * (r_des - r)];

% =================== Output Force and Moments ===================
F = u1;
M = u2;
ATT_des = [phi_des; theta_des; psi_des]; % Desired attitude
end
