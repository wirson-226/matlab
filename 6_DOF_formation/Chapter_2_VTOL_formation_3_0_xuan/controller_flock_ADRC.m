%%%% new adrc  %%%%
%%%% 需要调试   %%%%

function [F, M, ATT_des] = controller_flock_ADRC(t, state, des_state, params, global_state, global_des_state, agent_index)
% CONTROLLER_FLOCK_PID  Flocking controller for the quadrotor
%   Implements flocking control using desired acceleration as input to the attitude controller

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

% Controller gains
K1 = [0.08; 0.08; 0.2];  % Position proportional gain
K2 = [0.3; 0.3; 0.4];  % Neighbor position gain
Kv1 = [0.5; 0.5; 0.5]; % Velocity proportional gain
Kv2 = [0.5; 0.5;0.5]; % Neighbor velocity gain
Ka = [0.5; 0.5; 0.5];  % Acceleration reference gain


% ADRC parameters
persistent z_pos;
persistent z_att;

if isempty(z_pos)
    z_pos = zeros(3, 3);  % [z_x; z_y; z_z]
    z_att = zeros(3, 3);  % [z_phi; z_theta; z_psi]
end


% Number of agents
n = size(global_state.pos, 2);

% Initialize control forces
% control_forces = zeros(3, 1);

% Desired state for the current agent
des_state_i.pos = global_des_state.pos(:, agent_index);
des_state_i.vel = global_des_state.vel(:, agent_index);
des_state_i.acc = global_des_state.acc(:, agent_index);

% Current state for the current agent
state_i.pos = global_state.pos(:, agent_index);
state_i.vel = global_state.vel(:, agent_index);

% Calculate position and velocity errors
pos_err = des_state_i.pos - state_i.pos;
vel_err = des_state_i.vel - state_i.vel;

% Calculate position control input
u1 = K1 .* pos_err;

% Calculate velocity control input
uv = Kv1 .* vel_err;

% Calculate neighbor influence
u2 = zeros(3, 1);
for j = 1:n
    if agent_index ~= j
        pos_err_neighbor = (global_state.pos(:, j) - global_state.pos(:, agent_index)) - (global_des_state.pos(:, j) - global_des_state.pos(:, agent_index));
        vel_err_neighbor = (global_state.vel(:, j) - global_state.vel(:, agent_index)) - (global_des_state.vel(:, j) - global_des_state.vel(:, agent_index));
        u2 = u2 + K2 .* pos_err_neighbor + Kv2 .* vel_err_neighbor;
    end
end

% Calculate acceleration reference input
ua = Ka .* des_state_i.acc;

% Total control force
control_forces = u1 + uv + u2 + ua;

% Desired thrust and moments
F = 0;
M = zeros(3, 1);
ATT_des = zeros(3, 1);

% gains
Kp_att = [0.005; 0.005; 0.005]; % Attitude proportional gain
Kd_att = [0.002; 0.002; 0.002];    % Attitude derivative gain

% Desired accelerations
r1_des_ddot = control_forces(1);
r2_des_ddot = control_forces(2);
r3_des_ddot = control_forces(3);

% Desired thrust
u_1 = params.mass * (params.gravity + r3_des_ddot);

% Update ESO for position
z_pos(:,1) = ESO_NL(z_pos(:,1), state.pos(1), u_1, params, 'alt');
z_pos(:,2) = ESO_NL(z_pos(:,2), state.pos(2), u_1, params, 'alt');
z_pos(:,3) = ESO_NL(z_pos(:,3), state.pos(3), u_1, params, 'alt');

% Disturbance compensation
f_pos = z_pos(3,:);

% Desired roll and pitch angles
phi_des = (r1_des_ddot * sin(global_des_state.yaw(agent_index)) - r2_des_ddot * cos(global_des_state.yaw(agent_index))) / params.gravity;
theta_des = (r1_des_ddot * cos(global_des_state.yaw(agent_index)) + r2_des_ddot * sin(global_des_state.yaw(agent_index))) / params.gravity;
psi_des = global_des_state.yaw(agent_index);

% Desired angular velocities (assumed to be zero for hover)
p_des = 0;
q_des = 0;
r_des = global_des_state.yawdot(agent_index);

% Current state
phi = global_state.rot(1, agent_index);
theta = global_state.rot(2, agent_index);
psi = global_state.rot(3, agent_index);
p = global_state.omega(1, agent_index);
q = global_state.omega(2, agent_index);
r = global_state.omega(3, agent_index);

% Update ESO for attitude
z_att(:,1) = ESO_NL(z_att(:,1), phi, 0, params, 'roll');
z_att(:,2) = ESO_NL(z_att(:,2), theta, 0, params, 'pitch');
z_att(:,3) = ESO_NL(z_att(:,3), psi, 0, params, 'yaw');

% Disturbance compensation for attitude
f_phi = z_att(3,1);
f_theta = z_att(3,2);
f_psi = z_att(3,3);

u_2 = [Kp_att(1) * (phi_des - phi) + Kd_att(1) * (p_des - p) - f_phi;
      Kp_att(2) * (theta_des - theta) + Kd_att(2) * (q_des - q) - f_theta;
      Kp_att(3) * (psi_des - psi) + Kd_att(3) * (r_des - r) - f_psi];

% Output force and moments
F = u_1 - f_pos(3);
% F = u_1;
M = u_2;

ATT_des = [phi_des; theta_des; psi_des];
end
