
%%%%%% related functions  %%%%%%

function [desired_state] = traj_line(t, agent_index)
t_max = 20;
t = max(0, min(t, t_max));
t = t / t_max;

pos_base = 10 * t.^3 - 15 * t.^4 + 6 * t.^5;
vel_base = (30 / t_max) * t.^2 - (60 / t_max) * t.^3 + (30 / t_max) * t.^4;
acc_base = (60 / t_max^2) * t - (180 / t_max^2) * t.^2 + (120 / t_max^2) * t.^3;

% Formation offset (regular pentagon in the xy plane)
radius = 1; % Distance from the center of the formation
angle = 2 * pi * (agent_index - 1) / 5; % Angle for pentagon

formation_offset = radius * [cos(angle); sin(angle); 0];

% output desired state
desired_state.pos = formation_offset + [pos_base; 0; 0]; % move along x-axis
desired_state.vel = [vel_base; 0; 0];
desired_state.acc = [acc_base; 0; 0];
desired_state.yaw = 0;
desired_state.yawdot = 0;
end
