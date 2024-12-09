function [desired_state] = traj_circle(t, ~)

t_max = 4;  % Total time for the trajectory
t = max(0, min(t, t_max));  % Clamp time t to [0, t_max]
t_normalized = t / t_max;  % Normalize time

% Define parameters of the circle
radius = 5;  % Radius of the circle

% Calculate position, velocity, and acceleration
pos = radius * [cos(2*pi*t_normalized); sin(2*pi*t_normalized); 0];
vel = radius * 2*pi/t_max * [-sin(2*pi*t_normalized); cos(2*pi*t_normalized); 0];
acc = radius * (2*pi/t_max)^2 * [-cos(2*pi*t_normalized); -sin(2*pi*t_normalized); 0];

% Output desired state
desired_state.pos = pos;
desired_state.vel = vel;
desired_state.acc = acc;
desired_state.yaw = 0;
desired_state.yawdot = 0;

end
