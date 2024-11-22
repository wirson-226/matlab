function desired_state = traj_circle(t, agent_index)
    t_max = 20;
    t = max(0, min(t, t_max));
    t_norm = t / t_max;

    % Regular pentagon in xy plane with radius 1, counterclockwise
    radius = 1;
    angular_speed = 2 * pi / t_max; % Angular speed for full circle in t_max seconds
    angle = angular_speed * t - 2 * pi * (agent_index - 1) / 5;
    
    pos_base = radius * [cos(angle); sin(angle); 0];

    % Velocity and acceleration calculations
    vel_base = radius * angular_speed * [-sin(angle); cos(angle); 0];
    acc_base = radius * angular_speed^2 * [-cos(angle); -sin(angle); 0];

    % Output desired state
    desired_state.pos = pos_base;
    desired_state.vel = vel_base;
    desired_state.acc = acc_base;
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
end

