function desired_state = traj_formation_circle_lift(t, agent_index)
    t_max = 20; % 一圈的时间
    t_norm = t / t_max;

    % Regular pentagon in xy plane with radius 1, counterclockwise
    radius = 8;
    angular_speed = 2 * pi / t_max; % Angular speed for full circle in t_max seconds
    angle = angular_speed * t - 2 * pi * (agent_index - 1) / 6; % 六个智能体
    
    pos_base = radius * [cos(angle); sin(angle); t_norm]; % Position with z as a function of time

    % Output desired state
    desired_state.pos = pos_base;
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    desired_state.mode = 1;
end
