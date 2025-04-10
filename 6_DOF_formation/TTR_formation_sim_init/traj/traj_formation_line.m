function [desired_state] = traj_formation_line(t, agent_index)
%正六边形沿着y轴匀速直线运动的编队

    % Parameters
    radius = 8; % Radius of the hexagon
    velocity = 1; % Speed of the formation along y-axis
    
    % Calculate the angle for each agent in the hexagon
    angle = 2 * pi * (agent_index - 1) / 6;
    
    % Calculate position in hexagon formation
    x = radius * cos(angle);
    y = velocity * t; % Linear motion along y-axis
    z = radius * sin(angle);
    
    % Position vector
    pos = [x; y; z];
    
    % Output desired state
    desired_state.pos = pos;
    desired_state.yaw = 0;
    desired_state.yawdot = 0;
    desired_state.mode = 1;
end