function [desired_state] = traj_4point_step(t, state)
    % TRAJ_4POINT_STEP generates a trajectory where the aircraft stays at each point for a fixed time
    % before jumping to the next point.

    % Define the four points (x, y, z) in the 3D space
    points = [
        0, 0, 2.5;   % Point 1
        0, 0, 3.5;   % Point 2
        5, 0, 3.5;   % Point 3
        5, 5, 3.5    % Point 4
    ];

    % Total number of points
    num_points = size(points, 1);

    % Define the time to stay at each point
    T_stay = 4;  % Time to stay at each point (seconds)
    
    % Calculate the total time to complete the trajectory
    T_total = T_stay * num_points;

    % Calculate current segment based on time t
    segment_idx = floor(t / T_stay) + 1;  % Which segment are we in (1 to 4)
    
    % If t exceeds the total duration, hover at the last point
    if segment_idx > num_points
        segment_idx = num_points;
    end

    % Start and end points for the current segment
    p_start = points(mod(segment_idx - 2, num_points) + 1, :);
    p_end = points(segment_idx, :);

    % As we use step input, velocity and acceleration are directly zero
    % at each step jump
    pos = p_end;  % Set the position to the current end point
    vel = [0; 0; 0];  % No velocity between steps
    acc = [0; 0; 0];  % No acceleration

    % Yaw and yawdot calculation (constant yaw towards the next point)
    yaw = atan2(vel(2), vel(1));
    speed_sq = vel(1)^2 + vel(2)^2;
    if speed_sq > 1e-6
        yawdot = (vel(1) * acc(2) - vel(2) * acc(1)) / speed_sq;
    else
        yawdot = 0;
    end

    % Output the desired state
    desired_state.pos = pos(:);
    desired_state.vel = vel(:);
    desired_state.acc = acc(:);
    desired_state.yaw = yaw;
    desired_state.yawdot = yawdot;
    desired_state.Va = sqrt(vel(1)^2 + vel(2)^2 + vel(3)^2);  % True airspeed (zero in step)
    desired_state.mode = 2;  % Mode 2 for cruise

end
