function [desired_state] = traj_4point_cruise(t, state)
    % TRAJ_4POINT_CRUISE generates a trajectory that makes the aircraft cruise
    % through four fixed points. Once the aircraft reaches a point, it proceeds to the next.
    % mode = 1: copter, mode = 2: cruise, mode = 3: transition

    % Define the four points (x, y, z) in the 3D space
    points = [
        0, 0, 2.5;   % Point 1
        5, 0, 2.5;   % Point 2
        5, 5, 2.5;   % Point 3
        0, 5, 2.5    % Point 4
    ];

    % Total number of points
    num_points = size(points, 1);

    % Calculate the time duration to reach each point
    T_segment = 5;  % Time to reach each point (seconds)
    
    % Calculate current segment based on time t
    segment_idx = floor(t / T_segment) + 1;  % Which segment are we in (1 to 4)
    
    % If t exceeds the total duration, hover at the last point
    if segment_idx > num_points
        segment_idx = num_points;
    end

    % Start and end points for the current segment
    p_start = points(mod(segment_idx - 2, num_points) + 1, :);
    p_end = points(segment_idx, :);

    % Calculate the current position, velocity, and acceleration
    % Linear interpolation between the current and next point
    alpha = (t - (segment_idx - 1) * T_segment) / T_segment;  % Interpolation factor (0 to 1)

    pos = p_start + alpha * (p_end - p_start);  % Interpolated position
    vel = (p_end - p_start) / T_segment;  % Constant velocity between points
    acc = zeros(3, 1);  % No acceleration in a straight line, only during transitions

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
    desired_state.Va = sqrt(vel(1)^2 + vel(2)^2 + vel(3)^2);  % True airspeed
    desired_state.mode = 2;  % Mode 2 for cruise

end
