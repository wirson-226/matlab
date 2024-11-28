classdef WingPlot < handle
    % WINGPLOT Visualization class for fixed-wing aircraft

    properties (SetAccess = public)
        k = 0;
        qn;             % Aircraft number
        time = 0;       % Time
        state;          % Current state
        des_state;      % Desired state [x; y; z; xdot; ydot; zdot; pitch; roll; yaw]
        rot;            % Rotation matrix from body to world
        wingspan;       % Wingspan
        length;         % Aircraft length
        color;          % Aircraft color
        motor_pos;      % Position of the engines

        state_hist;     % Position history
        state_des_hist; % Desired position history
        time_hist;      % Time history
        max_iter;       % Max iteration
    end

    properties (SetAccess = private)
        h_3d
        h_wing;         % Wing plot handle
        h_tail;         % Tail plot handle
        h_pos_hist;     % Position history handle
        h_pos_des_hist; % Desired position history handle
        h_qn;           % Aircraft number handle
        text_dist;      % Distance between aircraft number and aircraft
    end

    methods
        % Constructor
        function W = WingPlot(qn, state, wingspan, length, color, max_iter, h_3d)
            W.qn = qn;
            W.state = state;
            W.wingspan = wingspan;
            W.length = length;
            W.color = color;
            W.rot = QuatToRot(W.state(7:9)); % Rotation matrix
            W.motor_pos = [W.state(1) W.state(2) W.state(3)]; % Aircraft position
            W.text_dist = W.wingspan / 3;
            W.des_state = W.state(1:6); % Only position and velocity for now

            W.max_iter = max_iter;
            W.state_hist = zeros(6, max_iter);
            W.state_des_hist = zeros(6, max_iter);
            W.time_hist = zeros(1, max_iter);

            % Initialize plot handle
            if nargin < 7, h_3d = gca; end
            W.h_3d = h_3d;
            hold(W.h_3d, 'on');
            W.h_pos_hist = plot3(W.h_3d, W.state(1), W.state(2), W.state(3), 'r.');
            W.h_pos_des_hist = plot3(W.h_3d, W.des_state(1), W.des_state(2), W.des_state(3), 'b.');
            W.h_wing = plot3(W.h_3d, [0 0], [0 0], [0 0], 'Color', W.color, 'LineWidth', 2); % placeholder for wing
            W.h_tail = plot3(W.h_3d, [0 0], [0 0], [0 0], 'Color', W.color, 'LineWidth', 2); % placeholder for tail
            W.h_qn = text( ...
                W.state(1) + W.text_dist, ...
                W.state(2) + W.text_dist, ...
                W.state(3) + W.text_dist, num2str(qn));
            hold(W.h_3d, 'off');
        end

        % Update aircraft state
        function UpdateWingState(W, state, time)
            W.state = state;
            W.time = time;
            W.rot = QuatToRot(state(7:10))'; % Body-to-world rotation matrix
        end

        % Update desired aircraft state
        function UpdateDesiredWingState(W, des_state)
            W.des_state = des_state;
        end

        % Update aircraft history
        function UpdateWingHist(W)
            W.k = W.k + 1;
            W.time_hist(W.k) = W.time;
            W.state_hist(:,W.k) = W.state(1:6);
            W.state_des_hist(:,W.k) = W.des_state(1:6);
        end

        % Update aircraft position
        function UpdateMotorPos(W)
            W.motor_pos = [W.state(1) W.state(2) W.state(3)];
        end

        % Truncate history
        function TruncateHist(W)
            W.time_hist = W.time_hist(1:W.k);
            W.state_hist = W.state_hist(:, 1:W.k);
            W.state_des_hist = W.state_des_hist(:, 1:W.k);
        end

        % Update aircraft plot
        function UpdateWingPlot(W, state, des_state, time)
            W.UpdateWingState(state, time);
            W.UpdateDesiredWingState(des_state);
            W.UpdateWingHist();
            W.UpdateMotorPos();

            % Update wing plot (draw a simple 3D plane representation)
            % Assuming a simple box shape for the wing
            wing_x = [-W.wingspan/2, W.wingspan/2, W.wingspan/2, -W.wingspan/2];
            wing_y = [-W.length/2, -W.length/2, W.length/2, W.length/2];
            wing_z = [0, 0, 0, 0];
            
            wing_points = W.rot * [wing_x; wing_y; wing_z];
            set(W.h_wing, 'XData', wing_points(1,:) + W.state(1));
            set(W.h_wing, 'YData', wing_points(2,:) + W.state(2));
            set(W.h_wing, 'ZData', wing_points(3,:) + W.state(3));

            % Update tail (a simple representation)
            tail_x = [-W.wingspan/5, W.wingspan/5];
            tail_y = [0, 0];
            tail_z = [-W.length/4, W.length/4];
            tail_points = W.rot * [tail_x; tail_y; tail_z];
            set(W.h_tail, 'XData', tail_points(1,:) + W.state(1));
            set(W.h_tail, 'YData', tail_points(2,:) + W.state(2));
            set(W.h_tail, 'ZData', tail_points(3,:) + W.state(3));

            % Update aircraft number position
            set(W.h_qn, 'Position', [W.state(1) + W.text_dist, W.state(2) + W.text_dist, W.state(3) + W.text_dist]);

            % Update position history
            set(W.h_pos_hist, 'XData', W.state_hist(1,1:W.k), 'YData', W.state_hist(2,1:W.k), 'ZData', W.state_hist(3,1:W.k));
            set(W.h_pos_des_hist, 'XData', W.state_des_hist(1,1:W.k), 'YData', W.state_des_hist(2,1:W.k), 'ZData', W.state_des_hist(3,1:W.k));

            drawnow;
        end
    end

end
