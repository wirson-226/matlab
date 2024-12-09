%%% 动画演示 --- 摆 %%%
addpath('images');


% main begins
% drawPendulumSimulation;

% Simulation parameters
dt = 0.05;       % Time step
T = 10;          % Total time
time = 0:dt:T;   % Time vector
speed = 1;


% Parameters
% Initialize the figure and set up key press functionality
fig = figure('Name', 'Pendulum Simulation', 'KeyPressFcn', @keyPressCallback);
global isPaused; % Global variable to track pause state
isPaused = false;

% Create a handle for the time text display
timeTextHandle = text(0.1, 1.1, 'Time: 0.00', 'FontSize', 12, 'Color', 'k');

% Animation loop
for t = time
    y = 0;                % Base stays stationary
    theta = pi / 4 * sin(2 * pi * t / T); % Pendulum oscillates

    % Call the drawPendulum function
    drawPendulum([y, theta, t]);

    % Update the time display
    % set(timeTextHandle, 'String', sprintf('Time: %.2f s', t));

    % Check if paused
    while isPaused
        pause(0.1); % Small delay to prevent busy-waiting
    end

    % Pause for animation frame
    pause(dt / speed);

    % Check if figure is valid
    if ~isvalid(fig)
        break;
    end
end




% main ends

%%% fuctions %%%


function drawPendulum(u)
    % Process inputs to the function
    y = u(1);       % Horizontal position of the base
    theta = u(2);   % Angle of the pendulum rod
    t = u(3);       % Current time

    % Drawing parameters
    L = 1;          % Length of the pendulum rod
    gap = 0.01;     % Gap between the base and rod
    width = 1.0;    % Width of the base
    height = 0.1;   % Height of the base

    % Define persistent variables
    persistent base_handle rod_handle

    % First time the function is called, initialize the plot and persistent vars
    if t == 0
        figure(1), clf;
        track_width = 3; % Width of the track
        hold on;
        plot([-track_width, track_width], [0, 0], 'k'); % Track line
        base_handle = drawBase(y, width, height, gap, [], 'normal');
        rod_handle = drawRod(y, theta, L, gap, height, [], 'normal');
        axis([-track_width, track_width, -L, 2 * track_width - L]);
        hold off;
    else
        % At every other time step, redraw base and rod
        drawBase(y, width, height, gap, base_handle);
        drawRod(y, theta, L, gap, height, rod_handle);
    end
end

% Function to draw the base
function handle = drawBase(y, width, height, gap, handle, mode)
    if isempty(handle)
        % Draw base for the first time
        x = [y - width / 2, y + width / 2, y + width / 2, y - width / 2];
        y_coords = [gap, gap, gap + height, gap + height];
        handle = fill(x, y_coords, 'b');
    else
        % Update base position
        x = [y - width / 2, y + width / 2, y + width / 2, y - width / 2];
        set(handle, 'XData', x);
    end
end

% Function to draw the rod
function handle = drawRod(y, theta, L, gap, height, handle, mode)
    x1 = y; 
    y1 = gap + height; 
    x2 = x1 + L * sin(theta);
    y2 = y1 - L * cos(theta);
    
    if isempty(handle)
        % Draw rod for the first time
        handle = plot([x1, x2], [y1, y2], 'r', 'LineWidth', 2);
    else
        % Update rod position
        set(handle, 'XData', [x1, x2], 'YData', [y1, y2]);
    end
end





% 增加 暂停继续 中断模块 时间显示加速减速等功能

function keyPressCallback(~, event)
    % Callback for key press
    global isPaused isRunning;
    persistent speed; % Animation speed
    if isempty(speed), speed = 1; end

    switch event.Key
        case 'space'
            isPaused = ~isPaused; % Toggle pause/resume
        case 'q'
            isRunning = false;   % Stop animation
            close(gcf);          % Close figure
        case 'r'
            speed = 1;           % Reset speed
            disp('Animation reset.');
        case 'uparrow'
            speed = speed + 0.1; % Increase speed
            disp(['Speed increased to ', num2str(speed)]);
        case 'downarrow'
            speed = max(speed - 0.1, 0.1); % Decrease speed (min 0.1)
            disp(['Speed decreased to ', num2str(speed)]);
        case 'w'
            ylim(get(gca, 'YLim') + [0.1, 0.1]); % Move view up
        case 's'
            ylim(get(gca, 'YLim') - [0.1, 0.1]); % Move view down
        case 'a'
            xlim(get(gca, 'XLim') - [0.1, 0.1]); % Move view left
        case 'd'
            xlim(get(gca, 'XLim') + [0.1, 0.1]); % Move view right
        case 'z'
            zoom(1.1); % Zoom in
        case 'x'
            zoom(0.9); % Zoom out
        otherwise
            disp(['Unhandled key: ', event.Key]); % Debug unknown keys
    end
end






% 图片注释参数表格附录展示

% The following code shows the setup diagram for the animation tool keyboard.
% 创建一个图形窗口并设置名称
fig = figure('name', 'Animation keyboard', 'NumberTitle', 'off');
% figure;
% 在窗口中显示图像
imshow('E:\documents\Codes\codes\matlab\6_DOF_formation\3-D+tilt_trirotor+Control\Airplane_sim_init_xuan\test_tools\images\keyboard_event.png'); % Load the image (ensure the file path is correct)
title('keyboard event');
