function [t_out, s_out] = simulation_3d_ttr_test(trajhandle, controlhandle)
% 在1.0基础推进了步长设置调整，自适应ode45与状态输出矩阵的规格
% NOTE: This script will not run as expected unless you fill in proper
% code in trajhandle and controlhandle
% You should not modify any part of this script except for the
% visualization part
%
% ***************** QUADROTOR SIMULATION *****************

% *********** YOU SHOULDN'T NEED TO CHANGE ANYTHING BELOW **********

addpath('utils');
addpath('traj');
addpath('controller');
addpath('test_tools');
addpath('test_airplane');

% real-time
real_time = true;

% max time
max_time = 30;

% parameters for simulation
params = sys_params;

%% **************************** FIGURES *****************************
disp('Initializing figures...');
figure;
axis equal;
grid on;
hold on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Airplane flight');
view(3);  % 3D视图
xlim([-10 10]); ylim([-10 10]); zlim([-5 5]); % 设置坐标轴范围


% axis equal
% grid on
% view(3);
% xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
set(gcf,'Renderer','OpenGL')

%% *********************** INITIAL CONDITIONS ***********************
disp('Setting initial conditions...');
tstep    = 0.01; % this determines the time step at which the solution is given
cstep    = 0.05; % image capture time interval
max_iter = max_time/cstep; % max iteration
nstep    = round(cstep/tstep); % 保证整数
time     = 0; % current time
err = []; % runtime errors

% Get start and stop position
des_start = trajhandle(0, []);
des_stop  = trajhandle(inf, []);
stop_pos  = des_stop.pos;
x0    = init_state(des_start.pos, 0);
xtraj = zeros(max_iter*nstep, length(x0));
% dtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);
% pos_4_plot = [0,0,0];
% att_4_plot = [0,0,0];
tilt_angle = [0,0]; % degrees


attitudetraj = zeros(max_iter*nstep, 3); % To record attitude (phi, theta, psi)
attitude_des_traj = zeros(max_iter*nstep, 3);
position_des_traj = zeros(max_iter*nstep, 3);
velocity_des_traj = zeros(max_iter*nstep,3);
tilt_des_traj = zeros(max_iter*nstep, 2);


x       = x0;        % state

pos_tol = 0.01;
vel_tol = 0.01;

% Initialize trajectory storage
actual_trajectory = []; % To store actual positions
desired_trajectory = []; % To store desired positions


%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');

% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;
    tic;

    % Run simulation
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x); % added att
    x    = xsave(end, :)'; % x =  [13 * 1] transform of each loop final result of xasve = [6 * 13] 0; 0.01; 0.02; 0.03; 0.04; 0.05;
    

    % Add state save for plotting
    % Current attitude  
    % 关键处，保证与时间步长迭代的一致性，同时数组规格统一
    att_current_save = zeros(length(tsave), 3);
    att_des_save = zeros(length(tsave), 3);
    position_des_save = zeros(length(tsave), 3);
    velocity_des_save = zeros(length(tsave), 3);
    des_tilt4_save = zeros(length(tsave), 2); 

    
    % Update trajectory
    actual_trajectory = [actual_trajectory; xsave(:, 1:3)]; % Store actual positions

    for i = 1:length(tsave)
        % Current attitude
        current_all_state = stateToQd(xsave(i, :));
        att_current_save(i, :) = current_all_state.rot'; % qd.rot = [phi; theta; yaw]; current [3 * 1]' = [1 * 3]
        
        % Desired attitude
        % tsave(i) 跟进时间步长迭代
        desired_state = trajhandle(tsave(i), current_all_state);
        [~, ~, desired_attitude,des_tilt4_save(i, :)] = controlhandle(tsave(i), current_all_state, desired_state, params); % 添加记录便于输出
        att_des_save(i, :) = desired_attitude';
        position_des_save(i, :) = desired_state.pos';
        velocity_des_save(i, :) = desired_state.vel';
        desired_trajectory = [desired_trajectory; desired_state.pos']; % Store desired positions
    end




    %% aircraft plot
    % 清除之前绘制
    cla; % 只清除当前窗口的内容
    % 绘制飞机模型 --- done 
    pos_4_plot = x(1:3)';
    rot_4_plot = QuatToRot(x(7:10));
    [phi,theta,psi]= RotToRPY_ZXY(rot_4_plot);
    att_4_plot = [phi,theta,psi];
    planeplot_ttr_test(pos_4_plot,att_4_plot,tilt_angle);
    tilt_angle = desired_state.tilt_angle;

    % Plot trajectories (actual and desired)
    plot3(actual_trajectory(:, 1), actual_trajectory(:, 2), actual_trajectory(:, 3), 'b-', 'LineWidth', 3);
    plot3(desired_trajectory(:, 1), desired_trajectory(:, 2), desired_trajectory(:, 3), 'r-', 'LineWidth', 3);


   
    % % % 更新视角：相机位置始终跟随飞机
    camera_target = pos_4_plot;  % 相机始终跟随飞机
    camera_position = camera_target + [-22, -10, 20];  % 设置相机位置，稍微偏离目标（例如20单位远）
    campos(camera_position);          % 设置相机位置


    % Save to traj 每个五步一存记录
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:); % end -1 取消每一步最后状态作为下一步初始的重复
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

    % Save attitudetraj
    position_des_traj((iter-1)*nstep+1:iter*nstep, :) = position_des_save(1:end-1, :); % desired pos
    velocity_des_traj((iter-1)*nstep+1:iter*nstep, :) = velocity_des_save(1:end-1, :); % desired pos
    tilt_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_tilt4_save(1:end-1, :); % desired tilt save with iter
    attitude_des_traj((iter-1)*nstep+1:iter*nstep, :) = att_des_save(1:end-1, :); % desired att
    attitudetraj((iter-1)*nstep+1:iter*nstep, :) = att_current_save(1:end-1,:); % real state att

    time = time + cstep; % Update simulation time
    t = toc;
    % Check to make sure ode45 is not timing out
    if(t> cstep*50)
        err = 'Ode45 Unstable';
        break;
    end

    % Pause to make real-time
    if real_time && (t < cstep)
        pause(cstep - t);
    end

    % Check termination criteria
    if terminate_check(x, time, stop_pos, pos_tol, vel_tol, max_time)
        break;
    end
    drawnow;  % 更新图形
end

%% ************************* POST PROCESSING *************************
% --- to do---plot 单独函数封装

% Truncate xtraj and ttraj  停留在当前步数状态数据，留存以防跳出流失数据
xtraj = xtraj(1:iter*nstep,:);  % Position and other state variables
ttraj = ttraj(1:iter*nstep);
attitudetraj = attitudetraj(1:iter*nstep, :); % Truncate attitude trajectory
attitude_des_traj = attitude_des_traj(1:iter*nstep, :); % Truncate desired attitude trajectory
position_des_traj = position_des_traj(1:iter*nstep, :); 
velocity_des_traj = velocity_des_traj(1:iter*nstep, :);
tilt_des_traj = tilt_des_traj(1:iter*nstep, :);

%% Plot position
h_pos = figure('Name', 'Position');
subplot(3,1,1);
plot(ttraj, xtraj(:,1), 'b', 'LineWidth', 1.5);  % Plot x position
hold on;
% 如果有预期轨迹，绘制预期轨迹
plot(ttraj, position_des_traj(:,1), 'r--', 'LineWidth', 1.5);  % Desired x position
xlabel('Time [s]');
ylabel('X [m]');
legend('Actual X', 'Desired X');
title('Position');
grid on;

subplot(3,1,2);
plot(ttraj, xtraj(:,2), 'b', 'LineWidth', 1.5);  % Plot y position
hold on;
plot(ttraj, position_des_traj(:,2), 'r--', 'LineWidth', 1.5);  % Desired y position
xlabel('Time [s]');
ylabel('Y [m]');
legend('Actual Y', 'Desired Y');
grid on;

subplot(3,1,3);
plot(ttraj, xtraj(:,3), 'b', 'LineWidth', 1.5);  % Plot z position
hold on;
plot(ttraj, position_des_traj(:,3), 'r--', 'LineWidth', 1.5);  % Desired z position
xlabel('Time [s]');
ylabel('Z [m]');
legend('Actual Z', 'Desired Z');
grid on;

%% Plot velocity
h_vel = figure('Name', 'Velocity');
subplot(3,1,1);
plot(ttraj, xtraj(:,4), 'b', 'LineWidth', 1.5);  % Plot x velocity
hold on;
plot(ttraj, velocity_des_traj(:,1), 'r--', 'LineWidth', 1.5);  % Desired x velocity
xlabel('Time [s]');
ylabel('Vx [m/s]');
legend('Actual Vx', 'Desired Vx');
title('Velocity');
grid on;

subplot(3,1,2);
plot(ttraj, xtraj(:,5), 'b', 'LineWidth', 1.5);  % Plot y velocity
hold on;
plot(ttraj, velocity_des_traj(:,2), 'r--', 'LineWidth', 1.5);  % Desired y velocity
xlabel('Time [s]');
ylabel('Vy [m/s]');
legend('Actual Vy', 'Desired Vy');
grid on;

subplot(3,1,3);
plot(ttraj, xtraj(:,6), 'b', 'LineWidth', 1.5);  % Plot z velocity
hold on;
plot(ttraj, velocity_des_traj(:,3), 'r--', 'LineWidth', 1.5);  % Desired z velocity
xlabel('Time [s]');
ylabel('Vz [m/s]');
legend('Actual Vz', 'Desired Vz');
grid on;

%% Plot attitudes
h_attitude = figure('Name', 'Attitude');
subplot(3,1,1);
plot(ttraj, rad2deg(attitudetraj(:,1)), 'b', 'LineWidth', 1.5);
hold on;
plot(ttraj, rad2deg(attitude_des_traj(:,1)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Roll [deg]');
legend('Actual Roll', 'Desired Roll');
title('Attitude');
grid on;

subplot(3,1,2);
plot(ttraj, rad2deg(attitudetraj(:,2)), 'b', 'LineWidth', 1.5);
hold on;
plot(ttraj, rad2deg(attitude_des_traj(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Pitch [deg]');
legend('Actual Pitch', 'Desired Pitch');
grid on;

subplot(3,1,3);
plot(ttraj, rad2deg(attitudetraj(:,3)), 'b', 'LineWidth', 1.5);
hold on;
plot(ttraj, rad2deg(attitude_des_traj(:,3)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Yaw [deg]');
legend('Actual Yaw', 'Desired Yaw');
grid on;


%% Plot 7 input -- thrust abc + tilt ab + elevon ab
% now just 2--tilt ab 
h_actutor = figure('Name', 'Actutor');
% subplot(3,1,1);
% plot(ttraj, rad2deg(attitudetraj(:,1)), 'b', 'LineWidth', 1.5);
% hold on;
% plot(ttraj, rad2deg(attitude_des_traj(:,1)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]');
% ylabel('Roll [deg]');
% legend('Actual Roll', 'Desired Roll');
% title('Actutor');
% grid on;
% 
% subplot(3,1,2);
% plot(ttraj, rad2deg(attitudetraj(:,2)), 'b', 'LineWidth', 1.5);
% hold on;
% plot(ttraj, rad2deg(attitude_des_traj(:,2)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]');
% ylabel('Pitch [deg]');
% legend('Actual Pitch', 'Desired Pitch');
% grid on;

subplot(3,1,1);
plot(ttraj, tilt_des_traj(:,1), 'b', 'LineWidth', 1.5); % arm a, right
hold on;
plot(ttraj, tilt_des_traj(:,2), 'r--', 'LineWidth', 1.5); % arm b, left
xlabel('Time [s]');
ylabel('titl_angle [deg]');
legend('right arm', 'left arm');
title('Actutor');
grid on;

if(~isempty(err))
    error(err);
end

disp('finished.')

t_out = ttraj;
s_out = xtraj;

end





%% 后续整合利用，目前暂存
function [ h_fig ] = plot_state( h_fig, state, time, name, type, view )
%PLOT_STATE visualize state data

if nargin < 6, view = 'sep'; end
if nargin < 5, type = 'vic'; end
if nargin < 4, name = 'pos'; end
if isempty(h_fig), h_fig = figure(); end
line_width = 2;

switch type
    case 'vic'
        line_color = 'r';
    case 'des'
        line_color = 'b';
    case 'est'
        line_color = 'g';
end

switch name
    case 'pos'
        labels = {'x [m]', 'y [m]', 'z [m]'};
    case 'vel'
        labels = {'xdot [m/s]', 'ydot [m/s]', 'zdot [m/s]'};
    case 'euler'
        labels = {'roll [rad]', 'pitch [rad]', 'yaw [rad]'};
end

figure(h_fig)
if strcmp(view, 'sep')
    % Plot seperate

    for i = 1:3
        subplot(3, 1, i)
        hold on
        plot(time, state(i,:), line_color, 'LineWidth', line_width);
        hold off
        xlim([time(1), time(end)])
        grid on
        xlabel('time [s]')
        ylabel(labels{i})
    end
elseif strcmp(view, '3d')
    % Plot 3d
    hold on
    plot3(state(1,:), state(2,:), state(3,:), line_color, 'LineWidth', line_width)
    hold off
    grid on
    xlabel(labels{1});
    ylabel(labels{2});
    zlabel(labels{3});
end

end
