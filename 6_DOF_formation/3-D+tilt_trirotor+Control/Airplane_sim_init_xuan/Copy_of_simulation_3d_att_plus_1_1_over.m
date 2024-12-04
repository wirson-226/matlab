function [t_out, s_out] = Copy_of_simulation_3d_att_plus_1_1_over(trajhandle, controlhandle)
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
max_time = 1;

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
xlim([-10 10]); ylim([-10 10]); zlim([-1 1]); % 设置坐标轴范围


% axis equal
% grid on
% view(3);
% xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]')
% set(gcf,'Renderer','OpenGL')

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
ttraj = zeros(max_iter*nstep, 1);
% pos_4_plot = [0,0,0];
% att_4_plot = [0,0,0];
tilt_angle = [0,0];


attitudetraj = zeros(max_iter*nstep, 3); % To record attitude (phi, theta, psi)
attitude_des_traj = zeros(max_iter*nstep, 3);

x       = x0;        % state

pos_tol = 0.01;
vel_tol = 0.01;




%% ************************* RUN SIMULATION *************************
disp('Simulation Running....');

% Main loop
for iter = 1:max_iter

    timeint = time:tstep:time+cstep;
    tic;

    % Run simulation
    [tsave, xsave] = ode45(@(t,s) quadEOM(t, s, controlhandle, trajhandle, params), timeint, x); % added att
    x    = xsave(end, :)'; % x =  [13 * 1] transform of each loop final result of xasve = [6 * 13] 0; 0.01; 0.02; 0.03; 0.04; 0.05;
    
    %% aircraft plot
    % 清除之前绘制
    cla; % 只清除当前窗口的内容
    % 绘制飞机模型 --- done 
    pos_4_plot = x(1:3)';
    rot_4_plot = QuatToRot(x(7:10));
    [phi,theta,psi]= RotToRPY_ZXY(rot_4_plot);
    att_4_plot = [phi,theta,psi];
    planeplot_ttr_test(pos_4_plot,att_4_plot,tilt_angle);


    % planeplot_ttr_test([1,1,1], [0,0,0], [0,0]);

    % % 绘制每架飞机的预期轨迹和实际轨迹 --- todo
    % plot3(desired_trajectories(1:t, 1, i), desired_trajectories(1:t, 2, i), desired_trajectories(1:t, 3, i), 'g--', 'LineWidth', 1.5); % 预期轨迹
    % plot3(current_trajectories(1:t, 1, i), current_trajectories(1:t, 2, i), current_trajectories(1:t, 3, i), 'b', 'LineWidth', 1);
    % 
    % % % 更新视角：相机位置始终跟随飞机
    camera_target = pos_4_plot;  % 相机始终跟随飞机
    camera_position = camera_target + [-22, -10, 20];  % 设置相机位置，稍微偏离目标（例如20单位远）
    campos(camera_position);          % 设置相机位置


    % Add attitude save
    % Current attitude  
    % 关键处，保证与时间步长迭代的一致性，同时数组规格统一
    att_current_save = zeros(length(tsave), 3);
    att_des_save = zeros(length(tsave), 3);

    for i = 1:length(tsave)
        % Current attitude
        current_all_state = stateToQd(xsave(i, :));
        att_current_save(i, :) = current_all_state.rot'; % qd.rot = [phi; theta; yaw]; current [3 * 1]' = [1 * 3]
        
        % Desired attitude
        % tsave(i) 跟进时间步长迭代
        desired_state = trajhandle(tsave(i), current_all_state);
        [~, ~, desired_attitude] = controlhandle(tsave(i), current_all_state, desired_state, params);
        att_des_save(i, :) = desired_attitude';
    end

    % Save to traj 每个五步一存记录
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:); % end -1 取消每一步最后状态作为下一步初始的重复
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

    % Save attitudetraj
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

% Truncate xtraj and ttraj  停留在当前步数状态数据，留存以防跳出流失数据
xtraj = xtraj(1:iter*nstep,:);
ttraj = ttraj(1:iter*nstep);
attitudetraj = attitudetraj(1:iter*nstep, :); % Truncate attitude trajectory
attitude_des_traj = attitude_des_traj(1:iter*nstep, :); % Truncate desired attitude trajectory
% 
% % Truncate saved variables
% QP.TruncateHist();
% 
% Plot position
% h_pos = figure('Name', ['Quad position']);
% plot_state(h_pos, xtraj(:,1:3), ttraj, 'pos', 'vic');
% plot_state(h_pos, xtraj(:,4:6), ttraj, 'pos', 'des');
% Plot velocity
% h_vel = figure('Name', ['Quad velocity']);
% plot_state(h_vel, QP.state_hist(4:6,:), ttraj, 'vel', 'vic');
% plot_state(h_vel, QP.state_des_hist(4:6,:), ttraj, 'vel', 'des');

% Plot attitudes
h_attitude = figure('Name', 'Quad Attitude');
subplot(3,1,1);
plot(ttraj, rad2deg(attitudetraj(:,1)), 'b', 'LineWidth', 1.5);
hold on;
plot(ttraj, rad2deg(attitude_des_traj(:,1)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Roll [deg]');
legend('Actual Roll', 'Desired Roll');
title('Quad Attitude');
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

if(~isempty(err))
    error(err);
end

disp('finished.')

t_out = ttraj;
s_out = xtraj;
% attitude_out = attitudetraj;
% attitude_des_out = attitude_des_traj;
end
