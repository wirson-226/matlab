function [t_out, s_out] = simulation_3d_ttr(trajhandle, controlhandle)
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

addpath('test_airplane');

% real-time
real_time = true;

% max time
max_time = 20;

% parameters for simulation
params = sys_params;

% %% **************************** FIGURES *****************************
disp('Initializing figures...');
figure;
axis equal;
grid on;
hold on;
xlabel('X_东_右');
ylabel('Y_北_前');
zlabel('Z_上');
title('TTR-VTOL flight');
% view(3);  % 3D视图
scale = 10;
% view(120,30);
x1= -100*scale;
x2= 100*scale;
y1= -100*scale;
y2= 100*scale;
z1= -100*scale;
z2= 100*scale;
xlim([x1 x2]); ylim([y1 y2]); zlim([z1 z2]); % 设置坐标轴范围
view(3);
% xlim([-10 10]); ylim([-10 10]); zlim([-10 10]); % 设置坐标轴范围
% set(gca, 'YDir', 'reverse');  % 'reverse' 将 y 轴正向反转


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
x0    = init_state(des_start.pos, 0); % pos + yaw 初始在这可调其余params中定义
xtraj = zeros(max_iter*nstep, length(x0));
ttraj = zeros(max_iter*nstep, 1);
% pos_4_plot = [0,0,0];
% att_4_plot = [0,0,0];
tilt_angle = [0,0]; % degrees

% 状态记录 for plot
attitudetraj = zeros(max_iter*nstep, 3); % To record attitude (phi, theta, psi)
omegatraj = zeros(max_iter*nstep, 3); % real state omega

position_des_traj = zeros(max_iter*nstep, 3);
velocity_des_traj = zeros(max_iter*nstep,3);
attitude_des_traj = zeros(max_iter*nstep, 3);
omega_des_traj = zeros(max_iter*nstep,3);
M_des_traj = zeros(max_iter*nstep,3);


% 执行器记录 for plot
tilt_des_traj = zeros(max_iter*nstep, 2);
throttle_des_traj = zeros(max_iter*nstep, 3);
elevon_des_traj = zeros(max_iter*nstep, 2);


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
    [tsave, xsave] = ode45(@(t,s) vtolEOM(t, s, controlhandle, trajhandle, params), timeint, x); % added att
    x    = xsave(end, :)'; % x =  [13 * 1] transform of each loop final result of xasve = [6 * 13] 0; 0.01; 0.02; 0.03; 0.04; 0.05;
    

    % Add state save for plotting
    % Current attitude  
    % 关键处，保证与时间步长迭代的一致性，同时数组规格统一
    att_current_save = zeros(length(tsave), 3);
    omega_current_save = zeros(length(tsave), 3);
    att_des_save = zeros(length(tsave), 3);
    position_des_save = zeros(length(tsave), 3);
    velocity_des_save = zeros(length(tsave), 3);

    % 执行器记录 7个
    des_tilt4_save = zeros(length(tsave), 2);     % 倾转 arm_a,b
    des_throttle4_save = zeros(length(tsave), 3); % 油门 throttle_a,b,c
    des_elevon4_save = zeros(length(tsave), 2);   % 舵面 elevon a,b

    
    % Update trajectory
    actual_trajectory = [actual_trajectory; xsave(:, 1:3)]; % Store actual positions

    for i = 1:length(tsave)
        % Current attitude
        current_all_state = stateToQd(xsave(i, :));
        att_current_save(i, :) = current_all_state.rot'; % qd.rot = [phi; theta; yaw]; current [3 * 1]' = [1 * 3]
        omega_current_save(i, :) = current_all_state.omega;
        
        % Desired attitude

        desired_state = trajhandle(tsave(i), current_all_state);
        
        % 控制器解算 执行器输入
        [des_from_ctrl,command,copter_cmd] = controlhandle(tsave(i), current_all_state, desired_state, params); % 添加记录便于输出
        
        % 执行器输入记录
        des_tilt4_save(i, :) = command.arm;          % 倾转 arm_a,b
        des_throttle4_save(i, :) = command.throttle; % 油门 throttle_a,b,c
        des_elevon4_save(i, :) = command.elevon;     % 舵面 elevon_a,b

        % 状态记录 --- todo 合并保存des_from_ctrl_save  --- 300 * 12
        position_des_save(i, :) = desired_state.pos';
        des_from_ctrl_save(i, :) = des_from_ctrl; % vel-att-omege-M 加速度与姿态是一层，解算出来的
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
    tilt_angle = rad2deg(command.arm);
    planeplot_ttr(pos_4_plot,att_4_plot,tilt_angle);


    % Plot trajectories (actual and desired)
    plot3(actual_trajectory(:, 1), actual_trajectory(:, 2), actual_trajectory(:, 3), 'b-', 'LineWidth', 3);
    plot3(desired_trajectory(:, 1), desired_trajectory(:, 2), desired_trajectory(:, 3), 'r-', 'LineWidth', 3);



    % % % 更新视角：相机位置始终跟随飞机
    % % % 轨迹跟随视角（第三人称跟随模式）
    % camera_target = pos_4_plot;  % 目标始终为飞机中心
    % camera_position = camera_target + [-0.8, -0.4, 0.6];  % 相机位于飞机后方稍高处
    % campos(camera_position);  % 设置相机位置
    % camtarget(camera_target); % 让相机朝向飞机


    % Save to traj 每个五步一存记录
    xtraj((iter-1)*nstep+1:iter*nstep,:) = xsave(1:end-1,:); % end -1 取消每一步最后状态作为下一步初始的重复
    ttraj((iter-1)*nstep+1:iter*nstep) = tsave(1:end-1);

    % Save traj
    position_des_traj((iter-1)*nstep+1:iter*nstep, :) = position_des_save(1:end-1, :); % desired pos
    velocity_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_from_ctrl_save(1:end-1, 1:3); % desired vel
    attitude_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_from_ctrl_save(1:end-1, 4:6); % desired att
    omega_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_from_ctrl_save(1:end-1, 7:9); % desired att
    M_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_from_ctrl_save(1:end-1, 10:12); % desired att

    attitudetraj((iter-1)*nstep+1:iter*nstep, :) = att_current_save(1:end-1,:); % real state att
    omegatraj((iter-1)*nstep+1:iter*nstep, :) = omega_current_save(1:end-1,:); % real state att



    tilt_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_tilt4_save(1:end-1, :); % desired tilt save with iter
    throttle_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_throttle4_save(1:end-1, :); % desired tilt save with iter
    elevon_des_traj((iter-1)*nstep+1:iter*nstep, :) = des_elevon4_save(1:end-1, :); % desired tilt save with iter

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
% --- todo---plot 单独函数封装

% Truncate xtraj and ttraj  停留在当前步数状态数据，留存以防跳出流失数据
xtraj = xtraj(1:iter*nstep,:);  % Position and other state variables
ttraj = ttraj(1:iter*nstep);
attitudetraj = attitudetraj(1:iter*nstep, :); % Truncate attitude trajectory
omegatraj = omegatraj(1:iter*nstep, :); % Truncate omega trajectory


attitude_des_traj = attitude_des_traj(1:iter*nstep, :); % Truncate desired attitude trajectory
omega_des_traj = omega_des_traj(1:iter*nstep, :); % Truncate desired attitude trajectory
position_des_traj = position_des_traj(1:iter*nstep, :); 
velocity_des_traj = velocity_des_traj(1:iter*nstep, :);
M_des_traj = M_des_traj(1:iter*nstep, :);

% for actuator
tilt_des_traj = tilt_des_traj(1:iter*nstep, :);
throttle_des_traj = throttle_des_traj(1:iter*nstep, :);
elevon_des_traj = elevon_des_traj(1:iter*nstep, :);


% 数据封存
desired_struct.pos = position_des_traj;
desired_struct.vel = velocity_des_traj;
desired_struct.att = attitude_des_traj;
desired_struct.omega = omega_des_traj;
desired_struct.M = M_des_traj;

actual_struct.att = attitudetraj;
actual_struct.omega = omegatraj;

actuator_struct.tilt = tilt_des_traj;
actuator_struct.throttle = throttle_des_traj;
actuator_struct.elevon = elevon_des_traj;


% 数据显示
plot_results(ttraj, xtraj, desired_struct, actual_struct, actuator_struct);


if(~isempty(err))
    error(err);
end

disp('finished.')

t_out = ttraj;
s_out = xtraj;

end







