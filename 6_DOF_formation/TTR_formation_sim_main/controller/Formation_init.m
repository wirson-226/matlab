%% main.m
clear; clc; close all;

% ==== 参数定义 ====
num_agents = 6;
n_dim = 2;  % 2D 平面
dt = 0.05;  % 仿真步长
T_total = 10;  % 仿真总时间
steps = T_total/dt;
u_max = 2.0;
r_safe = 1.0;
formation_mode = 'rotating'; % 可选 'fixed', 'moving', 'rotating'

% 控制器参数
ctrl.kp_formation = 15;
ctrl.kv_formation = 8;
ctrl.beta_consensus = 0.5;
ctrl.gamma_observer = 0.1;

% 状态初始化
x = 10 * rand(num_agents, n_dim); % 随机初始位置
v = zeros(num_agents, n_dim);      % 初始速度
d_hat = zeros(num_agents, n_dim);  % 扰动估计

% ==== 复杂障碍物设置 ====
% 静态障碍物（包括密集圆形障碍）
static_obstacles = [
    5, 5;
    -5, 5;
    0, 8;
    3, -3;
    -4, -6
];

% 动态障碍物初始位置与速度
moving_obstacles.pos = [0, 0; 8, -4];
moving_obstacles.vel = [0.1, 0.15; -0.08, 0.12];
obstacle_radius = 1.5;

% 存储轨迹用于绘图
state_hist.pos = zeros(num_agents, n_dim, steps);
state_hist.vel = zeros(num_agents, n_dim, steps);

% ==== 主循环 ====
for step = 1:steps
    t = (step-1)*dt;

    % 动态障碍物更新
    moving_obstacles.pos = moving_obstacles.pos + moving_obstacles.vel * dt;
    all_obstacles = [static_obstacles; moving_obstacles.pos];

    % 层一：一致性控制
    adjacency = generate_dynamic_graph(t, num_agents);
    u_consensus = consensus_control(x, v, d_hat, adjacency, u_max, ctrl);

    % 层二：可变形编队控制
    [u_formation, formation_targets] = formation_control(t, num_agents, x, v, formation_mode, ctrl);

    % 层三：CBF避障
    u_total = zeros(num_agents, n_dim);
    for i = 1:num_agents
        u_nominal = u_consensus(i,:) + u_formation(i,:);
        u_cbf = cbf_avoidance(x(i,:), v(i,:), u_nominal, all_obstacles, obstacle_radius, r_safe, u_max);
        u_total(i,:) = u_cbf;
    end

    % 更新状态
    [x, v, d_hat] = update_state(x, v, d_hat, u_total, dt);

    % 存储轨迹
    state_hist.pos(:,:,step) = x;
    state_hist.vel(:,:,step) = v;

    % 每隔若干步绘图
    if mod(step,10)==0
        plot_agents(x, all_obstacles, obstacle_radius, formation_targets);
        drawnow;
    end
end

%% 显示并保存最终一帧状态
figure('Name','Final Frame','Units','centimeters','Position',[5 5 18 12]);
plot_agents(x, all_obstacles, obstacle_radius, formation_targets);
saveas(gcf, 'final_frame.eps', 'epsc');
print(gcf, 'final_frame', '-dpdf');
print(gcf, 'final_frame', '-dpng', '-r300');

%% 绘制完整轨迹并保存
figure('Units','centimeters','Position',[5 5 18 12]);
hold on; grid on; axis equal;
for i = 1:num_agents
    plot(squeeze(state_hist.pos(i,1,:)), squeeze(state_hist.pos(i,2,:)),'LineWidth',1.5);
end
scatter(all_obstacles(:,1), all_obstacles(:,2), 100, 'r', 'filled');
title('Agent Trajectories'); xlabel('X (m)'); ylabel('Y (m)');
legend(arrayfun(@(i) sprintf('Agent %d', i), 1:num_agents, 'UniformOutput', false));

% 保存为矢量图
saveas(gcf, 'trajectory_result.eps', 'epsc');
print(gcf, 'trajectory_result', '-dpdf');
print(gcf, 'trajectory_result', '-dpng', '-r300');

% 另起一图绘制状态变化并保存
plot_state_history(state_hist, dt);
print(gcf, 'state_history_result', '-dpdf');
print(gcf, 'state_history_result', '-dpng', '-r300');


%% --- Functions ---
function [u_formation, formation_targets] = formation_control(t, num_agents, x, v, mode, ctrl)
    n_dim = 2;
    u_formation = zeros(num_agents, n_dim);
    formation_targets = zeros(num_agents, n_dim);
    alpha = 1 + 0.5*sin(0.1*t); % 可选动态缩放因子
    center = [0, 0];
    omega = 0.1; % 旋转角速度
    v_center = [0.2, 0.2]; % 平移速度

    if strcmp(mode, 'moving')
        center = v_center * t;
    elseif strcmp(mode, 'rotating')
        rotation_angle = omega * t;
        R = [cos(rotation_angle), -sin(rotation_angle); sin(rotation_angle), cos(rotation_angle)];
    else
        R = eye(2);
    end

    for i = 1:num_agents
        angle = 2*pi*(i-1)/num_agents;
        p0 = alpha * 5 * [cos(angle), sin(angle)];
        if strcmp(mode, 'rotating')
            p_rotated = R * p0';
            target = center + p_rotated';
        else
            target = center + p0;
        end
        formation_targets(i,:) = target;
        u_formation(i,:) = -ctrl.kp_formation * (x(i,:) - target) - ctrl.kv_formation * v(i,:);
    end
end

function adjacency = generate_dynamic_graph(t, num_agents)
    if mod(floor(t), 10) < 5
        adjacency = ones(num_agents) - eye(num_agents);
    else
        adjacency = diag(ones(1,num_agents-1),1) + diag(ones(1,num_agents-1),-1);
        adjacency(1,end) = 1; adjacency(end,1) = 1; % 环形连接
    end
end

function u_consensus = consensus_control(x, v, d_hat, adjacency, u_max, ctrl)
    num_agents = size(x,1);
    n_dim = size(x,2);
    u_consensus = zeros(num_agents, n_dim);

    for i = 1:num_agents
        for j = 1:num_agents
            if adjacency(i,j) > 0
                u_consensus(i,:) = u_consensus(i,:) - (x(i,:) - x(j,:)) - ctrl.beta_consensus*(v(i,:) - v(j,:));
            end
        end
        d_hat(i,:) = d_hat(i,:) - ctrl.gamma_observer * u_consensus(i,:);
        u_consensus(i,:) = u_consensus(i,:) - d_hat(i,:);
        u_consensus(i,:) = max(min(u_consensus(i,:), u_max), -u_max);
    end
end

function u_cbf = cbf_avoidance(x_i, v_i, u_nominal, obstacles, obstacle_radius, r_safe, u_max)
    n_dim = length(x_i);
    options = optimoptions('fmincon','Display','off');
    obj = @(u) norm(u - u_nominal)^2;
    A = []; b = []; Aeq = []; beq = [];
    lb = -u_max*ones(n_dim,1);
    ub = u_max*ones(n_dim,1);

    cons = {};
    for k = 1:size(obstacles,1)
        diff = x_i' - obstacles(k,:)';
        h = norm(diff)^2 - (r_safe + obstacle_radius)^2;
        dh = 2 * diff;
        cons{end+1} = @(u) dh'*(v_i'+u) + 1.0*h;    
    end

    nonlcon = @(u) deal([], cellfun(@(c) c(u), cons));
    [u_opt,~,exitflag] = fmincon(obj, u_nominal', A, b, Aeq, beq, lb, ub, nonlcon, options);

    if exitflag > 0
        u_cbf = u_opt';
    else
        u_cbf = u_nominal;
    end
end

function [x, v, d_hat] = update_state(x, v, d_hat, u_total, dt)
    v = v + u_total * dt;
    x = x + v * dt;
end

function plot_agents(x, obstacles, obstacle_radius, formation_targets)
    clf;
    hold on; grid on; axis equal;
    scatter(x(:,1), x(:,2), 50, 'b', 'filled');
    scatter(obstacles(:,1), obstacles(:,2), 100, 'r', 'filled');
    viscircles(obstacles, obstacle_radius*ones(size(obstacles,1),1),'LineStyle','--');
    plot(formation_targets(:,1), formation_targets(:,2), 'k*');

    % 添加连线显示形成队形
    num_agents = size(x,1);
    for i = 1:num_agents
        j = mod(i, num_agents) + 1; % 与下一个agent连接，形成闭环
        plot([x(i,1), x(j,1)], [x(i,2), x(j,2)], 'b--'); % 虚线连接实际位置
        plot([formation_targets(i,1), formation_targets(j,1)], [formation_targets(i,2), formation_targets(j,2)], 'k:'); % 虚线连接目标位置
    end

    xlim([-15, 15]); ylim([-15, 15]);
    title('Agent Positions and Formation Links');
end

% 状态量随时间变化图
function plot_state_history(state_hist, dt)
    [num_agents, n_dim, steps] = size(state_hist.pos);
    t = (0:steps-1) * dt;

    figure('Name','Position and Velocity vs Time','Units','centimeters','Position',[5,5,18,12]);
    for i = 1:num_agents
        subplot(2,1,1); hold on;
        plot(t, squeeze(state_hist.pos(i,1,:)), 'DisplayName', sprintf('Agent %d x', i));
        plot(t, squeeze(state_hist.pos(i,2,:)), '--', 'DisplayName', sprintf('Agent %d y', i));
        ylabel('Position [m]'); title('Position vs Time'); grid on;

        subplot(2,1,2); hold on;
        vel_norm = sqrt(sum(squeeze(state_hist.vel(i,:,:)).^2,1));
        plot(t, vel_norm, 'DisplayName', sprintf('Agent %d', i));
        ylabel('Speed [m/s]'); xlabel('Time [s]'); title('Velocity Magnitude vs Time'); grid on;
    end
    subplot(2,1,1); legend;
    subplot(2,1,2); legend;

    % 导出矢量图
    % saveas(gcf, 'state_history_result.eps', 'epsc');
    
end