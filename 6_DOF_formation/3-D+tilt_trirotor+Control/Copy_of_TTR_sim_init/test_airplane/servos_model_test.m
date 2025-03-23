clc; clear; close all;

%% **参数定义**
omega_n = 20;     % 舵机自然频率 (rad/s)
zeta = 0.8;       % 阻尼比 (增加阻尼避免过冲)
Ku = 1.2;         % 输入增益 (PWM 到角度映射)
theta_max = pi/3; % 最大偏转角度 (60°)
theta_dot_max = pi/2; % 最大角速度 (90°/s)
u_dead = 0.05;    % 死区 (PWM 低于该值时无动作)

% 仿真时间
tspan = [0 2];  
t_sampled = linspace(0, 2, 1000); % 采样时间点

% **输入信号**
u_type = 'step';  % 'step' | 'sin' | 'noise'
% u_type = 'sin';  % 'step' | 'sin' | 'noise'

switch u_type
    case 'step'
        u_sampled = 1 * (t_sampled > 0.2);  % 阶跃输入
    case 'sin'
        u_sampled = 0.8 * sin(2 * pi * 1 * t_sampled);  % 正弦输入 (1Hz)
    case 'noise'
        u_sampled = 0.5 * randn(size(t_sampled));  % 噪声输入
end

% **仿真模型**
theta0 = [0; 0]; % 初始角度和角速度
[t_out, theta] = ode45(@(t,theta) servo_model(t, theta, t_sampled, u_sampled), tspan, theta0);

%% **可视化**
figure;
subplot(2,1,1);
plot(t_sampled, u_sampled, 'k--', 'LineWidth', 1.5); hold on;
plot(t_out, rad2deg(theta(:,1)), 'b', 'LineWidth', 2);
xlabel('时间 (s)'); ylabel('舵机角度 (°)');
legend('输入信号 (PWM)','舵机角度 (°)');
title('非线性舵机响应');
grid on;

subplot(2,1,2);
plot(t_out, rad2deg(theta(:,2)), 'r', 'LineWidth', 2);
xlabel('时间 (s)'); ylabel('角速度 (°/s)');
legend('舵机角速度');
title('舵机角速度变化');
grid on;

%% **舵机非线性模型**
function dtheta_dt = servo_model(t, theta, t_in, u_in)
    % 参数
    omega_n = 20;  % 舵机自然频率
    zeta = 0.8;    % 阻尼比
    Ku = 1.2;      % 增益
    u_dead = 0.05; % 死区
    theta_max = pi/3; % 最大角度 (60°)
    theta_dot_max = pi/2; % 最大角速度 (90°/s)

    % **插值获取当前时间的输入信号**
    u = interp1(t_in, u_in, t, 'linear', 'extrap');

    % **死区效应**
    if abs(u) < u_dead
        u = 0;
    end

    % **二阶系统建模**
    theta_dot = theta(2);
    theta_ddot = omega_n^2 * (Ku * u - theta(1)) - 2 * zeta * omega_n * theta_dot;

    % **角速度饱和**
    if abs(theta_dot) > theta_dot_max
        theta_dot = sign(theta_dot) * theta_dot_max;
    end

    % **角度饱和**
    if abs(theta(1)) > theta_max
        theta(1) = sign(theta(1)) * theta_max;
        theta_dot = 0; % 防止震荡
    end

    % 返回导数
    dtheta_dt = [theta_dot; theta_ddot];
end
