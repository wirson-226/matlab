% 预备舵机 二阶 模型 带有预测补偿

clc; clear; close all;

%% **参数定义**
omega_n = 15;     % 舵机自然频率 (rad/s)
zeta = 0.7;       % 阻尼比 (防止过冲)
Ku = 1.2;         % PWM 到角度增益
theta_max = pi/3; % 最大倾转角 (60°) -- 倾转 向下偏转为正 -30°--100° 舵面--向下为正 +-50
theta_dot_max = pi/2; % 最大角速度 (90°/s)
u_dead = 0.05;    % 死区 (PWM 低于该值时无动作)

% 期望角度 & 预测增益
alpha_cmd = deg2rad(20);  % 期望舵机角度（20°）
K_pre = 1.1;  % 预补偿增益

% 仿真时间
tspan = [0 2];  
t_sampled = linspace(0, 2, 1000);

% **舵机输入**
u_input = ones(size(t_sampled)) * alpha_cmd;

% **仿真模型**
theta0 = [0; 0]; % 初始角度和角速度
[t_out, theta] = ode45(@(t,theta) servo_model(t, theta, t_sampled, u_input, K_pre), tspan, theta0);

%% **可视化**
figure;
subplot(2,1,1);
plot(t_out, rad2deg(theta(:,1)), 'b', 'LineWidth', 2);
xlabel('时间 (s)'); ylabel('舵机角度 (°)');
legend('舵机角度');
title('舵机响应优化 (预测补偿)');
grid on;

subplot(2,1,2);
plot(t_out, rad2deg(theta(:,2)), 'r', 'LineWidth', 2);
xlabel('时间 (s)'); ylabel('角速度 (°/s)');
legend('舵机角速度');
grid on;

%% **舵机模型（带预测补偿）**
function dtheta_dt = servo_model(t, theta, t_in, u_in, K_pre)
    omega_n = 15;  % 舵机自然频率
    zeta = 0.7;    % 阻尼比
    Ku = 1.2;      % 增益
    u_dead = 0.05; % 死区
    theta_max = pi/3; % 最大角度 (60°)
    theta_dot_max = pi/2; % 最大角速度 (90°/s)

    % **插值获取当前时间的输入**
    u = interp1(t_in, u_in, t, 'linear', 'extrap');

    % **预测补偿**
    u = K_pre * u;  

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
        theta_dot = 0;
    end

    % **返回导数**
    dtheta_dt = [theta_dot; theta_ddot];
end
