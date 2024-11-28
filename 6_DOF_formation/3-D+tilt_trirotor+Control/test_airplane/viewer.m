% Simple Test for WingPlot (Fixed-Wing Aircraft Visualization)

% 设置飞机的初始状态
% 初始位置 [x, y, z], 初始速度 [xdot, ydot, zdot], 初始姿态 [pitch, roll, yaw]
state = [0, 0, 100, 0, 10, -5, 0, 0, 0];  % Aircraft starts at [0, 0, 100] with a velocity [0, 10, -5]
wingspan = 10;   % Wingspan of the aircraft (meters)
length = 20;     % Length of the aircraft (meters)
color = 'b';     % Aircraft color
max_iter = 500;  % Maximum number of iterations for history tracking

% 创建一个图形窗口
figure;

% 创建 WingPlot 对象
W = WingPlot(1, state, wingspan, length, color, max_iter);

% 设置模拟时间
dt = 0.05;  % Time step
T = 20;     % Total simulation time
time = 0:dt:T;  % Time vector

% 设定飞机的期望飞行状态（简单的模拟为期望飞行沿着某个方向）
des_state = [0, 0, 100, 0, 10, -5, 0, 0, 0];  % Desired state stays constant (hovering)

% 模拟飞行过程
for t = time
    % 更新飞机的状态（简单模拟：飞机的x和y轴速度保持不变）
    state(1) = state(1) + state(4) * dt;  % Update x position based on velocity
    state(2) = state(2) + state(5) * dt;  % Update y position based on velocity
    state(3) = state(3) + state(6) * dt;  % Update z position based on velocity

    % 更新期望的状态（假设期望状态也沿着相同轨迹飞行）
    des_state(1) = des_state(1) + des_state(4) * dt;
    des_state(2) = des_state(2) + des_state(5) * dt;
    des_state(3) = des_state(3) + des_state(6) * dt;

    % 更新 WingPlot 可视化
    W.UpdateWingPlot(state, des_state, t);

    % 增加一些暂停以便观察
    pause(dt);
end

% 显示结束时的状态
disp('Simulation complete.');
