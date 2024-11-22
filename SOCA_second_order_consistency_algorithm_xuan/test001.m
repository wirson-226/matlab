% 四旋翼无人机的MPC控制器设计

% 定义采样时间和预测时域
Ts = 0.1;  % 采样时间
prediction_horizon = 20;  % 预测时域
control_horizon = 5;  % 控制时域

% 四旋翼无人机的线性化状态空间模型
A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];
B = [0 0 0 0;
     1 0 0 0;
     0 0 0 0;
     0 1 0 0;
     0 0 0 0;
     0 0 1 0];

C = eye(6);
D = zeros(6, 4);

% 创建状态空间模型
plant = ss(A, B, C, D, Ts);

% 创建 MPC 控制器对象
mpcobj = mpc(plant, Ts, prediction_horizon, control_horizon);

% 设置权重
mpcobj.Weights.ManipulatedVariables = 0.1 * ones(1, 4);
mpcobj.Weights.ManipulatedVariablesRate = 0.1 * ones(1, 4);
mpcobj.Weights.OutputVariables = ones(1, 6);

% 设置输入约束（假设推力在0到1之间）
for i = 1:4
    mpcobj.MV(i).Min = 0;
    mpcobj.MV(i).Max = 1;
end

% 设置初始条件
x0 = zeros(6, 1);  % 初始状态
u0 = zeros(4, 1);  % 初始输入
mpcstate_01 = mpcstate(mpcobj);  % 创建 MPC 状态对象

% 仿真参数
simulation_time = 20;  % 仿真总时间
num_steps = simulation_time / Ts;

% 定义期望轨迹（例如，在x-y平面上画一个圆）
t = (0:num_steps-1)' * Ts;
ref = [cos(t) sin(t) zeros(num_steps, 4)];

% 记录数据
X = zeros(num_steps, 6);
U = zeros(num_steps, 4);

% 主仿真循环
for k = 1:num_steps
    % 获取当前参考轨迹点
    r = ref(k, :)';
    
    % 计算MPC控制输入
    [u, mpcstate_01] = mpcmove(mpcobj, mpcstate_01, x0, r);
    
    % 更新状态
    x0 = A * x0 + B * u;
    
    % 记录数据
    X(k, :) = x0';
    U(k, :) = u';
end

% 绘制结果
figure;
subplot(2, 1, 1);
plot(t, X(:, 1:3));
title('Position');
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z');

subplot(2, 1, 2);
plot(t, U);
title('Control Inputs');
xlabel('Time (s)');
ylabel('Thrust');
legend('u1', 'u2', 'u3', 'u4');
