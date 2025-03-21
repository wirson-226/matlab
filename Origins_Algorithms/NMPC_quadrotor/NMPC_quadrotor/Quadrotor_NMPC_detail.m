% 初始化
clear;clc;
% 状态变量个数
nx = 12;
% 输出变量个数
ny = 12;
% 输入变量个数
nu = 4;
% 时间不长
Ts = 0.1;
% 预测时域和控制时域
p = 18;
m = 2;
% MPC参数
% 约束
MVMin=[0;0;0;0];
MVMax=[10;10;10;10];
RateMin={-2;-2;-2;-2};
RateMax={2;2;2;2};
% 权重
OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
ManipulatedVariables = [0.1 0.1 0.1 0.1];
ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
% 初始状态
x = [7;-10;0;0;0;0;0;0;0;0;0;0];
% 目标状态
MVTarget = [4.9 4.9 4.9 4.9]; 
nloptions.MVTarget = MVTarget;
Duration = 20;
% 记录上一步长的控制量
lastMV = MVTarget;
% 保存状态数据和输入数据
xHistory = x';
uHistory = lastMV;
mvs = [];
z = [];
data.lastMV = MVTarget';
% 主循环
for k = 1:(Duration/Ts)
    disp(['loop: ', num2str(k), '/',num2str((Duration/Ts))])
    % 设定预测时域的参考轨迹
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);
    % 获取实际状态
    xk = xHistory(k,:);
    % mpc计算输入量
    
    % 猜测初始值
    if isempty(z)
        X0 = repmat(xk,1,p);
    else
        X0 = z(nx+1:p*nx);
        X0 = [X0;X0(end-11:end)];
    end
    MV0 = repmat(MVTarget,m,1);
    xz = reshape(X0,p*nx,1);
    uz = reshape(MV0,m*nu,1);
    z0 = [xz; uz; 0];

    %计算线性不等式约束(A,B)
    A1  = [zeros(p*nu,p*nx), [eye(m*nu);[zeros(64,4),repmat(eye(nu),16,1)]], zeros(p*nu,1)];
    B1  = zeros(p*nu,1);
    B1(1:p*nu) = repmat(MVMax,p,1);
    A2  = [zeros(p*nu,p*nx), -[eye(m*nu);[zeros(64,4),repmat(eye(nu),16,1)]], zeros(p*nu,1)];
    B2  = zeros(p*nu,1);
    B2(1:p*nu) = repmat(-MVMin,p,1);
    A = [A1;A2];
    B = [B1;B2];

    % 计算zLB, zUB
    StateMin = -inf*ones(p,nx);
    StateMax = inf*ones(p,nx);
    xLB = reshape(StateMin', p*nx, 1);
    xUB = reshape(StateMax', p*nx, 1);
    nzu = m*nu;
    uLB = -Inf(nzu,1);
    uUB =  Inf(nzu,1);
    zLB = [xLB; uLB; 0];
    zUB = [xUB; uUB; Inf];

    % 设置CostFcn和ConFcn,
    data.state = xk;
    CostFcn = @(z)CostFcnWrapper(z,data,yref);
    ConFcn  = @(z)ConFcnWrapper(z,data,yref);
    % 求解NLP问题
    options = optimoptions(@fmincon,'MaxIterations',400,...
        'StepTolerance',1e-6,...
        'ConstraintTolerance',1e-6,...
        'OptimalityTolerance',1e-6,...
        'FunctionTolerance',0.01, ...
        'Display', 'off');
    [z, cost, ExitFlag, Out] = fmincon(CostFcn, z0, A, B, [], [], zLB, zUB, ConFcn,options);
    uk = z(217:220);
    lastMV = uk;
    data.lastMV = lastMV;
    % 仿真模型
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    % 保存输入量和状态量
    uHistory(k+1,:) = uk';
    xHistory(k+1,:) = XOUT(end,:);
end
%% 绘图
plotQuadrotorTrajectory;
figure;
hold on;
for i = 1:size(xHistory, 1)
    cla;
    hold on;
    plot3(xHistory(1:i,1),xHistory(1:i,2),xHistory(1:i,3),'r.');
    plot3(yreftot(:,1),yreftot(:,2),yreftot(:,3),'g.');
    drone_Animation(xHistory(i, 1:6));
    drawnow;
end

