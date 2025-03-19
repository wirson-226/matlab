% 车辆参数
% 轴距
egoWheelbase   = 2.8;
distToCenter   = 0.5*egoWheelbase;
% 初始位姿
egoInitialPose = [7,3.1,0];
% 目标位姿
egoTargetPose = [-distToCenter,0,0];
% 参考状态
ref = egoTargetPose;

% nmpc参数
% 采样时间
Ts = 0.1;
% 预测步长
p = 70;
% 控制步长
c = 70;
% 权重矩阵
Qp = diag([0.1 0.1 0]);
Rp = 0.01*eye(2);
Qt = diag([1 5 100]); 
Rt = 0.1*eye(2);
% 安全距离
safetyDistance = 0.1;

% NMPC信息
% 状态数目
nx = 3;
% 输出数目
ny = 3;
% 输入数目
nu = 2;
% 约束边界
MV1Min = -2;
MV1Max = 2;
MV2Min = -pi/4;
MV2Max = pi/4;

% 猜测初始解
X0 = [linspace(egoInitialPose(1),egoTargetPose(1),p)', ...
          linspace(egoInitialPose(2),egoInitialPose(2),p)'...
          zeros(p,1)];
MV0 = zeros(p,nu);
xz = reshape(X0',p*nx,1);
uz = reshape(MV0,p*nu,1);
z0 = [xz; uz; 0];

% 计算线性不等式约束(A,B)
% 这里是把输入变量（也叫操纵变量）的范围约束放在这里表示
% 在决策变量的上下限中就不需要再对输入变量进行约束了
A1  = [zeros(p*nu,p*nx), eye(p*nu), zeros(p*nu,1)];
B1  = zeros(p*nu,1);
for i = 1:p
    B1((i-1)*2+1) = MV1Max;
    B1((i-1)*2+2) = MV2Max;    
end
A2  = [zeros(p*nu,p*nx), -eye(p*nu), zeros(p*nu,1)];
B2  = zeros(p*nu,1);
for i = 1:p
    B2((i-1)*2+1) = -MV1Min;
    B2((i-1)*2+2) = -MV2Min;    
end
A = [A1;A2];
B = [B1;B2];

% 计算zLB, zUB
StateMin = -inf*ones(p,nx);
StateMax = inf*ones(p,nx);
xLB = reshape(StateMin', p*nx, 1);
xUB = reshape(StateMax', p*nx, 1);
nzu = 2*p;
uLB = -Inf(nzu,1);
uUB =  Inf(nzu,1);
zLB = [xLB; uLB; 0];
zUB = [xUB; uUB; Inf];
zUB(end) = 0;

% 设置data
data.PredictionHorizon = p;
data.state = egoInitialPose';
data.NumOfStates = 3;
data.MVIndex = [1,2];

% 设置CostFcn和ConFcn
CostFcn = @(z)CostJacFcnWrapper(z,data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);
ConFcn  = @(z)ConJacFcnWrapper(z,data,ref,Qp,Rp,Qt,Rt,distToCenter,safetyDistance);

% 求解NLP问题
options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',20,...
    'StepTolerance',0.01,...
    'ConstraintTolerance',0.01,...
    'OptimalityTolerance',0.01,...
    'FunctionTolerance',0.01, ...
    'Display', 'off',...
    'SpecifyConstraintGradient', true);
[cineq, ceq] = ConFcn(z0);
[z, cost, ExitFlag, Out] = fmincon(CostFcn, z0, A, B, [], [], zLB, zUB, ConFcn,options);

% 从求得的解中还原出状态和输入
Xopt = reshape(z(1:210),3,70)';
MVopt = reshape(z(211:350),2,70)';
% 可视化
timeLength = size(Xopt,1);
for ct = 1:timeLength
    helperSLVisualizeParking(Xopt(ct,:), MVopt(ct,2));
    pause(0.05);
end