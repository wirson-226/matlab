% 车辆参数
% 轴距
egoWheelbase   = 2.8;
distToCenter   = 0.5*egoWheelbase;
% 初始位姿
egoInitialPose = [7,3.1,0];
% 目标位姿
egoTargetPose = [-distToCenter,0,0];

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
% 最大迭代次数
maxIter = 20;

% 定义nmpc对象
% 状态数目
nx = 3;
% 输出数目
ny = 3;
% 输入数目
nu = 2;
% 调用nlmpc库函数
nlobj = nlmpc(nx,ny,nu);
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = c;
nlobj.MV(1).Min = -2;
nlobj.MV(1).Max = 2;
nlobj.MV(2).Min = -pi/4;
nlobj.MV(2).Max = pi/4;
% 状态转换函数
nlobj.Model.StateFcn = "parkingVehicleStateFcn";
% 状态转换函数雅可比
nlobj.Jacobian.StateFcn = "parkingVehicleStateJacobianFcn";
% 目标函数
nlobj.Optimization.CustomCostFcn = "parkingCostFcn";
nlobj.Optimization.ReplaceStandardCost = true;
% 目标函数雅可比
nlobj.Jacobian.CustomCostFcn = "parkingCostJacobian";
% 不等式约束
nlobj.Optimization.CustomIneqConFcn = "parkingIneqConFcn";
% 不等式约束雅可比
nlobj.Jacobian.CustomIneqConFcn = "parkingIneqConFcnJacobian";
% 求解器选项
nlobj.Optimization.SolverOptions.FunctionTolerance = 0.01;
nlobj.Optimization.SolverOptions.StepTolerance = 0.01;
nlobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;
nlobj.Optimization.SolverOptions.OptimalityTolerance = 0.01;
nlobj.Optimization.SolverOptions.MaxIter = maxIter;
nlobj.Optimization.SolverOptions.Display = 'iter';
opt = nlmpcmoveopt;
% 猜测初始解
opt.X0 = [linspace(egoInitialPose(1),egoTargetPose(1),p)', ...
          linspace(egoInitialPose(2),egoInitialPose(2),p)'...
          zeros(p,1)];
opt.MV0 = zeros(p,nu);
paras = {egoTargetPose,Qp,Rp,Qt,Rt,distToCenter,safetyDistance}';
nlobj.Model.NumberOfParameters = numel(paras);
opt.Parameters = paras;

% 仿真过程
x0 = egoInitialPose';
u0 = [0;0];
[mv,nloptions,info] = nlmpcmove(nlobj,x0,u0,[],[],opt);
% 可视化
timeLength = size(info.Xopt,1);
for ct = 1:timeLength
    % 可视化
    helperSLVisualizeParking(info.Xopt(ct,:), info.MVopt(ct,2));
    pause(0.05);
end