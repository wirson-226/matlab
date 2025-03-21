% NLMPC参数
% 状态数目
nx = 4;
% 输出数目
ny = 2;
% 输入变量数目
nu = 1;
% 定义nlmpc对象
nlobj = nlmpc(nx, ny, nu);
% 采样时间
Ts = 0.1;
nlobj.Ts = Ts;
% 预测步长
nlobj.PredictionHorizon = 10;
% 控制步长
nlobj.ControlHorizon = 5;
% 状态函数
nlobj.Model.StateFcn = "pendulumDT0";
nlobj.Model.IsContinuousTime = false;
nlobj.Model.NumberOfParameters = 1;
nlobj.Model.OutputFcn = 'pendulumOutputFcn';
nlobj.Jacobian.OutputFcn = @(x,u,Ts) [1 0 0 0; 0 0 1 0];
nlobj.Weights.OutputVariables = [3 3];
nlobj.Weights.ECR = eps;
nlobj.OV(1).Min = -10;
nlobj.OV(1).Max = 10;
nlobj.MV.Min = -100;
nlobj.MV.Max = 100;
nlobj.Weights.ManipulatedVariablesRate = 0.1;
nlobj.Optimization.SolverOptions.Display = 'off';
nlobj.Optimization.SolverOptions.SpecifyObjectiveGradient = false;
nlobj.Optimization.SolverOptions.SpecifyConstraintGradient = false;

% 卡尔曼滤波
EKF = extendedKalmanFilter(@pendulumStateFcn, @pendulumMeasurementFcn);

%% 闭环仿真
x = [0;0;-pi;0];
y = [x(1);x(3)];
EKF.State = x;
mv = 0;
yref1 = [0 0];
yref2 = [5 0];

nloptions = nlmpcmoveopt;
nloptions.Parameters = {Ts};

% 循环运行20s
Duration = 20;
xHistory = x;
for ct = 1:(20/Ts)
    disp(ct)
    % 设置参考输出
    % 参考输出在10s左右的时候发生了变化，需要控制跟随
    if ct*Ts<10
        yref = yref1;
    else
        yref = yref2;
    end
    % 从输出值来得到状态值，卡尔滤波估计
    xk = correct(EKF, y);
    % 计算最优控制输入
    [mv,nloptions,info] = nlmpcmove(nlobj,xk,mv,yref,[],nloptions);   
    % 预测状态，卡尔曼滤波估计
    predict(EKF, [mv; Ts]);
    % 把输入施加到被控对象仿真模型
    x = pendulumDT0(x,mv,Ts);
    % 模拟传感器得到带噪声的观测数据
    y = x([1 3]) + randn(2,1)*0.01; 
    % 保存数据以便可视化
    xHistory = [xHistory x]; %#ok<*AGROW>
end

%%
dyn.l = 1;
for i = 1:size(xHistory,2)
    drawCartPole(i*Ts,xHistory(:,i),dyn);
    drawnow;
    pause(0.1);
end

% Plot the closed-loop response.
figure
subplot(2,2,1)
plot(0:Ts:Duration,xHistory(1,:))
xlabel('time')
ylabel('z')
title('cart position')
subplot(2,2,2)
plot(0:Ts:Duration,xHistory(2,:))
xlabel('time')
ylabel('zdot')
title('cart velocity')
subplot(2,2,3)
plot(0:Ts:Duration,xHistory(3,:))
xlabel('time')
ylabel('theta')
title('pendulum angle')
subplot(2,2,4)
plot(0:Ts:Duration,xHistory(4,:))
xlabel('time')
ylabel('thetadot')
title('pendulum velocity')
