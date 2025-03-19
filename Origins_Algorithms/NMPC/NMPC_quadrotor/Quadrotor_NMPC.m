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
% 定义mpc对象
nlmpcobj = nlmpc(nx, ny, nu);
nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";
nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;
nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = p;
nlmpcobj.ControlHorizon = m;
% nlmpcobj.MV = struct( ...
%     Min={0;0;0;0}, ...
%     Max={10;10;10;10}, ...
%     RateMin={-2;-2;-2;-2}, ...
%     RateMax={2;2;2;2} ...
%     );
nlmpcobj.MV = struct( ...
    Min={0;0;0;0}, ...
    Max={10;10;10;10}...
    );
nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];
% 初始状态
x = [7;-10;0;0;0;0;0;0;0;0;0;0];
% 目标状态
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [4.9 4.9 4.9 4.9]; 
mv = nloptions.MVTarget;
Duration = 20;
% 记录上一步长的控制量
lastMV = mv;
% 保存状态数据和输入数据
xHistory = x';
uHistory = lastMV;
% 主循环
for k = 1:(Duration/Ts)
    disp(['loop: ', num2str(k), '/',num2str((Duration/Ts))])
    % 设定预测时域的参考轨迹
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);
    % 获取实际状态
    xk = xHistory(k,:);
    % mpc计算输入量
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nloptions);
    lastMV = uk;
    % 仿真模型
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    % 保存输入量和状态量
    uHistory(k+1,:) = uk';
    xHistory(k+1,:) = XOUT(end,:);
end
%% 绘图
outmp4 = VideoWriter('drone.mp4','MPEG-4');
open(outmp4);
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
    frame = getframe(gcf);
    writeVideo(outmp4,frame);
    writeVideo(outmp4,frame);
    writeVideo(outmp4,frame);
    writeVideo(outmp4,frame);
    pause(0.1);
end
close(outmp4);

