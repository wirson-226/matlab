% NLMPC参数
% 状态数目
nx = 4;
% 输出数目
ny = 2;
% 输入变量数目
nu = 1;
% 采样时间
Ts = 0.1;
% 预测步长
p = 10;
% 控制步长
c = 5;
% 输入变量的范围
MVMin = -100;
MVMax = 100;

%% 闭环仿真
x = [0;0;-pi;0];
y = [x(1);x(3)];
mv = 0;
yref1 = [0 0];
yref2 = [5 0];

% 循环运行20s
Duration = 20;
xHistory = x;
mvs = [];
z = [];
data.lastMV = 0;
for ct = 1:(20/Ts)
    disp(ct)
    % 设置参考输出
    % 参考输出在10s左右的时候发生了变化，需要控制跟随
    if ct*Ts<10
        yref = yref1;
    else
        yref = yref2;
    end
    xk = x;

    % 猜测初始值
    if isempty(z)
        X0 = repmat(xk,1,p);
    else
        X0 = z(nx+1:p*nx);
        X0 = [X0;X0(end-3:end)];
    end
    MV0 = zeros(c,nu);
    xz = reshape(X0,p*nx,1);
    uz = reshape(MV0,c*nu,1);
    z0 = [xz; uz; 0];

    %计算线性不等式约束(A,B)
    A1  = [zeros(p*nu,p*nx), [eye(c*nu);[zeros(5,4),ones(5,1)]], zeros(p*nu,1)];
    B1  = zeros(p*nu,1);
    for i = 1:p
        B1(i) = MVMax;
    end
    A2  = [zeros(p*nu,p*nx), -[eye(c*nu);[zeros(5,4),ones(5,1)]], zeros(p*nu,1)];
    B2  = zeros(p*nu,1);
    for i = 1:p
        B2(i) = -MVMin;
    end
    A = [A1;A2];
    B = [B1;B2];

    % 计算zLB, zUB
    StateMin = -inf*ones(p,nx);
    StateMax = inf*ones(p,nx);
    xLB = reshape(StateMin', p*nx, 1);
    xUB = reshape(StateMax', p*nx, 1);
    nzu = c;
    uLB = -Inf(nzu,1);
    uUB =  Inf(nzu,1);
    zLB = [xLB; uLB; 0];
    zUB = [xUB; uUB; Inf];

    % 设置CostFcn和ConFcn,
    data.state = x;
    CostFcn = @(z)CostFcnWrapper(z,data,yref);
    ConFcn  = @(z)ConFcnWrapper(z,data,yref);

   
    % 求解NLP问题
    options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',400,...
        'StepTolerance',1e-6,...
        'ConstraintTolerance',1e-6,...
        'OptimalityTolerance',1e-6,...
        'FunctionTolerance',0.01, ...
        'Display', 'off');
    [z, cost, ExitFlag, Out] = fmincon(CostFcn, z0, A, B, [], [], zLB, zUB, ConFcn,options);
    % 从求得的解中获取当前循环的输入
    mv = z(41);
    data.lastMV = mv;
    mvs = [mvs,mv];
    % 施加到被控对象仿真模型中
    x = pendulumDT0(x,mv,Ts);
    % 保存数据以便可视化
    xHistory = [xHistory x]; %#ok<*AGROW>
end

dyn.l = 1;
for i = 1:size(xHistory,2)
    drawCartPole(i*Ts,xHistory(:,i),dyn);
    drawnow;
    pause(0.1);
end

%%
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
