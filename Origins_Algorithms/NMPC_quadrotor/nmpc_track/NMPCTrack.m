function steer = NMPCTrack(x, y, phi, refxs, refys, refths, ff, ds, p)
% 输入参数
% x       当前车辆的x坐标
% y       当前车辆的y坐标
% phi     当前车辆的航向角
% refxs   参考路径的x坐标
% refys   参考路径的y坐标
% refths  参考路径的航向角
% ff      前轮转角的前馈两，作为热启动参数
% ds      预测时域点集的距离间隔
% p       预测步长数

% 输出参数
% steer 前轮转角

% 目标函数
CostFcn = @(X)CostFcuntion(X, refxs, refys, refths, p);

% 约束函数
ConFcn = @(X)ConFunction(X, [x;y;phi], ds, p);

% 猜测初始决策变量,热启动
x0  = refxs;
y0  = refxs;
th0 = refths;
u0  = ones(p,1)*ff;
z0 = [x0; y0; th0; u0];

% 计算决策变量的上下限
StateMin = -inf*ones(p,3);
StateMax =  inf*ones(p,3);
xLB = reshape(StateMin', p*3, 1);
xUB = reshape(StateMax', p*3, 1);
uLB = -ones(p,1)*0.8;
uUB =  ones(p,1)*0.8;
zLB = [xLB; uLB];
zUB = [xUB; uUB];

% 求解最优化
options = optimoptions(@fmincon,'Algorithm','sqp','MaxIterations',40,...
    'StepTolerance',1e-6,...
    'ConstraintTolerance',1e-6,...
    'OptimalityTolerance',1e-6,...
    'FunctionTolerance',0.01, ...
    'Display', 'off');
[z, ~, ~, ~] = fmincon(CostFcn, z0, [], [], [], [], zLB, zUB, ConFcn,options);
steer = z(3*p+1);
end

% 约束函数
function [cineq, ceq] = ConFunction(X, X0, ds, p)
% X   决策变量实际状态
% X0  决策变量初始值
% ds  行驶距离间隔
% p   预测步长数

% 车辆轴距参数
L = 2.6;
% 不等式约束
cineq = [];
% 等式约束
ceq = zeros(3*p,1);

Ui  = X(3*p+1:4*p);
Xi  = zeros(p+1, 3);
Xi(1,:) = X0';
Xi(2:end,1) = X(1:p);
Xi(2:end,2) = X(p+1:2*p);
Xi(2:end,3) = X(2*p+1:3*p);
Xi = Xi';
ic = 1:3; 
for i = 1:p
    uk  = Ui(i);
    xk  = Xi(:,i);
    xk1 = xk;
    
    xk1(1) = xk(1) + ds*cos(xk(3));
    xk1(2) = xk(2) + ds*sin(xk(3));
    xk1(3) = xk(3) + (tan(uk)/L)*ds;
    ceq(ic) = Xi(:,i+1) - xk1(:);
    ic = ic + 3;
end
end

% 目标函数
function cost = CostFcuntion(X, refxs, refys, refths, p)
% X 决策变量实际状态
% refxs  预测时域内的参考点x坐标
% refys  预测时域内的参考点y坐标
% refths 预测时域内的参考点航向角
% p      预测步长数
    xs   = X(1:p);
    ys   = X(p+1:2*p);
    ths  = wrapTo2Pi(X(2*p+1:3*p));
    us   = X(3*p+1:4*p);
    Qx   = 1;
    Qy   = 1;
    Qth  = 0.5;
    Qu   = 0.01;
    cost = 0;
    for i = 1:p
        cost = cost + Qx*(xs(i) - refxs(i)).^2;
        cost = cost + Qy*(ys(i) - refys(i)).^2;
        dth =  angdiff(ths(i), refths(i));
        cost = cost + Qth*(dth).^2;
        cost = cost + Qu*us(i).^2;
    end
end