%% 推导J和C矩阵
% 定义角度
% phi: roll angle
% theta: pitch angle
% psi: yaw angle
syms phi(t) theta(t) psi(t)

% 惯性坐标系到机体坐标系的转换矩阵
W = [ 1,  0,        -sin(theta);
      0,  cos(phi),  cos(theta)*sin(phi);
      0, -sin(phi),  cos(theta)*cos(phi) ];
% R_ZYX机体坐标系到惯性坐标系的转换矩阵
R = rotationMatrixEulerZYX(phi,theta,psi);
% 定义惯量符号
syms Ixx Iyy Izz
% Jacobian that relates body frame to inertial frame velocities
I = [Ixx, 0, 0; 0, Iyy, 0; 0, 0, Izz];
J = W.'*I*W;
% Coriolis matrix
dJ_dt = diff(J);
h_dot_J = [diff(phi,t), diff(theta,t), diff(psi,t)]*J;
grad_temp_h = transpose(jacobian(h_dot_J,[phi theta psi]));
C = dJ_dt - 1/2*grad_temp_h;
C = subsStateVars(C,t);

%% 求状态方程表达式并生成代码
% 定义固定参数和输入变量
% k: 升力系数
% l: 转轴距离
% m: 质量
% b: 阻力常数
% g: 重力常数
% ui: 4个输入量
syms k l m b g u1 u2 u3 u4
% phi, theta, psi3个方向的姿态扭矩
tau_beta = [l*k*(-u2+u4); l*k*(-u1+u3); b*(-u1+u2-u3+u4)];
% 总推力
T = k*(u1+u2+u3+u4);
syms x(t) y(t) z(t)
% 状态变量
state = [x; y; z; phi; theta; psi; diff(x,t); diff(y,t); ...
    diff(z,t); diff(phi,t); diff(theta,t); diff(psi,t)];
state = subsStateVars(state,t);
f = [ % Set time-derivative of the positions and angles
      state(7:12);
      % Equations for linear accelerations of the center of mass
      -g*[0;0;1] + R*[0;0;T]/m;
      % Euler–Lagrange equations for angular dynamics
      inv(J)*(tau_beta - C*state(10:12))
];
f = subsStateVars(f,t);
% Replace fixed parameters with given values here
IxxVal = 1.2;
IyyVal = 1.2;
IzzVal = 2.3;
kVal = 1;
lVal = 0.25;
mVal = 2;
bVal = 0.2;
gVal = 9.81;
f = subs(f, [Ixx Iyy Izz k l m b g], ...
    [IxxVal IyyVal IzzVal kVal lVal mVal bVal gVal]);
f = simplify(f);
% 生成代码
matlabFunction(f,"File","QuadrotorStateFcn", ...
    "Vars",{state,control});

%% 计算状态转换函数的雅克比矩阵函数，并生成代码
% Calculate Jacobians for nonlinear prediction model
A = jacobian(f,state);
control = [u1; u2; u3; u4];
B = jacobian(f,control);
matlabFunction(A,B,"File","QuadrotorStateJacobianFcn", ...
    "Vars",{state,control});



function [Rz,Ry,Rx] = rotationMatrixEulerZYX(phi,theta,psi)
% Euler ZYX angles convention
    Rx = [ 1,           0,          0;
           0,           cos(phi),  -sin(phi);
           0,           sin(phi),   cos(phi) ];
    Ry = [ cos(theta),  0,          sin(theta);
           0,           1,          0;
          -sin(theta),  0,          cos(theta) ];
    Rz = [cos(psi),    -sin(psi),   0;
          sin(psi),     cos(psi),   0;
          0,            0,          1 ];
    if nargout == 3
        % Return rotation matrix per axes
        return;
    end
    % Return rotation matrix from body frame to inertial frame
    Rz = Rz*Ry*Rx;
end

function stateExpr = subsStateVars(timeExpr,var)
    if nargin == 1 
        var = sym("t");
    end
    repDiff = @(ex) subsStateVarsDiff(ex,var);
    stateExpr = mapSymType(timeExpr,"diff",repDiff);
    repFun = @(ex) subsStateVarsFun(ex,var);
    stateExpr = mapSymType(stateExpr,"symfunOf",var,repFun);
    stateExpr = formula(stateExpr);
end

function newVar = subsStateVarsFun(funExpr,var)
    name = symFunType(funExpr);
    name = replace(name,"_Var","");
    stateVar = "_" + char(var);
    newVar = sym(name + stateVar);
end

function newVar = subsStateVarsDiff(diffExpr,var)
    if nargin == 1 
      var = sym("t");
    end
    c = children(diffExpr);
    if ~isSymType(c{1},"symfunOf",var)
      % not f(t)
      newVar = diffExpr;
      return;
    end
    if ~any([c{2:end}] == var)
      % not derivative wrt t only
      newVar = diffExpr;
      return;
    end
    name = symFunType(c{1});
    name = replace(name,"_Var","");
    extension = "_" + join(repelem("d",numel(c)-1),"") + "ot";
    stateVar = "_" + char(var);
    newVar = sym(name + extension + stateVar);
end