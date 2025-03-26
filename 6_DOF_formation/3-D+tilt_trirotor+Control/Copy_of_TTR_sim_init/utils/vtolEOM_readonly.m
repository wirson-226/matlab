function sdot = vtolEOM_readonly(t, s, F, M, params)
%   todo --坐标系检查-- Done
%   空气动力相关部分，参考：《Aerodynamic modeling of a delta‑wing UAV for model‑based navigation》 采用model B 
%   
%   F M，由dynamics in controller 计算得出（controller 出command delta fo actutor, 然后解算出合力与力矩）机体下合力与合力矩  
%   在机身是 xyz右前上 在世界是 ned 北东地

% INPUTS:
% t      - 1 x 1, time
% s      - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p,
% q, r]  应该是1 * 13, 一行十三列; 如果是 13 * 1 , 用：'；'
% F      - 1 x 1, thrust output from controller (only used in simulation)
% F 修改为三维 F = [Fx, Fy, Fz];
% M      - 3 x 1, moments output from controller (only used in simulation)
% params - struct, output from nanoplus() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot   - 13 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, nanoplus


%************ EQUATIONS OF MOTION ************************
% Limit the force and moments due to actuator limits 添加动力特性限制
% A = [0.25,                      0, -0.5/params.arm_length;
%      0.25,  0.5/params.arm_length,                      0;
%      0.25,                      0,  0.5/params.arm_length;
%      0.25, -0.5/params.arm_length,                      0];
% 
% prop_thrusts = A*[F;M(1:2)]; % Not using moment about Z-axis for limits
% prop_thrusts_clamped = max(min(prop_thrusts, params.maxF/4), params.minF/4);
% 
% B = [                 1,                 1,                 1,                  1;
%                       0, params.arm_length,                 0, -params.arm_length;
%      -params.arm_length,                 0, params.arm_length,                 0];
% F = B(1,:)*prop_thrusts_clamped;
% M = [B(2:3,:)*prop_thrusts_clamped; M(3)];

% Assign states
x = s(1);
y = s(2);
z = s(3);
xdot = s(4);
ydot = s(5);
zdot = s(6);
qW = s(7);
qX = s(8);
qY = s(9);
qZ = s(10);
p = s(11);
q = s(12);
r = s(13);
quat = [qW; qX; qY; qZ];

% 转换逻辑：世界坐标系(ENU) -> 机体坐标系(RFS)
wRb = QuatToRot(quat);  % 世界到机体的旋转矩阵

% 坐标系映射变换矩阵
% 世界系(ENU): x-East, y-North, z-Up
% 机体系(RFS): x-Right, y-Front, z-Up
R_mapping = [0 1 0;   % 世界x(East) -> 机体y(Front)
             1 0 0;   % 世界y(North) -> 机体x(Right)
             0 0 1];  % 世界z(Up) -> 机体z(Up)

% 加速度世界坐标系解算
% 使用映射矩阵确保坐标系正确变换
% accel = 1 / params.mass * (R_mapping * wRb * [0; 0; F(3)] - [0; 0; params.mass * params.gravity]);
accel = 1 / params.mass * (R_mapping * wRb * F - [0; 0; params.mass * params.gravity]);
% Display the results
disp('Acc-inertialframe:');
disp(accel);

% Angular velocity body xyz
K_quat = 2; %this enforces the magnitude 1 constraint for the quaternion
quaterror = 1 - (qW^2 + qX^2 + qY^2 + qZ^2);
qdot = -1/2*[0, -p, -q, -r;...
             p,  0, -r,  q;...
             q,  r,  0, -p;...
             r, -q,  p,  0] * quat + K_quat*quaterror * quat;

% Angular acceleration
% 包含陀螺力矩
omega = [p;q;r];
pqrdot   = params.invI * (M - cross(omega, params.I*omega));

% Assemble sdot
sdot = zeros(13,1);
sdot(1)  = xdot;
sdot(2)  = ydot;
sdot(3)  = zdot;
sdot(4)  = accel(1);
sdot(5)  = accel(2);
sdot(6)  = accel(3);
sdot(7)  = qdot(1);
sdot(8)  = qdot(2);
sdot(9)  = qdot(3);
sdot(10) = qdot(4);
sdot(11) = pqrdot(1);
sdot(12) = pqrdot(2);
sdot(13) = pqrdot(3);

end

