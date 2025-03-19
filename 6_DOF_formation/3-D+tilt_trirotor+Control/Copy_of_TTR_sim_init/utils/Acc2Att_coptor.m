% 旋翼模式加速度解算 - 对比测试
clc; clear; close all;

% 设置参数
psi_cmd = deg2rad(0);  % 偏航角（例：30°）
g = 9.81;              % 重力加速度(m/s^2)
m = 3;                 % 质心质量 (kg)

% 期望加速度（示例值）
acc_des = [1; 5; 0];  % [ax; ay; az]

%% ====================== 小角度近似方法 ======================
theta_cmd_approx = (acc_des(1)*sin(psi_cmd) - acc_des(2)*cos(psi_cmd)) / g;
phi_cmd_approx = (acc_des(1)*cos(psi_cmd) + acc_des(2)*sin(psi_cmd)) / g;

%% ====================== 完整非线性方法 ======================

% 期望总推力向量（ENU坐标）
F_des = m * [acc_des(1); acc_des(2); g + acc_des(3)];

% 期望机体Z轴方向 (Up)
zb_des = F_des / norm(F_des);

% 期望机体X轴水平投影方向
xc = [sin(psi_cmd); cos(psi_cmd); 0];

% 期望机体Y轴方向
yb_des = cross(zb_des, xc);
yb_des = yb_des / norm(yb_des);

% 重新计算期望机体X轴方向，确保正交
xb_des = cross(yb_des, zb_des);

% 构造期望旋转矩阵
R_des = [xb_des, yb_des, zb_des];

% 解算欧拉角（ZXY顺序）
phi_cmd_exact = atan2(R_des(3,2), R_des(3,3));
theta_cmd_exact = asin(R_des(3,1));
% psi_des 已知，不需重新计算（但可用作校验）
% psi_des_check = atan2(R_des(2,1), R_des(1,1));

%% ====================== 结果显示 ======================

% 绘制坐标轴和加速度矢量
figure;
quiver3(0,0,0,1,0,0,'r','LineWidth',2); hold on; % 东(X)
quiver3(0,0,0,0,1,0,'g','LineWidth',2);          % 北(Y)
quiver3(0,0,0,0,0,1,'b','LineWidth',2);          % 上(Z)

% 绘制期望加速度矢量
quiver3(0,0,0,acc_des(1),acc_des(2),acc_des(3),'m','LineWidth',3);

% 添加坐标轴标签
text(1.1,0,0,'East (X)','FontSize',12,'Color','r');
text(0,1.1,0,'North (Y)','FontSize',12,'Color','g');
text(0,0,1.1,'Up (Z)','FontSize',12,'Color','b');

% 显示加速度信息
acc_label = sprintf('a_{des} = [%.2f, %.2f, %.2f]', acc_des(1), acc_des(2), acc_des(3));
text(acc_des(1), acc_des(2), acc_des(3), acc_label,'FontSize',12,'Color','m');

% 小角度方法计算的姿态角
angle_label_approx = sprintf(['\\bf Small Angle Approximation\\rm\n'...
    '\\phi = %.2f^{\\circ}  \\theta = %.2f^{\\circ}  \\psi = %.2f^{\\circ}'],...
    rad2deg(phi_cmd_approx), rad2deg(theta_cmd_approx), rad2deg(psi_cmd));
text(-1, -1, 1, angle_label_approx, 'FontSize', 12, 'BackgroundColor', 'w', 'Color', 'blue');

% 完整非线性方法计算的姿态角
angle_label_exact = sprintf(['\\bf Nonlinear Solution\\rm\n'...
    '\\phi = %.2f^{\\circ}  \\theta = %.2f^{\\circ}  \\psi = %.2f^{\\circ}'],...
    rad2deg(phi_cmd_exact), rad2deg(theta_cmd_exact), rad2deg(psi_cmd));
text(-1, -1, 0.5, angle_label_exact, 'FontSize', 12, 'BackgroundColor', 'w', 'Color', 'red');

% 设置图像参数
grid on; axis equal;
xlabel('East (X)'); ylabel('North (Y)'); zlabel('Up (Z)');
title('Quadrotor Acceleration to Attitude Angle Comparison');
view(120,30);
