function world_3D = BodyToWorldAccel(body_3D, roll, pitch, yaw)
% BodyToWorldAccel 将机体坐标系加速度转换到世界坐标系
%
% 输入:
% body_3D - 机体坐标系加速度 [3x1 向量]
% roll - 横滚角 (弧度)
% pitch - 俯仰角 (弧度)
% yaw - 偏航角 (弧度)
%
% 输出:
% world_3D - 世界坐标系加速度 [3x1 向量]

% 构建旋转矩阵（ZXY顺序）
Rx = [1, 0, 0;
      0, cos(roll), -sin(roll);
      0, sin(roll), cos(roll)];

Ry = [cos(pitch), 0, sin(pitch);
      0, 1, 0;
     -sin(pitch), 0, cos(pitch)];

Rz = [cos(yaw), -sin(yaw), 0;
      sin(yaw), cos(yaw), 0;
      0, 0, 1];

% 组合旋转矩阵（注意顺序：Rz * Rx * Ry）
R_world_to_body = Rz * Rx * Ry;

% 坐标系映射（东北天 -> 右前上）
R_mapping = [0 1 0; % 世界x(East) -> 机体y(Front)
             1 0 0; % 世界y(North) -> 机体x(Right)
             0 0 1]; % 世界z(Up) -> 机体z(Up)

% 由于是反向变换，需要对旋转矩阵和映射矩阵取转置
% 转置可以得到逆变换
world_3D = (R_mapping' * R_world_to_body')' * body_3D;
end