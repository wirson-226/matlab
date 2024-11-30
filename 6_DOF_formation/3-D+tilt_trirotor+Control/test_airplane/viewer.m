%% 设置飞行器的位置和姿态
% position = [10, 10, 10];    % 飞机位置
% attitude = deg2rad([45, 45, 45]);    % 飞机姿态：roll pitch yaw, 弧度rad;rad = deg * pi / 180，正方向跟随坐标轴
% tilt_angle = [90, 45];     % 电机倾转角 顺序为右，左，向下倾转为正 类pitch y轴正向；尾部固定。为c；

% position = [0, 0, 0];    % 飞机位置
% attitude = deg2rad([0, 0, 0]);    % 飞机姿态：roll pitch yaw, 弧度rad;rad = deg * pi / 180，正方向跟随坐标轴
% tilt_angle = [0, 0];     % 电机倾转角 顺序为右，左，向下倾转为正 类pitch y轴正向；尾部固定。为c；
% omega = [0, 0]; 


%% 调用函数进行可视化
% WingPlot(position, attitude, wingspan, length, tail_span, engine_pos);
% planeplot_ttr(position, attitude,tilt_angle); % 完成倾转，添加原点---
% planeplot_ttr_test(position, attitude,tilt_angle,omega);
planeplot_ttr_animation;
