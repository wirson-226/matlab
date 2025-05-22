
clear
close all
clc

%% main process
tic
X0 = [1;0.5;1;-0.6; 2;1;-2;0.8; -2;0.6;-1;-1; -3;1.4;0;0.1];

% 精度更高
% tspan = [0, 50];
% 计算更快
tspan = linspace(0, 50, 5000); % 仅输出5000个点
options = odeset('RelTol', 1e-2, 'AbsTol', 1e-4); % 放宽容差

% [t,X] = ode45(@threeFollowers_formation_tracking_simulation, tspan, X0, options);
[t,X] = ode45(@threeFollowers_IAPF2022_simulation, tspan, X0, options);
toc

disp('Calculate End!!!')

Draw_threeFollowers_formation_tracking_simulation();

% %% 播放提示音
% load train
% sound(y, Fs)
