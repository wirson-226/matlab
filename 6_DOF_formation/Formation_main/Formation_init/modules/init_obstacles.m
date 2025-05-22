%% modules/init_obstacles.m (修改版)
function [static_obs, moving_obs, obstacle_radius] = init_obstacles()
    % 静态障碍物 - 调整位置以适应编队任务
    % 原来的障碍物距离太远，调整到编队活动区域内
    static_obs = [1.5, 1.5;     % 右上区域
                  -1.5, 1.5;    % 左上区域  
                  0, -2.0;      % 下方区域
                  2.0, -1.0;    % 右下区域
                  -2.5, -0.5];  % 左侧区域
    
    % 动态障碍物 - 初始位置和速度
    moving_obs.pos = [0.5, -0.5;   % 第一个动态障碍物
                      -1.0, 1.0];   % 第二个动态障碍物
    moving_obs.vel = [0.1, 0.1;    % 第一个障碍物速度
                      -0.05, 0.08]; % 第二个障碍物速度

    moving_obs.pos = [];   % 第二个动态障碍物
    moving_obs.vel = []; % 第二个障碍物速度
    % 障碍物半径 - 调整为更合理的大小
    obstacle_radius = 0.3;  % 原来1.5太大，改为0.3
end
