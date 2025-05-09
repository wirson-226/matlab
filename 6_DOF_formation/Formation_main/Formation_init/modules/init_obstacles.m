% modules/init_obstacles_grouped.m
function [static_obs, moving_obs, obstacle_radius] = init_obstacles()
   % === 静态障碍物分布 ===
    static_obs = [
        -10, -10;
        -7, 5;
        -3, 10;
        0, -8;
        3, 6;
        7, -4;
        10, 9
    ];

    % === 移动障碍物初始位置与速度 ===
    moving_obs.pos = [
        -6, 0;
         6, 0;
         0, 0
    ];
    moving_obs.vel = [
        0.08,  0.15;
       -0.07, -0.1;
        0.05, -0.05
    ];

    % === 障碍物半径 ===
    obstacle_radius = 1.5;
end
