% modules/init_obstacles.m
function [static_obs, moving_obs, obstacle_radius] = init_obstacles()
    static_obs = [5,5;-5,5;0,8;3,-3;-4,-6];
    moving_obs.pos = [0,0; 8,-4];
    moving_obs.vel = [0.1,0.15;-0.08,0.12];
    obstacle_radius = 1.5;
end