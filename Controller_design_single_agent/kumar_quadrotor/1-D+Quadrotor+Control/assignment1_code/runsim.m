close all;
clear;

% Hover
z_des = 0;

% Step
% z_des = 1;

% Given trajectory generator
trajhandle = @(t) fixed_set_point(t, z_des);

% This is your controller
controlhandle = @controller(z,z_des,3,1,);

% Run simulation with given trajectory generator and controller
[t, z] = height_control(trajhandle, controlhandle);
