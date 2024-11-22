function params = sys_params()
% SYS_PARAMS basic parameters for the quadrotor

m = 0.18; % kg
g = 9.81; % m/s^2
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738];

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.arm_length = 0.086; % m

params.minF = 0.0;
params.maxF = 2.0 * m * g;

% ESO parameters
% Corresponding to the element which is designed to be controlled

params.alpha1 = 3; % pos
params.alpha2 = 3; % vel
params.alpha3 = 3; % acc

% Corresponding to the disturbance
params.beta1 = 4;
params.beta2 = 4;
params.beta3 = 4;
params.Ts = 0.01;  % Sampling time
params.delta = 0.05;  % 非线性函数fal的参数

% ESO parameters for different controllers
params.beta1_roll = 450;
params.beta2_roll = 13000;
params.beta3_roll = 370000;

params.beta1_pitch = 600;
params.beta2_pitch = 24000;
params.beta3_pitch = 980000;

params.beta1_yaw = 550;
params.beta2_yaw = 20000;
params.beta3_yaw = 650000;

params.beta1_alt = 450;
params.beta2_alt = 13000;
params.beta3_alt = 370000;

params.b_roll = 80;
params.b_pitch = 118;
params.b_yaw = 100;
params.b_alt = 1;




%%%%%%% 参数归零 %%%%%%%
% ESO parameters
% Corresponding to the element which is designed to be controlled
% 
% params.alpha1 = 0; % pos
% params.alpha2 = 0; % vel
% params.alpha3 = 0; % acc
% 
% % Corresponding to the disturbance
% params.beta1 = 0;
% params.beta2 = 0;
% params.beta3 = 0;
% params.Ts = 0.01;  % Sampling time
% params.delta = 0.0;  % 非线性函数fal的参数

% params.beta1_roll = 0;
% params.beta2_roll = 0;
% params.beta3_roll = 0;
% 
% params.beta1_pitch = 0;
% params.beta2_pitch = 0;
% params.beta3_pitch = 0;
% 
% params.beta1_yaw = 0;
% params.beta2_yaw = 0;
% params.beta3_yaw = 0;
% 
% params.beta1_alt = 0;
% params.beta2_alt = 0;
% params.beta3_alt = 0;
% 
% params.b_roll = 0;
% params.b_pitch = 0;
% params.b_yaw = 0;
% params.b_alt = 0;

end

