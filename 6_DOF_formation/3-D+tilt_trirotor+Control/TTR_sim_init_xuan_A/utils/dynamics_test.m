% 动力学测试用
% Get aircraft parameters from sys_params
params = sys_params();

% Example inputs
% state = [0, 0, -20, params.V_min, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % m/s (ground speed)
state = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % m/s (ground speed)
% elevon_a = deg2rad(10);
% elevon_b = deg2rad(10);

%% 满油平飞 测试用
% command.throttle = [1,1,0];
% command.arm = [pi/2,pi/2];
% command.elevon = [elevon_a, elevon_b];
% command.elevon = [params.elevon_max, params.elevon_max];

%% 过渡 hovering to cruise 定高加速 测试用 
% command.throttle = [2/3,2/3,1/3];
% command.elevon = [0,0]; 
% command.arm = [pi/6,pi/6];



%% 旋翼悬停平衡计算测试 参考Media--TTR_arm_a
% 偏航plan A
arm_a = 0;
arm_b = -arm_a;
% tc = (params.mass * params.gravity * 1/3) / (params.T_max * cos(params.arm_c));
tc = (1/3) / cos(params.arm_c);
ta = 2 * (params.l1/params.l2) * tc * cos(params.arm_c);
tb = ta;

% 偏航plan B
% tc = 1/3 ;
% arm_a = atan2(params.k_f, params.l3);
% arm_b = -arm_a;
% % ta = (tc * params.k_f) / ( 2 * params.l3 * sin(arm_a));
% ta = tc / (2 * cos(arm_a));
% tb = ta;

command.throttle = [ta,tb,tc];
command.elevon = [0,0]; 
command.arm = [arm_a,arm_b];



%% Call the function to calculate aerodynamic forces and moments
[force, moment] = all_forces_moments(state, command, params);


% 重力
Fg = params.mass * params.gravity;


% Display the results
disp('Forces:');
disp(force);

disp('Moments:');
disp(moment);


disp('arm_c:');
disp(rad2deg(params.arm_c));

%% 显示设计性能
disp('系统设计性能计算V_max: m/s');
disp(params.V_max);
disp('系统设计性能计算V_min: m/s');
disp(params.V_min);
% disp('系统设计性能计算phi_max: deg');
% disp(rad2deg(params.phi_max));
% disp('系统设计性能计算R_min:  m');
% disp(params.R_min);

% % 创建一个图形窗口并设置名称
% fig = figure('name', 'arm_a calculation reference', 'NumberTitle', 'off');
% % figure;
% % 在窗口中显示图像
% imshow('E:\documents\Codes\codes\matlab\6_DOF_formation\3-D+tilt_trirotor+Control\TTR_sim_init_xuan_A\Medias\TTR_arm_a.jpg'); % Load the image (ensure the file path is correct)
% title('神的笔记');