function params = sys_params()
% 全部参数定义  ---  控制参数 仅有 ESO 待添加 PID formation 等等
% aerodynamics, rotor parameters, and control-related parameters.
% 偏航尾部电机自平衡


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Physical Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kumar老机型参数 目前可用

m = 0.18; % kg
g = 9.81; % m/s^2
I = [0.00025,   0,          2.55e-6;
     0,         0.000232,   0;
     2.55e-6,   0,          0.0003738]; % kg m^2 

params.mass = m;
params.I    = I;
params.invI = inv(I);
params.Jx =  0.00025;
params.Jy =  0.000232;
params.Jz =  0.0003738;
params.Jxz = 0.00000255; % 一般忽略，两个数量级的差

params.gravity = g;
params.arm_length = 0.086; % m

params.minF = 0.0;
TW_ratio = 3 ; % 推重比
params.maxF = TW_ratio * m * params.gravity;  % 最大合力 -- z向

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%新机型 待调试匹配pid 后续再用
% params.mass = 0.295;  % kg
% params.g = 9.81;      % m/s^2, gravity
% params.I = [0.00229, 0, 0;
%             0, 0.00244, 0;
%             0, 0, 0.00462];  % kg m^2, inertia matrix for the quadrotor
% params.Jxz = 0.0000952;  % kg m^2


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.north0 = 0.0;  % initial north position
params.east0 = 0.0;   % initial east position
params.down0 = 0.0;  % initial down position
params.u0 = 0.0;  % initial velocity along body x-axis
params.v0 = 0.0;  % initial velocity along body y-axis
params.w0 = 0.0;  % initial velocity along body z-axis

params.phi0 = 0.0;  % initial roll angle
params.theta0 = 0.0;  % initial pitch angle
params.psi0 = 0.0;  % initial yaw angle
params.p0 = 0.0;  % initial roll rate
params.q0 = 0.0;  % initial pitch rate
params.r0 = 0.0;  % initial yaw rate

% params.init = [params.north0,params.east0,params.down0,
%                params.u0,params.v0,params.w0,
%                params.phi0,params.theta0,params.psi0,
%                params.p0,params.q0,params.r0]; % initial state 初始状态
%                有init_state 函数 存到了x0

params.Va0 = sqrt(params.u0^2 + params.v0^2 + params.w0^2);  % initial airspeed

% 风速
params.w_ns = 0.0; % 目前限定为0，待修改测试抗扰算法
params.w_es = 0.0;
params.w_ds = 0.0;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aerodynamic Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.rho = 1.2682;  % kg/m^3, air density
params.S_wing = 0.0773;  % m^2, wing area
params.b = 0.45;  % m, wing span
params.c = 0.00203;  % m, mean aerodynamic chord
params.S_prop = 0.0006;  % m^2, propeller area
params.e_os = 0.9;  % Oswald efficiency factor
params.AR = (params.b^2) / params.S_wing;  % Aspect ratio




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal Coefficients
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.C_L_0 = 0.15;
params.C_D_0 = 0.059195;
params.C_m_0 = 0.0135;
params.C_L_alpha = 4.52;
params.C_D_alpha = 0.03;
params.C_m_alpha = -2.74;
params.C_L_q = 7.95;
params.C_D_q = 0.0;
params.C_m_q = -38.21;
params.C_L_delta_e = 0.18;
params.C_D_delta_e = 0.0135;
params.C_m_delta_e = -1.13;
params.M = 50.0;
params.alpha0 = deg2rad(5.0);  % convert degrees to radians
params.epsilon = 0.16;
params.C_D_p = 0.0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Lateral Coefficients
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.C_Y_0 = 0.0;
params.C_ell_0 = 0.0;
params.C_n_0 = 0.0;
params.C_Y_beta = -0.98;
params.C_ell_beta = -0.06;
params.C_n_beta = 0.044;
params.C_Y_p = 0.0;
params.C_ell_p = -0.51;
params.C_n_p = 0.069;
params.C_Y_r = 0.0;
params.C_ell_r = 0.25;
params.C_n_r = -0.095;
params.C_Y_delta_a = 0.075;
params.C_ell_delta_a = 0.17;
params.C_n_delta_a = -0.011;
params.C_Y_delta_r = 0.19;
params.C_ell_delta_r = 0.0024;
params.C_n_delta_r = -0.069;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Propeller Parameters (Motor and Propeller)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.D_prop = 20 * 0.0254;  % prop diameter in m
params.K_V = 3750.0;  % from datasheet RPM/V
params.K_Q = (1 / params.K_V) * 60 / (2 * pi);  % KQ in N-m/A, V-s/rad
params.R_motor = 0.042;  % ohms
params.i0 = 1.5;  % no-load (zero-torque) current (A)
params.ncells = 3.0;  % battery cell count
params.V_max = 3.7 * params.ncells;  % max voltage for specified number of battery cells
params.C_Q2 = -0.01664;
params.C_Q1 = 0.004970;
params.C_Q0 = 0.005230;
params.C_T2 = -0.1079;
params.C_T1 = -0.06044;
params.C_T0 = 0.09357;
params.k_motor = 102.5;
params.kTp = 0.0;
params.kOmega = 0.0;
params.C_prop = 1.0;
params.k_f = 0.02; % thrust to torque todo 修改

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotor Arm Lengths and Tilt Angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.l1 = 0.009;     % m, front motor X-axis force arm 俯仰
params.l2 = 0.018;     % m, rear motor X-axis force arm  俯仰 大屁股 力臂
params.l3 = 0.011;     % m, front motor Y-axis force arm 滚转
params.arm_max = deg2rad(120.0);  % max motor arm tilt angle in radians  执行器限制
params.arm_min = deg2rad(-30.0);  % min motor arm tilt angle in radians
params.delta_lr_max = deg2rad(45.0);  % max elevon deflection angle
params.delta_lr_min = deg2rad(-45.0);  % min elevon deflection angle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thrust and Torque Limits for Motor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% params.T_max = 4.0;  % N, max thrust produced by a single motor
params.T_max = params.maxF/3;  % 三旋翼垂起 执行器限制 single rotor thrust
params.T_min = 0.0;
params.T_percent_max = 1.0;  % maximum throttle value thrust abc 油门表示 目前不需要
params.T_percent_min = 0.0;  % minimum throttle value



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 偏航自定义 plan A 尾部自平衡 参考media---new_Mx_set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.arm_c = atan2(params.k_f, params.l2);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path planner Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.WS_ratio = params.mass/params.S_wing; % 翼载荷
% Calculate maximum cruise speed
params.V_max = sqrt((4 * params.T_max) / (params.rho * params.S_wing * params.C_D_0)); % 最大平飞速度 和推重比有关， T = D, 螺旋桨的话还和桨叶尺寸和转速有关
% Calculate minimum cruise speed (V_min)
params.V_min = sqrt((2 * m * params.gravity) / (params.rho * params.S_wing * params.C_L_0)); % 最小平飞速度和翼载荷有关 L = D



% dubin path 固定翼杜宾曲线相关限制 最小转弯半径，巡航速度
% airspeed commanded by planner
params.Va_planner = 25.0;

% max possible roll angle
params.phi_max = deg2rad(30);

% minimum turn radius
params.R_min = params.Va_planner^2 / params.gravity / tan(params.phi_max);





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ESO Parameters (Extended State Observer)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.b0 = 0.07;
params.alpha1 = 3;  % Position ESO gain
params.alpha2 = 3;  % Velocity ESO gain
params.alpha3 = 3;  % Acceleration ESO gain
params.beta1 = 4;  % Position disturbance ESO gain
params.beta2 = 4;  % Velocity disturbance ESO gain
params.beta3 = 4;  % Acceleration disturbance ESO gain
params.Ts = 0.01;  % Sampling time

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Additional Control Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.b_roll = 80;
params.b_pitch = 118;
params.b_yaw = 100;
params.b_alt = 1;
params.delta = 0.05;  % Non-linear function parameter for ESO

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculation Variables for Dynamics
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.gamma = params.Jx * params.Jz - (params.Jxz ^ 2);
params.gamma1 = (params.Jxz * (params.Jx - params.Jy + params.Jz)) / params.gamma;
params.gamma2 = (params.Jz * (params.Jz - params.Jy) + (params.Jxz ^ 2)) / params.gamma;
params.gamma3 = params.Jz / params.gamma;
params.gamma4 = params.Jxz / params.gamma;
params.gamma5 = (params.Jz - params.Jx) / params.Jy;
params.gamma6 = params.Jxz / params.Jy;
params.gamma7 = ((params.Jx - params.Jy) * params.Jx + (params.Jxz ^ 2)) / params.gamma;
params.gamma8 = params.Jx / params.gamma;

% Control Coefficients
params.C_p_0 = params.gamma3 * params.C_ell_0 + params.gamma4 * params.C_n_0;
params.C_p_beta = params.gamma3 * params.C_ell_beta + params.gamma4 * params.C_n_beta;
params.C_p_p = params.gamma3 * params.C_ell_p + params.gamma4 * params.C_n_p;
params.C_p_r = params.gamma3 * params.C_ell_r + params.gamma4 * params.C_n_r;
params.C_p_delta_a = params.gamma3 * params.C_ell_delta_a + params.gamma4 * params.C_n_delta_a;
params.C_p_delta_r = params.gamma3 * params.C_ell_delta_r + params.gamma4 * params.C_n_delta_r;
params.C_r_0 = params.gamma4 * params.C_ell_0 + params.gamma8 * params.C_n_0;
params.C_r_beta = params.gamma4 * params.C_ell_beta + params.gamma8 * params.C_n_beta;
params.C_r_p = params.gamma4 * params.C_ell_p + params.gamma8 * params.C_n_p;
params.C_r_r = params.gamma4 * params.C_ell_r + params.gamma8 * params.C_n_r;
params.C_r_delta_a = params.gamma4 * params.C_ell_delta_a + params.gamma8 * params.C_n_delta_a;
params.C_r_delta_r = params.gamma4 * params.C_ell_delta_r + params.gamma8 * params.C_n_delta_r;

end
