function params = sys_params()
% 全部参数定义  ---  控制参数 仅有 ESO 待添加 PID formation 等等
% aerodynamics, rotor parameters, and control-related parameters.
% 偏航尾部电机自平衡


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Physical Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% kumar老机型参数 目前可用

m = 2;% (X1500)  % 0.18; % kg
g = 9.81; % m/s^2
params.mass = m;
% params.Jx =  0.00025;
% params.Jy =  0.000232;
% params.Jz =  0.0003738;
% params.Jxz = 0.00000255; % 一般忽略，两个数量级的差 

params.Jx =  0.1015378; % kg * m^2 -- e-9-- g * mm^2
params.Jy =  0.0882566;
params.Jz =  0.1877412;
params.Jxz = 0.0010278; % 一般忽略，两个数量级的差 X1500

I = [params.Jx,    0,            params.Jxz;
     0,            params.Jy,    0;
     params.Jxz,   0,            params.Jz]; % kg m^2 

params.I    = I;
params.invI = inv(I);
params.gravity = g;
params.minF = 0.0;
TW_ratio = 3 ; % 推重比
params.maxF = TW_ratio * m * params.gravity;  % 最大合力 -- z向

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial Conditions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.north0 = 0.0;  % initial north position
params.east0 = 0.0;   % initial east position
params.up = 0.0;  % initial down position
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

% 风速 -- ENU 东北天坐标系
params.w_es = 0.0; % 目前限定为0，待修改测试抗扰算法
params.w_ns = 0.0;
params.w_us = 0.0;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Aerodynamic Parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.rho = 1.2682;  % kg/m^3, air density
params.S_wing = 0.6004; % (X1500) % 0.0773;  % m^2, wing area(X450)
params.b =1.5; % (X1500)  0.45;  % m, wing span
params.c = 0.55; % (X1500)  % 0.00203;  % m, mean aerodynamic chord


params.S_prop = 0.0006;  % m^2, propeller area
params.e_os = 0.9;  % Oswald efficiency factor
params.AR = (params.b^2) / params.S_wing;  % Aspect ratio




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Longitudinal Coefficients
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.C_L_0 = 0.509;
params.C_D_0 = 0.023;
params.C_m_0 = 0.0135;
params.C_L_alpha = 7.61;
params.C_D_alpha = 0.13;
params.C_m_alpha = -2.74;
params.C_L_q = 7.95;
params.C_D_q = 0.0;
params.C_m_q = -38.21;
params.C_L_delta_e = 0.18;
params.C_D_delta_e = 0.0135;
params.C_m_delta_e = -2.13;
params.M = 50.0;
params.alpha0 = deg2rad(3.0);  % convert degrees to radians
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
% params.V_max = 3.7 * params.ncells;  % max voltage for specified number of battery cells
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


params.k_f = 0.02454; % thrust to torque todo 修改

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Rotor Arm Lengths and Tilt Angles
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% params.l1 = 3*0.09;     % m, front motor X-axis force arm 俯仰
% params.l2 = 3*0.18;     % m, rear motor X-axis force arm  俯仰 大屁股 力臂 0.18*mg = Mx max--0.31752
% params.l3 = 3*0.11;     % m, front motor Y-axis force arm 滚转            0.11*mg = My max--0.194  2*0.11*mg = Mz max--0.388
params.l1 = 0.27;     % m, front motor X-axis force arm 俯仰
params.l2 = 0.54;     % m, rear motor X-axis force arm  俯仰 大屁股 力臂 0.54*mg = Mx max--10.584 n*m
params.l3 = 0.33;     % m, front motor Y-axis force arm 滚转            0.33*mg = My max--6.468 n*m  0.5*2*0.33*mg = Mz max--5.874

params.arm_max = deg2rad(120.0);  % max motor arm tilt angle in radians  执行器限制
params.arm_min = deg2rad(-30.0);  % min motor arm tilt angle in radians
params.elevon_max = deg2rad(45.0);  % max elevon deflection angle
params.elevon_min = deg2rad(-45.0);  % min elevon deflection angle

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thrust and Torque Limits for Motor
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% params.T_max = 4.0;  % N, max thrust produced by a single motor
params.T_max = params.maxF/3;  % 三旋翼垂起 执行器限制 single rotor thrust --mg
params.T_min = 0.0;
params.T_percent_max = 1.0;  % maximum throttle value thrust abc 油门表示 目前不需要
params.T_percent_min = 0.0;  % minimum throttle value



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 偏航自定义 plan A 尾部自平衡 参考media---new_Mx_set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.arm_c = atan2(params.k_f, params.l2);

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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control parameters --- mode 1 copter
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
params.Ts = 0.01;     % 采样时间（10ms）
params.sigma = 0.05;  % 微分低通滤波器时间常数

% % ----------position loop-------------
params.pe_kp = 3;
params.pe_ki = 0.0;
params.pe_kd = 0;
params.vx_sat_limit = 10;  % m/s

params.pn_kp = 3;
params.pn_ki = 0.0;
params.pn_kd = 0;
params.vy_sat_limit = 10;  % m/s

params.pu_kp = 3;
params.pu_ki = 0.00;
params.pu_kd = 0;
params.vz_sat_limit = 10;  % m/s

% % ---------- velocity loop -------------
% % the outputs are phi_desired and theta_desired
params.vh_kp = 4;       % horizontal
params.vh_ki = 0;
params.vh_kd = 1;

params.vx_kp = params.vh_kp;
params.vx_ki = params.vh_ki;
params.vx_kd = params.vh_kd;
params.acc_x_sat_limit = 2 * params.gravity;

params.vy_kp = params.vh_kp;
params.vy_ki = params.vh_ki;
params.vy_kd = params.vh_kd;
params.acc_y_sat_limit = 2 * params.gravity;

% % the output is collective thrust
params.vz_kp = 4;  % 8.0
params.vz_ki = 0 ; % 2.0
params.vz_kd = 1;
params.acc_z_sat_limit = 2 * params.gravity;

% % TODO: 加速度的限幅值和角度约束和拉力约束是有对应关系的，这个进行辨识后要算出来。

% % ---------- attitude loop -------------
params.roll_kp = 6;
params.roll_ki = 0.0;
params.roll_kd = 0.1;
params.roll_input_limit = 30. * pi / 180.;  % rad
params.roll_rate_sat_limit = 180.0 * pi / 180.0 ; % rad/s

params.pitch_kp = 6;
params.pitch_ki = 0.0;
params.pitch_kd = 0.1;
params.pitch_input_limit = 30. * pi / 180.;  % rad
params.pitch_rate_sat_limit = 180.0 * pi / 180.0 ; % rad/s

params.yaw_kp = 4;
params.yaw_ki = 0.0;
params.yaw_kd = 0.2;
params.yaw_rate_sat_limit = 90.0 * pi / 180.0;  % rad/s

% % ---------- attitude rate loop -------------
params.roll_rate_kp = 6;
params.roll_rate_ki = 0.1;
params.roll_rate_kd = 0.05;

params.pitch_rate_kp = 6;
params.pitch_rate_ki = 0.1;
params.pitch_rate_kd = 0.05;

params.yaw_rate_kp = 4;
params.yaw_rate_ki = 0.1;
params.yaw_rate_kd = 0.05;

% params.My_limit = 0.00019; % roll
% params.Mx_limit = 0.00032; % pitch
% params.Mz_limit = 0.00038; % yaw

% X1500
params.My_limit = 6.468;       % roll
params.Mx_limit = 10.584;      % pitch
params.Mz_limit = 5.874;       % yaw

% 俯仰 0.54*mg = Mx max--10.584 n*m
% 滚转 0.33*mg = My max--6.468 n*m  0.5*2*0.33*mg = Mz max--5.874

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
params.Va_planner = 0.5*(params.V_max + params.V_min);

% max possible roll angle
params.phi_max = deg2rad(60);
params.phi_min = deg2rad(-60);
params.theta_max = deg2rad(60);
params.theta_min = deg2rad(-45);
params.psi_max = deg2rad(160);
params.psi_min = deg2rad(-160);
params.course_max = deg2rad(60);

% minimum turn radius
params.R_min = params.Va_planner^2 / params.gravity / tan(params.phi_max);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Control parameters --- mode 2 cruise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ----------roll loop-------------
% get transfer function data for delta_a to phi
% wn_roll = 6.0;  % 7 old (003) 12.0 / 18.0(004)
% zeta_roll = 1.1; %old (003) 1.15  /1.4(004)
% params.roll_kp = wn_roll^2 / TF.a_phi2;
% params.roll_kd = (2.0 * zeta_roll * wn_roll - TF.a_phi1) / TF.a_phi2;
params.roll_cruise_kp = 0.035;
params.roll_cruise_kd = 0.0012;
% print('roll_kp = ',params.roll_kp,'  roll_kd = ',params.roll_kd);

% ----------course loop-------------
% wn_course = wn_roll / 20.0;
% zeta_course = 1.0;
% params.course_kp = 2.0 * zeta_course * wn_course * params.Va_planner / params.gravity;
% params.course_ki = wn_course ^ 2 * params.Va_planner / params.gravity;
params.course_kp = 0.003;
params.course_ki = 0.001;
% print('course_kp = ',params.course_kp,'  course_ki = ',params.course_ki);

% ----------yaw damper-------------
params.yaw_damper_p_wo = 0.5 ; % (old) 1/0.5
params.yaw_damper_kr = 0.5; % (old) 0.5

% ----------pitch loop-------------
% wn_pitch = 26.0;   % old 24.0/30.0(003) /25.0(004)
% zeta_pitch = 1.25;  % old 0.707/0.95(003) /1.25(004)
% params.pitch_kp = (wn_pitch ^ 2 - TF.a_theta2) / TF.a_theta3;
% params.pitch_kd = (2.0 * zeta_pitch * wn_pitch - TF.a_theta1) / TF.a_theta3;
params.pitch_cruise_kp = -500.0;
params.pitch_cruise_kd = -25.0;
% params.K_theta_DC = pitch_kp * TF.a_theta3 / (TF.a_theta2 + pitch_kp * TF.a_theta3);
% print('pitch_kp = ',params.pitch_kp,'   pitch_kd = ', params.pitch_kd);


% ----------altitude loop-------------
% wn_altitude = wn_pitch / 30.0;
% zeta_altitude = 1.31 ; %old 1.31(005)/
% params.altitude_kp = 2.0 * zeta_altitude * wn_altitude / K_theta_DC / params.Va0;
% params.altitude_ki = wn_altitude ^ 2 / K_theta_DC / params.Va_planner;
params.altitude_zone = 10.0;  % moving saturation limit around current altitude
% params.altitude_kd = 0.0;
params.altitude_kp = 0.0;
params.altitude_ki = 0.0;
% print('altitude_kp = ',params.altitude_kp,'  altitude_ki = ',params.altitude_ki);

% ---------airspeed hold using throttle---------------
% wn_airspeed_throttle = 8.5  % old 3.0/
% zeta_airspeed_throttle = 7.5    % 0.707
% airspeed_throttle_kp = (2.0 * zeta_airspeed_throttle * wn_airspeed_throttle - TF.a_V1) / TF.a_V2
% airspeed_throttle_ki = wn_airspeed_throttle ^ 2 / TF.a_V2
params.airspeed_throttle_kp = 0.75;
params.airspeed_throttle_ki = 0.35;
params.airspeed_throttle_kd = 0.0;
% print('airspeed_kp = ', params.airspeed_throttle_kp,'   airspeed_ki = ',params.airspeed_throttle_ki, ' airspeed_kd = ',params.airspeed_throttle_kd);



end
