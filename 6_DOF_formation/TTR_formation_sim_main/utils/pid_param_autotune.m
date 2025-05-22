
function params = pid_param_autotune(TF)
% 自动根据线性化传递函数 TF 和巡航空速 Va 计算 PID 参数
% 输入:
%   TF - struct，包含线性化参数 a_phi1, a_phi2, a_theta1, a_theta2, a_theta3, a_V1, a_V2
%   Va - 巡航空速
% 输出:
%   params - 含有全部控制器增益的结构体

params = sys_params();
Va = 1.2* params.V_min;


% ------ Roll loop (aileron to roll angle) ------
wn_roll = 10.0;           % 横滚自然频率 [rad/s]
zeta_roll = 0.707;        % 阻尼比
params.roll_kp = wn_roll^2 / TF.a_phi2;
params.roll_kd = (2*zeta_roll*wn_roll - TF.a_phi1) / TF.a_phi2;

% ------ Course loop (outer loop of roll) ------
wn_course = wn_roll / 20.0;
zeta_course = 1.0;
params.course_kp = 2 * zeta_course * wn_course * Va / 9.81;
params.course_ki = wn_course^2 * Va / 9.81;

% ------ Pitch loop (elevator to pitch angle) ------
wn_pitch = 6.0;
zeta_pitch = 0.707;
params.pitch_kp = (wn_pitch^2 - TF.a_theta2) / TF.a_theta3;
params.pitch_kd = (2*zeta_pitch*wn_pitch - TF.a_theta1) / TF.a_theta3;

% DC gain for altitude loop
K_theta_DC = params.pitch_kp * TF.a_theta3 / (TF.a_theta2 + params.pitch_kp * TF.a_theta3);

% ------ Altitude loop ------
wn_altitude = wn_pitch / 30.0;
zeta_altitude = 1.0;
params.altitude_kp = 2*zeta_altitude*wn_altitude / (K_theta_DC * Va);
params.altitude_ki = wn_altitude^2 / (K_theta_DC * Va);

% ------ Airspeed throttle loop ------
wn_airspeed = 1.0;
zeta_airspeed = 1.0;
params.airspeed_throttle_kp = (2*zeta_airspeed*wn_airspeed - TF.a_V1) / TF.a_V2;
params.airspeed_throttle_ki = wn_airspeed^2 / TF.a_V2;
params.airspeed_throttle_kd = 0;

% ------ Yaw damper ------
params.yaw_damper_p_wo = 1.0;
params.yaw_damper_kr = 0.5;

end