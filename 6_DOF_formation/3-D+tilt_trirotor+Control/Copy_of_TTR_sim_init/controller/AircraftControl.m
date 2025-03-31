classdef AircraftControl
    properties
        % PID controllers for position to velocity loops (ENU)
        ve_from_pe  % East
        vn_from_pn  % North
        vu_from_pu  % Up

        % PID controllers for velocity to acceleration loops (ENU)
        acc_e_from_ve
        acc_n_from_vn
        acc_u_from_vu

        % PID controllers for angle to angular rate loops (ENU) YXZ
        roll_rate_from_roll
        pitch_rate_from_pitch
        yaw_rate_from_yaw

        % PID controllers for angular rate to moment loops (ENU) YXZ
        My_from_roll_rate
        Mx_from_pitch_rate
        Mz_from_yaw_rate

        % PD controllers for roll and pitch with rate
        aileron_from_roll
        elevator_from_pitch

        % PI controllers for course, altitude
        roll_from_course
        pitch_from_altitude
        
        % Transfer function for yaw damper
        yaw_damper

        % PID controller for airspeed
        throttle_from_airspeed
    end


    methods
        % Constructor
        function obj = AircraftControl(params)
            % Initialize PID controllers for position loops (ENU)
            obj.ve_from_pe = PIDControl(params.pe_kp, params.pe_ki, params.pe_kd, params.Ts, params.sigma, params.vx_sat_limit);
            obj.vn_from_pn = PIDControl(params.pn_kp, params.pn_ki, params.pn_kd, params.Ts, params.sigma, params.vy_sat_limit);
            obj.vu_from_pu = PIDControl(params.pu_kp, params.pu_ki, params.pu_kd, params.Ts, params.sigma, params.vz_sat_limit);

            % Initialize PID controllers for velocity to acceleration loops (ENU)
            obj.acc_e_from_ve = PIDControl(params.vx_kp, params.vx_ki, params.vx_kd, params.Ts, params.sigma, params.acc_x_sat_limit);
            obj.acc_n_from_vn = PIDControl(params.vy_kp, params.vy_ki, params.vy_kd, params.Ts, params.sigma, params.acc_y_sat_limit);
            obj.acc_u_from_vu = PIDControl(params.vz_kp, params.vz_ki, params.vz_kd, params.Ts, params.sigma, params.acc_z_sat_limit);

            % Initialize PID controllers for angle to angular rate loops (ENU)
            obj.roll_rate_from_roll  = PIDControl(params.roll_kp, params.roll_ki, params.roll_kd, params.Ts, params.sigma, params.roll_rate_sat_limit);
            obj.pitch_rate_from_pitch = PIDControl(params.pitch_kp, params.pitch_ki, params.pitch_kd, params.Ts, params.sigma, params.pitch_rate_sat_limit);
            obj.yaw_rate_from_yaw    = PIDControl(params.yaw_kp, params.yaw_ki, params.yaw_kd, params.Ts, params.sigma, params.yaw_rate_sat_limit);

            % Initialize PID controllers for angular rate to moment loops (ENU)
            obj.My_from_roll_rate = PIDControl(params.roll_rate_kp, params.roll_rate_ki, params.roll_rate_kd, params.Ts, params.sigma, params.My_limit);
            obj.Mx_from_pitch_rate = PIDControl(params.pitch_rate_kp, params.pitch_rate_ki, params.pitch_rate_kd, params.Ts, params.sigma, params.Mx_limit);
            obj.Mz_from_yaw_rate = PIDControl(params.yaw_rate_kp, params.yaw_rate_ki, params.yaw_rate_kd, params.Ts, params.sigma, params.Mz_limit);

            % Initialize PD controllers for roll and pitch with rate
            obj.aileron_from_roll = PDControlWithRate(params.roll_cruise_kp, params.roll_cruise_kd, params.phi_max);
            obj.elevator_from_pitch = PDControlWithRate(params.pitch_cruise_kp, params.pitch_cruise_kd, params.theta_max);

            % Initialize PI controllers for course and altitude
            obj.roll_from_course = PIControl(params.course_kp, params.course_ki, params.Ts, params.course_max);
            obj.pitch_from_altitude = PIControl(params.altitude_kp, params.altitude_ki, params.Ts, deg2rad(30));

            % Initialize PID controller for airspeed
            obj.throttle_from_airspeed = PIDControl(params.airspeed_throttle_kp, params.airspeed_throttle_ki, ...
                                                     params.airspeed_throttle_kd, params.Ts, params.sigma, 1.0);
        end
    end
end

% % Initialize yaw damper transfer function
% obj.yaw_damper = TransferFunction(params.yaw_damper_kr, [1, params.yaw_damper_p_wo], params.Ts);



% Helper function for degree-to-radian conversion
function r = deg2rad(deg)
    r = deg * pi / 180;
end

%% 注解
%% PID 控制器引用
% 控制器参数
% kp = 2.0;      % 比例增益
% ki = 0.5;      % 积分增益
% kd = 1.0;      % 微分增益
% Ts = 0.01;     % 采样时间（10ms）
% sigma = 0.05;  % 微分低通滤波器时间常数
% limit = 10;    % 控制输出限制

% 创建PID控制器对象
% pid = PIDControl(kp, ki, kd, Ts, sigma, limit);

% method 1 --- 调用update方法
% 初始化参数
% y_ref = 1.0;  % 参考值
% y = 0.8;      % 测量值

% [u, pid] = pid.update(y_ref, y);

% 输出控制量
% disp(['控制输出: ', num2str(u)]);


% method 2 --- 调用updateWithRate方法
% 初始化参数
% y_ref = 1.0;  % 参考值
% y = 0.8;      % 测量值
% ydot = -0.1;  % 输出变化率

% [u, pid] = pid.updateWithRate(y_ref, y, ydot);
% 
% % 输出控制量
% disp(['控制输出 (带速率): ', num2str(u)]);



% 重置控制器状态
% [~, pid] = pid.update(y_ref, y, true);

% 控制输出超出限制 [-limit, limit]
% u = 15;  % 假设u过大
% u_sat = pid.saturate(u);

% disp(['饱和输出: ', num2str(u_sat)]);


%% 关键注意事项
% 低通滤波器：sigma 影响微分项的平滑效果，需要根据实际情况调整。
% 采样时间 Ts：应与实际系统的采样周期匹配。
% 反饱和积分项：控制器实现了抗积分饱和，避免积分项过度累积。

