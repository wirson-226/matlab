function [actuator] = actuator_assignment(force, moment, state, params)
    %   反向执行器分配，基于输入的力和力矩解算执行器输出（推力与力矩）
    %   机体坐标系 
    %   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
    %   state.rot = [phi; theta; psi], state.omega = [p; q; r]
    %   refernce -- media - assignmenet
    %   忽略空气动力学 todo --- 添加 aero --- doing
    
    % 分解输入的总力和力矩
    fx = force(1); % 前向推力
    fy = force(2); % 侧向推力
    fz = force(3); % 垂直推力
    Mx = moment(1); % 滚转力矩
    My = moment(2); % 俯仰力矩
    Mz = moment(3); % 偏航力矩

    % % 副翼和升降舵的偏转角（这里假设对称，不考虑额外的滚转控制）
    elevator = 0;   % 旋翼模式
    aileron = 0;  
    elevon_r = elevator + aileron;
    elevon_l = elevator - aileron;

    %% 计算 aero forces and moments
    % Relative velocity components (ground speed - wind speed)
    u_r = state.vel(1) - params.w_ns;
    v_r = state.vel(2) - params.w_es;
    w_r = state.vel(3) - params.w_ds;

    p = state.omega(1);
    q = state.omega(2);
    r = state.omega(3);

    % Compute airspeed (magnitude of velocity)
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    % Va = 0; % 忽略空气动力学
    
    % Compute angle of attack (alpha) and sideslip angle (beta)
    if Va == 0
        % Handle case when airspeed is zero (avoid NaN)
        alpha = 0;
        beta = 0;
    else
        if u_r == 0
            alpha = pi / 2;  % If u_r is zero, set alpha to 90 degrees
        else
            alpha = atan2(w_r, u_r);  % Angle of attack
        end
        
        beta = asin(v_r / Va);  % Sideslip angle
    end

    % Get MAV parameters
    C_L_0 = params.C_L_0;
    C_L_alpha = params.C_L_alpha;
    C_D_0 = params.C_D_0;
    C_D_alpha = params.C_D_alpha;
    C_Y_0 = params.C_Y_0;
    C_Y_beta = params.C_Y_beta;
    C_L_q = params.C_L_q;
    C_D_q = params.C_D_q;
    C_M_0 = params.C_m_0;
    C_M_alpha = params.C_m_alpha;
    C_M_q = params.C_m_q;
    C_M_delta_e = params.C_m_delta_e;
    C_Y_p = params.C_Y_p;
    C_Y_r = params.C_Y_r;
    C_ell_0 = params.C_ell_0;
    C_ell_beta = params.C_ell_beta;
    C_ell_p = params.C_ell_p;
    C_ell_r = params.C_ell_r;
    C_n_0 = params.C_n_0;
    C_n_beta = params.C_n_beta;
    C_n_p = params.C_n_p;
    C_n_r = params.C_n_r;
    MAV = params;  % Aircraft parameters from sys_params

    % Calculate Lift and Drag coefficients
    % Calculate C_L using the provided formulas
    term_1 = exp(-MAV.M * (alpha - MAV.alpha0));
    term_2 = exp(MAV.M * (alpha + MAV.alpha0));
    sigma = (1 + term_1 + term_2) / ((1 + term_1) * (1 + term_2));
    C_L = (1 - sigma) * (C_L_0 + C_L_alpha * alpha) + sigma * (2 * sign(alpha) * (sin(alpha)^2) * cos(alpha));

    % Calculate C_D using the provided formula
    C_D = MAV.C_D_p + (C_L_0 + C_L_alpha * alpha)^2 / (pi * MAV.e_os * MAV.AR);

    % Compute Lift and Drag Forces
    % 考虑空速为零
    rVS_2 = MAV.rho * (Va^2) * MAV.S_wing / 2;
    if Va == 0
        F_lift = 0;
        F_drag = 0;
        F_Y = 0;
    else
    F_lift = rVS_2 * (C_L + C_L_q * MAV.c / (2 * Va) * q + MAV.C_L_delta_e * elevator);
    F_drag = rVS_2 * (C_D + C_D_q * MAV.c / (2 * Va) * q + MAV.C_D_delta_e * elevator);
    F_Y = rVS_2 * (C_Y_0 + C_Y_beta * beta + C_Y_p * MAV.b / (2 * Va) * p + C_Y_r * MAV.b / (2 * Va) * r + MAV.C_Y_delta_a * aileron);
    end

    % Compute moments (torques) in body frame 
    % 考虑空速为零
    if Va == 0
        Aero_Mx = 0;
        Aero_My = 0;
        Aero_Mz = 0;
    else
    Aero_Mx = rVS_2 * MAV.b * (C_n_0 + C_n_beta * beta + C_n_p * MAV.b / (2 * Va) * p + C_n_r * MAV.b / (2 * Va) * r + MAV.C_n_delta_a * aileron + MAV.C_n_delta_r * 0) ; % 滚转
    Aero_My = rVS_2 * MAV.c * (C_M_0 + C_M_alpha * alpha + C_M_q * MAV.c / (2 * Va) * q + C_M_delta_e * elevator); % 俯仰
    Aero_Mz = rVS_2 * MAV.b * (C_ell_0 + C_ell_beta * beta + C_ell_p * MAV.b / (2 * Va) * p + C_ell_r * MAV.b / (2 * Va) * r + MAV.C_ell_delta_a * aileron + MAV.C_ell_delta_r * 0);% 偏航
    end

    % Mx = Aero_Mx + MAV.l3 * (thrust_prop_a_z - thrust_prop_b_z) + torque_prop_a_x - torque_prop_b_x; % 滚转 不忽略ab反扭倾转映射
    % My = Aero_My + MAV.l1 * (thrust_prop_a_z + thrust_prop_b_z) - thrust_prop_c_y * MAV.l2 - torque_prop_c_y; % 俯仰 不忽略c反扭倾转映射

    %% 测试隔离用
    % Aero_Mx = 0;
    % Aero_My = 0;
    % Aero_Mz = 0;

    % Compute longitudinal forces in body frame
    fx_aero = -F_lift * sin(alpha) - F_drag * cos(alpha) ; % 前右上 正向
    fz_aero =  F_lift * cos(alpha) + F_drag * sin(alpha) ;
    % fy_aero = F_Y;
    fy_aero = 0; % 忽略侧滑角


    %% 合并 空气动力学项 aero
    fx = fx - fx_aero;
    fy = fy - fy_aero;
    fz = fz - fz_aero;
    Mx = Mx - Aero_Mx;
    My = My - Aero_My;
    Mz = Mz - Aero_Mz;

    %% 计算电机的推力（假设推力沿着 Z 轴）

    % thrust_c_z = fz/3; % 假设悬停
    thrust_c_z = (fz - (My/params.l1))/3; % 细化求解 taz + tbz + tcz = Fz (-fz_aero); taz + tbz  = (My + tcz*l2)/l1 ---- 3tcz + My/l1 = Fz;
    thrust_b_z = (My + params.l2 * thrust_c_z)/(2*params.l1) + Mx/(2 * params.l3);
    thrust_a_z = (My + params.l2 * thrust_c_z)/(2*params.l1) - Mx/(2 * params.l3);
    thrust_a_x = (fx + Mz/(params.k_f + params.l3))/2;
    thrust_b_x = (fx - Mz/(params.k_f + params.l3))/2;



    % disp('fx:');
    % disp(fx);
    % 
    % disp('thrust_a_x:');
    % disp(thrust_a_x);

    % 计算总推力
    thrust_a_total = sqrt(thrust_a_x^2 + thrust_a_z^2);  % 电机 A 的总推力
    thrust_b_total = sqrt(thrust_b_x^2 + thrust_b_z^2);  % 电机 B 的总推力

    % 计算尾电机的侧向推力（假设推力沿着 Y 轴）
    thrust_c_total = thrust_c_z / cos(params.arm_c);

    % 计算 倾转角 rad
    arm_a = atan2(thrust_a_x,thrust_a_z);
    arm_b = atan2(thrust_b_x,thrust_b_z);

    % 将推力转换为油门输入
    throttle_a = thrust_to_throttle(thrust_a_total, params);  % 电机 A 的油门
    throttle_b = thrust_to_throttle(thrust_b_total, params);  % 电机 B 的油门
    throttle_c = thrust_to_throttle(thrust_c_total, params);  % 尾电机的油门
    

    % % 副翼和升降舵的偏转角（这里假设对称，不考虑额外的滚转控制）
    % elevator = 0;   % 旋翼模式
    % aileron = 0;  
    % elevon_r = elevator + aileron;
    % elevon_l = elevator - aileron;


    % 执行器 限制 油门 0-1, arm [-30,120], elevon [-45,45] deg
    arm_a = saturate(arm_a, params.arm_min,params.arm_max);
    arm_b = saturate(arm_b, params.arm_min,params.arm_max);
    throttle_a = saturate(throttle_a, params.T_percent_min,params.T_percent_max);  % 电机 A 的油门
    throttle_b = saturate(throttle_b, params.T_percent_min,params.T_percent_max);  % 电机 B 的油门
    throttle_c = saturate(throttle_c, params.T_percent_min,params.T_percent_max);  % 尾电机的油门
    elevon_r = saturate(elevon_r, params.elevon_min, params.elevon_max);
    elevon_l = saturate(elevon_l, params.elevon_min, params.elevon_max);

    % 返回执行器的控制指令（包含推力和偏转角） 行向量
    actuator.arm = [arm_a, arm_b];  % 返回倾转角
    actuator.throttle = [throttle_a, throttle_b, throttle_c];  % 返回油门
    actuator.elevon = [elevon_r, elevon_l];  % 返回副翼的偏转角（升降舵）r l 向上偏转为正 rad
end

function [throttle] = thrust_to_throttle(thrust, params)
    % 根据推力计算油门输入
    % thrust: 目标推力
    % params: 飞行器参数（包括最大推力和电机常数等）
    
    % 这里假设推力与油门线性关系
    throttle = thrust / params.T_max;  % 假设油门与最大推力成比例
end

% function [thrust, torque] = motor_thrust_torque(delta_t, MAV)
%     % Calculate the thrust and torque produced by a motor based on throttle input
%     % delta_t: throttle input [0, 1]
%     % MAV: Aircraft parameters from sys_params
%     thrust = delta_t * MAV.T_max;  % Thrust is proportional to throttle
%     torque = thrust * MAV.k_f;    % Torque is proportional to thrust and motor's thrust constant
% end
% torque = thrust * params.k_f