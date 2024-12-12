function [force, moment] = all_forces_moments(state, actuator, params)
    % AERODYNAMIC_FORCES_MOMENTS calculates the aerodynamic forces and moments in the body axis system
    % based on the provided ground speed, wind speed, and control surfaces (delta).
    %
    % Inputs:
    % ground_speed - Aircraft ground speed [u, v, w] (body frame) (m/s)
    % wind_speed - Wind speed [u_w, v_w, w_w] (wind frame) (m/s)
    % delta - Control surfaces deflections (structure with aileron_l, aileron_r, throttle_a, throttle_b, etc.)
    % xyz 前右上，abc电机分别 位置右左尾 转向顺逆逆 带来反扭 逆顺顺 其中逆是朝z正方向 所以是 + - -
    % thrust a = b 但是不一定 = c ，不过 a+b/c = l2/l1 a + b + c = F
    % params - Aircraft parameters structure (params from sys_params)
    %
    % Outputs: 机体坐标系 xyz 前右上
    % force - Aerodynamic forces [Fx, Fy, Fz] (N)
    % moment - Aerodynamic moments [Mx, My, Mz] (N·m)

    % Relative velocity components (ground speed - wind speed)
    u_r = state(4) - params.w_ns;
    v_r = state(5) - params.w_es;
    w_r = state(6) - params.w_ds;

    p = state(11);
    q = state(12);
    r = state(13);

    % Compute airspeed (magnitude of velocity)
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    
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

    % Display debug values for key parameters
    disp(['u_r: ', num2str(u_r), ', v_r: ', num2str(v_r), ', w_r: ', num2str(w_r)]);
    disp(['Va: ', num2str(Va), ', alpha: ', num2str(alpha), ', beta: ', num2str(beta)]);

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

    % Control surface deflections (elevator, aileron) 
    % 应该向下偏转为负，向上为正 俯仰抬头 以此给 +My---rad 向上偏转为正
    elevator = 1/2 * (actuator.elevon_l + actuator.elevon_r);  % Equivalent deflection for elevons (flying wing)
    aileron = 1/2 * (actuator.elevon_l - actuator.elevon_r);

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


    % Display the aerodynamics results
    disp('Aero_forces:');
    disp(['F_lift: ', num2str(F_lift)]);
    disp(['F_drag: ', num2str(F_drag)]);
    disp(['F_Y: ', num2str(F_Y)]);

    % Compute propeller thrust and torque
    [thrust_prop_a, torque_prop_a] = motor_thrust_torque(actuator.throttle_a, MAV);
    [thrust_prop_b, torque_prop_b] = motor_thrust_torque(actuator.throttle_b, MAV);
    [thrust_prop_c, torque_prop_c] = motor_thrust_torque(actuator.throttle_c, MAV);
    
    % Propeller thrust in body frame (with arm lengths) 注意是弧度 rad
    c_arm_a = cos(actuator.arm_a);
    s_arm_a = sin(actuator.arm_a);
    c_arm_b = cos(actuator.arm_b);
    s_arm_b = sin(actuator.arm_b);

    % 倾转分量旋转矩阵 拆分捞逼形式
    thrust_prop_a_x = thrust_prop_a * s_arm_a;
    thrust_prop_b_x = thrust_prop_b * s_arm_b;
    thrust_prop_a_z = thrust_prop_a * c_arm_a;
    thrust_prop_b_z = thrust_prop_b * c_arm_b;

    torque_prop_a_x = torque_prop_a * s_arm_a;
    torque_prop_b_x = torque_prop_b * s_arm_b;
    torque_prop_a_z = torque_prop_a * c_arm_a;
    torque_prop_b_z = torque_prop_b * c_arm_b;

    % Compute longitudinal forces in body frame
    fx = -F_lift * sin(alpha) - F_drag * cos(alpha) + thrust_prop_a_x + thrust_prop_b_x; % 前右上 正向
    fz = F_lift * cos(alpha) + F_drag * sin(alpha) + thrust_prop_a_z + thrust_prop_b_z;

    % Compute lateral forces in body frame
    fy = F_Y;

    % propulsion relative forces
    disp('Prop_forces:');
    disp(['thrust_prop_a: ', num2str(thrust_prop_a)]);
    disp(['thrust_prop_b: ', num2str(thrust_prop_b)]);
    disp(['thrust_prop_c: ', num2str(thrust_prop_c)]);

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

    % Display the aerodynamics results
    disp('Aero_moments:');
    disp(['Aero_Mx: ', num2str(Aero_Mx)]);
    disp(['Aero_My: ', num2str(Aero_My)]);
    disp(['Aero_Mz: ', num2str(Aero_Mz)]);

    Mx = Aero_Mx + MAV.l3 * (thrust_prop_a_z - thrust_prop_b_z) + torque_prop_a_x - torque_prop_b_x; % 滚转
    My = Aero_My + MAV.l1 * (thrust_prop_a_z + thrust_prop_b_z) - thrust_prop_c * MAV.l2; % 俯仰
    Mz = Aero_Mz + MAV.l3 * (thrust_prop_a_x - thrust_prop_b_x) + torque_prop_a_z - torque_prop_b_z - torque_prop_c;% 偏航



    % Mx = rVS_2 * MAV.b * (C_n_0 + C_n_beta * beta + C_n_p * MAV.b / (2 * Va) * p + C_n_r * MAV.b / (2 * Va) * r + MAV.C_n_delta_a * aileron + MAV.C_n_delta_r * 0) + MAV.l3 * (thrust_prop_a_z - thrust_prop_b_z) + torque_prop_a_x - torque_prop_b_x; % 滚转
    % My = rVS_2 * MAV.c * (C_M_0 + C_M_alpha * alpha + C_M_q * MAV.c / (2 * Va) * q + C_M_delta_e * elevator) + MAV.l1 * (thrust_prop_a_z + thrust_prop_b_z) - thrust_prop_c * MAV.l2; % 俯仰
    % Mz = rVS_2 * MAV.b * (C_ell_0 + C_ell_beta * beta + C_ell_p * MAV.b / (2 * Va) * p + C_ell_r * MAV.b / (2 * Va) * r + MAV.C_ell_delta_a * aileron + MAV.C_ell_delta_r * 0) + MAV.l3 * (thrust_prop_a_x - thrust_prop_b_x) + torque_prop_a_z - torque_prop_b_z - torque_prop_c;% 偏航

    % Return the forces and moments in body frame
    force = [fx; fy; fz];   % Aerodynamic forces [Fx, Fy, Fz] in body frame
    moment = [Mx; My; Mz];  % Aerodynamic moments [Mx, My, Mz] in body frame
end

function [thrust, torque] = motor_thrust_torque(delta_t, MAV)
    % Calculate the thrust and torque produced by a motor based on throttle input
    % delta_t: throttle input [0, 1]
    % MAV: Aircraft parameters from sys_params
    thrust = delta_t * MAV.T_max;  % Thrust is proportional to throttle
    torque = thrust * MAV.k_f;    % Torque is proportional to thrust and motor's thrust constant
end
