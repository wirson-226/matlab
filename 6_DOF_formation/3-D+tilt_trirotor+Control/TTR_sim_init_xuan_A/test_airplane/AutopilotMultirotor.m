classdef AutopilotMultirotor
    properties
        % PID Controllers for each loop
        vx_from_pn
        vy_from_pe
        vz_from_pd
        acc_x_from_vx
        acc_y_from_vy
        acc_z_from_vz
        roll_rate_from_roll
        pitch_rate_from_pitch
        yaw_rate_from_yaw
        Mx_from_roll_rate
        My_from_pitch_rate
        Mz_from_yaw_rate
        % State and Commanded State
        true_state
        commanded_state
    end
    
    methods
        function obj = AutopilotMultirotor(ts_control)
            % Initialize PID controllers for each loop
            obj.vx_from_pn = PIDControl(ts_control, AP.pn_kp, AP.pn_ki, AP.pn_kd, AP.vx_sat_limit);
            obj.vy_from_pe = PIDControl(ts_control, AP.pe_kp, AP.pe_ki, AP.pe_kd, AP.vy_sat_limit);
            obj.vz_from_pd = PIDControl(ts_control, AP.pd_kp, AP.pd_ki, AP.pd_kd, AP.vz_sat_limit);
            
            obj.acc_x_from_vx = PIDControl(ts_control, AP.vx_kp, AP.vx_ki, AP.vx_kd, AP.acc_x_sat_limit);
            obj.acc_y_from_vy = PIDControl(ts_control, AP.vy_kp, AP.vy_ki, AP.vy_kd, AP.acc_y_sat_limit);
            obj.acc_z_from_vz = PIDControl(ts_control, AP.vz_kp, AP.vz_ki, AP.vz_kd, AP.acc_z_sat_limit);
            
            obj.roll_rate_from_roll = PIDControl(ts_control, AP.roll_kp, AP.roll_ki, AP.roll_kd, AP.roll_rate_sat_limit);
            obj.pitch_rate_from_pitch = PIDControl(ts_control, AP.pitch_kp, AP.pitch_ki, AP.pitch_kd, AP.pitch_rate_sat_limit);
            obj.yaw_rate_from_yaw = PIDControl(ts_control, AP.yaw_kp, AP.yaw_ki, AP.yaw_kd, AP.yaw_rate_sat_limit);
            
            obj.Mx_from_roll_rate = PIDControl(ts_control, AP.roll_rate_kp, AP.roll_rate_ki, AP.roll_rate_kd, 0);
            obj.My_from_pitch_rate = PIDControl(ts_control, AP.pitch_rate_kp, AP.pitch_rate_ki, AP.pitch_rate_kd, 0);
            obj.Mz_from_yaw_rate = PIDControl(ts_control, AP.yaw_rate_kp, AP.yaw_rate_ki, AP.yaw_rate_kd, 0);
            
            obj.true_state = MsgState();
            obj.commanded_state = MsgState();
        end
        
        % Position Loop (from position to desired velocity)
        function v_cmd = positionLoop(obj, position_cmd)
            vn_cmd = obj.vx_from_pn.update(position_cmd.pn_cmd, obj.true_state.north);
            ve_cmd = obj.vy_from_pe.update(position_cmd.pe_cmd, obj.true_state.east);
            vd_cmd = obj.vz_from_pd.update(position_cmd.pd_cmd, -obj.true_state.altitude);
            v_cmd = MsgVelocity(vn_cmd, ve_cmd, vd_cmd, position_cmd.psi_cmd);
        end
        
        % Velocity Loop (from velocity to desired angle and thrust)
        function attitude_cmd = velocityLoop(obj, v_cmd)
            yaw_cmd = wrap_multi(v_cmd.psi_cmd, -pi, pi);  % Yaw angle in [-pi, pi]
            c_psi_c = cos(yaw_cmd);
            s_psi_c = sin(yaw_cmd);
            
            acc_x_cmd = obj.acc_x_from_vx.update(v_cmd.vn_cmd, obj.true_state.vx);
            acc_y_cmd = obj.acc_y_from_vy.update(v_cmd.ve_cmd, obj.true_state.vy);
            acc_z_cmd = obj.acc_z_from_vz.update(v_cmd.vd_cmd, obj.true_state.vz) - TMAV.gravity;
            
            pitch_cmd = atan2(-acc_x_cmd * c_psi_c - acc_y_cmd * s_psi_c, -acc_z_cmd);
            pitch_cmd = wrap_multi(pitch_cmd, -pi/2, pi/2);  % Pitch angle in [-pi/2, pi/2]
            c_theta_c = cos(pitch_cmd);
            s_theta_c = sin(pitch_cmd);
            
            roll_cmd = atan2(c_theta_c * (-acc_x_cmd * s_psi_c + acc_y_cmd * c_psi_c), -acc_z_cmd);
            roll_cmd = wrap_multi(roll_cmd, -pi/2, pi/2);  % Roll angle in [-pi/2, pi/2]
            
            thrust_k_cmd = -TMAV.mass * acc_z_cmd / (c_phi_c * c_theta_c);
            thrust_i_cmd = TMAV.mass * acc_y_cmd / (s_theta_c * c_psi_c);
            
            attitude_cmd = MsgAttitude(roll_cmd, pitch_cmd, yaw_cmd, thrust_k_cmd, thrust_i_cmd);
        end
        
        % Attitude Loop (from angle to desired angular velocity)
        function [roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd] = attitudeLoop(obj, attitude_cmd)
            roll_cmd = attitude_cmd.phi_cmd;
            pitch_cmd = attitude_cmd.theta_cmd;
            yaw_cmd = attitude_cmd.psi_cmd;
            
            roll_cmd = saturate(roll_cmd, -AP.roll_input_limit, AP.roll_input_limit);
            pitch_cmd = saturate(pitch_cmd, -AP.pitch_input_limit, AP.pitch_input_limit);
            
            roll_rate_cmd = obj.roll_rate_from_roll.update(roll_cmd, obj.true_state.phi);
            pitch_rate_cmd = obj.pitch_rate_from_pitch.update(pitch_cmd, obj.true_state.theta);
            yaw_rate_cmd = obj.yaw_rate_from_yaw.update(yaw_cmd, obj.true_state.psi);
        end
        
        % Attitude Rate Loop (from angular velocity to desired torque)
        function [torque_x_cmd, torque_y_cmd, torque_z_cmd] = attitudeRateLoop(obj, roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd)
            torque_x_cmd = obj.Mx_from_roll_rate.update(roll_rate_cmd, obj.true_state.p);
            torque_y_cmd = obj.My_from_pitch_rate.update(pitch_rate_cmd, obj.true_state.q);
            torque_z_cmd = obj.Mz_from_yaw_rate.update(yaw_rate_cmd, obj.true_state.r);
        end
        
        % Power Distribution (converts thrust and torque to motor inputs)
        function [throttle_a_cmd, throttle_b_cmd, throttle_c_cmd, arm_a_cmd, arm_b_cmd] = powerDistribution(thrust_k_cmd, thrust_i_cmd, torque_x_cmd, torque_y_cmd, torque_z_cmd)
            thrust_k_cmd = thrust_k_cmd / TMAV.F_max;
            thrust_i_cmd = thrust_i_cmd / TMAV.F_max;
            torque_x_cmd = torque_x_cmd / TMAV.F_max;
            torque_y_cmd = torque_y_cmd / TMAV.F_max;
            torque_z_cmd = torque_z_cmd / TMAV.F_max;
            
            temp1 = 1/2 * (thrust_i_cmd + torque_z_cmd / TMAV.l3);
            temp2 = 1 / TMAV.l2 * (thrust_k_cmd * TMAV.l1 - torque_y_cmd);
            temp3 = 1/2 * ((thrust_k_cmd - temp2) + torque_x_cmd / TMAV.l3);
            temp4 = thrust_k_cmd - temp3 - temp2;
            temp5 = temp1 / temp3;
            temp6 = (thrust_i_cmd - temp1) / temp4;
            
            throttle_a_cmd = saturate(temp3, TMAV.T_min, TMAV.T_max);
            throttle_b_cmd = saturate(temp4, TMAV.T_min, TMAV.T_max);
            throttle_c_cmd = saturate(temp2, TMAV.T_min, TMAV.T_max);
            arm_a_cmd = saturate(temp5, TMAV.arm_min, TMAV.arm_max);
            arm_b_cmd = saturate(temp6, TMAV.arm_min, TMAV.arm_max);
        end
        
        % Update function (integrates all loops)
        function [delta, obj] = update(obj, position_cmd, state)
            obj.true_state = state;
            
            v_cmd = obj.positionLoop(position_cmd);
            attitude_cmd = obj.velocityLoop(v_cmd);
            [roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd] = obj.attitudeLoop(attitude_cmd);
            [torque_x_cmd, torque_y_cmd, torque_z_cmd] = obj.attitudeRateLoop(roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd);
            [throttle_a, throttle_b, throttle_c, arm_a, arm_b] = obj.powerDistribution(attitude_cmd.thrust_k_cmd, attitude_cmd.thrust_i_cmd, torque_x_cmd, torque_y_cmd, torque_z_cmd);
            
            delta = MsgDelta(0, 0, throttle_a, throttle_b, arm_a, arm_b, throttle_c);
            
            obj.commanded_state.north = position_cmd.pn_cmd;
            obj.commanded_state.east = position_cmd.pe_cmd;
            obj.commanded_state.altitude = -position_cmd.pd_cmd;
            obj.commanded_state.vx = v_cmd.vn_cmd;
            obj.commanded_state.vy = v_cmd.ve_cmd;
            obj.commanded_state.vz = v_cmd.vd_cmd;
            obj.commanded_state.phi = roll_rate_cmd;
            obj.commanded_state.theta = pitch_rate_cmd;
            obj.commanded_state.psi = yaw_rate_cmd;
            obj.commanded_state.p = torque_x_cmd;
            obj.commanded_state.q = torque_y_cmd;
            obj.commanded_state.r = torque_z_cmd;
        end
    end
end
