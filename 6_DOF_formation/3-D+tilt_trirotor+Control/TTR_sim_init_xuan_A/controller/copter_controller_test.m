function [des_from_ctrl,command] = copter_controller_test(t, state, des_state, params)
%   CONTROLLER  Controller for the quadrotor
%   
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%%   Current state
%   phi = state.rot(1);
%   theta = state.rot(2);
%   psi = state.rot(3);
%   p = state.omega(1);
%   q = state.omega(2);
%   r = state.omega(3);


%   des_state: The desired states are:
%   des_state.pos = [x; y; z], yaw_cmd = des_state.yaw
%   des_state.mode = [1]; 1 -- hovering  or 2 -- cruise or 3 -- transition

%   Using these current and desired states, you have to compute the desired
%   controls：des_from_ctrl = [vn_cmd, ve_cmd, vd_cmd, phi_cmd, theta_cmd,, yaw_cmd, p_cmd, q_cmd, r_cmd, Mx_cmd, My_cmd, Mz_cmd];


%% Application
% params = sys_params();
controller = AircraftControl(params);
% u_vx = controller.vx_from_pn.update(y_ref, y, reset_flag);


%% Position control -- velocity_des from position error using PIDControl(class)世界坐标系下
% PID gains from params..... also the limits of every virables
% pos_err = des_state.pos - state.pos; % pn_err = pos_err（1）,pe_err = pos_err（2),ps_err = pos_err（3), 北东天方向
vn_cmd = controller.vn_from_pn.update(des_state.pos(1), state.pos(1)); % n north
ve_cmd = controller.ve_from_pe.update(des_state.pos(2), state.pos(2)); % e east
vs_cmd = controller.vs_from_ps.update(des_state.pos(3), state.pos(3)); % s sky 这里写成d


%% velocity control -- attitudes cmd with acceleration_des from velocity error using PIDControl(class) 世界坐标系下
% vel_err_n = vn_cmd - state.vel(1);
% vel_err_e = ve_cmd - state.vel(2);
% vel_err_s = vs_cmd - state.vel(3);
acc_des = [0,0,0]; %  n e s 北东天
acc_des(1) = controller.acc_n_from_vn.update(vn_cmd, state.vel(1)); 
acc_des(2) = controller.acc_e_from_ve.update(ve_cmd, state.vel(2));
acc_des(3) = controller.acc_s_from_vs.update(vs_cmd, state.vel(3));




% 加速度到角度，解算线性简化 悬停平飞假设
% phi_cmd = (acc_des(1) * sin(des_state.yaw) - acc_des(2) * cos(des_state.yaw)) / params.gravity;
% theta_cmd = (acc_des(1) * cos(des_state.yaw) + acc_des(2) * sin(des_state.yaw)) / params.gravity;
psi_cmd = des_state.yaw;
yaw_cmd = wrap(psi_cmd, limit=pi);  % 偏航角，[-pi, pi]

% 第二种解算表达
theta_cmd = atan2(acc_des(1) * cos(psi_cmd) + acc_des(2) * sin(psi_cmd), params.gravity + acc_des(3));
theta_cmd = wrap(theta_cmd, limit=pi/ 2);  % 俯仰角，[-pi/2., pi/2.]
phi_cmd = atan2(cos(theta_cmd) * (acc_des(1) * sin(psi_cmd) - acc_des(2) * cos(psi_cmd)), params.gravity + acc_des(3));
phi_cmd = wrap(phi_cmd, limit=pi/ 2);      % 滚转角，[-pi/2., pi/2.]

% 避免奇异后限制范围
phi_cmd = saturate(phi_cmd, -params.roll_input_limit, params.roll_input_limit);
theta_cmd = saturate(theta_cmd, -params.pitch_input_limit, params.pitch_input_limit);


% Compute desired force 
thrust_n_cmd = params.mass * acc_des(1);
thrust_e_cmd = params.mass * acc_des(2);
thrust_s_cmd = params.mass * (acc_des(3) + params.gravity);

bRw = RPYToRot(state.rot);
force_cmd_body = bRw * [thrust_n_cmd; thrust_e_cmd; thrust_s_cmd]; % world to body nes --- xyz


%% attitude control -- p_cmd, q_cmd, r_cmd from accitudes error using PIDControl(class) 机体坐标系下
% phi_err = phi_cmd - state.rot(1); % roll
% theta_err = theta_cmd - state.rot(2); % pitch
% psi_err = psi_cmd - state.rot(3); % yaw

p_cmd = controller.roll_rate_from_roll.update(phi_cmd, state.rot(1));
q_cmd = controller.pitch_rate_from_pitch.update(theta_cmd, state.rot(2));
r_cmd = controller.yaw_rate_from_yaw.update(psi_cmd, state.rot(3));

%% omega control-- xyz 轴力矩 Mx_cmd, My_cmd, Mz_cmd from omega error using PIDControl(class) 机体坐标系下
% p_err = p_cmd - state.omega(1); % roll_rate
% q_err = q_cmd - state.omega(2); % pitch_rate
% r_err = r_cmd - state.omega(3); % yaw_rate
Mx_cmd = controller.Mx_from_roll_rate.update(p_cmd, state.omega(1)); %  with p_err
My_cmd = controller.My_from_pitch_rate.update(q_cmd, state.omega(2)); %  with q_err
Mz_cmd = controller.Mz_from_yaw_rate.update(r_cmd, state.omega(3)); %  with r_err

moment_cmd_body = [Mx_cmd; My_cmd; Mz_cmd];

%% 期望状态输出 1 * 12 --- vel-att-omege-M
des_from_ctrl = [vn_cmd, ve_cmd, vd_cmd, phi_cmd, theta_cmd, yaw_cmd, p_cmd, q_cmd, r_cmd, Mx_cmd, My_cmd, Mz_cmd];


%% 执行器命令结算 -- 控制分配  - 机体坐标系
command = actuator_assignment(force_cmd_body, moment_cmd_body, state, params);

%% 测试用
% command.throttle = [ta,tb,tc];
% command.elevon = [0,0]; 
% command.arm = [arm_a,arm_b];


u_r = state.vel(1) - params.w_ns;
v_r = state.vel(2) - params.w_es;
w_r = state.vel(3) - params.w_ds;


%% mode 2 cruise 固定翼模式

% Compute airspeed (magnitude of velocity)
Va = sqrt(u_r^2 + v_r^2 + w_r^2);

    while des_state.mode == 2
        throttle = controller.throttle_from_airspeed.update(des_state.Va, Va);
        ta = throttle/2;
        tb = ta;
        tc = 0;
        yaw_cmd = atan2(des_state.pos(2) - state.pos(2), des_state.pos(1) - state.pos(1)); % 期望偏航角
        pitch_cmd = controller.pitch_from_altitude.update(des_state.pos(3), state.pos(3));
        roll_cmd = controller.roll_from_course.update(yaw_cmd, state.rot(3));
        elevator_cmd = controller.elevator_from_pitch.update(pitch_cmd, state.rot(2));
        aileron_cmd = controller.aileron_from_roll.update(roll_cmd, state.rot(1));
        elevon_r = elevator_cmd + aileron_cmd;
        elevon_l = elevator_cmd - aileron_cmd;
        arm_a = 0;
        arm_b = 0;
    
        %% 测试用
        command.throttle = [ta,tb,tc];
        command.elevon = [elevon_r,elevon_l]; 
        command.arm = [arm_a,arm_b];
    end
    

end
