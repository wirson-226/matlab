function [des_from_ctrl,command] = copter_controller_needtodo(t, state, des_state, params)
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


%%   des_state: The desired states are:
%   des_state.pos = [x; y; z], yaw_cmd = des_state.yaw，Va_cmd = des_state.Va
%   des_state.mode = [1]; 1 -- hovering  or 2 -- cruise or 3 -- transition

%   Using these current and desired states, you have to compute the desired
%   controls：des_from_ctrl = [vn_cmd, ve_cmd, vd_cmd, phi_cmd, theta_cmd,, yaw_cmd, p_cmd, q_cmd, r_cmd, Mx_cmd, My_cmd, Mz_cmd];


%% Application
% params = sys_params();
% addpath('utils');
% addpath(genpath('E:\documents\Codes\codes\matlab\6_DOF_formation\3-D+tilt_trirotor+Control\TTR_sim_init_xuan_A'));
controller = AircraftControl(params);
% u_vx = controller.vx_from_pn.update(y_ref, y, reset_flag);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mode 1 copter 固定翼模式
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
% 测试用
% acc_des = [0,2,0]; %  n e s 北东天

acc_des(1) = controller.acc_n_from_vn.update(vn_cmd, state.vel(1)); 
acc_des(2) = controller.acc_e_from_ve.update(ve_cmd, state.vel(2));
acc_des(3) = controller.acc_s_from_vs.update(vs_cmd, state.vel(3));




% 加速度到角度，解算线性简化 悬停平飞假设

% psi_cmd = des_state.yaw;
psi_cmd = deg2rad(0);
psi_cmd = wrap(psi_cmd, pi);  % 偏航角，[-pi, pi]

% 第一种解算表达
phi_cmd = (acc_des(1) * sin(psi_cmd) - acc_des(2) * cos(psi_cmd)) / params.gravity;
theta_cmd = (acc_des(1) * cos(psi_cmd) + acc_des(2) * sin(psi_cmd)) / params.gravity;

% 第二种解算表达
% theta_cmd = atan2(acc_des(1) * cos(psi_cmd) + acc_des(2) * sin(psi_cmd), params.gravity + acc_des(3));
% phi_cmd = atan2(cos(theta_cmd) * (acc_des(1) * sin(psi_cmd) - acc_des(2) * cos(psi_cmd)), params.gravity + acc_des(3));

phi_cmd = wrap(phi_cmd, pi/ 2);      % 滚转角，[-pi/2., pi/2.]
theta_cmd = wrap(theta_cmd, pi/ 2);  % 俯仰角，[-pi/2., pi/2.]


% 避免奇异后限制范围
phi_cmd = saturate(phi_cmd, -params.roll_input_limit, params.roll_input_limit);
theta_cmd = saturate(theta_cmd, -params.pitch_input_limit, params.pitch_input_limit);


% Compute desired force 
thrust_n_cmd = params.mass * acc_des(1);
thrust_e_cmd = params.mass * acc_des(2);
thrust_s_cmd = params.mass * (acc_des(3) + params.gravity);

bRw = RPYtoRot_ZXY(state.rot(1),state.rot(2),state.rot(3));
force_cmd_body = bRw * [thrust_n_cmd; thrust_e_cmd; thrust_s_cmd]; % world to body nes --- xyz 悬停假设
force_cmd_body =  [0; 0; force_cmd_body(3)]; % world to body nes --- xyz 悬停假设

% disp('body_forces:');
% disp(['body_forces_x: ', num2str(force_cmd_body(1))]);
% disp(['body_forces_y: ', num2str(force_cmd_body(2))]);
% disp(['body_forces_z: ', num2str(force_cmd_body(3))]);

%% attitude control -- p_cmd, q_cmd, r_cmd from accitudes error using PIDControl(class) 机体坐标系下


% phi_cmd= deg2rad(15);
% theta_cmd= deg2rad(15);
% psi_cmd= deg2rad(0);
p_cmd = controller.roll_rate_from_roll.update(phi_cmd, state.rot(1));
q_cmd = controller.pitch_rate_from_pitch.update(theta_cmd, state.rot(2));
r_cmd = controller.yaw_rate_from_yaw.update(psi_cmd, state.rot(3));

%% omega control-- xyz 轴力矩 Mx_cmd, My_cmd, Mz_cmd from omega error using PIDControl(class) 机体坐标系下
% p_err = p_cmd - state.omega(1); % roll_rate
% q_err = q_cmd - state.omega(2); % pitch_rate
% r_err = r_cmd - state.omega(3); % yaw_rate
Mx_cmd = -controller.Mx_from_roll_rate.update(p_cmd, state.omega(1)); %  with p_err 正俯仰产生x负加速度(负反馈)，正滚转产生y正加速度（正反馈），所以加-
My_cmd = controller.My_from_pitch_rate.update(q_cmd, state.omega(2)); %  with q_err  % Compute error  error = y_ref - y;
Mz_cmd = controller.Mz_from_yaw_rate.update(r_cmd, state.omega(3)); %  with r_err  正偏航产生正力矩 没有加速度解算，合理

moment_cmd_body = [Mx_cmd; My_cmd; Mz_cmd];

% moment_cmd_body = [0; 0; Mz_cmd];  % 得到控制输出但是阻断 为0的是仅显示 不接动力学

%% 期望状态输出 1 * 12 --- vel-att-omege-M
des_from_ctrl = [vn_cmd, ve_cmd, vs_cmd, phi_cmd, theta_cmd, psi_cmd, p_cmd, q_cmd, r_cmd, Mx_cmd, My_cmd, Mz_cmd];


%% 执行器命令结算 -- 控制分配  - 机体坐标系
command = actuator_assignment(force_cmd_body, moment_cmd_body, state, params);

%% 测试用
% command.throttle = [ta,tb,tc];
% command.elevon = [0,0]; 
% command.arm = [arm_a,arm_b];





%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mode 2 cruise 固定翼模式
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute airspeed (magnitude of velocity)
% u_r = state.vel(1) - params.w_ns;
% v_r = state.vel(2) - params.w_es;
% w_r = state.vel(3) - params.w_ds;
% 
% Va = sqrt(u_r^2 + v_r^2 + w_r^2);

    % while des_state.mode == 2
    %     throttle = controller.throttle_from_airspeed.update(des_state.Va, Va);
    %     vn_cmd = des_state.vel(1);
    %     ve_cmd = des_state.vel(2);
    %     vs_cmd = des_state.vel(3);
    %     ta = throttle/2;
    %     tb = ta;
    %     tc = 0;
    %     yaw_cmd = atan2(des_state.pos(2) - state.pos(2), des_state.pos(1) - state.pos(1)); % 期望偏航角
    %     pitch_cmd = controller.pitch_from_altitude.update(des_state.pos(3), state.pos(3));
    %     roll_cmd = controller.roll_from_course.update(yaw_cmd, state.rot(3));
    %     elevator_cmd = controller.elevator_from_pitch.update(pitch_cmd, state.rot(2), state.omega(2));
    %     aileron_cmd = controller.aileron_from_roll.update(roll_cmd, state.rot(1), state.omega(1));
    %     elevon_r = elevator_cmd + aileron_cmd;
    %     elevon_l = elevator_cmd - aileron_cmd;
    %     arm_a = deg2rad(90);
    %     arm_b = arm_a;
    % 
    %     %% 测试用
    %     command.throttle = [ta,tb,tc];
    %     command.elevon = [elevon_r,elevon_l]; 
    %     command.arm = [arm_a,arm_b];
    %     %% 期望状态输出 1 * 12 --- vel-att-omege-M
    %     des_from_ctrl = [vn_cmd, ve_cmd, vs_cmd, roll_cmd, pitch_cmd, yaw_cmd, 0, 0, 0, 0, 0, 0];
    % 
    % end
    

end
