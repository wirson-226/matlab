function [des_from_ctrl,command] = cruise_controller(t, state, des_state, params)
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

addpath('utils');
addpath(genpath('E:\documents\Codes\codes\matlab\6_DOF_formation\3-D+tilt_trirotor+Control\TTR_sim_init_xuan_A'));

params = sys_params();
controller = AircraftControl(params);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mode 2 cruise 固定翼模式
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute airspeed (magnitude of velocity)
u_r = state.vel(1) - params.w_ns;
v_r = state.vel(2) - params.w_es;
w_r = state.vel(3) - params.w_ds;

Va = sqrt(u_r^2 + v_r^2 + w_r^2);
% Va = 15 ; % m/s


throttle = controller.throttle_from_airspeed.update(des_state.Va, Va);
vn_cmd = des_state.vel(1);
ve_cmd = des_state.vel(2);
vs_cmd = des_state.vel(3);
ta = throttle/2;
tb = ta;
tc = 0;
yaw_cmd = atan2(des_state.pos(2) - state.pos(2), des_state.pos(1) - state.pos(1)); % 期望偏航角
pitch_cmd = controller.pitch_from_altitude.update(des_state.pos(3), state.pos(3));
roll_cmd = controller.roll_from_course.update(yaw_cmd, state.rot(3));
elevator_cmd = controller.elevator_from_pitch.update(pitch_cmd, state.rot(2), state.omega(2));
aileron_cmd = controller.aileron_from_roll.update(roll_cmd, state.rot(1), state.omega(1));
elevon_r = elevator_cmd + aileron_cmd;
elevon_l = elevator_cmd - aileron_cmd;
arm_a = deg2rad(90);
arm_b = arm_a;

%% 测试用
command.throttle = [ta,tb,tc];
command.elevon = [elevon_r,elevon_l]; 
command.arm = [arm_a,arm_b];
%% 期望状态输出 1 * 12 --- vel-att-omege-M
des_from_ctrl = [vn_cmd, ve_cmd, vs_cmd, roll_cmd, pitch_cmd, yaw_cmd, 0, 0, 0, 0, 0, 0];


    

end
