function[sdot] = vtolEOM(t, s, controlhandle, trajhandle, params)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 13 x 1, state vector = [x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r]
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from sys_params() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 13 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, sys_params

% convert state to quad stuct for control
current_state = stateToQd(s);

% Get desired_state
desired_state = trajhandle(t, current_state); % stuct 结构

% get control outputs
[DES_ctrl,command] = controlhandle(t, current_state, desired_state, params);  % s: [13 * 1]; 这里的输出应该是执行器command 
%   而后 利用 all_foreces_moments得到FM
[force, moment] = all_forces_moments(s, command, params); % s 行向量结构


% %% 测试用 机体坐标系合力合力矩
% % Display the results
% disp('Forces:');
% disp(force);
% 
% disp('Moments:');
% disp(moment);

% compute derivative
sdot = vtolEOM_readonly(t, s, force, moment, params);  % sdot : [13 * 1]; att_des_save: [3 * 1];
% sdot = vtolEOM_readonly(t, s, DES_ctrl, moment, params);  % sdot : [13 * 1]; att_des_save: [3 * 1];



end

%% 参考
% % Example inputs
% ground_speed = [10, 0, -1, 0, 0, 0, 10, 0, -1,10, 0, -1,10];  % m/s (ground speed)
% wind_speed = [0, 0, 0];      % m/s (wind speed)
% delta = struct('elevon_l', 0.1, 'elevon_r', -0.1, 'throttle_a', 0.8, 'throttle_b', 0.8,'throttle_c', 0.8, 'arm_a', 0.2, 'arm_b', 0.2);
% 
% % Get aircraft parameters from sys_params
% params = sys_params();
% 
% % Call the function to calculate aerodynamic forces and moments
% [force, moment] = all_forces_moments(ground_speed, wind_speed, delta, params);