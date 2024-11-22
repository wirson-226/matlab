function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% u = params.mass*(30.8*(s_des(1,1)-s(1,1))+6.8*(s_des(2,1)-s(2,1))+params.gravity);
u = params.mass*(35.8*(s_des(1,1)-s(1,1))+6.81*(s_des(2,1)-s(2,1))+params.gravity);


% FILL IN YOUR CODE HERE


end

