function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls
u1 = params.mass*(params.gravity+des_state.acc(2,1)+8*(des_state.vel(2,1)-state.vel(2,1))+35.8*(des_state.pos(2,1)-state.pos(2,1)));
phic=-1*(des_state.acc(1,1)+5.8*(des_state.vel(1,1)-state.vel(1,1))+35.8*(des_state.pos(1,1)-state.pos(1,1)))/params.gravity;
% phic_diff1=-1*(3.8*(des_state.vel(1,1)-state.vel(1,1))+10.8*(des_state.pos(1,1)-state.pos(1,1)))/params.gravity;
% phic_diff2=diff(phic_diff1);
% u2 = params.Ixx*(phic_diff2+3.8*(phic_diff1-state.omega)+10.8*(phic-state.rot));
u2=1*(0-state.omega)+100.8*(phic-state.rot);
% u2=0;
% params
% FILL IN YOUR CODE HERE

end

