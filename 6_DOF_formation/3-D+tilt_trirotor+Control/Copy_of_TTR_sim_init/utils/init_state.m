function [ s ] = init_state( start, yaw )
%INIT_STATE Initialize 13 x 1 state vector
params = sys_params;

s     = zeros(13,1);
phi0   = params.phi0;
theta0 = params.theta0;
psi0   = yaw;
Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
Quat0  = RotToQuat(Rot0);
s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = params.u0;        %xdot
s(5)  = params.v0;        %ydot
s(6)  = params.w0;        %zdot
s(7)  = Quat0(1); %qw
s(8)  = Quat0(2); %qx
s(9)  = Quat0(3); %qy
s(10) = Quat0(4); %qz
s(11) = params.p0 ;       %p
s(12) = params.q0;        %q
s(13) = params.r0;        %r

end
% 
% params.north0 = 0.0;  % initial north position
% params.east0 = 0.0;   % initial east position
% params.up = 0.0;  % initial down position
% params.u0 = 0.0;  % initial velocity along body x-axis
% params.v0 = 10.0;  % initial velocity along body y-axis -- todo--测试cruise
% params.w0 = 0.0;  % initial velocity along body z-axis
% 
% params.phi0 = 0.0;  % initial roll angle
% params.theta0 = 0.0;  % initial pitch angle
% params.psi0 = 0.0;  % initial yaw angle
% params.p0 = 0.0;  % initial roll rate
% params.q0 = 0.0;  % initial pitch rate
% params.r0 = 0.0;  % initial yaw rate