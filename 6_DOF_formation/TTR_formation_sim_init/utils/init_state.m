function [ s ] = init_state( )
%INIT_STATE Initialize 13 x 1 state vector
params = sys_params;

s     = zeros(13,1);
phi0   = params.phi0;
theta0 = params.theta0;
psi0   = params.psi0;
Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
Quat0  = RotToQuat(Rot0);
s(1)  = params.east0; %x
s(2)  = params.north0; %y
s(3)  = params.up; %z
s(4)  = params.u0;        %xdot
s(5)  = params.v0;        %ydot
s(6)  = params.w0;        %zdot
s(7)  = Quat0(1); %qw
s(8)  = Quat0(2); %qx
s(9)  = Quat0(3); %qy
s(10) = Quat0(4); %qz
s(11) = params.p0;       %p
s(12) = params.q0;        %q
s(13) = params.r0;        %r

end
