function [desired_state] = traj_helix(t, state)
% TRAJ_HELIX generates a helix trajectory where the nose points along the trajectory direction.
% 添加倾转角度

if nargin < 4, z_max = 2.5; end
if nargin < 3, r = 5; end
T       = 12;       % finishing time 快速 大角度
% T       = 12*3;       % finishing time 慢速 小角度

T_hover = T/5; % 悬停状态

if t >= T
    % hover at the final position
    x       = cos(2*pi)*r;
    y       = sin(2*pi)*r;
    z       = z_max;
    pos     = [x; y; z];
    vel     = zeros(3,1);
    acc     = zeros(3,1);
    yaw     = atan2(vel(2), vel(1));
    yawdot  = 0;
    
else
    % trajectory generation using quintic polynomial
    t0      = 0;
    tf      = T;
    M       = ...
       [1    t0  t0^2    t0^3    t0^4    t0^5;
        0    1   2*t0    3*t0^2  4*t0^3  5*t0^4;
        0    0   2       6*t0    12*t0^2 20*t0^3;
        1    tf  tf^2    tf^3    tf^4    tf^5;
        0    1   2*tf    3*tf^2  4*tf^3  5*tf^4;
        0    0   2       6*tf    12*tf^2 20*tf^3];
    b       = ...
       [0    0;
        0    0;
        0    0;
        2*pi z_max;
        0    0;
        0    0];
    a       = M\b;
    out     = a(1,:) + a(2,:)*t + a(3,:)*t^2 + a(4,:)*t^3 + a(5,:)*t^4 + a(6,:)*t^5;
    outd    = a(2,:) + 2*a(3,:)*t + 3*a(4,:)*t^2 + 4*a(5,:)*t^3 + 5*a(6,:)*t^4;
    outdd   = 2*a(3,:) + 6*a(4,:)*t + 12*a(5,:)*t^2 + 20*a(6,:)*t^3;
    beta    = out(1,1);
    betad   = outd(1,1);
    betadd  = outdd(1,1);
    z       = out(1,2);
    zd      = outd(1,2);
    zdd     = outdd(1,2);
    % position
    x       = cos(beta)*r;
    y       = sin(beta)*r;
    pos     = [x; y; z];
    % velocity
    xd      = -r*sin(beta)*betad;
    yd      =  r*cos(beta)*betad;
    vel     = [xd; yd; zd];
    % acceleration
    xdd     = -r*cos(beta)*betad^2 - r*sin(beta)*betadd;
    ydd     = -r*sin(beta)*betad^2 + r*cos(beta)*betadd;
    acc     = [xdd; ydd; zdd];
    % yaw and yawdot
    yaw = atan2(vel(2), vel(1));
    speed_sq = vel(1)^2 + vel(2)^2;
    if speed_sq > 1e-6
        yawdot = (vel(1)*acc(2) - vel(2)*acc(1)) / speed_sq;
    else
        yawdot = 0;
    end
end

% output desired state
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;
desired_state.mode = 1;  % Mode copter
end
