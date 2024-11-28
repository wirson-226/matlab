function [desired_state] = traj_helix_adjust(t, ~)

% Default parameters
r = 5;       % Radius of the helix
z_max = 2.5; % Maximum z-coordinate

% Normalize time
T = 12;       % Total time
t_max = T;    % Maximum time
t = max(0, min(t, t_max));
t_normalized = t / t_max;

if t_normalized >= 1
    % Hover controller input when time t is greater than or equal to T
    beta    = 2*pi;
    betad   = 0;
    betadd  = 0;
    z       = z_max;
    zd      = 0;
    zdd     = 0;
else
    % Position controller input using quintic polynomial trajectory
    M       = [1    0    0     0      0      0;
               0    1    0     0      0      0;
               0    0    2     0      0      0;
               1    1    1     1      1      1;
               0    1    2     3      4      5;
               0    0    2     6     12     20];
    b       = [0    0;
               0    0;
               0    0;
               2*pi z_max;
               0    0;
               0    0];
    a       = M\b;
    
    % Compute position, velocity, and acceleration using the quintic polynomial
    out     = a(1,:) + a(2,:)*t_normalized + a(3,:)*t_normalized^2 + ...
              a(4,:)*t_normalized^3 + a(5,:)*t_normalized^4 + a(6,:)*t_normalized^5;
    outd    = a(2,:) + 2*a(3,:)*t_normalized + 3*a(4,:)*t_normalized^2 + ...
              4*a(5,:)*t_normalized^3 + 5*a(6,:)*t_normalized^4;
    outdd   = 2*a(3,:) + 6*a(4,:)*t_normalized + 12*a(5,:)*t_normalized^2 + 20*a(6,:)*t_normalized^3;
    
    beta    = out(1,1);  % Angle beta
    betad   = outd(1,1); % Angular velocity
    betadd  = outdd(1,1);% Angular acceleration
    z       = out(1,2);  % Z-coordinate
    zd      = outd(1,2); % Z-velocity
    zdd     = outdd(1,2);% Z-acceleration
end

% Compute position, velocity, and acceleration in Cartesian coordinates
x       = cos(beta)*r;
y       = sin(beta)*r;
pos     = [x; y; z];

xd      = -y*betad;   % X-velocity
yd      =  x*betad;   % Y-velocity
vel     = [xd; yd; zd];

xdd     = -x*betad^2 - y*betadd;   % X-acceleration
ydd     = -y*betad^2 + x*betadd;   % Y-acceleration
acc     = [xdd; ydd; zdd];

% Yaw and yaw rate (not varying in this helical trajectory)
yaw = 0;
yawdot = 0;

% Output desired state struct
desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
