% Parameters
m = 1.5;    % Mass of the drone (kg)
g = 9.81;   % Gravitational acceleration (m/s^2)
I = diag([0.05, 0.05, 0.1]);  % Inertia matrix

% PID gains for position control (X, Y, Z)
Kp_pos = [5, 5, 10];
Ki_pos = [0.1, 0.1, 0.1];
Kd_pos = [3, 3, 5];

% PID gains for attitude control (Roll, Pitch, Yaw)
Kp_att = [8, 8, 6];
Ki_att = [0.2, 0.2, 0.1];
Kd_att = [4, 4, 3];

% Initial conditions
r = [0; 0; 0];     % Position (X, Y, Z)
v = [0; 0; 0];     % Velocity (X, Y, Z)
att = [0; 0; 0];   % Attitude (Roll, Pitch, Yaw)
omega = [0; 0; 0]; % Angular velocity

% Desired trajectory (for demonstration, a simple circular trajectory)
t = 0:0.01:10;     % Time vector
radius = 5;
r_des = [radius * cos(0.1 * t); radius * sin(0.1 * t); 1 + 0.1 * sin(0.05 * t)];  % Desired position

% PID control loop
dt = 0.01;  % Time step
for i = 1:length(t)
    % Error in position and velocity
    e_pos = r_des(:, i) - r;
    e_vel = [0; 0; 0] - v;  % Assuming desired velocity is zero for this example
    
    % Position control (PID)
    u_pos = Kp_pos .* e_pos + Kd_pos .* e_vel;
    
    % Convert position control output to desired angles
    phi_des = (u_pos(2) / g);   % Desired roll
    theta_des = -(u_pos(1) / g);  % Desired pitch
    psi_des = 0;  % Assuming we want to maintain no yaw rotation for simplicity

    % Error in attitude
    e_att = [phi_des; theta_des; psi_des] - att;
    
    % Attitude control (PID)
    u_att = Kp_att .* e_att + Kd_att .* (-omega);
    
    % Update position, velocity, and attitude
    F = [0; 0; m * g] + u_pos;  % Total force
    r_dot = v;
    v_dot = F / m;
    
    % Torque (for roll, pitch, yaw control)
    tau = u_att;
    
    % Update angular velocity and attitude
    omega_dot = inv(I) * (tau - cross(omega, I * omega));
    att_dot = omega;
    
    % Update states using simple Euler integration
    r = r + r_dot * dt;
    v = v + v_dot * dt;
    att = att + att_dot * dt;
    omega = omega + omega_dot * dt;
    
    % Visualization (for simplicity, we'll just plot the position)
    plot3(r(1), r(2), r(3), 'ro');
    hold on;
    plot3(r_des(1, :), r_des(2, :), r_des(3, :), 'b--');
    axis([-6 6 -6 6 0 3]);
    drawnow;
end
