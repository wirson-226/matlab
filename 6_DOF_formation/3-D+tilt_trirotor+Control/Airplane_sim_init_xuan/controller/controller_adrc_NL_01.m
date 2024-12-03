function [F, M] = controller_adrc_NL_01(t, state, des_state, params)
    % CONTROLLER  ADRC Controller for the quadrotor
    %
    %   state: The current state of the robot with the following fields:
    %   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
    %   state.rot = [phi; theta; psi], state.omega = [p; q; r]
    %
    %   des_state: The desired states are:
    %   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
    %   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
    %   des_state.yawdot
    %
    %   params: robot parameters

    % Controller gains
    Kp_pos = [10; 10; 15];  % Position proportional gain
    Kd_pos = [6; 6; 9];  % Position derivative gain

    % ADRC parameters
    b0 = params.b0;

    persistent z_pos;
    persistent z_att;

    if isempty(z_pos)
        z_pos = zeros(3, 3);  % [z_x; z_y; z_z]
        z_att = zeros(3, 3);  % [z_phi; z_theta; z_psi]
    end

    % Position control
    pos_err = des_state.pos - state.pos;
    vel_err = des_state.vel - state.vel;

    acc_des = des_state.acc + Kp_pos .* pos_err + Kd_pos .* vel_err;

    % Desired accelerations
    r1_des_ddot = acc_des(1);
    r2_des_ddot = acc_des(2);
    r3_des_ddot = acc_des(3);

    % Desired thrust
    u1 = params.mass * (params.gravity + r3_des_ddot);

    % Update ESO for position
    z_pos(:,1) = ESO_NL(z_pos(:,1), state.pos(1), u1, params);
    z_pos(:,2) = ESO_NL(z_pos(:,2), state.pos(2), u1, params);
    z_pos(:,3) = ESO_NL(z_pos(:,3), state.pos(3), u1, params);

    % Disturbance compensation
    f_pos = z_pos(3,:);

    % Desired roll and pitch angles
    phi_des = (r1_des_ddot * sin(des_state.yaw) - r2_des_ddot * cos(des_state.yaw)) / params.gravity;
    theta_des = (r1_des_ddot * cos(des_state.yaw) + r2_des_ddot * sin(des_state.yaw)) / params.gravity;
    psi_des = des_state.yaw;

    % Desired angular velocities (assumed to be zero for hover)
    p_des = 0;
    q_des = 0;
    r_des = des_state.yawdot;

    % Current state
    phi = state.rot(1);
    theta = state.rot(2);
    psi = state.rot(3);
    p = state.omega(1);
    q = state.omega(2);
    r = state.omega(3);

    % Update ESO for attitude
    z_att(:,1) = ESO_NL(z_att(:,1), phi, 0, params);
    z_att(:,2) = ESO_NL(z_att(:,2), theta, 0, params);
    z_att(:,3) = ESO_NL(z_att(:,3), psi, 0, params);

    % Disturbance compensation for attitude
    f_phi = z_att(3,1);
    f_theta = z_att(3,2);
    f_psi = z_att(3,3);

    % Attitude control using PD control laws with disturbance compensation
    Kp_att = [200; 200; 200]; % Attitude proportional gain
    Kd_att = [50; 50; 50];    % Attitude derivative gain

    u2 = [Kp_att(1) * (phi_des - phi) + Kd_att(1) * (p_des - p) - f_phi;
          Kp_att(2) * (theta_des - theta) + Kd_att(2) * (q_des - q) - f_theta;
          Kp_att(3) * (psi_des - psi) + Kd_att(3) * (r_des - r) - f_psi];

    % Output force and moments
    F = u1 - f_pos(3);
    M = u2;

end
