    % 测试 inverse_actuator_assignment 函数
    params = sys_params();
    % Original state vector
    state_vec = [0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0];
    
    % Initialize state as a struct
    state = struct();
    Fg = params.mass * params.gravity;
    % Convert to struct
    state.pos = state_vec(1:3);        % Position
    state.vel = state_vec(4:6);        % Velocity
    quat = [state_vec(7),state_vec(8),state_vec(9),state_vec(10)];
    state.rotmatrix = QuatToRot(quat);        % Rotation (Euler angles or quaternion)
    state.rot = RotToRPY_ZXY(state.rotmatrix);
    state.omega = state_vec(11:13);    % Angular velocity (omega)
    
    % Display the result
    disp(state);
 


    %% 悬停      --- mode 1
    % force = [0; 0; Fg];   % [fx, fy, fz] in Newtons (侧向, 前向, 垂直)
    % moment = [0; 0; 0];     % [My, Mx, Mz] in Newton-meters (滚转, 俯仰, 偏航)
    
    %% 固定翼巡航 --- mode 2
    force = [0; 1.5; Fg];   % [fx, fy, fz] in Newtons (侧向, 前向, 垂直)
    moment = [0; 0; 0];     % [My, Mx, Mz] in Newton-meters (滚转, 俯仰, 偏航)   

    %% 过渡 加速平飞  mode 3
    % force = [0; 1; Fg];   % [fx, fy, fz] in Newtons (侧向, 前向, 垂直)
    % moment = [0; 0; 0];     % [My, Mx, Mz] in Newton-meters (滚转, 俯仰, 偏航)


    % 调用 inverse_actuator_assignment 函数
    % 反解 struct
    actuator = actuator_assignment(force, moment, state, params); 
    
    % 打印执行器输出结果
    fprintf('Actuator Outputs:\n');
    fprintf('Arm Angles(ab-rl) (rad):\n');
    disp(rad2deg(actuator.arm));  % 显示倾转角
    fprintf('Throttle Inputs(abc-rlt):\n');
    disp(actuator.throttle);  % 显示油门输入
    fprintf('Elevon Deflections(rl):\n');
    disp(actuator.elevon);  % 显示副翼偏转角


    % 正解
    % 验证结束成功 加范围约束 1*13
    [force_test, moment_test] = all_forces_moments(state_vec, actuator, params);
    % Display the results
    disp('Forces(XYZ-RFU):');
    disp(force_test);
    
    disp('Moments(YXZ-Roll Pitch Yaw):');
    disp(moment_test);


  