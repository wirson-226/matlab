
    % 测试 inverse_actuator_assignment 函数
    params = sys_params();
    state = [0, 0, -20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % 

    
    % 输入的力和力矩（模拟值）除了重力
    force = [0.5; 0; 1.7658];   % [fx, fy, fz] in Newtons (前向, 侧向, 垂直)
    moment = [0; 0; 0];     % [Mx, My, Mz] in Newton-meters (滚转, 俯仰, 偏航)
    
    % 调用 inverse_actuator_assignment 函数
    % 反解
    actuator = actuator_assignment(force, moment, state, params);
    
    % 打印执行器输出结果
    fprintf('Actuator Outputs:\n');
    fprintf('Arm Angles (rad):\n');
    disp(rad2deg(actuator.arm));  % 显示倾转角
    fprintf('Throttle Inputs:\n');
    disp(actuator.throttle);  % 显示油门输入
    fprintf('Elevon Deflections:\n');
    disp(actuator.elevon);  % 显示副翼偏转角


    % 正解
    % 验证结束成功 加范围约束
    [force, moment] = all_forces_moments(state, actuator, params);
    % Display the results
    disp('Forces:');
    disp(force);
    
    disp('Moments:');
    disp(moment);
