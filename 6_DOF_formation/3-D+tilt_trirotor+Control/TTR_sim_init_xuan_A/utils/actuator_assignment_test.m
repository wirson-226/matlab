    % 测试 inverse_actuator_assignment 函数
    params = sys_params();
    state = [0, 0, -20, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0];  % 

    
    % 输入的力和力矩（模拟值）除了重力
    % Mz_aero max     0.0116; My_aero max    0.4455  min 0.0003;Mx_aero  max   0.0007


    %% 悬停      --- mode 1
    % force = [0; 0; 1.7658];   % [fx, fy, fz] in Newtons (前向, 侧向, 垂直)
    % moment = [0; 0; 0];     % [Mx, My, Mz] in Newton-meters (滚转, 俯仰, 偏航)
    
    %% 固定翼巡航 --- mode 2
    % force = [1.5; 0; 1.7658];   % [fx, fy, fz] in Newtons (前向, 侧向, 垂直)
    % moment = [0; 0; 0];     % [Mx, My, Mz] in Newton-meters (滚转, 俯仰, 偏航)   

    %% 过渡 加速平飞  mode 3
    force = [1.5; 0; 1.7658];   % [fx, fy, fz] in Newtons (前向, 侧向, 垂直)
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
    [force_test, moment_test] = all_forces_moments(state, actuator, params);
    % Display the results
    disp('Forces:');
    disp(force_test);
    
    disp('Moments:');
    disp(moment_test);



    % % % 创建一个图形窗口并设置名称
    % fig = figure('name', 'assignment calculation reference', 'NumberTitle', 'off');
    % % figure;
    % % 在窗口中显示图像
    % imshow('E:\documents\Codes\codes\matlab\6_DOF_formation\3-D+tilt_trirotor+Control\TTR_sim_init_xuan_A\Medias\assignment_new.png'); % Load the image (ensure the file path is correct)
    % title('神的笔记');