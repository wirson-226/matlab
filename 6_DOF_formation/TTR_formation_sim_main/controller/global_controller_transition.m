function [des_from_ctrl, command, copter_cmd, transition_state] = global_controller_transition(t, state, des_state, params, transition_state)
    % 定义模式常量
    persistent MODE
    if isempty(MODE)
        MODE.HOVER = 1;
        MODE.TRANSITION_PREPARE = 2;
        MODE.TRANSITION_EXECUTE = 3;
        MODE.CRUISE = 4;
    end
    
    % 初始化过渡状态
    if nargin < 5 || isempty(transition_state)
        transition_state = struct(...
            'current_mode', MODE.HOVER, ...
            'mode_start_time', t, ...
            'transition_progress', 0, ...
            'prev_velocity', zeros(3,1), ...
            'target_velocity', zeros(3,1), ...
            'transition_type', 'hover_to_cruise');
    end
    
    % 计算真实空速
    u_r = state.vel(1) - params.w_es;
    v_r = state.vel(2) - params.w_ns;
    w_r = state.vel(3) - params.w_us;
    Va = sqrt(u_r^2 + v_r^2 + w_r^2);
    
    % 创建控制器实例
    controller = AircraftControl(params);
    
    % 模式切换决策
    [next_mode, transition_state] = determine_mode_transition(...
        transition_state, state, des_state, params, Va);
    
    % 根据模式选择控制策略
    switch next_mode
        case MODE.HOVER
            [des_from_ctrl, command, copter_cmd] = hover_mode_control(...
                state, des_state, params, controller);
            
        case MODE.TRANSITION_PREPARE
            [des_from_ctrl, command, copter_cmd, transition_state] = transition_prepare_control(...
                t, state, des_state, params, controller, transition_state);
            
        case MODE.TRANSITION_EXECUTE
            [des_from_ctrl, command, copter_cmd, transition_state] = transition_execute_control(...
                t, state, des_state, params, controller, transition_state);
            
        case MODE.CRUISE
            [des_from_ctrl, command, copter_cmd] = cruise_mode_control(...
                state, des_state, params, controller);
    end
    
    % 更新当前模式
    transition_state.current_mode = next_mode;
end

function [next_mode, transition_state] = determine_mode_transition(t,transition_state, state, des_state, params, Va)
    MODE = struct(...
        'HOVER', 1, ...
        'TRANSITION_PREPARE', 2, ...
        'TRANSITION_EXECUTE', 3, ...
        'CRUISE', 4);
    
    current_mode = transition_state.current_mode;
    
    switch current_mode
        case MODE.HOVER
            % 判断是否开始过渡准备（悬停到巡航）
            if is_transition_ready(state, des_state, params)
                % 根据距离决定是否需要过渡模式
                if norm(des_state.pos - state.pos) > params.transition_distance_threshold
                    next_mode = MODE.TRANSITION_PREPARE;
                    transition_state.transition_type = 'hover_to_cruise';
                else
                    % 短距离直接保持悬停模式
                    next_mode = MODE.HOVER;
                end
                transition_state.mode_start_time = t;
            else
                next_mode = MODE.HOVER;
            end
            
        case MODE.CRUISE
            % 判断是否需要返回悬停
            if is_return_to_hover_required(t,state, des_state, params)
                next_mode = MODE.TRANSITION_PREPARE;
                transition_state.mode_start_time = t;
                transition_state.transition_type = 'cruise_to_hover';
            else
                next_mode = MODE.CRUISE;
            end
            
        case MODE.TRANSITION_PREPARE
            % 判断是否进入主过渡
            if is_transition_prepare_complete(t,transition_state, state, des_state)
                next_mode = MODE.TRANSITION_EXECUTE;
                transition_state.mode_start_time = t;
            else
                next_mode = MODE.TRANSITION_PREPARE;
            end
            
        case MODE.TRANSITION_EXECUTE
            % 判断过渡是否完成
            switch transition_state.transition_type
                case 'hover_to_cruise'
                    if Va >= des_state.Va * 0.95  % 允许5%的速度误差
                        next_mode = MODE.CRUISE;
                        transition_state.mode_start_time = t;
                    else
                        next_mode = MODE.TRANSITION_EXECUTE;
                    end
                    
                case 'cruise_to_hover'
                    % 检查是否已经降低到悬停速度并接近目标位置
                    if Va <= params.hover_velocity_threshold && ...
                       norm(state.pos - des_state.pos) <= params.hover_position_tolerance
                        next_mode = MODE.HOVER;
                        transition_state.mode_start_time = t;
                    else
                        next_mode = MODE.TRANSITION_EXECUTE;
                    end
            end
    end
    
    % 更新过渡进度
    transition_state.transition_progress = (t - transition_state.mode_start_time) / params.transition_duration;
end

function [des_from_ctrl, command, copter_cmd, transition_state] = transition_prepare_control(...
    t, state, des_state, params, controller, transition_state)
    
    % 计算平滑过渡的加速度和姿态
    progress = transition_state.transition_progress;
    
    % 根据过渡类型选择不同的过渡策略
    switch transition_state.transition_type
        case 'hover_to_cruise'
            acc_des = linear_interpolation(...
                [0, 0, 0], ... % 起始加速度
                [0, params.cruise_acceleration, 0], ...  % 目标加速度（前向）
                progress);
            
            % 计算渐进式姿态变化
            [roll_cmd, pitch_cmd] = smooth_attitude_transition(progress, 'forward');
            
        case 'cruise_to_hover'
            acc_des = linear_interpolation(...
                [0, params.cruise_acceleration, 0], ... % 起始加速度
                [0, 0, 0], ...  % 目标加速度（悬停）
                progress);
            
            % 计算渐进式姿态变化
            [roll_cmd, pitch_cmd] = smooth_attitude_transition(progress, 'hover');
    end
    
    % 计算期望总推力向量
    F_des = params.mass * [acc_des(1); acc_des(2); params.gravity + acc_des(3)];
    
    % 坐标变换
    force_cmd_body = WorldToBodyAccel(F_des, state.rot(1), state.rot(2), state.rot(3));
    
    % 力矩命令计算
    moment_cmd_body = calculate_moment_command(controller, roll_cmd, pitch_cmd, state);
    
    % 生成命令
    des_from_ctrl = prepare_transition_state(state, des_state, acc_des, roll_cmd, pitch_cmd);
    copter_cmd = [force_cmd_body; moment_cmd_body];
    command = actuator_assignment(force_cmd_body, moment_cmd_body, state, params, 2);
    
    % 存储目标速度和加速度
    transition_state.target_velocity = des_state.Va * [cos(des_state.yaw), sin(des_state.yaw), 0];
    transition_state.prev_acceleration = acc_des;
end

function [des_from_ctrl, command, copter_cmd, transition_state] = transition_execute_control(...
    t, state, des_state, params, controller, transition_state)
    
    % 根据过渡类型选择不同的执行策略
    switch transition_state.transition_type
        case 'hover_to_cruise'
            % 渐进增加前向速度
            current_velocity = interpolate_velocity(...
                [0, 0, 0], ...
                transition_state.target_velocity, ...
                transition_state.transition_progress);
            
            % 速度控制
            acc_des = calculate_velocity_error_acceleration(current_velocity, state.vel, params);
            
            % 姿态控制
            [roll_cmd, pitch_cmd] = calculate_transition_attitude(current_velocity, state);
            
        case 'cruise_to_hover'
            % 渐进减小速度，接近目标位置
            current_velocity = interpolate_velocity(...
                transition_state.target_velocity, ...
                [0, 0, 0], ...
                transition_state.transition_progress);
            
            % 位置和速度控制混合
            pos_error = des_state.pos - state.pos;
            vel_error = current_velocity - state.vel;
            
            % 混合加速度控制
            acc_des = saturate_vector(...
                pos_error * params.position_gain + ...
                vel_error * params.velocity_gain, ...
                params.max_acceleration);
            
            % 姿态控制（逐渐回到水平）
            [roll_cmd, pitch_cmd] = calculate_hover_transition_attitude(current_velocity, state);
    end
    
    % 计算期望总推力向量
    F_des = params.mass * [acc_des(1); acc_des(2); params.gravity + acc_des(3)];
    
    % 坐标变换
    force_cmd_body = WorldToBodyAccel(F_des, state.rot(1), state.rot(2), state.rot(3));
    
    % 力矩命令计算
    moment_cmd_body = calculate_moment_command(controller, roll_cmd, pitch_cmd, state);
    
    % 生成命令
    des_from_ctrl = prepare_transition_state(state, des_state, acc_des, roll_cmd, pitch_cmd);
    copter_cmd = [force_cmd_body; moment_cmd_body];
    command = actuator_assignment(force_cmd_body, moment_cmd_body, state, params, 2);
end

% 新增辅助函数：计算从巡航到悬停的姿态过渡
function [roll_cmd, pitch_cmd] = calculate_hover_transition_attitude(t,velocity, state)
    % 逐渐回到水平姿态
    speed = norm(velocity);
    
    % 横滚角：逐渐归零
    roll_cmd = -state.rot(1) * (1 - speed / max(speed, 0.1));
    
    % 俯仰角：逐渐平整
    pitch_cmd = -state.rot(2) * (1 - speed / max(speed, 0.1));
    
    % 限幅
    roll_cmd = saturate(roll_cmd, deg2rad(-30), deg2rad(30));
    pitch_cmd = saturate(pitch_cmd, deg2rad(-30), deg2rad(30));
end

% 其他辅助函数保持不变

% 悬停模式控制
function [des_from_ctrl, command, copter_cmd] = hover_mode_control(t,state, des_state, params, controller)
    % 原悬停模式控制逻辑
    % 此处保留原代码实现
    %% Position control -- velocity_des from position error using PIDControl(class)世界坐标系下
    % PID gains from params ..... also the limits of every virables
    % pos_err = des_state.pos - state.pos; % 123-xyz-enu
    ve_cmd = controller.ve_from_pe.update(des_state.pos(1), state.pos(1)); % e
    vn_cmd = controller.vn_from_pn.update(des_state.pos(2), state.pos(2)); % n
    vu_cmd = controller.vu_from_pu.update(des_state.pos(3), state.pos(3)); % u
    
    
    %% velocity control -- attitudes cmd with acceleration_des from velocity error using PIDControl(class) 世界坐标系下
    
    % 123 xyz enu
    % 测试用
    % acc_des = [0,1,0]; % -- todo 修改坐标轴右手定则 东北天 enu -- done--todo --无执行器Done--有执行器--
    % test--done
    
    acc_des(1) = controller.acc_e_from_ve.update(ve_cmd, state.vel(1)); 
    acc_des(2) = controller.acc_n_from_vn.update(vn_cmd, state.vel(2));
    acc_des(3) = controller.acc_u_from_vu.update(vu_cmd, state.vel(3));
    
    
    
    %% 非线性 加速度到角度，解算
    % psi_cmd = des_state.yaw;
    psi_cmd = deg2rad(0);
    psi_cmd = wrap(psi_cmd, pi);  % 偏航角，[-pi, pi]
    
    % -- 完整非线性
    % 期望总推力向量（ENU坐标）
    F_des = params.mass * [acc_des(1); acc_des(2); params.gravity + acc_des(3)];
    
    % 期望机体Z轴方向 (Up)
    zb_des = F_des / norm(F_des);
    
    % 期望机体X轴水平投影方向
    xc = [sin(psi_cmd); cos(psi_cmd); 0];
    
    % 期望机体Y轴方向
    yb_des = cross(zb_des, xc);
    yb_des = yb_des / norm(yb_des);
    
    % 重新计算期望机体X轴方向，确保正交
    xb_des = cross(yb_des, zb_des);
    
    % 构造期望旋转矩阵
    R_des = [xb_des, yb_des, zb_des];
    
    % 解算欧拉角（ZXY顺序）
    roll_cmd = atan2(R_des(3,2), R_des(3,3));
    pitch_cmd = asin(R_des(3,1));
    
     % 姿态环测试用
    % phi_cmd= deg2rad(25);
    % theta_cmd= deg2rad(25);
    roll_cmd = wrap(roll_cmd, pi/3);      % 滚转角，[-pi/3., pi/3.]
    pitch_cmd = wrap(pitch_cmd, pi/3);  % 俯仰角，[-pi/3., pi/3.]
    
    % 避免奇异后限制范围
    roll_cmd = saturate(roll_cmd, -params.roll_input_limit, params.roll_input_limit);
    pitch_cmd = saturate(pitch_cmd, -params.pitch_input_limit, params.pitch_input_limit);
    
    % 期望合力转到机体坐标系 ENU -- RFU -- xyz--todo--Done
    % 测试
    force_cmd_body =  WorldToBodyAccel(F_des, state.rot(1),state.rot(2),state.rot(3)); % 高度控制映射机体坐标系
    
    
    %% attitude control -- p_cmd, q_cmd, r_cmd from accitudes error using PIDControl(class) 机体坐标系下
    
    % phi_cmd= deg2rad(15);
    % theta_cmd= deg2rad(15);
    % psi_cmd= deg2rad(0);
    p_cmd = controller.roll_rate_from_roll.update(roll_cmd, state.rot(1));
    q_cmd = controller.pitch_rate_from_pitch.update(pitch_cmd, state.rot(2));
    r_cmd = controller.yaw_rate_from_yaw.update(psi_cmd, state.rot(3));
    
    %% omega control-- xyz rfu 轴力矩 Mx_cmd, My_cmd, Mz_cmd from omega error using PIDControl(class) 机体坐标系下
    % todo -- 检查加速度方向分解对应-Done
    My_cmd = controller.My_from_roll_rate.update(p_cmd, state.omega(1)); %   with p_err    正滚转产生右正加速度（正反馈），所以加-
    Mx_cmd = controller.Mx_from_pitch_rate.update(q_cmd, state.omega(2)); %  with q_err    正俯仰产生前负加速度(负反馈)，
    Mz_cmd = controller.Mz_from_yaw_rate.update(r_cmd, state.omega(3)); %    with r_err    正偏航产生正力矩 没有加速度解算，合理
    
    moment_cmd_body = [My_cmd; Mx_cmd; Mz_cmd]; % roll pitch yaw
    
    % moment_cmd_body = [0; 0; Mz_cmd];  % 得到控制输出但是阻断 为0的是仅显示 不接动力学
    
    %% 期望状态输出 1 * 12 --- vel-att-omege-M
    % 可选替换 - ACC--Moment
    % des_from_ctrl = [ve_cmd, vn_cmd, vu_cmd, phi_cmd, theta_cmd, psi_cmd, p_cmd, q_cmd, r_cmd, M_y, M_x, M_z];
    des_from_ctrl = [ve_cmd, vn_cmd, vu_cmd, roll_cmd, pitch_cmd, psi_cmd, p_cmd, q_cmd, r_cmd, acc_des(1), acc_des(2), acc_des(3)];
    copter_cmd = [force_cmd_body; moment_cmd_body]; % 力与力矩期望
    
    % %% 执行器命令结算 -- 控制分配  - 机体坐标系 phi theta psi YXZ 前右上 roll pitch yaw 储存顺序
    command = actuator_assignment(force_cmd_body, moment_cmd_body, state, params, mode);
end

% 巡航模式控制
function [des_from_ctrl, command, copter_cmd] = cruise_mode_control(t,state, des_state, params, controller)
    % 原巡航模式控制逻辑
    % 此处保留原代码实现
     % 期望航向角计算（使用更稳定的导航算法）
    dx = des_state.pos(1) - state.pos(1);
    dy = des_state.pos(2) - state.pos(2);
    
    % 使用 atan2 计算期望航向角，并处理特殊情况
    yaw_cmd = wrapToPi(atan2(dy, dx));
    yaw_cmd = deg2rad(0); % 测试偏航 稳定直线
    % 路径跟踪增强
    % 引入交叉误差和航向误差
    path_distance = 0; % sqrt(dx^2 + dy^2);
    cross_track_error = -sin(yaw_cmd - state.rot(3)) * path_distance;
    % heading_error = wrapToPi(yaw_cmd - state.rot(3));
    
    % 速度控制（增加饱和和平滑）
    throttle = saturate(controller.throttle_from_airspeed.update(des_state.Va+5, Va), 0, 2);
    ta = throttle/2;
    tb = ta;
    tc = 0;
    
    % 高度控制（增加前馈和微分项）
    % altitude_error = des_state.pos(3) - state.pos(3);
    altitude_error = 15 - state.pos(3); % 测试俯仰 定高15
    pitch_cmd = controller.pitch_from_altitude.update(des_state.pos(3), state.pos(3)) + ...
        0.1 * altitude_error + ... % 比例项
        0.05 * (altitude_error - prev_altitude_error) / t; % 微分项
    
    % 横滚角控制（引入交叉误差）
    k_track_error = 0.5; % 直线平飞测试 取消跟踪 
    roll_cmd = controller.roll_from_course.update(yaw_cmd, state.rot(3)) + ...
        k_track_error * cross_track_error; % 交叉误差补偿
    
    % 限制横滚角在合理范围
    roll_cmd = saturate(roll_cmd, deg2rad(-30), deg2rad(30));
    
    % 升降舵和副翼控制
    elevator_cmd = controller.elevator_from_pitch.update(pitch_cmd, state.rot(2), state.omega(2));
    aileron_cmd = controller.aileron_from_roll.update(roll_cmd, state.rot(1), state.omega(1));
    
    % 升降舵和副翼混合控制
    elevon_r = -(elevator_cmd + aileron_cmd);
    elevon_l = -(elevator_cmd - aileron_cmd);
    
    % 控制面限幅
    elevon_r = saturate(elevon_r, deg2rad(-45), deg2rad(45));
    elevon_l = saturate(elevon_l, deg2rad(-45), deg2rad(45));
    
    % 倾转角度
    arm_a = deg2rad(90);
    arm_b = arm_a;
    
    roll_cmd = wrap(roll_cmd, pi/3);      % 滚转角，[-pi/3., pi/3.]
    pitch_cmd = wrap(pitch_cmd, pi/3);  % 俯仰角，[-pi/3., pi/3.]
    
    % 避免奇异后限制范围
    roll_cmd = saturate(roll_cmd, -params.roll_input_limit, params.roll_input_limit);
    pitch_cmd = saturate(pitch_cmd, -params.pitch_input_limit, params.pitch_input_limit);
    % 命令输出
    command.throttle = [ta, tb, tc];
    command.elevon = [elevon_r, elevon_l];
    command.arm = [arm_a, arm_b];
    
    % 输出期望状态 (1 * 12)

    s = [state.pos(1),state.pos(2),state.pos(3),state.vel(1),state.vel(2),state.vel(3),0,0,0,0,state.omega(1),state.omega(2),state.omega(3),];
    [force, moment] = all_forces_moments(s, command, params);
    copter_cmd = [force; moment]; % 
    des_from_ctrl = [0, Va, 0, roll_cmd, pitch_cmd, yaw_cmd, moment(1), moment(2), moment(3), 0, 0, 0];

end



% 辅助函数
function acc = linear_interpolation(start_acc, end_acc, progress)
    acc = start_acc + progress * (end_acc - start_acc);
end

function [roll, pitch] = smooth_attitude_transition(progress)
    % S曲线插值
    roll = sin(pi/2 * progress);
    pitch = sin(pi/2 * progress);
end

function moment_cmd = calculate_moment_command(controller, roll_cmd, pitch_cmd, state)
    % 计算力矩命令
    p_cmd = controller.roll_rate_from_roll.update(roll_cmd, state.rot(1));
    q_cmd = controller.pitch_rate_from_pitch.update(pitch_cmd, state.rot(2));
    r_cmd = 0;  % 固定偏航
    
    My_cmd = controller.My_from_roll_rate.update(p_cmd, state.omega(1));
    Mx_cmd = controller.Mx_from_pitch_rate.update(q_cmd, state.omega(2));
    Mz_cmd = 0;
    
    moment_cmd = [My_cmd; Mx_cmd; Mz_cmd];
end

% 准备过渡状态
function des_from_ctrl = prepare_transition_state(state, des_state, acc_des)
    % 生成过渡状态的期望控制量
    des_from_ctrl = [
        state.vel(1), state.vel(2), state.vel(3), ... % 速度
        0, 0, 0, ... % 姿态角
        0, 0, 0, ... % 角速率
        acc_des(1), acc_des(2), acc_des(3) % 加速度
    ];
end

% 切换条件判断函数
function ready = is_transition_ready(state, des_state, params)
    % 综合判断是否准备好开始过渡
    velocity_condition = norm(state.vel) < params.max_hover_velocity;
    altitude_condition = abs(state.pos(3) - des_state.pos(3)) < params.altitude_tolerance;
    
    ready = velocity_condition && altitude_condition;
end

function complete = is_transition_prepare_complete(transition_state, state, des_state)
    % 判断过渡准备是否完成
    progress_condition = transition_state.transition_progress >= 1;
    
    complete = progress_condition;
end

function return_required = is_return_to_hover_required(state, des_state, params)
    % 判断是否需要返回悬停
    position_deviation = norm(state.pos - des_state.pos) > params.max_cruise_deviation;
    velocity_low = norm(state.vel) < params.min_cruise_velocity;
    
    return_required = position_deviation || velocity_low;
end
