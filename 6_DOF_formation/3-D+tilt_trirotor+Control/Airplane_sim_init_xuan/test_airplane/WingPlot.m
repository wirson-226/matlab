function WingPlot(position, attitude, wingspan, length, tail_span, engine_pos)
    % position: [x, y, z] - 飞行器的位置
    % orientation: [roll, pitch, yaw] - 飞行器的姿态角度（单位：弧度）
    % wingspan: 翼展（米）
    % length: 飞行器的长度（米）
    % tail_span: 尾翼的跨度（米）
    % engine_pos: 发动机位置，表示发动机在机身上的相对位置 [x, y, z]

    % 创建一个新的图形窗口
    figure;
    hold on;
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');

    % 设置背景颜色
    set(gca, 'Color', [1, 1, 1]); % 设置背景为浅灰色

    % 旋转矩阵计算：先计算旋转矩阵，再将所有部件应用相同的旋转
    R = eul2rotm(attitude(1,:)); % 姿态旋转矩阵

        % 生成飞行器的机身（一个简单的长方体）
    fuselage_length = length * 0.6;  % 假设机身长度为飞行器总长的 60%
    fuselage_width = 1;               % 假设机身宽度为 1 米
    fuselage_height = 0.5;            % 假设机身高度为 0.5 米

    fuselage_vertices = [
        -fuselage_length / 2, -fuselage_width / 2, -fuselage_height / 2;  % 后左下
        fuselage_length / 2, -fuselage_width / 2, -fuselage_height / 2;   % 后右下
        fuselage_length / 2, fuselage_width / 2, -fuselage_height / 2;    % 后右上
        -fuselage_length / 2, fuselage_width / 2, -fuselage_height / 2;   % 后左上
        -fuselage_length / 2, -fuselage_width / 2, fuselage_height / 2;   % 前左下
        fuselage_length / 2, -fuselage_width / 2, fuselage_height / 2;    % 前右下
        fuselage_length / 2, fuselage_width / 2, fuselage_height / 2;     % 前右上
        -fuselage_length / 2, fuselage_width / 2, fuselage_height / 2;    % 前左上
    ];

    % 旋转和位移机身
    fuselage_vertices_rot = (R * fuselage_vertices')';  % 应用旋转
    fuselage_vertices_rot(:,1) = fuselage_vertices_rot(:,1) + position(1);  % 位置偏移
    fuselage_vertices_rot(:,2) = fuselage_vertices_rot(:,2) + position(2);
    fuselage_vertices_rot(:,3) = fuselage_vertices_rot(:,3) + position(3);

    % 绘制机身
    faces = [
        1, 2, 6, 5;
        2, 3, 7, 6;
        3, 4, 8, 7;
        4, 1, 5, 8;
        1, 2, 3, 4;
        5, 6, 7, 8
    ];
    patch('Vertices', fuselage_vertices_rot, 'Faces', faces, ...
        'FaceColor', 'r', 'FaceAlpha', 0.6);
    % 生成飞行器的机翼（delta-wing）
    wing_vertices = [
        length/2, 0, 0;                   % 机翼的机头位置
        length, -wingspan / 2, 0; % 机翼的左翼后端
        length, wingspan / 2, 0;  % 机翼的右翼后端
    ];
%     wing_vertices = [
%     -fuselage_length/2, -wing_span/2, 0;   % 左翼根
%     -fuselage_length/2, wing_span/2, 0;    % 右翼根
%     fuselage_length/2, 0, 0;               % 机头
% ];

    % 旋转和位移机翼位置
    wing_vertices_rot = (R * wing_vertices')';  % 应用旋转
    wing_vertices_rot(:,1) = wing_vertices_rot(:,1) + position(1);  % 位置偏移
    wing_vertices_rot(:,2) = wing_vertices_rot(:,2) + position(2);
    wing_vertices_rot(:,3) = wing_vertices_rot(:,3) + position(3);

    % 绘制机翼
    patch(wing_vertices_rot(:,1), wing_vertices_rot(:,2), wing_vertices_rot(:,3), 'b', 'FaceAlpha', 0.6);

    % 生成尾翼（简单的矩形尾翼）
    tail_vertices = [
        -tail_span / 2, 0, 0;     % 尾翼的左端
        tail_span / 2, 0, 0;      % 尾翼的右端
        tail_span / 2, 0, length / 6;  % 尾翼的下端
        -tail_span / 2, 0, length / 6; % 尾翼的下端
    ];

    % 旋转和位移尾翼
    tail_vertices_rot = (R * tail_vertices')';  % 应用旋转
    tail_vertices_rot(:,1) = tail_vertices_rot(:,1) + position(1);  % 位置偏移
    tail_vertices_rot(:,2) = tail_vertices_rot(:,2) + position(2);
    tail_vertices_rot(:,3) = tail_vertices_rot(:,3) + position(3);

    % 绘制尾翼
    patch(tail_vertices_rot(:,1), tail_vertices_rot(:,2), tail_vertices_rot(:,3), 'g', 'FaceAlpha', 0.6);
    % 
    % % 绘制发动机位置（假设两个发动机位置）
    % engine1_pos = engine_pos + [1, 0, -length/4];  % 发动机 1 位置
    % engine2_pos = engine_pos + [-1, 0, -length/4]; % 发动机 2 位置
    % plot3(engine1_pos(1), engine1_pos(2), engine1_pos(3), 'ko', 'MarkerFaceColor', 'k');
    % plot3(engine2_pos(1), engine2_pos(2), engine2_pos(3), 'ko', 'MarkerFaceColor', 'k');

    % 设置视角和图形属性
    view(3);   % 3D视图
    axis([-50, 50, -50, 50, -50, 50]); % 设置坐标轴范围
    title('Delta Wing Aircraft Visualization');
end

