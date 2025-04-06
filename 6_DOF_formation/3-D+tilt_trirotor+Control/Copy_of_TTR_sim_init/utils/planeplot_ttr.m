function planeplot_ttr(position, attitude,tilt_angle)
%  position(x,y,z),实时位置；
%  attitude（roll，pitch，yaw）rad 实时姿态；
%  tilt_angle(a,b), degrees 顺序为右，左，跟随机体旋转正方向倾转为正；尾部固定。为c；
%  tips shift + enter 同时修改变量名；ctrl + h 替换；ctrl + r 注释；f5 运行；
%  plot steps--定义全部点与尺寸--平移对齐转轴--倾转rotor a，b电机--整体旋转--整体平移---所有点依次连接上色
%  这段代码中的加速度朝向顺序是[x, y, z]，分别对应东、北、天方向（ENU坐标系） 机头朝向 Y
%  无人机模型的机头朝向是沿着机体坐标系的Y轴正方向，这符合"右前上"(RFU)坐标系中Y轴指向前方的定义。
%  --todo 修改坐标系对应  XYZ-东北天-右前上 --Done  


    %% Define your points (38 points)
    a = 0.009;  % m
    l = 5 * a;  % m
    l_p = 4 * a;
    l_p2 = l_p * cos(pi/4);
    
    points = [
        4.5*l, 0, 0; 0, -l, 0; 0, 0, 6*a; 0, l, 0; -10*l, 0, 0;
        -10*l, -10*l, 0; -10*l, 10*l, 0; -6*l, -3*l, 5*a; -6*l, 3*l, 5*a;
        -1.5*l, -5*l, 0; -1.5*l, 5*l, 0; -13*l, 0, 0;
        -1.5*l + l_p + l_p2, -5*l - l_p, 0; -1.5*l + l_p + l_p2, -5*l + l_p, 0;
        -1.5*l + l_p, -5*l + l_p + l_p2, 0; -1.5*l - l_p, -5*l + l_p + l_p2, 0;
        -1.5*l - l_p - l_p2, -5*l + l_p, 0; -1.5*l - l_p - l_p2, -5*l - l_p, 0;
        -1.5*l - l_p, -5*l - l_p - l_p2, 0; -1.5*l + l_p, -5*l - l_p - l_p2, 0;
        -1.5*l + l_p + l_p2, 5*l - l_p, 0; -1.5*l + l_p + l_p2, 5*l + l_p, 0;
        -1.5*l + l_p, 5*l + l_p + l_p2, 0; -1.5*l - l_p, 5*l + l_p + l_p2, 0;
        -1.5*l - l_p - l_p2, 5*l + l_p, 0; -1.5*l - l_p - l_p2, 5*l - l_p, 0;
        -1.5*l - l_p, 5*l - l_p - l_p2, 0; -1.5*l + l_p, 5*l - l_p - l_p2, 0;
        -13*l + l_p + l_p2, 0 - l_p, 0; -13*l + l_p + l_p2, 0 + l_p, 0;
        -13*l + l_p, 0 + l_p + l_p2, 0; -13*l - l_p, 0 + l_p + l_p2, 0;
        -13*l - l_p - l_p2, 0 + l_p, 0; -13*l - l_p - l_p2, 0 - l_p, 0;
        -13*l - l_p, 0 - l_p - l_p2, 0; -13*l + l_p, 0 - l_p - l_p2, 0;
        -1.5*l, 0, 0; -1.5*l, 0, -3*a
    ];
    
    %% 增加尺寸定义  -- 原机翼展 0.45m --图画成0.9m了
    % scale =  3.33333*0.5; % X1500
    scale =  15;  % 测试姿态用
    points = scale * points;

    
    %% 平移至电机旋转轴对齐原点坐标轴位置
    points = points + scale*[1.5*l,0,0];
    

    %% 绘制
    % Define the mesh (faces as triplets of point indices)
    mesh = [
        1, 2, 3; 1, 3, 4; 1, 2, 4; 5, 2, 3; 5, 2, 4; 5, 3, 4;
        2, 5, 6; 4, 5, 7; 8, 2, 6; 8, 6, 5; 8, 2, 5; 9, 4, 5;
        9, 7, 5; 9, 4, 7; 10, 13, 14; 10, 14, 15; 10, 15, 16; 10, 16, 17;
        10, 17, 18; 10, 18, 19; 10, 19, 20; 10, 20, 13; 11, 21, 22;
        11, 22, 23; 11, 23, 24; 11, 24, 25; 11, 25, 26; 11, 26, 27;
        11, 27, 28; 11, 21, 28; 12, 29, 30; 12, 30, 31; 12, 31, 32;
        12, 32, 33; 12, 33, 34; 12, 35, 34; 12, 35, 36; 12, 36, 29;
        37, 38, 10; 37, 38, 11; 37, 38, 12; 37, 38, 1
    ];
    
    % Define colors for different faces (e.g., rotor faces, arm faces, etc.)
    colors = [
        0.1, 0.2, 0.9;   % Blue for base/arm faces
        0.8, 0.1, 0.1;   % Red for rotor_a faces
        0.1, 0.8, 0.1;   % Green for rotor_b faces
        0.8, 0.8, 0.1;   % Yellow for rotor_c faces
        0.6, 0.6, 0.6;   % Grey for body faces
        0.9, 0.9, 0.9;   % Light grey for connecting faces
    ];
     




    %% 电机倾转定义
    % 提取 Rotor A 和 Rotor B 的面
    rotor_a_faces = mesh(15:22, :);  % Rotor A faces
    rotor_b_faces = mesh(23:30, :);  % Rotor B faces

    % 提取 Rotor A 和 Rotor B 的顶点
    rotor_a_indices = unique(rotor_a_faces(:)); % Rotor A 顶点的全局索引
    rotor_b_indices = unique(rotor_b_faces(:)); % Rotor B 顶点的全局索引
    rotor_a_points = points(rotor_a_indices, :); % 获取 Rotor A 的独立顶点
    rotor_b_points = points(rotor_b_indices, :); % 获取 Rotor B 的独立顶点

    % 调整 Rotor A 和 Rotor B 面的索引为局部索引
    rotor_a_faces_local = changem(rotor_a_faces, 1:numel(rotor_a_indices), rotor_a_indices);
    rotor_b_faces_local = changem(rotor_b_faces, 1:numel(rotor_b_indices), rotor_b_indices);

    %% 添加绕 Y 轴的俯仰旋转---前面有对齐坐标轴步骤
    % 定义 Rotor A 和 Rotor B 的俯仰角（单位：弧度）
    pitch_a = deg2rad(tilt_angle(1)); % Rotor A 的俯仰角
    pitch_b = deg2rad(tilt_angle(2)); % Rotor B 的俯仰角

    % 绕 x 轴的旋转矩阵
    R_pitch_a = [cos(pitch_a), 0, sin(pitch_a);
                 0, 1, 0;
                 -sin(pitch_a), 0, cos(pitch_a)];
    R_pitch_b = [cos(pitch_b), 0, sin(pitch_b);
                 0, 1, 0;
                 -sin(pitch_b), 0, cos(pitch_b)];

    % 对 Rotor A 和 Rotor B 的顶点进行俯仰旋转
    rotor_a_points_rotated = (R_pitch_a * rotor_a_points')'; % 转置操作
    rotor_b_points_rotated = (R_pitch_b * rotor_b_points')';




    % 对每个点进行姿态旋转
    % 定义飞行器的姿态角（单位：弧度）
    phi = attitude(1,1);   % 滚转角
    theta = attitude(1,2); % 俯仰角
    psi = attitude(1,3);   % 偏航角    
    
    % 使用自定义函数获取旋转矩阵
    R = rotation_matrix(phi, theta, psi);
    
    % 对每个点进行姿态旋转
    rotated_points = (R * points')';  % 转置后进行矩阵乘法，再转置回来
   
    % 更新 `points` 为旋转后的结果
    points = rotated_points;
    rotor_a_points_rotated = (R * rotor_a_points_rotated')'; % a
    rotor_b_points_rotated = (R * rotor_b_points_rotated')'; % b


    %% 对每个点进行平移 
    % 右前上 对应 东北天， XYZ坐标的转换
    R_pos = [0, 1, 0; 
             1, 0, 0; 
             0, 0, 1];    

    position_adjust= (R_pos * position')';
    points = points + position_adjust;
    rotor_a_points_rotated = rotor_a_points_rotated + position_adjust;% a
    rotor_b_points_rotated = rotor_b_points_rotated + position_adjust;% b


    %% 坐标反转矩阵---目前不变，后续针对高度酌情修改
    % convert North-East Down to East-North-Up for rendering
    % 定义旋转矩阵 机头朝 -- 北前Y
    R_ATT = [0, 1, 0; 
             1, 0, 0; 
             0, 0, 1];    

    % R_z = [1, 0, 0; 
    %        0, 1, 0;
    %        0, 0, 1];    
    % % 对每个点进行旋转
    rotated_points_z = (R_ATT * points')';  % 转置后进行矩阵乘法，再转置回来    
    % % 更新 `points` 为旋转后的结果
    points = rotated_points_z;
    rotor_a_points_rotated = (R_ATT * rotor_a_points_rotated')'; % a
    rotor_b_points_rotated = (R_ATT * rotor_b_points_rotated')'; % b

    % % 绘制所有部分在一张图中
    % cla;
    % figure;
    % hold on;

    % 绘制 Rotor A
    for i = 1:size(rotor_a_faces_local, 1)
        % Extract the points of the current face
        p1 = rotor_a_points_rotated(rotor_a_faces_local(i, 1), :);
        p2 = rotor_a_points_rotated(rotor_a_faces_local(i, 2), :);
        p3 = rotor_a_points_rotated(rotor_a_faces_local(i, 3), :);

        % Set color for Rotor A (Red)
        color = [0.8, 0.1, 0.1]; % Red

        % Plot the triangle (face) with the selected color
        fill3([p1(1), p2(1), p3(1)], ...
              [p1(2), p2(2), p3(2)], ...
              [p1(3), p2(3), p3(3)], color, 'FaceAlpha', 0.7);
    end

    % 绘制 Rotor B
    for i = 1:size(rotor_b_faces_local, 1)
        % Extract the points of the current face
        p1 = rotor_b_points_rotated(rotor_b_faces_local(i, 1), :);
        p2 = rotor_b_points_rotated(rotor_b_faces_local(i, 2), :);
        p3 = rotor_b_points_rotated(rotor_b_faces_local(i, 3), :);

        % Set color for Rotor B (Green)
        color = [0.1, 0.8, 0.1]; % Green

        % Plot the triangle (face) with the selected color
        fill3([p1(1), p2(1), p3(1)], ...
              [p1(2), p2(2), p3(2)], ...
              [p1(3), p2(3), p3(3)], color, 'FaceAlpha', 0.7);
        end

    % 绘制其他部分（例如主结构或 Rotor C）
    other_faces = [mesh(1:14, :); mesh(31:end, :)]; % 其余部分的面
    for i = 1:size(other_faces, 1)
        % Extract the points of the current face
        p1 = points(other_faces(i, 1), :);
        p2 = points(other_faces(i, 2), :);
        p3 = points(other_faces(i, 3), :);

        % Set color for the other parts (Blue)
        color = [0.1, 0.2, 0.9]; % Blue

        % Plot the triangle (face) with the selected color
        fill3([p1(1), p2(1), p3(1)], ...
              [p1(2), p2(2), p3(2)], ...
              [p1(3), p2(3), p3(3)], color, 'FaceAlpha', 0.7);
    end


    % %% 添加原点显示
    % % 绘制原点为小球
    [X, Y, Z] = sphere(20); % 创建球体
    sphere_radius = 0.2; % 设置球体半径
    surf(sphere_radius * X, sphere_radius * Y, sphere_radius * Z, ...
         'EdgeColor', 'none', 'FaceColor', [1, 0, 0], 'FaceAlpha', 0.2);

    % 添加原点的标注
    text(0, 0, 0, '原点', 'FontSize', 6, 'Color', 'r', 'HorizontalAlignment', 'center');

end


function new_indices = changem(indices, new_vals, old_vals)
    [~, loc] = ismember(indices, old_vals);
    new_indices = new_vals(loc);
end


function R = rotation_matrix(phi,theta,psi)

R = [cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), ...
     cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta), ...
     -cos(phi)*sin(theta); ...
     -cos(phi)*sin(psi),...
     cos(phi)*cos(psi), ...
     sin(phi);...
     cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi),...
     sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi),...
     cos(phi)*cos(theta)];

end
