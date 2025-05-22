function plot_results(ttraj, xtraj, desired, actual, actuator)
% 绘图模块 - 用于 VTOL 仿真结果展示
% 输入:
%   ttraj     - 时间轴
%   xtraj     - 实际状态轨迹 (Nx13)
%   desired   - struct 包含期望值（位置、速度、姿态、角速度）
%   actual    - struct 包含实际值（姿态、角速度）
%   actuator  - struct 包含执行器命令（tilt、throttle、elevon）

%% Position (X, Y, Z) - Optimized
figure('Name', 'Position', 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

labels = {'X', 'Y', 'Z'};
ylabels = {'X [m]', 'Y [m]', 'Z [m]'};

for k = 1:3
    ax = nexttile;
    plot(ttraj, xtraj(:,k), 'b-', ...
         ttraj, desired.pos(:,k), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(['Position ', labels{k}]);
    grid on; box on;
    legend({'Actual', 'Desired'}, 'Location', 'northeast', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end


%% Velocity (Vx, Vy, Vz) - Optimized
figure('Name', 'Velocity', 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

labels = {'V_x', 'V_y', 'V_z'};
ylabels = {'V_x [m/s]', 'V_y [m/s]', 'V_z [m/s]'};

for k = 1:3
    ax = nexttile;
    plot(ttraj, xtraj(:, k+3), 'b-', ...
         ttraj, desired.vel(:,k), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(['Velocity ', labels{k}]);
    grid on; box on;
    legend({'Actual', 'Desired'}, 'Location', 'northeast', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end


%% Attitude (Roll, Pitch, Yaw) - Optimized
figure('Name', 'Attitude', 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

labels = {'\phi', '\theta', '\psi'};
ylabels = {'\phi [deg]', '\theta [deg]', '\psi [deg]'};
titles = {'Roll Angle', 'Pitch Angle', 'Yaw Angle'};

for k = 1:3
    ax = nexttile;
    plot(ttraj, rad2deg(actual.att(:,k)), 'b-', ...
         ttraj, rad2deg(desired.att(:,k)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(titles{k});
    grid on; box on;
    legend({'Actual', 'Desired'}, 'Location', 'northeast', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

%% Angular Rates (p, q, r) - Optimized
figure('Name', 'Angular Velocity', 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

labels = {'p', 'q', 'r'};
ylabels = {'p [deg/s]', 'q [deg/s]', 'r [deg/s]'};
axes_array = gobjects(1,3);

for k = 1:3
    ax = nexttile;
    axes_array(k) = ax;
    plot(ttraj, rad2deg(actual.omega(:,k)), 'b-', ...
         ttraj, rad2deg(desired.omega(:,k)), 'r--', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(['Angular Rate ', labels{k}]);
    grid on; box on;
    legend({'Actual', 'Desired'}, 'Location', 'northeast', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end

%% Actuator Inputs (with separate legends)
figure('Name', 'Actuators', 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
tiledlayout(3,1, 'TileSpacing','compact', 'Padding','compact');

% --- Tilt Angles ---
ax1 = nexttile;
plot(ttraj, rad2deg(actuator.tilt(:,1)), 'b-', ...
     ttraj, rad2deg(actuator.tilt(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Tilt [deg]');
title('Tilt Angles');
grid on; box on;
legend({'Tilt a', 'Tilt b'}, 'Location', 'northeast', ...
       'Box','off','FontSize',10,'Interpreter','latex');

% --- Throttle ---
ax2 = nexttile;
plot(ttraj, actuator.throttle(:,1), 'b-', ...
     ttraj, actuator.throttle(:,2), 'r--', ...
     ttraj, actuator.throttle(:,3), 'g-.', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Throttle');
title('Throttle Inputs');
grid on; box on;
legend({'Throttle a', 'Throttle b', 'Throttle c'}, ...
       'Location', 'northeast', 'Box','off','FontSize',10,'Interpreter','latex');

% --- Elevon Deflections ---
ax3 = nexttile;
plot(ttraj, rad2deg(actuator.elevon(:,1)), 'b-', ...
     ttraj, rad2deg(actuator.elevon(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Elevon [deg]');
title('Elevon Deflections');
grid on; box on;
legend({'Elevon Right', 'Elevon Left'}, 'Location', 'northeast', ...
       'Box','off','FontSize',10,'Interpreter','latex');

% 统一字体风格
set([ax1, ax2, ax3], 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);


%% Moments (each subplot with individual legend)
figure('Name', 'Moments', 'Units', 'centimeters', 'Position', [5, 5, 36, 20]);
tiledlayout(3,1, 'TileSpacing', 'compact', 'Padding', 'compact');

labels = {'My', 'Mx', 'Mz'};
ylabels = {'My [Nm]', 'Mx [Nm]', 'Mz [Nm]'};

for k = 1:3
    ax = nexttile;
    plot(ttraj, desired.M(:,k), 'b', 'LineWidth', 1.5);
    xlabel('Time [s]');
    ylabel(ylabels{k});
    title(labels{k});
    grid on; box on;
    legend(labels{k}, 'Location', 'northeast', ...
           'Box', 'off', 'FontSize', 10, 'Interpreter', 'latex');
    set(ax, 'FontName', 'Times New Roman', 'FontSize', 12, 'LineWidth', 1);
end



end
