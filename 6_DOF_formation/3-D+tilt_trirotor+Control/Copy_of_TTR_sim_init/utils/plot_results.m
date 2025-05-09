function plot_results(ttraj, xtraj, desired, actual, actuator)
% 绘图模块 - 用于 VTOL 仿真结果展示
% 输入:
%   ttraj     - 时间轴
%   xtraj     - 实际状态轨迹 (Nx13)
%   desired   - struct 包含期望值（位置、速度、姿态、角速度）
%   actual    - struct 包含实际值（姿态、角速度）
%   actuator  - struct 包含执行器命令（tilt、throttle、elevon）

% %% Position
% figure('Name', 'Position');
% subplot(3,1,1);
% plot(ttraj, xtraj(:,1), 'b', ttraj, desired.pos(:,1), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('X [m]'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,2);
% plot(ttraj, xtraj(:,2), 'b', ttraj, desired.pos(:,2), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Y [m]'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,3);
% plot(ttraj, xtraj(:,3), 'b', ttraj, desired.pos(:,3), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Z [m]'); legend('Actual', 'Desired'); grid on;

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


% %% Velocity
% figure('Name', 'Velocity');
% subplot(3,1,1);
% plot(ttraj, xtraj(:,4), 'b', ttraj, desired.vel(:,1), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Vx'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,2);
% plot(ttraj, xtraj(:,5), 'b', ttraj, desired.vel(:,2), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Vy'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,3);
% plot(ttraj, xtraj(:,6), 'b', ttraj, desired.vel(:,3), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Vz'); legend('Actual', 'Desired'); grid on;


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


% %% Attitude (Roll, Pitch, Yaw)
% figure('Name', 'Attitude');
% subplot(3,1,1);
% plot(ttraj, rad2deg(actual.att(:,1)), 'b', ttraj, rad2deg(desired.att(:,1)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('\phi [deg]'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,2);
% plot(ttraj, rad2deg(actual.att(:,2)), 'b', ttraj, rad2deg(desired.att(:,2)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('\theta [deg]'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,3);
% plot(ttraj, rad2deg(actual.att(:,3)), 'b', ttraj, rad2deg(desired.att(:,3)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('\psi [deg]'); legend('Actual', 'Desired'); grid on;

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


% %% Angular Rates (p, q, r)
% figure('Name', 'Angular Velocity');
% subplot(3,1,1);
% plot(ttraj, rad2deg(actual.omega(:,1)), 'b', ttraj, rad2deg(desired.omega(:,1)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('p [deg/s]'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,2);
% plot(ttraj, rad2deg(actual.omega(:,2)), 'b', ttraj, rad2deg(desired.omega(:,2)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('q [deg/s]'); legend('Actual', 'Desired'); grid on;
% subplot(3,1,3);
% plot(ttraj, rad2deg(actual.omega(:,3)), 'b', ttraj, rad2deg(desired.omega(:,3)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('r [deg/s]'); legend('Actual', 'Desired'); grid on;

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



% %% Actuator
% figure('Name', 'Actuators');
% subplot(3,1,1);
% plot(ttraj, rad2deg(actuator.tilt(:,1)), 'b', ttraj, rad2deg(actuator.tilt(:,2)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Tilt [deg]'); legend('tilt a', 'tilt b'); grid on;
% subplot(3,1,2);
% plot(ttraj, actuator.throttle(:,1), 'b', ttraj, actuator.throttle(:,2), 'r--', ttraj, actuator.throttle(:,3), 'g--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Throttle'); legend('throttle a', 'throttle b', 'throttle c'); grid on;
% subplot(3,1,3);
% plot(ttraj, rad2deg(actuator.elevon(:,1)), 'b', ttraj, rad2deg(actuator.elevon(:,2)), 'r--', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Elevon [deg]'); legend('elevon right', 'elevon left'); grid on;

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


% %% Moment
% figure('Name', 'Moments');
% subplot(3,1,1);
% plot(ttraj, desired.M(:,1), 'b', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('My [Nm]'); legend('M_y'); grid on;
% subplot(3,1,2);
% plot(ttraj, desired.M(:,2), 'b', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Mx [Nm]'); legend('M_x'); grid on;
% subplot(3,1,3);
% plot(ttraj, desired.M(:,3), 'b', 'LineWidth', 1.5);
% xlabel('Time [s]'); ylabel('Mz [Nm]'); legend('M_z'); grid on;


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
