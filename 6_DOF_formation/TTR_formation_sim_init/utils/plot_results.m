function plot_results(ttraj, xtraj, desired, actual, actuator)
% 绘图模块 - 用于 VTOL 仿真结果展示
% 输入:
%   ttraj     - 时间轴
%   xtraj     - 实际状态轨迹 (Nx13)
%   desired   - struct 包含期望值（位置、速度、姿态、角速度）
%   actual    - struct 包含实际值（姿态、角速度）
%   actuator  - struct 包含执行器命令（tilt、throttle、elevon）

%% Position
figure('Name', 'Position');
subplot(3,1,1);
plot(ttraj, xtraj(:,1), 'b', ttraj, desired.pos(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('X [m]'); legend('Actual', 'Desired'); grid on;
subplot(3,1,2);
plot(ttraj, xtraj(:,2), 'b', ttraj, desired.pos(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Y [m]'); legend('Actual', 'Desired'); grid on;
subplot(3,1,3);
plot(ttraj, xtraj(:,3), 'b', ttraj, desired.pos(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Z [m]'); legend('Actual', 'Desired'); grid on;

%% Velocity
figure('Name', 'Velocity');
subplot(3,1,1);
plot(ttraj, xtraj(:,4), 'b', ttraj, desired.vel(:,1), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Vx'); legend('Actual', 'Desired'); grid on;
subplot(3,1,2);
plot(ttraj, xtraj(:,5), 'b', ttraj, desired.vel(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Vy'); legend('Actual', 'Desired'); grid on;
subplot(3,1,3);
plot(ttraj, xtraj(:,6), 'b', ttraj, desired.vel(:,3), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Vz'); legend('Actual', 'Desired'); grid on;

%% Attitude (Roll, Pitch, Yaw)
figure('Name', 'Attitude');
subplot(3,1,1);
plot(ttraj, rad2deg(actual.att(:,1)), 'b', ttraj, rad2deg(desired.att(:,1)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\phi [deg]'); legend('Actual', 'Desired'); grid on;
subplot(3,1,2);
plot(ttraj, rad2deg(actual.att(:,2)), 'b', ttraj, rad2deg(desired.att(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\theta [deg]'); legend('Actual', 'Desired'); grid on;
subplot(3,1,3);
plot(ttraj, rad2deg(actual.att(:,3)), 'b', ttraj, rad2deg(desired.att(:,3)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('\psi [deg]'); legend('Actual', 'Desired'); grid on;

%% Angular Rates (p, q, r)
figure('Name', 'Angular Velocity');
subplot(3,1,1);
plot(ttraj, rad2deg(actual.omega(:,1)), 'b', ttraj, rad2deg(desired.omega(:,1)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('p [deg/s]'); legend('Actual', 'Desired'); grid on;
subplot(3,1,2);
plot(ttraj, rad2deg(actual.omega(:,2)), 'b', ttraj, rad2deg(desired.omega(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('q [deg/s]'); legend('Actual', 'Desired'); grid on;
subplot(3,1,3);
plot(ttraj, rad2deg(actual.omega(:,3)), 'b', ttraj, rad2deg(desired.omega(:,3)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('r [deg/s]'); legend('Actual', 'Desired'); grid on;

%% Actuator
figure('Name', 'Actuators');
subplot(3,1,1);
plot(ttraj, rad2deg(actuator.tilt(:,1)), 'b', ttraj, rad2deg(actuator.tilt(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Tilt [deg]'); legend('arm a', 'arm b'); grid on;
subplot(3,1,2);
plot(ttraj, actuator.throttle(:,1), 'b', ttraj, actuator.throttle(:,2), 'r--', ttraj, actuator.throttle(:,3), 'g--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Throttle'); legend('a', 'b', 'c'); grid on;
subplot(3,1,3);
plot(ttraj, rad2deg(actuator.elevon(:,1)), 'b', ttraj, rad2deg(actuator.elevon(:,2)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Elevon [deg]'); legend('right', 'left'); grid on;

%% Moment
figure('Name', 'Moments');
subplot(3,1,1);
plot(ttraj, desired.M(:,1), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('My [Nm]'); legend('Des M_y'); grid on;
subplot(3,1,2);
plot(ttraj, desired.M(:,2), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Mx [Nm]'); legend('Des M_x'); grid on;
subplot(3,1,3);
plot(ttraj, desired.M(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Mz [Nm]'); legend('Des M_z'); grid on;

end
