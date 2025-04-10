function plot_results(t, x, desired, actual, actuator, num_agents, colors)
% 多无人机统一绘图函数，展示位置、速度、姿态、角速度、执行器与控制矩

N = length(t);
labels = {'X','Y','Z'}; 
states = {'att','omega','pos','vel'};  % 保持索引一致
unit = {'[deg]','[deg/s]','[m]','[m/s]'};
names = {'Attitude','Angular Velocity','Position','Velocity'};


% ==== Position, Velocity, Attitude, Angular Velocity ====
for fig = 1:4
    figure('Name', names{fig});
    for k = 1:3
        subplot(3,1,k); hold on;
        legend_entries = {};
        for i = 1:num_agents
            idx = (i-1)*N + (1:N);
            if fig <= 2  % 对应姿态/角速度，从结构体中取
                actual_data = actual.(states{fig})(idx, k);
                desired_data = desired.(states{fig})(idx, k);
            else  % 对应位置/速度，从x中取
                actual_data = x(idx, (fig-2)*3 + k - 3);  % fig=3:pos, 4:vel
                desired_data = desired.(states{fig})(idx, k);
            end

            if fig >= 3
                actual_data = rad2deg(actual_data);
                desired_data = rad2deg(desired_data);
            end
            plot(t, actual_data, '-', 'Color', colors(i,:), 'LineWidth', 1.2);
            plot(t, desired_data, '--', 'Color', colors(i,:), 'LineWidth', 1.2);
            legend_entries{end+1} = ['A' num2str(i)];
            legend_entries{end+1} = ['A' num2str(i) ' des'];
        end
        xlabel('Time [s]');
        ylabel([labels{k} ' ' unit{fig}]);
        legend(legend_entries, 'Location', 'best');
        grid on;
    end
end

% ==== Position Error Norm ====
figure('Name', 'Position Error Norm'); hold on;
for i = 1:num_agents
    idx = (i-1)*N + (1:N);
    err = desired.pos(idx,:) - x(idx,1:3);
    err_norm = vecnorm(err, 2, 2);
    plot(t, err_norm, 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time [s]'); ylabel('||Position Error|| [m]');
legend(arrayfun(@(j) ['A' num2str(j)], 1:num_agents, 'UniformOutput', false));
grid on;

% ==== Velocity Error Norm ====
figure('Name', 'Velocity Error Norm'); hold on;
for i = 1:num_agents
    idx = (i-1)*N + (1:N);
    err = desired.vel(idx,:) - x(idx,4:6);
    err_norm = vecnorm(err, 2, 2);
    plot(t, err_norm, 'Color', colors(i,:), 'LineWidth', 1.5);
end
xlabel('Time [s]'); ylabel('||Velocity Error|| [m/s]');
legend(arrayfun(@(j) ['A' num2str(j)], 1:num_agents, 'UniformOutput', false));
grid on;

% ==== Actuators ====
figure('Name', 'Actuators');
subplot(3,1,1); hold on;
for i = 1:num_agents
    idx = (i-1)*N + (1:N);
    plot(t, rad2deg(actuator.tilt(idx,1)), '-', 'Color', colors(i,:), 'LineWidth', 1.2);
    plot(t, rad2deg(actuator.tilt(idx,2)), '--', 'Color', colors(i,:), 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('Tilt [deg]'); legend('arm a', 'arm b'); grid on;

subplot(3,1,2); hold on;
for i = 1:num_agents
    idx = (i-1)*N + (1:N);
    plot(t, actuator.throttle(idx,1), '-', 'Color', colors(i,:), 'LineWidth', 1.2);
    plot(t, actuator.throttle(idx,2), '--', 'Color', colors(i,:), 'LineWidth', 1.2);
    plot(t, actuator.throttle(idx,3), ':', 'Color', colors(i,:), 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('Throttle'); legend('a','b','c'); grid on;

subplot(3,1,3); hold on;
for i = 1:num_agents
    idx = (i-1)*N + (1:N);
    plot(t, rad2deg(actuator.elevon(idx,1)), '-', 'Color', colors(i,:), 'LineWidth', 1.2);
    plot(t, rad2deg(actuator.elevon(idx,2)), '--', 'Color', colors(i,:), 'LineWidth', 1.2);
end
xlabel('Time [s]'); ylabel('Elevon [deg]'); legend('right','left'); grid on;

% ==== Moments ====
figure('Name', 'Desired Moments');
for k = 1:3
    subplot(3,1,k); hold on;
    for i = 1:num_agents
        idx = (i-1)*N + (1:N);
        plot(t, desired.M(idx,k), 'Color', colors(i,:), 'LineWidth', 1.2);
    end
    xlabel('Time [s]'); ylabel(['M' labels{k} ' [Nm]']);
    legend(arrayfun(@(j) ['A' num2str(j)], 1:num_agents, 'UniformOutput', false));
    grid on;
end
end
