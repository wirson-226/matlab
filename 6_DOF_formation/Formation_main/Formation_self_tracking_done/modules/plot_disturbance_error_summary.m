% modules/plot_disturbance_error_summary.m
function plot_disturbance_error_summary(d_hist, dhat_hist, dt, ~)
    [N, ~, T] = size(d_hist); t = (0:T-1)*dt;
    e = zeros(1, T);
    for i = 1:N
        ei = vecnorm(squeeze(d_hist(i,:,:) - dhat_hist(i,:,:)), 2, 1);  % [1 × T]
        e = e + ei;
    end
    e = e / N;  % 平均扰动估计误差范数

    % === 绘图 ===
    figure('Name','Disturbance Estimation Error Summary');
    plot(t, e, 'LineWidth', 2); hold on;
    title('Average Disturbance Estimation Error Norm');
    xlabel('Time [s]'); ylabel('||d - \hat{d}||_{avg}');
    grid on;

    % === 数值输出 ===
    IAE = trapz(t, abs(e));
    ISE = trapz(t, e.^2);
    max_err = max(e);
    steady_err = mean(e(end - round(T*0.1):end));  % 最后10%窗口平均值

    fprintf('[扰动估计性能指标]\n');
    fprintf('IAE (积分绝对误差)     : %.4f\n', IAE);
    fprintf('ISE (积分平方误差)     : %.4f\n', ISE);
    fprintf('MAX (最大误差范数)     : %.4f\n', max_err);
    fprintf('STEADY (稳态平均误差) : %.4f\n', steady_err);
end
