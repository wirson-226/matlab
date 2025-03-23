%%%% 测试用 %%%%

% 初始化PID控制器
pid = PIDControl(1.0, 0.5, 0.1, 0.01, 0.05, 10); % 参数：kp, ki, kd, Ts, sigma, limit

% 设置参考值和当前值
y_ref = 1.0;
y = 0.5;

% 更新控制器并获取饱和后的控制输出
[u_sat, pid] = pid.update(y_ref, y);

% 显示控制输出
disp(['Saturated Control Output: ', num2str(u_sat)]);
