%% README.m - Formation Control Simulation Module Index
%
% 本文件概述了 modules 文件夹中所有函数模块的功能与调用关系。
% 所有模块用于支持多智能体编队控制、扰动估计、避障与动画输出等任务。

%% 目录结构
% modules/
% ├── init_params.m              % 初始化控制参数与仿真常量
% ├── init_state.m               % 初始化智能体状态与数据记录结构
% ├── init_obstacles.m           % 初始化静态/动态障碍物参数
% ├── init_video.m               % 初始化视频输出对象（MP4）
% ├── update_moving_obstacles.m  % 更新动态障碍物位置
% ├── consensus_control.m        % 一致性控制器 + 扰动观测器更新
% ├── formation_control.m        % 可变形队形控制器（支持旋转/平移）
% ├── cbf_avoidance_batch.m      % 基于CBF的避障优化器（批量处理所有agent）
% ├── render_frame.m             % 绘制每一帧图像供视频录制使用
% ├── record_history.m           % 状态与扰动估计记录器
% ├── plot_disturbance_estimation.m % 绘制真实扰动与估计值对比图
% └── save_all_figures.m         % 批量保存所有figure为 EPS/PDF/PNG

%% 模块调用说明
% 在 main.m 中，推荐按以下方式调用模块函数：
%
%   [params...] = init_params();
%   [x, v, ...] = init_state(...);
%   [static_obs, moving_obs, ...] = init_obstacles();
%   vwriter = init_video(...);
%   moving_obs.pos = update_moving_obstacles(...);
%   u_consensus = consensus_control(...);
%   [u_formation, targets] = formation_control(...);
%   u_total = cbf_avoidance_batch(...);
%   frame = render_frame(...);
%   [histories...] = record_history(...);
%   plot_disturbance_estimation(...);
%   save_all_figures();

%% 联系方式
% 如需反馈建议、合作或引用，请注明来源或联系开发者。