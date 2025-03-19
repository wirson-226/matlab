% 【复现】复杂环境下自适应决策和避障的异构固定翼无人机集群协同搜索方法
% 本文研究了适用干复杂环境的导构无人机协同搜索技术。在应用中，固定翼无人机
% 空投旋翼无人机快速部罢集群，同时，固定翼无人机作为通信中继节点，进一步提
% 升集群的协同搜索性能。针对异构无人机的协同搜索需求，提出了一种满足无人机
% 机动性约束的跳跃网格决策方法，发展了一种参数动态选择方法，使搜索决策更加
% 响应任务需求，以及一种低功耗的搜索信息传递方法。带宽进行了设计，显着提高
% 了无人机的通信效率。仿真结果表明异构无人机具有自适应决策能力，威胁规避和
% 碰撞规避。异构集群在通信约束下的协同搜索性能较同构集群有显着提高。

% 总共三个demo，运行比较久，只演示demo3，另外两个结果图已经放在文件夹里了
%% Add path
addpath Function_System;
addpath Function_Initiate;
addpath Function_Plot;
addpath Function_Communication;
addpath Function_Search;
addpath Function_Genetic_Algorithm;
addpath Function_Objective;

% demo_index=1, simulation of 4 homogeneous UAVs, with communication constraints and stationary obstacles
% demo_index=2, simulation of 5 heterogeneous UAVs, with communication constraints and stationary obstacles
% demo_index=3, simulation of 5 heterogeneous UAVs, with communication constraints and dynamic obstacles

demo_index=3;
demo_path=['Demo/Demo' sprintf('%d',demo_index)];
demo_name=['demo' sprintf('%d',demo_index) '.m'];
addpath(demo_path);                     
run(demo_name)
