%*********************************************************************
% Discription:  Initialize UAVs' search information by ground station
% input:        UAV                 UAVs structure array
% input:        GS                  Ground station information
% putput:       UAV                 UAVs structure array
%*********************************************************************

function UAV=Init_UAV_Map(UAV,GS)
[~,uav_num]=size(UAV);
for k=1:uav_num
    UAV(k).proba=GS.probability;        % Update target existance probability matrix
    UAV(k).uncer=GS.uncertainty;        % Update environment uncertainty matrix
    UAV(k).obser=GS.observation;        % Update observation information matrix
end
end

% 
% 这段代码的作用是 在任务开始前，确保所有 UAV 共享相同的环境感知信息，这样可以：
% 
% 统一 UAV 的环境认知，保证所有 UAV 使用相同的初始搜索信息。
% 
% 提高搜索效率，避免 UAV 之间因信息不一致而产生重复搜索或信息偏差。
% 
% 适用于协作搜索任务，特别是 多 UAV 目标探测、环境勘察、灾害监测 等应用。
% 
% 总的来说，这个函数 将地面站的搜索信息同步给所有 UAV，在 多无人机协作搜索任务 中至关重要。