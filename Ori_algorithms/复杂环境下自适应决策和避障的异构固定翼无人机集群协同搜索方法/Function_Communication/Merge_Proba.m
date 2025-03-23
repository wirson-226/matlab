%***************************************************************************
% Discription:  Merging the target existance probability of another UAV
% input:        uav1                UAV structure (Self)
% input:        uav2                UAV structure (source of information)
% putput:       proba               Merged target existance probability matrix
%***************************************************************************

function proba = Merge_Proba(uav1,uav2)

[cover_grid1,cover_num1]=Seeker(uav1);                          % Obtain the grids covered by the FOV of uav1
[cover_grid2,cover_num2]=Seeker(uav2);                          % Obtain the grids covered by the FOV of uav2


% Compare whether there is overlap between two detection areas. 
% If there is overlap, change the coordinates of overlap grids from information source UAV
% to negative and do not participate in fusion, so that the probability of targets in the 
% coverage area is based on the self detection results

for i=1:cover_num1
    for j=1:cover_num2
        if cover_grid1(i,1)==cover_grid2(j,1)&&...
                cover_grid1(i,2)==cover_grid2(j,2)
            cover_grid2(j,1)=-1;
            cover_grid2(j,2)=-1;
        end        
    end    
end
% Initialize the target probability matrix with the current UAV information
proba=uav1.proba;
for i=1:cover_num2
    if cover_grid2(i,1)>0&&cover_grid2(i,2)>0                   % Determine non overlapping areas
        xg=cover_grid2(i,1);                                    % Obtain the x-direction position of the grid
        yg=cover_grid2(i,2);                                    % Obtain the y-direction position of the grid
        proba(xg,yg)=uav2.proba(xg,yg);                         % Integrate the target existance probability of UAV2 in UAV1
    end   
end
end



% 这段代码的作用是 让 UAV1 参考 UAV2 的搜索信息，但不覆盖 UAV1 自己已经探测到的区域，其优点包括：
% 
% 避免信息重复覆盖：
% 
% UAV 只会更新自己 未覆盖 的网格，而不会更改自己已探测的区域。
% 
% 提高搜索效率：
% 
% 共享目标概率信息，提高多无人机协同搜索能力。
% 
% 适用于多 UAV 目标探测任务：
% 
% 比如 灾难救援、军事侦察、环境监测，确保不同 UAV 互相补充信息，而不会干扰彼此的决策。