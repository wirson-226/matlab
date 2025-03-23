%*****************************************************************
% Discription:  Under the constraint of communication range, 
%               achieve mutual communication function between UAVs
% input:        i                   Current UAV index
% input:        uav                 UAV structure
% input:        UAV                 UAVs structure array
% putput:       uav                 UAV structure
%*****************************************************************

function uav=Com_UAV(i,uav,UAV)

[~,uav_num]=size(UAV);                                      % Obtain UAVs number
member=0;                                                   % UAVs number within communication range
for n=1:uav_num
    if n==i
        continue;                                           % n=i repersents self
    end
    d=sqrt((UAV(i).x-UAV(n).x)^2+(UAV(i).y-UAV(n).y)^2);    % Calculate the distance between UAVs

    %% Information transmission begins when the distance between UAVs is less than the communication range
    if d<UAV(n).com_distance
        member=member+1;
        uav.teammates(member,1)=n;                          % Recored the teammate index
        uav.teammates(member,2)=UAV(n).x;                   % Recored the teammate's x-coordinate
        uav.teammates(member,3)=UAV(n).y;                   % Recored the teammate's y-coordinate
        [uav,update_info]=Update_Search_Coop(uav,UAV(n));
        if uav.task==1
            uav=Merge_Search_Info(uav,update_info);
        end
    end
end
end


% 这段代码实现了 多无人机间的通信，在 UAVs 之间共享目标搜索信息，主要功能如下：

% 计算所有 UAV 之间的距离，判断哪些 UAV 在通信范围内。
% 
% 记录通信范围内的 UAV 信息，包括编号和位置信息。
% 
% 调用 Update_Search_Coop() 进行 UAV 之间的搜索信息更新。
% 
% 如果 UAV 正在执行搜索任务，则合并搜索信息，提高环境感知能力