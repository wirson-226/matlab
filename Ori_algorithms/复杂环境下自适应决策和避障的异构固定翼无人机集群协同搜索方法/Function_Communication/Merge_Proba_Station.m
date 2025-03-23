%***************************************************************************
% Discription:  Merging the target existance probability of UAV
% input:        uav1                UAV structure
% input:        proba               Target existance probability matrix
% output:       proba               Merged target existance probability matrix
%***************************************************************************

function proba = Merge_Proba_Station(proba,uav)
uav_temp=uav;                                   
uav_temp.x=uav.xs;
uav_temp.y=uav.ys;
uav_temp.xg=uav.xsg;
uav_temp.yg=uav.ysg;
[cover_grid,cover_num]=Seeker(uav_temp);        % Obtain the grids covered by the FOV of uav
for i=1:cover_num
    xg=cover_grid(i,1);                         % Obtain the x-direction position of the grid
    yg=cover_grid(i,2);                         % Obtain the y-direction position of the grid
    proba(xg,yg)=uav_temp.proba(xg,yg);         % Integrate the target existance probability of the UAV
end
end

% 这段代码的作用是 让 UAV 将其搜索的目标存在概率信息同步到地面站或其他共享存储矩阵，其优势包括：
% 
% 增强多 UAV 任务的协作能力：
% 
% UAV 之间可以共享目标检测信息，提高搜索效率。
% 
% 提高地面站的目标感知能力：
% 
% 让地面站获得 UAV 观测数据，以便后续任务规划