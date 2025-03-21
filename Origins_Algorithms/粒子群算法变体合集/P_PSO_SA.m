%% 结合自适应惯性权重的混合粒子群算法
function [GBest,G,GB] = P_PSO_SA(N,Max_iteration,lb,ub,dim,fobj)


D = dim;                           % 粒子维数
T = Max_iteration;                         % 最大迭代次数
c1 = 1.5;                        % 学习因子1
c2 = 1.5;                        % 学习因子2
Xmax = ub;                        % 搜索变量最大值
Xmin = lb;                       % 搜索变量最小值
%% 冷却表参数
L = 100;                         % 马可夫链长度
K = 0.99;                        % 衰减参数
Te = 100;                        % 初始温度
P = 0;                           % Metropolis过程中总接受点
%% 惯性因子相关参数
k = 0.6;                         % 控制因子
Wstart = 0.9;                    % 初始惯性权重
Wend = 0.4;                      % 终止惯性权重
Vmax = 1;                        % 速度最大值
Vmin = -1;                       % 速度最小值
tic;
%% 初始化种群个体（限定位置和速度）
PreX = rand(N,D) .* (Xmax-Xmin)+Xmin;
v = rand(N,D) * (Vmax-Vmin)+Vmin;            % 速度
%% 初始化个体最优位置和最优值
P = PreX;                          % 个体最优位置
PBest = ones(N,1);
for i = 1:N
    PBest(i) = fobj(PreX(i,:));   % 个体最优值   一个 1*N 的矩阵
end
%% 初始化全局最优位置和最优值
G = ones(1,D);
GBest = inf;
for i = 1:N
    if PBest(i) < GBest         % 求最小值，故选取最小的数
        G = P(i,:);             % 全局最优位置
        GBest = PBest(i);       % 个体最优值  一个值
    end
end
GB = ones(1,T);
%% 按照公式依次迭代
for t = 1:T                 % 迭代次数
    Te = K*Te;              % 降温
    for i = 1:N            % 维数
        %% 更新个体最优位置和最优值
        changer1 = (fobj(PreX(i,:))-PBest(i))/ Te ;
        p1 = exp(changer1);
        if( PBest(i) - fobj(PreX(i,:)) > 0 )  | (p1 > rand(0,1))
            P(i,:) = PreX(i,:);
            PBest(i) = fobj(PreX(i,:));     % 一个 1*N 的矩阵
        end
        %% 更新全局最优位置和最优值
        changer2 = (PBest(i)-GBest)/ Te ;
        p2 = exp(changer2);
        if( GBest - PBest(i) > 0 )  | (p2 > rand(0,1))
            G = P(i,:);
            GBest = PBest(i);             % 一个值
        end
        %% 计算动态惯性权重值
        w = (Wstart-Wend)*tan(0.875*(1-(t/T)^k))+Wend;
        %% 跟新位置和速度值
        v(i,:) = w*v(i,:)+c1*rand*(P(i,:)-PreX(i,:))+c2*rand*(G-PreX(i,:));
        NextX(i,:) = PreX(i,:)+v(i,:);
        % %% 边界条件处理
        % for ii = 1:D
        %     if (v(i,ii)>Vmax)  |  (v(i,ii)< Vmin)    % 速度吸收
        %         v(i,ii)=rand * (Vmax-Vmin)+Vmin;
        %     end
        %     if (NextX(i,ii)>Xmax)  |  (NextX(i,ii)< Xmin)    % x 吸收
        %         NextX(i,ii)=rand * (Xmax-Xmin)+Xmin;
        %     end
        % end       % 边界条件
        %% 是否全局最优解
        if ( GBest > fobj(NextX(i,:)))
            % 此为新的最优解
            G = NextX(i,:);
            GBest = fobj(NextX(i,:));
        end
        %%  Metropolis过程
        changer3 = (fobj(NextX(i,:))-fobj(PreX(i,:)))/ Te ;
        p3 = exp(changer3);
        if( fobj(PreX(i,:)) - fobj(NextX(i,:)) > 0 )  | (p3 > rand(0,1))
            % 接受新解
            PreX(i,:) = NextX(i,:);
            P = P+1;
        end
    end
    %% 记录历代全局最优值
    GB(t)=GBest;
end
time = toc;
G ;                        %最优个体
GBest = GB(end)    ;       %最优值
row = find(GB == GB(end));
N = row(1);
