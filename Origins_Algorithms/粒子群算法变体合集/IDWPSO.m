%% 动态调整惯性权重改进粒子群算法（IDWPSO）
function [Gbest,g,gb] = IDWPSO(N,Max_iteration,lb,ub,dim,fobj)


% N = N;                  % 群体粒子个数
D = dim;                    % 粒子维数
T = Max_iteration;                  % 最大迭代次数
c1 = 1.5;                 % 学习因子1
c2 = 1.5;                 % 学习因子2
F = 0.5;                  % 放缩因子
p1 = 1;
q1 = 3;
sigma = 0.1;              % 惯性调整因子
CR = 0.1;                 % 交叉概率
Wmax = 0.8;               % 惯性权重最大值
Wmin = 0.4;               % 惯性权重最小值
Xmax = ub;                 % 位置最大值
Xmin = lb;                % 位置最小值
Vmax = 1;                 % 速度最大值
Vmin = -1;                % 速度最小值
tic;
%% 初始化种群个体（限定位置和速度）
x = rand(N,D) .* (Xmax-Xmin)+Xmin;
v = rand(N,D) .* (Vmax-Vmin)+Vmin;
%% 初始化个体最优位置和最优值
p = x;                          % 个体最优位置
pbest = ones(N,1);
for i = 1:N
    pbest(i) = fobj(x(i,:));   % 个体最优值   一个 1*N 的矩阵
end
%% 初始化全局最优位置和最优值
g = ones(1,D);
gbest = inf;
for i = 1:N
    if pbest(i) < gbest         % 求最小值，故选取最小的数
        g = p(i,:);             % 全局最优位置
        gbest = pbest(i);       % 个体最优值  一个值
    end
end
gb = ones(1,T);
%% 按照公式依次迭代直到满足精度或者迭代次数
for i = 1:T                     % 迭代次数
    for j = 1:N                 % 维数
        %% 更新个体最优位置和最优值
        if fobj(x(j,:)) < pbest(j)       % 求最小值，故选取最小的数
            p(j,:) = x(j,:);
            pbest(j) = fobj(x(j,:));     % 一个 1*N 的矩阵
        end
        %% 更新全局最优位置和最优值
        if pbest(j) < gbest
            g = p(j,:);
            gbest = pbest(j);             % 一个值
        end
        %% 计算动态惯性权重值
        w = Wmin+(Wmax-Wmin)*exp(-i/T)+sigma*betarnd(p1,q1);
        v(j,:) = w*v(j,:)+c1*rand*(p(j,:)-x(j,:))+c2*rand*(g-x(j,:));
        %% 跟新位置和速度值
        if rand < CR
            for ii = 1:D
                r1 = randperm(N,1);
                while r1 == j
                    r1 = randperm(N,1);
                end         % r1 
                
                r2 = randperm(N,1);
                while (r2==j) & (r2==r1)
                    r2 = randperm(N,1)
                end         % r2
                
                r3 = randperm(N,1);
                while (r3==j) & (r3==r1) & (r3==r2)
                    r3 = randperm(N,1)
                end         % r3
                
                x(j,ii) = x(r1,ii)+F*(x(r2,ii)-x(r3,ii));
            end             % ii 
        else
            x(j,:) = x(j,:)+v(j,:);
        end
        %% 边界条件处理
        % for ii = 1:D
        %     if (v(j,ii)>Vmax)  |  (v(j,ii)< Vmin)    % 速度吸收
        %         v(j,ii)=rand * (Vmax-Vmin)+Vmin;
        %     end
        %     if (x(j,ii)>Xmax)  |  (x(j,ii)< Xmin)    % x 吸收
        %         x(j,ii)=rand * (Xmax-Xmin)+Xmin;
        %     end
        % end       % 边界条件
        
    end        % 维数循环
    %% 记录历代全局最优值
    gb(i)=gbest;
end        % 迭代循环
time =toc;
g;                         %最优个体
Gbest = gb(end);           %最优值
row = find(gb == gb(end));
N = row(1);


