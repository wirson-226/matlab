function  [y,trace,result]=qpsoforlstm(p_train,t_train,p_test,t_test)
PopNum=10;% 种群数
Maxstep=10;%最大寻优迭代数
dim=4;% 寻优维度

xmin=[1    1  10 0.001];%分别对两个隐含层的节点 训练次数与学习率寻优
xmax=[200 200  100 0.01];%比如第一个隐含层节点的范围是1-200

for i=1:PopNum%随机初始化速度,随机初始化位置
    for j=1:dim
        if j==dim% % 隐含层节点与训练次数是整数 学习率是浮点型
            pop(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);
        else
            pop(i,j)=round((xmax(j)-xmin(j))*rand+xmin(j));  %
        end
    end
end

% calculate the fitness_value of Pop
pbest = pop;
gbest = zeros(1,dim);
data1 = zeros(Maxstep,PopNum,dim);
data2 = zeros(Maxstep,PopNum);
for i = 1:PopNum
    fit(i) = fitness(pop(i,:),p_train,t_train,p_test,t_test);
    f_pbest(i) = fit(i);
end
g = min(find(f_pbest == min(f_pbest(1:PopNum))));
gbest = pbest(g,:);
f_gbest = f_pbest(g);

%-------- in the loop -------------
for step = 1:Maxstep
    
    mbest =sum(pbest(:))/PopNum;
    % linear weigh factor
    b = 1-step/Maxstep*0.5;
    data1(step,:,:) = pop;
    data2(step,:) = fit;
    for i = 1:PopNum
        a = rand(1,dim);
        u = rand(1,dim);
        p = a.*pbest(i,:)+(1-a).*gbest;
        pop(i,:) = p + b*abs(mbest-pop(i,:)).*...
            log(1./u).*(1-2*(u >= 0.5));
        % boundary detection
        
        for j=1:dim
            if j ==dim
                if pop(i,j)>xmax(j) | pop(i,j)<xmin(j)
                    pop(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);  %
                end
            else
                pop(i,j)=round(pop(i,j));
                if pop(i,j)>xmax(j) | pop(i,j)<xmin(j)
                    pop(i,j)=round((xmax(j)-xmin(j))*rand+xmin(j));  %
                end
            end
        end
        
        
        fit(i) = fitness(pop(i,:),p_train,t_train,p_test,t_test);
        if fit(i) < f_pbest(i)
            pbest(i,:) = pop(i,:);
            f_pbest(i) = fit(i);
        end
        if f_pbest(i) < f_gbest
            gbest = pbest(i,:);
            f_gbest = f_pbest(i);
        end
    end
    trace(step)=f_gbest;
    step,f_gbest,gbest
    result(step,:)=gbest;
end
y=gbest;

end