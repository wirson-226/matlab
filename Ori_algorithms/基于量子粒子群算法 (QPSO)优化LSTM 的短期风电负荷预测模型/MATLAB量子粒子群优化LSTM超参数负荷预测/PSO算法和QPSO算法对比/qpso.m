function  [y,trace]=qpso(PopNum,Maxstep,dim,lb,ub,fobj)
% PopNum;% 种群数
% Maxstep;%最大寻优迭代数
% dim;% 寻优维度

xmin=lb*ones(1,dim);
xmax=ub*ones(1,dim);
% % init of population
for i=1:PopNum%随机初始化速度,随机初始化位置
    for j =1:dim
        pop(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);
    end
end

% calculate the fitness_value of Pop
pbest = pop;
data1 = zeros(Maxstep,PopNum,dim);
data2 = zeros(Maxstep,PopNum);
for i = 1:PopNum
    fit(i) = fobj(pop(i,:)); 
    f_pbest(i) = fit(i);
end
[f_gbest,g]=min(f_pbest);

gbest = pbest(g,:);

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
        pop(i,:) = p + b*abs(mbest-pop(i,:)).*log(1./u).*(1-2*(u >= 0.5));
        
        % boundary detection
        for j=1:dim
            if pop(i,j)>xmax(j) | pop(i,j)<xmin(j)
                    pop(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);  %
            end
        end
        fit(i) = fobj(pop(i,:)); 
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
end
y=gbest;

end