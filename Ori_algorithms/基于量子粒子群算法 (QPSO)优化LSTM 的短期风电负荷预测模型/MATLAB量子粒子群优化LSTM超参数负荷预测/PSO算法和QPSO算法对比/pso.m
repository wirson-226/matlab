function [xm ,trace]=pso(N,M,D,lb,ub,fobj)
c1=1.5;
c2=1.5;
w=1.2;
% 定义寻优边界
xmin=lb*ones(1,D);
xmax=ub*ones(1,D);
for i=1:N%随机初始化速度,随机初始化位置
    for j =1:D
        x(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);
    end
    v(i,:)=rand(1,D);
end
%------先计算各个粒子的适应度，并初始化Pi和Pg----------------------
for i=1:N
    p(i)=fobj(x(i,:));
    y(i,:)=x(i,:);
    
end
[fg,index]=min(p);
pg = x(index,:);             %Pg为全局最优

%------进入主要循环，按照公式依次迭代------------

for t=1:M
    
    for i=1:N
        v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(pg-x(i,:));
        x(i,:)=x(i,:)+v(i,:);
        
        % boundary detection
        for j=1:D
            if x(i,j)>xmax(j) | x(i,j)<xmin(j)
                    x(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);  %
            end
        end
        temp=fobj(x(i,:));
        if temp<p(i)
            p(i)=temp;
            y(i,:)=x(i,:);
        end
        
        if p(i)<fg
            pg=y(i,:);
            fg=p(i);
        end
    end
    trace(t)=fg;
end
xm = pg;