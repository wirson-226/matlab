function [xm ,trace,result]=psoforlstm(p_train,t_train,p_test,t_test)

N=5;
M=10;
c1=1.5;
c2=1.5;
w=0.8;
D=4;

% 定义寻优边界
xmin=[1    1  10 0.001];%分别两个对隐含层节点 训练次数与学习率寻优
xmax=[200 200  100 0.01];%比如第一个隐含层节点的范围是1-200


for i=1:N%随机初始化速度,随机初始化位置
    for j=1:D
        if j==D% % 隐含层节点与训练次数是整数 学习率是浮点型
            x(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);
        else
            x(i,j)=round((xmax(j)-xmin(j))*rand+xmin(j));  %
        end
    end
    
    v(i,:)=rand(1,D);
end

%------先计算各个粒子的适应度，并初始化Pi和Pg----------------------
for i=1:N
    p(i)=fitness(x(i,:),p_train,t_train,p_test,t_test);
    y(i,:)=x(i,:);
    
end
[fg,index]=min(p);
pg = x(index,:);             %Pg为全局最优

%------进入主要循环，按照公式依次迭代------------

for t=1:M
    
    for i=1:N
        v(i,:)=w*v(i,:)+c1*rand*(y(i,:)-x(i,:))+c2*rand*(pg-x(i,:));
        x(i,:)=x(i,:)+v(i,:);
        
        
        for j=1:D
            if j ~=D
                x(i,j)=round(x(i,j));
            end
            if x(i,j)>xmax(j) | x(i,j)<xmin(j)
                if j==D
                    x(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);  %
                else
                    x(i,j)=round((xmax(j)-xmin(j))*rand+xmin(j));  %
                end
            end
        end
        temp=fitness(x(i,:),p_train,t_train,p_test,t_test);
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
    result(t,:)=pg;
    t,fg,pg
end
xm = pg;