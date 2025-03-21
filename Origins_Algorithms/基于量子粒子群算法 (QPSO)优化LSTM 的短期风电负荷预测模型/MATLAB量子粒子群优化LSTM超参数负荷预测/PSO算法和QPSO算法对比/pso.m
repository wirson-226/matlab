function [xm ,trace]=pso(N,M,D,lb,ub,fobj)
c1=1.5;
c2=1.5;
w=1.2;
% ����Ѱ�ű߽�
xmin=lb*ones(1,D);
xmax=ub*ones(1,D);
for i=1:N%�����ʼ���ٶ�,�����ʼ��λ��
    for j =1:D
        x(i,j)=(xmax(j)-xmin(j))*rand+xmin(j);
    end
    v(i,:)=rand(1,D);
end
%------�ȼ���������ӵ���Ӧ�ȣ�����ʼ��Pi��Pg----------------------
for i=1:N
    p(i)=fobj(x(i,:));
    y(i,:)=x(i,:);
    
end
[fg,index]=min(p);
pg = x(index,:);             %PgΪȫ������

%------������Ҫѭ�������չ�ʽ���ε���------------

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