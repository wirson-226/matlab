clc
clear
close all

n=4;
l=0.1;
K1=kron(eye(2),[-2,-4]);
A=kron(eye(2),[1,l;0,1]);
B=kron(eye(2),[0;l]);
gamma=0.95;
theta=0.95;
P=dare(A+B*K1,B,eye(4),eye(2));
K=-inv(B'*P*B+eye(2))*B'*P*(A+B*K1);
L=[1 0 0 -1;
    -1 1 0 0;
    0 -1 1 0;
    0 0 -1 1];
c=1;alpha=0.999999;
K3=0.003; K4=0.003;
% K3=0.03; K4=0.03;
%K3=0;
te_total=600; %*0.1s
t_total=60; %/s
eig(A+B*K1);

% 仿真初始位置
x0=[2;0;0;2;0;0.8;-2;3;-2;1.5;0;1;3.5;-2.4;-2;0.5];

% 实验初始位置
% x0=[120;0;90;2;-30;0.8;0;3;-135;1.5;60;1;105;-2.4;-60;0.5];
% x0=[23.2;-1.2;6.5;0.5;-41.9;0.3;63.0;-1;-162.2;-2.5;-2.6;-0.3;-37.2;0.7;-78.2;2.8];
% 李萨如编队信息
% 仿真编队参数
% r=2;w=0.314;d=3.5;
% 实验编队参数
% r=80;w=0.157;d=140;
% r=60; d=105; w=0.15;
% t=0:l:t_total;
% for i=1:n
%     h(4*i-3:4*i,:)=[r*sin(w*t)+d*cos(2*pi*(i-1)/n); w*r*cos(w*t); r*sin(2*w*t)+d*sin(2*pi*(i-1)/n); 2*w*r*cos(2*w*t)];
% end

% %圆形编队
r=2;w=0.314;
t=0:l:t_total;
for i=1:n
    h(4*i-3:4*i,:)=[r*cos(w*t+2*pi*(i-1)/n);-w*r*sin(w*t+2*pi*(i-1)/n);r*sin(w*t+2*pi*(i-1)/n);w*r*cos(w*t+2*pi*(i-1)/n)];
end

% 仿真障碍物数据
[xo,yo,zo]=cylinder(0.25,50);
p_obs1=[1.5,1.5];
p_obs2=[-1.5,-1.5];
p_obs3=[-1.3,2.5];
d_safe=0.8;
d0=1;
b=4;

% 实验障碍物数据
% [xo,yo,zo]=cylinder(10,50);
% p_obs1=[145,40];
% p_obs2=[-40,-160];
% p_obs3=[-60,100];
% d_safe=1000;
% d0=1000;
% b=200;
%初始化x矩阵以及误差e矩阵
x=zeros(4*n,te_total+1);
e=zeros(4*n,te_total+1);

for k=1:1:te_total
    x(:,1)=x0;
    e(:,1)=0*ones(4*n,1);
    %各智能体与障碍物1之间的势场力
    d_obs1=zeros(n,51);
    d_obs2=zeros(n,51);
    d_obs3=zeros(n,51);
    for i=1:1:n
        d_obs1(i,:)=((x(4*i-3,k)*ones(1,51)-(p_obs1(1)*ones(1,51)+xo(1,:))).^2+(x(4*i-1,k)*ones(1,51)-(p_obs1(2)*ones(1,51)+yo(1,:))).^2).^0.5;
        d_obs2(i,:)=((x(4*i-3,k)*ones(1,51)-(p_obs2(1)*ones(1,51)+xo(1,:))).^2+(x(4*i-1,k)*ones(1,51)-(p_obs2(2)*ones(1,51)+yo(1,:))).^2).^0.5;
        d_obs3(i,:)=((x(4*i-3,k)*ones(1,51)-(p_obs3(1)*ones(1,51)+xo(1,:))).^2+(x(4*i-1,k)*ones(1,51)-(p_obs3(2)*ones(1,51)+yo(1,:))).^2).^0.5;
    end
    %初始化各智能体与障碍物之间的势场力矩阵
    F_obs1=zeros(2*n,1);
    F_obs2=zeros(2*n,1);
    F_obs3=zeros(2*n,1);
    min_d1=zeros(4,1);
    a1=zeros(4,1);
    min_d2=zeros(4,1);
    a2=zeros(4,1);
    min_d3=zeros(4,1);
    a3=zeros(4,1);
    for i=1:1:n
        [min_d1(i),a1(i)]=min(d_obs1(i,:));
        [min_d2(i),a2(i)]=min(d_obs2(i,:));
        [min_d3(i),a3(i)]=min(d_obs3(i,:));
    end
    for i=1:1:n
        if min_d1(i)<d_safe && min_d1(i)~=0
            F_obs1(2*i-1:2*i,:)=(b/min_d1(i)-1/d_safe)^2*(b/(min_d1(i)^2))*[(x(4*i-3,k)-(p_obs1(1)+xo(a1(i))))/min_d1(i);(x(4*i-1,k)-(p_obs1(2)+yo(a1(i))))/min_d1(i)];
        end
        
        if min_d2(i)<d_safe && min_d2(i)~=0
            F_obs2(2*i-1:2*i,:)=(b/min_d2(i)-1/d_safe)^2*(b/(min_d2(i)^2))*[(x(4*i-3,k)-(p_obs2(1)+xo(a2(i))))/min_d2(i);(x(4*i-1,k)-(p_obs2(2)+yo(a2(i))))/min_d2(i)];
        end
        
        if min_d3(i)<d_safe && min_d3(i)~=0
            F_obs3(2*i-1:2*i,:)=(b/min_d3(i)-1/d_safe)^2*(b/(min_d3(i)^2))*[(x(4*i-3,k)-(p_obs3(1)+xo(a3(i))))/min_d3(i);(x(4*i-1,k)-(p_obs3(2)+yo(a3(i))))/min_d3(i)];
        end
    end
    
    % 各智能体之间的势场力
    % 各智能体间距离
    d_ij=zeros(n,n);
    F_ij=zeros(2*n,1);
    for i=1:1:n
        for j=1:1:n
            d_ij(i,j)=((x(4*i-3,k)-x(4*j-3,k))^2+(x(4*i-1,k)-x(4*j-1,k))^2)^0.5;
        end
    end
    for i=1:1:n
        for j=1:1:n
            if d_ij(i,j)<=d0 && d_ij(i,j)~=0
                %F_ij(2*i-1:2*i,1)=F_ij(2*i-1:2*i,1)+exp(d_ij(i,j))/((exp(d_ij(i,j))-exp(d0))^2*d_ij(i,j))*[x(4*i-3,k)-x(4*j-3,k);x(4*i-1,k)-x(4*j-1,k)];
                F_ij(2*i-1:2*i,1)=F_ij(2*i-1:2*i,1)+(b/exp(d_ij(i,j))-1/exp(d0))^2*(b/(exp(d_ij(i,j))))*[(x(4*i-3,k)-x(4*j-3,k))/d_ij(i,j);(x(4*i-1,k)-x(4*j-1,k))/d_ij(i,j)];
            end
        end
    end
    x(:,k+1)=(kron(eye(n),A+B*K1)+kron(L,B*K))*x(:,k)-kron(eye(n),B*K1)*h(:,k)+kron(L,B*K)*(e(:,k)-h(:,k))+kron(eye(n),B)*kron(eye(2*n),[0 1])*((h(:,k+1)-h(:,k))/l)+K3*kron(eye(n),B)*(F_obs1+F_obs2+F_obs3)+K4*kron(eye(n),B)*F_ij;
    e(:,k+1)=kron(eye(n),A^k)*(x0-h(:,1))-(x(:,k+1)-h(:,k+1));
end

t1=[];
t2=[];
t3=[];
t4=[];

for k=1:1:te_total
    f1=norm(e(1:4,k))-c*power(alpha,k-1);
    f2=norm(e(5:8,k))-c*alpha^(k-1);
    f3=norm(e(9:12,k))-c*power(alpha,k-1);
    f4=norm(e(13:16,k))-c*power(alpha,k-1);

    if f1>0||f1==0
        e(1:4,k)=zeros(4,1);
        l1=numel(t1);
        trigger=k;
        t1(l1+1)=trigger;
    end
    
    if f2>0||f2==0
        e(5:8,k)=zeros(4,1);
        l2=numel(t2);
        trigger=k;
        t2(l2+1)=trigger;
    end
    
    if f3>0||f3==0
        e(9:12,k)=zeros(4,1);
        l3=numel(t3);
        trigger=k;
        t3(l3+1)=trigger;
    end
    
    if f4>0||f4==0
        e(13:16,k)=zeros(4,1);
        l4=numel(t4);
        trigger=k;
        t4(l4+1)=trigger;
    end
    
    if f1>0||f1==0||f2==0||f2>0||f3==0||f3>0||f4==0||f4>0
        for i=trigger:1:te_total
            %各智能体与障碍物1之间的势场力
            d_obs1=zeros(n,51);
            d_obs2=zeros(n,51);
            d_obs3=zeros(n,51);
            for j=1:1:n
                d_obs1(j,:)=((x(4*j-3,i)*ones(1,51)-(p_obs1(1)*ones(1,51)+xo(1,:))).^2+(x(4*j-1,i)*ones(1,51)-(p_obs1(2)*ones(1,51)+yo(1,:))).^2).^0.5;
                d_obs2(j,:)=((x(4*j-3,i)*ones(1,51)-(p_obs2(1)*ones(1,51)+xo(1,:))).^2+(x(4*j-1,i)*ones(1,51)-(p_obs2(2)*ones(1,51)+yo(1,:))).^2).^0.5;
                d_obs3(j,:)=((x(4*j-3,i)*ones(1,51)-(p_obs3(1)*ones(1,51)+xo(1,:))).^2+(x(4*j-1,i)*ones(1,51)-(p_obs3(2)*ones(1,51)+yo(1,:))).^2).^0.5;
            end
            %初始化各智能体与障碍物之间的势场力矩阵
            F_obs1=zeros(2*n,1);
            F_obs2=zeros(2*n,1);
            F_obs3=zeros(2*n,1);
            min_d1=zeros(4,1);
            a1=zeros(4,1);
            min_d2=zeros(4,1);
            a2=zeros(4,1);
            min_d3=zeros(4,1);
            a3=zeros(4,1);
            for j=1:1:n
                [min_d1(j),a1(j)]=min(d_obs1(j,:));
                [min_d2(j),a2(j)]=min(d_obs2(j,:));
                [min_d3(j),a3(j)]=min(d_obs3(j,:));
            end
            for j=1:1:n
                if min_d1(j)<d_safe||min_d1(j)==d_safe
                    F_obs1(2*j-1:2*j,:)=(b/min_d1(j)-1/d_safe)^2*(b/(min_d1(j)^2))*[(x(4*j-3,i)-(p_obs1(1)+xo(a1(j))))/min_d1(j);(x(4*j-1,i)-(p_obs1(2)+yo(a1(j))))/min_d1(j)];
                end
                
                if min_d2(j)<d_safe||min_d2(j)==d_safe
                    F_obs2(2*j-1:2*j,:)=(b/min_d2(j)-1/d_safe)^2*(b/(min_d2(j)^2))*[(x(4*j-3,i)-(p_obs2(1)+xo(a2(j))))/min_d2(j);(x(4*j-1,i)-(p_obs2(2)+yo(a2(j))))/min_d2(j)];
                end
                
                if min_d3(j)<d_safe||min_d3(j)==d_safe
                    F_obs3(2*j-1:2*j,:)=(b/min_d3(j)-1/d_safe)^2*(b/(min_d3(j)^2))*[(x(4*j-3,i)-(p_obs3(1)+xo(a3(j))))/min_d3(j);(x(4*j-1,i)-(p_obs3(2)+yo(a3(j))))/min_d3(j)];
                end
            end
            
             % 各智能体之间的势场力
             % 各智能体间距离
             d_ij=zeros(n,n);
             F_ij=zeros(2*n,1);
             for I=1:1:n
                 for j=1:1:n
                     d_ij(I,j)=((x(4*I-3,i)-x(4*j-3,i))^2+(x(4*I-1,i)-x(4*j-1,i))^2)^0.5;
                 end
             end
             for I=1:1:n
                 for j=1:1:n
                     if d_ij(I,j)<=d0 && d_ij(I,j)~=0
                         %F_ij(2*I-1:2*I,1)=F_ij(2*I-1:2*I,1)+exp(d_ij(I,j))/((exp(d_ij(I,j))-exp(d0))^2*d_ij(I,j))*[x(4*I-3,i)-x(4*j-3,i);x(4*I-1,i)-x(4*j-1,i)];
                         F_ij(2*I-1:2*I,1)=F_ij(2*I-1:2*I,1)+(b/exp(d_ij(I,j))-1/exp(d0))^2*(b/(exp(d_ij(I,j))))*[(x(4*I-3,i)-x(4*j-3,i))/d_ij(I,j);(x(4*I-1,i)-x(4*j-1,i))/d_ij(I,j)];
                     end
                 end
             end
            
           x(:,i+1)=(kron(eye(n),A+B*K1)+kron(L,B*K))*x(:,i)-kron(eye(n),B*K1)*h(:,i)+kron(L,B*K)*(e(:,i)-h(:,i))+kron(eye(n),B)*kron(eye(2*n),[0 1])*((h(:,i+1)-h(:,i))/l)+K3*kron(eye(n),B)*(F_obs1+F_obs2+F_obs3)+K4*kron(eye(n),B)*F_ij;
            if f1>0||f1==0
                e(1:4,i+1)=A^(i-trigger)*(x(1:4,trigger)-h(1:4,trigger))-(x(1:4,i+1)-h(1:4,i+1));
            end
            if f2>0||f2==0
                e(5:8,i+1)=A^(i-trigger)*(x(5:8,trigger)-h(5:8,trigger))-(x(5:8,i+1)-h(5:8,i+1));
            end
            if f3>0||f3==0
                e(9:12,i+1)=A^(i-trigger)*(x(9:12,trigger)-h(9:12,trigger))-(x(9:12,i+1)-h(9:12,i+1));
            end
            if f4>0||f4==0
                e(13:16,i+1)=A^(i-trigger)*(x(13:16,trigger)-h(13:16,trigger))-(x(13:16,i+1)-h(13:16,i+1));
            end
        end
    end
end

z=x'-h';
for i=1:1:4
    z1=z(:,1:4);
    z2=z(:,5:8);
    z3=z(:,9:12);
    z4=z(:,13:16);
end


ep1=sum(z1,2);
ep2=sum(z2,2);
ep3=sum(z3,2);
ep4=sum(z4,2);

ep=[ep1,ep2,ep3,ep4];
[e_max,index1]=max(ep,[],2);
[e_min,index2]=min(ep,[],2);
error=e_max-e_min;

% figure(1)
% rectangle('position', [1, 1, 0.5, 0.5],'Curvature',1,'FaceColor','k');%绘制障碍物,前面两个数是位置，后面两个数是大小


figure(1)
hold on

%sphere(50);%球由50*50个面组成
rectangle('position', [1.25, 1.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');%绘制障碍物,前面两个数是位置，后面两个数是大小
rectangle('position', [-1.75, -1.75, 0.5, 0.5],'Curvature',1,'FaceColor','k');
rectangle('position', [-1.55, 2.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');

c1=zeros(2,51,3);%获得o阵大小和x相同

for i=1:1:2
    for j=1:1:51
        c1(i,j,1)=1;
        c1(i,j,2)=0;
        c1(i,j,3)=0;%红
    end
end
%%
i=te_total;
z=0.5*ones(1,te_total);
z_1=0.5*ones(1,5);
x_1=[x(1,i) x(5,i) x(9,i) x(13,i) x(1,i)];
x_2=[x(3,i) x(7,i) x(11,i) x(15,i) x(3,i)];
plot3(x(1,1:i),x(3,1:i),z,'m.',x(5,1:i),x(7,1:i),z,'g.',x(9,1:i),x(11,1:i),z,'r.',x(13,1:i),x(15,1:i),z,'b.',x_1,x_2,z_1,'k--','markersize',10)
plot3(h(1,1:i),h(3,1:i),z,'y--',h(5,1:i),h(7,1:i),z,'y--',h(9,1:i),h(11,1:i),z,'y--',h(13,1:i),h(15,1:i),z,'y--','LineWidth',1.2)
plot3(x(1,1),x(3,1),0.5,'mo','LineWidth',2,'markersize',15)
plot3(x(5,1),x(7,1),0.5,'go','LineWidth',2,'markersize',15)
plot3(x(9,1),x(11,1),0.5,'ro','LineWidth',2,'markersize',15)
plot3(x(13,1),x(15,1),0.5,'bo','LineWidth',2,'markersize',15)
plot3(x(1,i),x(3,i),0.5,'mP','LineWidth',2,'markersize',15)
plot3(x(5,i),x(7,i),0.5,'gP','LineWidth',2,'markersize',15)
plot3(x(9,i),x(11,i),0.5,'rP','LineWidth',2,'markersize',15)
plot3(x(13,i),x(15,i),0.5,'bP','LineWidth',2,'markersize',15)
surf(xo+p_obs1(1),yo+p_obs1(2),zo*0.6,c1);%绘制半径为0.5的球
surf(xo+p_obs2(1),yo+p_obs2(2),zo*0.6,c1);%绘制半径为0.5的球
surf(xo+p_obs3(1),yo+p_obs3(2),zo*0.6,c1);%绘制半径为0.5的球
view(30,60)
grid on
grid minor
xlabel('X-axis/m');
ylabel('Y-axis/m');
zlabel('Z-axis/m');
legend('Agent1','Agent2','Agent3','Agent4');

figure(2)
hold on
one1=1*ones(1,length(t1));
plot(t1,one1,'m.','markersize',10)
one2=2*ones(1,length(t2));
plot(t2,one2,'g.','markersize',10)
one3=3*ones(1,length(t3));
plot(t3,one3,'r.','markersize',10)
one4=4*ones(1,length(t4));
plot(t4,one4,'b.','markersize',10)
grid on
grid minor
xlabel('iteration');
ylabel('triggering instants of each agent')
legend('Agent1','Agent2','Agent3','Agent4');

figure(3)
hold on
plot(t,error(:,1),'r-','LineWidth',2)
grid on
grid minor
xlabel('time(s)');
ylabel('formation error');