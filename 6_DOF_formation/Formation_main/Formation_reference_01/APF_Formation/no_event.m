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
% c=1;alpha=0.999999; % Removed: Event-triggering parameter
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

% % 李萨如编队信息
% % 仿真编队参数
% % r=2;w=0.314;d=3.5;
% % 实验编队参数
% % r=80;w=0.157;d=140;
% % r=60; d=105; w=0.15;
% % t=0:l:t_total;
% % for i=1:n
% %     h(4*i-3:4*i,:)=[r*sin(w*t)+d*cos(2*pi*(i-1)/n); w*r*cos(w*t); r*sin(2*w*t)+d*sin(2*pi*(i-1)/n); 2*w*r*cos(2*w*t)];
% % end

%圆形编队
r=2;w=0.314;
t=0:l:t_total; % t_total/l should match te_total if t is used for indexing h
% Ensure h has enough columns for h(:,k+1) in the loop
if length(t) < te_total + 1
    t_corrected_end = (te_total)*l; % k goes up to te_total, so h needs te_total+1 columns
    t = 0:l:t_corrected_end;
end
h=zeros(4*n,length(t)); % Preallocate h

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

%初始化x矩阵 %以及误差e矩阵 (e matrix for event triggering is removed)
x=zeros(4*n,te_total+1);
% e=zeros(4*n,te_total+1); % Removed: This 'e' was for event-triggering

% Corrected initialization: Should be outside the main simulation loop
x(:,1)=x0;
% e(:,1)=0*ones(4*n,1); % Removed: Initialization of event-triggering 'e'

for k=1:1:te_total
    % x(:,1)=x0; % Buggy: Removed from inside the loop
    % e(:,1)=0*ones(4*n,1); % Buggy: Removed from inside the loop

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
    min_d1=zeros(n,1); % Changed size from 4 to n for consistency
    a1=zeros(n,1);     % Changed size from 4 to n
    min_d2=zeros(n,1); % Changed size from 4 to n
    a2=zeros(n,1);     % Changed size from 4 to n
    min_d3=zeros(n,1); % Changed size from 4 to n
    a3=zeros(n,1);     % Changed size from 4 to n
    for i=1:1:n
        [min_d1(i),a1(i)]=min(d_obs1(i,:));
        [min_d2(i),a2(i)]=min(d_obs2(i,:));
        [min_d3(i),a3(i)]=min(d_obs3(i,:));
    end
    for i=1:1:n
        if min_d1(i)<d_safe && min_d1(i)~=0
            F_obs1(2*i-1:2*i,:)=(b/min_d1(i)-1/d_safe)^2*(b/(min_d1(i)^2))*[(x(4*i-3,k)-(p_obs1(1)+xo(1,a1(i))))/min_d1(i);(x(4*i-1,k)-(p_obs1(2)+yo(1,a1(i))))/min_d1(i)]; % Corrected xo/yo indexing
        end
        
        if min_d2(i)<d_safe && min_d2(i)~=0
            F_obs2(2*i-1:2*i,:)=(b/min_d2(i)-1/d_safe)^2*(b/(min_d2(i)^2))*[(x(4*i-3,k)-(p_obs2(1)+xo(1,a2(i))))/min_d2(i);(x(4*i-1,k)-(p_obs2(2)+yo(1,a2(i))))/min_d2(i)]; % Corrected xo/yo indexing
        end
        
        if min_d3(i)<d_safe && min_d3(i)~=0
            F_obs3(2*i-1:2*i,:)=(b/min_d3(i)-1/d_safe)^2*(b/(min_d3(i)^2))*[(x(4*i-3,k)-(p_obs3(1)+xo(1,a3(i))))/min_d3(i);(x(4*i-1,k)-(p_obs3(2)+yo(1,a3(i))))/min_d3(i)]; % Corrected xo/yo indexing
        end
    end
    
    % 各智能体之间的势场力
    % 各智能体间距离
    d_ij=zeros(n,n);
    F_ij=zeros(2*n,1);
    for i=1:1:n
        for j=1:1:n
            if i == j % Added to avoid self-influence if d_ij(i,i) is 0
                continue;
            end
            d_ij(i,j)=((x(4*i-3,k)-x(4*j-3,k))^2+(x(4*i-1,k)-x(4*j-1,k))^2)^0.5;
        end
    end
    for i=1:1:n
        for j=1:1:n
            if i == j % Added to avoid self-influence
                continue;
            end
            if d_ij(i,j)<=d0 && d_ij(i,j)~=0
                %F_ij(2*i-1:2*i,1)=F_ij(2*i-1:2*i,1)+exp(d_ij(i,j))/((exp(d_ij(i,j))-exp(d0))^2*d_ij(i,j))*[x(4*i-3,k)-x(4*j-3,k);x(4*i-1,k)-x(4*j-1,k)];
                F_ij(2*i-1:2*i,1)=F_ij(2*i-1:2*i,1)+(b/exp(d_ij(i,j))-1/exp(d0))^2*(b/(exp(d_ij(i,j))))*[(x(4*i-3,k)-x(4*j-3,k))/d_ij(i,j);(x(4*i-1,k)-x(4*j-1,k))/d_ij(i,j)];
            end
        end
    end
    
    % Modified state update: e(:,k) is replaced by x(:,k)
    x(:,k+1)=(kron(eye(n),A+B*K1)+kron(L,B*K))*x(:,k) ...
              -kron(eye(n),B*K1)*h(:,k) ...
              +kron(L,B*K)*(x(:,k)-h(:,k)) ... % MODIFIED: e(:,k) changed to x(:,k)
              +kron(eye(n),B)*kron(eye(2*n),[0 1])*((h(:,k+1)-h(:,k))/l) ...
              +K3*kron(eye(n),B)*(F_obs1+F_obs2+F_obs3) ...
              +K4*kron(eye(n),B)*F_ij;
              
    % e(:,k+1)=kron(eye(n),A^k)*(x0-h(:,1))-(x(:,k+1)-h(:,k+1)); % Removed: This 'e' was for event-triggering
end

% The entire second 'for' loop for event triggering is removed.
% Original start of removed event-triggering loop:
% t1=[];
% t2=[];
% t3=[];
% t4=[];
% for k=1:1:te_total
%    ... (event triggering logic) ...
% end

% Formation error calculation (this part remains relevant)
z=x'-h'; 
% Ensure z has the correct number of columns if te_total was adjusted for h
if size(z,1) > te_total +1
    z = z(1:te_total+1,:);
end
if size(z,2) < 4*n % Check if h was shorter than x
    warning('Mismatch in length of x and h trajectories for error calculation. Error might be incorrect.');
    % Pad z or adjust, for now, assume it's mostly fine for plotting existing data.
    % This can happen if t for h was not long enough.
    % For simplicity, we'll proceed, but this indicates h might not cover all x points.
end


z1=zeros(size(z,1),4); % Preallocate
z2=zeros(size(z,1),4);
z3=zeros(size(z,1),4);
z4=zeros(size(z,1),4);

if size(z,2) >= 4
    z1=z(:,1:4);
end
if size(z,2) >= 8
    z2=z(:,5:8);
end
if size(z,2) >= 12
    z3=z(:,9:12);
end
if size(z,2) >= 16
    z4=z(:,13:16);
end


ep1=sum(z1,2);
ep2=sum(z2,2);
ep3=sum(z3,2);
ep4=sum(z4,2);
ep=[ep1,ep2,ep3,ep4];
e_max=zeros(size(ep,1),1); % Preallocate
e_min=zeros(size(ep,1),1); % Preallocate
if ~isempty(ep)
    [e_max,~]=max(ep,[],2); % index1 not used later
    [e_min,~]=min(ep,[],2); % index2 not used later
end
error_formation=e_max-e_min; % Renamed from 'error' to avoid conflict with any 'error' function

% figure(1)
% rectangle('position', [1, 1, 0.5, 0.5],'Curvature',1,'FaceColor','k');%绘制障碍物,前面两个数是位置，后面两个数是大小
figure(1)
hold on
%sphere(50);%球由50*50个面组成
rectangle('position', [1.25, 1.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');%绘制障碍物,前面两个数是位置，后面两个数是大小
rectangle('position', [-1.75, -1.75, 0.5, 0.5],'Curvature',1,'FaceColor','k');
rectangle('position', [-1.55, 2.25, 0.5, 0.5],'Curvature',1,'FaceColor','k');
c1_surf_color=zeros(2,51,3);%获得o阵大小和x相同 % Renamed c1 to c1_surf_color to avoid conflict
for i=1:1:2
    for j_surf=1:1:51 % Renamed j to j_surf
        c1_surf_color(i,j_surf,1)=1;
        c1_surf_color(i,j_surf,2)=0;
        c1_surf_color(i,j_surf,3)=0;%红
    end
end
%%
plot_idx_end=te_total; % Use te_total for plotting up to the simulated time
z_plot_level=0.5*ones(1,plot_idx_end); % Renamed z to z_plot_level
z_1_plot_level=0.5*ones(1,5); % Renamed z_1 to z_1_plot_level

% Ensure indices for x do not exceed dimensions
last_valid_k = size(x,2);
if plot_idx_end > last_valid_k
    plot_idx_end = last_valid_k;
    z_plot_level=0.5*ones(1,plot_idx_end);
end
final_t_idx = plot_idx_end; % The last time index for plotting trajectories, usually te_total or te_total+1 for states

% Ensure x values at final_t_idx are valid
x_1_final=[x(1,final_t_idx) x(5,final_t_idx) x(9,final_t_idx) x(13,final_t_idx) x(1,final_t_idx)];
x_2_final=[x(3,final_t_idx) x(7,final_t_idx) x(11,final_t_idx) x(15,final_t_idx) x(3,final_t_idx)];

plot3(x(1,1:final_t_idx),x(3,1:final_t_idx),z_plot_level(1:final_t_idx),'m.',...
      x(5,1:final_t_idx),x(7,1:final_t_idx),z_plot_level(1:final_t_idx),'g.',...
      x(9,1:final_t_idx),x(11,1:final_t_idx),z_plot_level(1:final_t_idx),'r.',...
      x(13,1:final_t_idx),x(15,1:final_t_idx),z_plot_level(1:final_t_idx),'b.',...
      x_1_final,x_2_final,z_1_plot_level,'k--','markersize',10)

% Ensure h plotting indices match available h data length
h_len = size(h,2);
plot_h_idx_end = min(final_t_idx, h_len);
z_h_plot_level = 0.5*ones(1,plot_h_idx_end);


plot3(h(1,1:plot_h_idx_end),h(3,1:plot_h_idx_end),z_h_plot_level,'y--',...
      h(5,1:plot_h_idx_end),h(7,1:plot_h_idx_end),z_h_plot_level,'y--',...
      h(9,1:plot_h_idx_end),h(11,1:plot_h_idx_end),z_h_plot_level,'y--',...
      h(13,1:plot_h_idx_end),h(15,1:plot_h_idx_end),z_h_plot_level,'y--','LineWidth',1.2)

plot3(x(1,1),x(3,1),0.5,'mo','LineWidth',2,'markersize',15)
plot3(x(5,1),x(7,1),0.5,'go','LineWidth',2,'markersize',15)
plot3(x(9,1),x(11,1),0.5,'ro','LineWidth',2,'markersize',15)
plot3(x(13,1),x(15,1),0.5,'bo','LineWidth',2,'markersize',15)

plot3(x(1,final_t_idx),x(3,final_t_idx),0.5,'mP','LineWidth',2,'markersize',15)
plot3(x(5,final_t_idx),x(7,final_t_idx),0.5,'gP','LineWidth',2,'markersize',15)
plot3(x(9,final_t_idx),x(11,final_t_idx),0.5,'rP','LineWidth',2,'markersize',15)
plot3(x(13,final_t_idx),x(15,final_t_idx),0.5,'bP','LineWidth',2,'markersize',15)

surf(xo+p_obs1(1),yo+p_obs1(2),zo*0.6,c1_surf_color);%绘制半径为0.5的球
surf(xo+p_obs2(1),yo+p_obs2(2),zo*0.6,c1_surf_color);%绘制半径为0.5的球
surf(xo+p_obs3(1),yo+p_obs3(2),zo*0.6,c1_surf_color);%绘制半径为0.5的球
view(30,60)
grid on
grid minor
xlabel('X-axis/m');
ylabel('Y-axis/m');
zlabel('Z-axis/m');
legend('Agent1','Agent2','Agent3','Agent4');

% Figure 2 for triggering instants is removed as t1, t2, t3, t4 are no longer generated.
% figure(2)
% hold on
% one1=1*ones(1,length(t1));
% plot(t1,one1,'m.','markersize',10)
% one2=2*ones(1,length(t2));
% plot(t2,one2,'g.','markersize',10)
% one3=3*ones(1,length(t3));
% plot(t3,one3,'r.','markersize',10)
% one4=4*ones(1,length(t4));
% plot(t4,one4,'b.','markersize',10)
% grid on
% grid minor
% xlabel('iteration');
% ylabel('triggering instants of each agent')
% legend('Agent1','Agent2','Agent3','Agent4');

figure(3)
hold on
% Plotting formation error: Ensure t (time vector for x-axis) matches length of error_formation
time_axis_error = (0:length(error_formation)-1)*l; % Create time axis matching error_formation length
if length(time_axis_error) > length(error_formation)
    time_axis_error = time_axis_error(1:length(error_formation));
end

plot(time_axis_error,error_formation(:,1),'r-','LineWidth',2) % Assuming error_formation is a column vector
grid on
grid minor
xlabel('time(s)');
ylabel('formation error');