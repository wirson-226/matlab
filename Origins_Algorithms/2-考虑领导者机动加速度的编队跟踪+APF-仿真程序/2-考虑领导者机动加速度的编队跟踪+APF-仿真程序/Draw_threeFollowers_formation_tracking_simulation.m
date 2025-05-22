
% close all
tic

Drone0_xp=X(:,1);
Drone0_xv=X(:,2);
Drone0_yp=X(:,3);
Drone0_yv=X(:,4);

Drone1_xp=X(:,5);
Drone1_xv=X(:,6);
Drone1_yp=X(:,7);
Drone1_yv=X(:,8);

Drone2_xp=X(:,9);
Drone2_xv=X(:,10);
Drone2_yp=X(:,11);
Drone2_yv=X(:,12);

Drone3_xp=X(:,13);
Drone3_xv=X(:,14);
Drone3_yp=X(:,15);
Drone3_yv=X(:,16);
%% figure position x/t of the leader and the five followers
figure;
plot(t,Drone0_xp,'g','LineWidth',3.0); %green
hold on
plot(t,Drone1_xp,'r','LineWidth',1.2); %red
hold on
plot(t,Drone2_xp,'b','LineWidth',1.2); %blue
hold on
plot(t,Drone3_xp,'k','LineWidth',1.2); %black
hold on

xlabel('$t/s$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Position \ P_x/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Leader$','$Follower_1$','$Follower_2$','$Follower_3$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

%% figure position y/t of the leader and the five followers
figure;
plot(t,Drone0_yp,'g','LineWidth',3.0); %green
hold on
plot(t,Drone1_yp,'r','LineWidth',1.2); %red
hold on
plot(t,Drone2_yp,'b','LineWidth',1.2); %blue
hold on
plot(t,Drone3_yp,'k','LineWidth',1.2); %black
hold on

xlabel('$t/s$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Position \ P_y/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Leader$','$Follower_1$','$Follower_2$','$Follower_3$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

%% figure velocity v_x/t of the leader and the five followers
figure;
plot(t,Drone0_xv,'g','LineWidth',3.0); %green
hold on
plot(t,Drone1_xv,'r','LineWidth',1.2); %red
hold on
plot(t,Drone2_xv,'b','LineWidth',1.2); %blue
hold on
plot(t,Drone3_xv,'k','LineWidth',1.2); %black
hold on

xlabel('$t/s$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Velocity \ V_x/ m \dot s^{_1}/t$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Leader$','$Follower_1$','$Follower_2$','$Follower_3$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

%% figure velocity v_y/t of the leader and the five followers
figure;
plot(t,Drone0_yv,'g','LineWidth',3.0); %green
hold on
plot(t,Drone1_yv,'r','LineWidth',1.2); %red
hold on
plot(t,Drone2_yv,'b','LineWidth',1.2); %blue
hold on
plot(t,Drone3_yv,'k','LineWidth',1.2); %black
hold on

xlabel('$t/s$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Velocity \ V_y/ m s^{_1}/t$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Leader$','$Follower_1$','$Follower_2$','$Follower_3$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

%% figure 2D position x/y within the simulation period of the leader and five followers
figure;
plot(Drone0_xp,Drone0_yp,'g','LineWidth',3.0);
hold on
plot(Drone1_xp,Drone1_yp,'r','LineWidth',1.2);
hold on
plot(Drone2_xp,Drone2_yp,'b','LineWidth',1.2);
hold on
plot(Drone3_xp,Drone3_yp,'k','LineWidth',1.2);
hold on
plot(Drone0_xp(1),Drone0_yp(1),'d','MarkerSize',20,'MarkerEdgeColor','g');
hold on
plot(Drone1_xp(1),Drone1_yp(1),'d','MarkerSize',20,'MarkerEdgeColor','r');
hold on
plot(Drone2_xp(1),Drone2_yp(1),'d','MarkerSize',20,'MarkerEdgeColor','b');
hold on
plot(Drone3_xp(1),Drone3_yp(1),'d','MarkerSize',20,'MarkerEdgeColor','k');

% 障碍物可视化
obstacles = [0,  -0.9;
             -1.5, 0.1;
              -1.5, 0.3]';
obs_radius = 0.2;
for i = 1:size(obstacles,2)
    viscircles(obstacles(:,i)', obs_radius, 'Color', 'k', 'LineStyle', '--');
end

% axis([-9 7 -8 8]);
axis equal

xlabel('$Position \ P_x/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Position \ P_y/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Leader$','$Follower_1$','$Follower_2$','$Follower_3$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

%% figure 2D position x/y at the final time of the leader and five followers
figure
plot(Drone0_xp(end),Drone0_yp(end),'*','MarkerSize',10,'MarkerEdgeColor','g');
hold on
plot(Drone1_xp(end),Drone1_yp(end),'*','MarkerSize',10,'MarkerEdgeColor','r');
hold on
plot(Drone2_xp(end),Drone2_yp(end),'*','MarkerSize',10,'MarkerEdgeColor','b');
hold on
plot(Drone3_xp(end),Drone3_yp(end),'*','MarkerSize',10,'MarkerEdgeColor','k');
hold on
x_c1=[Drone1_xp(end),Drone1_xp(end),Drone3_xp(end)]';
y_c1=[Drone1_yp(end),Drone1_yp(end),Drone3_yp(end)]';
TR = triangulation([1,2,3],x_c1,y_c1); %表示成三角网格
[cc_c1,r_c1] = circumcenter(TR); 
deg=0:360;           %这是角度的取值，0~360间隔越小圆越平滑
rx_c1=cc_c1(1)+r_c1*cosd(deg);
ry_c1=cc_c1(2)+r_c1*sind(deg); %这三句根据圆心和半径生成圆的数据
plot(rx_c1,ry_c1,'r','LineWidth',1.5); %画出外接圆

hold on
cc_c2=[Drone0_xp(end),Drone0_yp(end)];
r_c2=norm(cc_c2-[Drone1_xp(end),Drone1_yp(end)]);
rx_c2=cc_c2(1)+r_c2*cosd(deg);
ry_c2=cc_c2(2)+r_c2*sind(deg); %这三句根据圆心和半径生成圆的数据
plot(rx_c2,ry_c2,'r','LineWidth',1.5); %画出外接圆
hold off;

% axis([-9 7 -8 8]);
axis equal

xlabel('$Position \ P_x/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Position \ P_y/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Leader$','$Follower_1$','$Follower_2$','$Follower_3$','$Follower_4$','$Follower_5$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

%% figure Alpha/t of the five followers
% figure;
% plot(t,Alpha_1,'g','LineWidth',1.2); %green
% hold on
% plot(t,Alpha_2,'r','LineWidth',1.2); %red
% hold on
% plot(t,Alpha_3,'b','LineWidth',1.2); %blue
% hold on
% plot(t,Alpha_4,'k','LineWidth',1.2); %black
% hold on
% plot(t,Alpha_5,'m','LineWidth',1.2); 
% hold on
% 
% xlabel('$t/s$','interpreter','latex','FontName','Times New Roman','FontSize',15);
% ylabel('$ \alpha /t$','interpreter','latex','FontName','Times New Roman','FontSize',15);
% h=legend('$Follower_1$','$Follower_2$','$Follower_3$','$Follower_4$','$Follower_5$');
% set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
% grid on

%% figure 3D position x/y of the leader and five followers
figure;
plot3(t,Drone0_xp,Drone0_yp,'g','LineWidth',3.0);
hold on
plot3(t,Drone1_xp,Drone1_yp,'r','LineWidth',1.2);
hold on
plot3(t,Drone2_xp,Drone2_yp,'b','LineWidth',1.2);
hold on
plot3(t,Drone3_xp,Drone3_yp,'k','LineWidth',1.2);
hold on

axis equal 
 
xlabel('$Time / s$','interpreter','latex','FontName','Times New Roman','FontSize',15);
ylabel('$Position x/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
zlabel('$Position y/ m$','interpreter','latex','FontName','Times New Roman','FontSize',15);
h=legend('$Drone0(pos_{x,y})$','$Drone1(pos_{x,y})$','$Drone2(pos_{x,y})$','$Drone3(pos_{x,y})$');
set(h,'interpreter','latex','FontName','Times New Roman','FontSize',15);
grid on

toc