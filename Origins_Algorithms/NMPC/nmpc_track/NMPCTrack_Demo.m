% 加载路径
load pathpoint.mat;
refPath     = referencePathFrenet(waypoints); 
% 第一个点
startp      = refPath.interpolate(0);
% 最后一个点
endp        = refPath.interpolate(realmax);
maxdist     = endp(6);
% 待跟踪路径
pp          = refPath.interpolate(0:0.1:maxdist);
xs          = pp(:,1)';
ys          = pp(:,2)';
ths         = pp(:,3)';
dt          = 0.1;

% 车辆初始状态
L = 2.4;
drivedist   = 0;
vehx    = startp(1);
vehy    = startp(2);
vehth   = startp(3);
vehspd  = 2;

actxs  = vehx;
actys  = vehy;
actths = vehth;
for i = 1:1000
    disp(['loop ', num2str(i)])
    if drivedist > maxdist
        break;
    end
    % 计算前馈转角
    curvaturep = refPath.interpolate(drivedist);
    curvature  = curvaturep(4);
    disp(curvature)
    ff = atan(curvature*L);
    
    % NMPC跟踪
    p = 10;
    ds = 0.1;
    refdata = refPath.interpolate(drivedist + (1:p)*ds);
    refxs  = refdata(:,1);
    refys  = refdata(:,2);
    refths = refdata(:,3); 
    steer = NMPCTrack(vehx, vehy, vehth, refxs, refys, refths, ff, ds, 10);
    % 运动学模型仿真
    ds        = dt*vehspd;
    drivedist = drivedist + ds;
    vehx      = vehx  + ds*cos(vehth);
    vehy      = vehy  + ds*sin(vehth);
    vehth     = vehth + ds*tan(steer)/L;
    actxs     = [actxs, vehx];%#ok
    actys     = [actys, vehy];%#ok
    actths    = [actths, vehth];%#ok
end

%% 绘图
veh_h = 5;
veh_w = 1.8;

left  = refPath.frenet2global((0:1:maxdist)'*[1, 0, 0, 0, 0, 0]+[0,0,0, 2,0,0]);
right = refPath.frenet2global((0:1:maxdist)'*[1, 0, 0, 0, 0, 0]+[0,0,0,-2,0,0]);
hold on;
plot(actxs, actys, 'LineWidth',2);
plot(xs, ys);
plot(left(:,1),left(:,2),'k');
plot(right(:,1),right(:,2),'k');
legend(gca, {'实际轨迹','期望轨迹'});
axis equal;