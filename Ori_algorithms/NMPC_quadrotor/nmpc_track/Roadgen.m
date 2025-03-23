function Roadgen
fig = figure('UserData',[]);
th = title('选点时，鼠标左键选点，鼠标右键选点结束');
hold on;
axis equal;
axis([0,100,0,100]);
grid minor;
data.fig = fig;
data.ax  = gca;
data.th  = th;
uicontrol('Style','pushbutton','String','选点','Position',[20,50,60,20],'Callback',@select_points, 'userdata', data);
uicontrol('Style','pushbutton','String','生成道路数据','Position',[20,20,60,20],'Callback',@gen_road,'userdata', data);
end

function select_points(obj, event)
    cla;
    hold on;
    button = 1;
    i = 1;
    n = 1;
    while button(i) == 1
       [x,y, button] = ginput(1);
       x_n(n) = x; % save all points you continue getting
       y_n(n) = y;
       plot(x,y,'ro')
       drawnow
       n=n+1;
    end
    data = get(obj,'userdata');
    fdata = get(data.fig, 'userdata');
    points = [x_n;y_n]';
    fdata.points = points;
    set(data.fig, 'userdata',fdata);
    plot(points(:,1),points(:,2),'marker', 's');
    axis equal;
    grid minor;
end

function gen_road(obj, event)
    data = get(obj,'userdata');
    fdata = get(data.fig, 'userdata');
    if ~isempty(fdata)
        ref = referencePathFrenet(fdata.points);
        p = ref.interpolate(100000000);
        points = ref.interpolate(0:1:p(6));
        plot(points(:,1),points(:,2),'color','r','marker', 'o','MarkerFaceColor','r');
        l = 2.61;
        sth = rad2deg(points(1,3));
        sx  =  points(1,1) + l*cos(points(1,3));
        sy  =  points(1,2) + l*sin(points(1,3));
        set(data.th, 'string', ['carsim内初始位置是',mat2str([sx,sy,sth])]);
        waypoints = points(:,1:2);
        save("pathpoint.mat","waypoints");
        axis equal;
        grid minor;
    end
end