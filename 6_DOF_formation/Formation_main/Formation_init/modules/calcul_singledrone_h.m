%% 轨迹生成函数
function h = calcul_singledrone_h(w,r,time)
    t=time;
    h=zeros(4,1);
    h=[r*cos(w*t);
       -w*r*sin(w*t);
       r*sin(w*t);
       w*r*cos(w*t)];
end

