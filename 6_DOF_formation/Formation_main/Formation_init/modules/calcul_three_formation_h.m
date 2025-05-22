
function h = calcul_three_formation_h(w,r,time)
    t=time;
    h=zeros(12,1);
    for i=1:3
        h((4*(i-1)+1):4*i,:)=[r*cos(w*t+2*(i-1)*pi/3);
                               -w*r*sin(w*t+2*(i-1)*pi/3);
                               r*sin(w*t+2*(i-1)*pi/3);
                               w*r*cos(w*t+2*(i-1)*pi/3)];
    end
end