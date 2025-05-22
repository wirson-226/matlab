function hDer = calcul_singledrone_hDer(w,r,time)
    t=time;
    hDer=zeros(4,1);
    hDer=[-w*r*sin(w*t);
        -w*w*r*cos(w*t);
           w*r*cos(w*t);
        -w*w*r*sin(w*t)];
    end 