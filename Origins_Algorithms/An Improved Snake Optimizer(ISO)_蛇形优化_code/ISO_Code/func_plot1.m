% This function draw the benchmark functions

function func_plot(func_name)

[lb,ub,dim,fobj]=Get_Functions_details(func_name);

switch func_name 
       case 'F1' 
        x=-100:2:100; y=x; %[-100,100]
        
    case 'F2' 
        x=-10:2:10; y=x; %[-10,10]
        
    case 'F3' 
        x=-100:2:100; y=x; %[-100,100]
        
    case 'F4' 
        x=-100:2:100; y=x; %[-100,100]
    case 'F5' 
        x=-30:2:30; y=x; %[-5,5]
    case 'F6' 
        x=-100:2:100; y=x; %[-100,100]
    case 'F7' 
        x=-1.28:0.03:1.28;  y=x;  %[-1,1]
    case 'F8' 
        x=-500:10:500;y=x; %[-500,500]
    case 'F9' 
        x=-5.12:0.1:5.12;   y=x; %[-5,5]    
    case 'F10' 
        x=-32:0.5:32; y=x;%[-500,500]
    case 'F11' 
        x=-600:10:600; y=x;%[-0.5,0.5]
    case 'F12' 
        x=-50:1:50; y=x;%[-pi,pi]
    case 'F13' 
        x=-50:0.8:50; y=x;%[-3,1]
    case 'F14' 
        x=-65:2:65; y=x;%[-100,100]
    case 'F15' 
        x=-5:0.1:5; y=x;%[-5,5]
    case 'F16' 
        x=-5:0.01:5; y=x;%[-5,5]
    case 'F17' 
        x=-5:0.1:5; y=x;%[-5,5]
    case 'F18' 
        x=-2:0.06:2; y=x;%[-5,5]
    case 'F19' 
        x=0:0.1:1; y=x;%[-5,5]
    case 'F20' 
        x=0:0.1:1; y=x;%[-5,5]        
    case 'F21' 
        x=0:0.1:10; y=x;%[-5,5]
    case 'F22' 
        x=0:0.1:10; y=x;%[-5,5]     
    case 'F23' 
        x=0:0.1:10; y=x;%[-5,5]  
end    

    

L=length(x);
f=[];

for i=1:L
    for j=1:L
        if strcmp(func_name,'F15')==0 && strcmp(func_name,'F19')==0 && strcmp(func_name,'F20')==0 && strcmp(func_name,'F21')==0 && strcmp(func_name,'F22')==0 && strcmp(func_name,'F23')==0
            f(i,j)=fobj([x(i),y(j)]);
        end
        if strcmp(func_name,'F15')==1
            f(i,j)=fobj([x(i),y(j),0,0]);
        end
        if strcmp(func_name,'F19')==1
            f(i,j)=fobj([x(i),y(j),0]);
        end
        if strcmp(func_name,'F20')==1
            f(i,j)=fobj([x(i),y(j),0,0,0,0]);
        end       
        if strcmp(func_name,'F21')==1 || strcmp(func_name,'F22')==1 ||strcmp(func_name,'F23')==1
            f(i,j)=fobj([x(i),y(j),0,0]);
        end          
    end
end

contour(x, y, f);
colormap(jet);
colorbar;  
end

