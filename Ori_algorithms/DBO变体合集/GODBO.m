% pop是种群规模，M是最大迭代次数，fobj是适应度函数
% pNum：滚球蜣螂的数量   
% 需要注意的是种群的大小以及各个子种群的大小都是可以根据实际问题自行调整的
function [fMin,bestX,Convergence_curve]=GODBO(pop,Max_FEs,c,d,dim,fobj)
    P_percent=0.2; % 滚球蜣螂所占整个种群的比例
    pNum=round(pop*P_percent);    % 滚球蜣螂的数量  
    lb=c.*ones(1,dim);% 下边界 a vector
    ub=d.*ones(1,dim);% 上边界 a vector
    Convergence_curve=[];
    % Initialization
    for i=1:pop
        x(i,:)=lb+(ub-lb).*rand(1,dim);  
        fit(i)=fobj(x(i,:));                       
    end
    pFit=fit;                       
    pX=x; 
    XX=pX;    
    [fMin,bestI]=min(fit); % fMin denotes the global optimum fitness value
    bestX=x(bestI,:);      % bestX denotes the global optimum position corresponding to fMin
    % Start updating the solutions.
    t=1;
    while t<Max_FEs    
        [fmax,B]=max(fit);
        worse=x(B,:); % 全局最差位置  
        r2=rand(1);
        for i=1:pNum    
            if(r2<0.9)
                a=rand(1,1);
                if (a>0.1)
                    a=1;
                else
                    a=-1;
                end
                x(i,:)=pX(i,:)+0.8*abs(pX(i,:)-worse)+a*0.1*(XX(i,:)); % Equation (1)
            else
                aaa=randperm(180,1);
                if(aaa==0||aaa==90||aaa==180)
                    x(i,:)=pX(i,:);   
                end
                theta=aaa*pi/180;
                x(i,:)=pX(i,:)+tan(theta).*abs(pX(i,:)-XX(i,:));    % Equation (2) 
            end
            % -----------------------------------------------------------------
            % OPPOSITION-BASED LEARNING
            for j=1:dim
                x_o(i,j)=lb(j)+ub(j)-x(i,j);
                if x_o(i,j)<x(i,j)
                    x(i,j)=x_o(i,j);
                else
                    x(i,j)=x(i,j);
                end
            end
            % -----------------------------------------------------------------
            x(i,:)=Bounds(x(i,:),lb,ub);
            fit(i)=fobj(x(i,:));
            % FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            % Convergence_curve(FES) = fMin;
            % if FES>=Max_FEs
            %     return;
            % end

        end 
        [fMMin,bestII]=min(fit);   
        bestXX=x(bestII,:);% 当前最优解
        R=1-t/Max_FEs; 
        Xnew1=bestXX.*(1-R); 
        Xnew2=bestXX.*(1+R);                    % Equation (3)
        Xnew1=Bounds(Xnew1,lb,ub);   
        Xnew2=Bounds(Xnew2,lb,ub);
  
        Xnew11=bestX.*(1-R); 
        Xnew22=bestX.*(1+R);                     % Equation (5)
        Xnew11=Bounds(Xnew11,lb,ub);
        Xnew22=Bounds(Xnew22,lb,ub);
        for i=(pNum+1):12                  % Equation (4)
            x(i,:)=bestXX+((rand(1,dim)).*(pX(i,:)-Xnew1)+(rand(1,dim)).*(pX(i,:)-Xnew2));
            x(i,:)=Bounds(x(i,:),Xnew1,Xnew2 );
            fit(i )=fobj(x(i,:));
            % FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            % Convergence_curve(FES) = fMin;
            % if FES>=Max_FEs
            %     return;
            % end
        end
        for i=13:19                  % Equation (6)
            % x(i,:)=pX(i,:)+((randn(1)).*(pX(i,:)-Xnew11)+((rand(1,dim)).*(pX(i,:)-Xnew22)));
            % -----------------------------------------------------------------------------
            % 最优值引导策略
            lbd=0.25;
            x(i,:)=pX(i,:)+((randn(1)).*(pX(i,:)-Xnew11)+((rand(1,dim)).*(pX(i,:)-Xnew22)))+lbd.*(bestXX-pX(i,:));
            % -----------------------------------------------------------------------------
            x(i,:)=Bounds(x(i,:),lb,ub); 
            fit(i)=fobj(x(i,:));
        end 
        for j=20:pop                 % Equation (7)
           x(j,:)=bestX+randn(1,dim).*((abs((pX(j,:)-bestXX)))+(abs((pX(j,:)-bestX))))./2;
           x(j,:)=Bounds(x(j,:),lb,ub);
           fit(j)=fobj(x(j,:));
           % FES = FES + 1;  % 每调用一次目标函数，增加评价计数
           %  Convergence_curve(FES) = fMin;
           %  if FES>=Max_FEs
           %      return;
           %  end
        end
        % Update the individual's best fitness vlaue and the global best fitness value
        XX=pX;
        for i=1:pop 
            if(fit(i)<pFit(i))
                pFit(i)=fit(i);
                pX(i,:)=x(i,:);
            end
            if(pFit(i)<fMin)
                fMin=pFit(i);
                bestX=pX(i,:);
            end
        end
        Convergence_curve(t)=fMin;
        t= t+1 ;
    end
end

% Application of simple limits/bounds
function s=Bounds(s,Lb,Ub)
  % Apply the lower bound vector
  temp=s;
  I=temp<Lb;
  temp(I)=Lb(I);
  % Apply the upper bound vector 
  J=temp>Ub;
  temp(J)=Ub(J);
  % Update this new move 
  s=temp;
end