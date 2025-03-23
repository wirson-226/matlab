%% beta分布 raw
% a=[0.5,5,1,2,2];
% b=[0.5,1,3,2,5];
% x=0:0.1:1;
% syms t;
% hold on;
% for i=1:5
%     ft=t^(a(i)-1)*(1-t)^(b(i)-1);
%     B=int(ft,t,0,1);
%     fx=(1/B).*(x.^(a(i)-1)).*(1-x).^(b(i)-1);
%     plot(x,fx,'LineWidth',2);
% end
% axis tight % 按紧凑方式显示坐标轴范围，即坐标轴范围为绘图数据范围。
% legend('a=0.5,b=0.5','a=5,b=1','a=1,b=3','a=2,b=2','a=2,b=5','Location', 'Best');
% ylim([0 2.5])
% xlabel('x');
% ylabel('PDF');

%% beta分布 api
% x=0:0.1:1;
% pdf_beta = betapdf(x, 0.5, 0.5);
% hold on;
% xlabel('x');
% ylabel('PDF');
% plot(x, pdf_beta, 'r', 'LineWidth', 2);

%% MDBO
% -----------------------------------------------------------------------------------------------------------
function [fMin,bestX,Convergence_curve]=MDBO(pop,Max_FEs,c,d,dim,fobj)
    % The population size of producers accounts for "P_percent" percent of the total population size
    P_percent = 0.2;
     Convergence_curve=[];
    % The population size of the producers
    pNum=round(pop*P_percent);       
    lb=c.*ones(1,dim);    % Lower limit/bounds/     a vector
    ub=d.*ones(1,dim);    % Upper limit/bounds/     a vector
    %Initialization
    for i=1:pop
        x(i,:)=lb+(ub-lb).*rand(1,dim);
        fit(i)=fobj(x(i,:));                 
    end
    pFit=fit;
    pX=x; 
    XX=pX;    
    % fMin denotes the global optimum fitness value
    [fMin,bestI]=min(fit);      
    % bestX denotes the global optimum position corresponding to fMin
    bestX=x(bestI,:); 
     FES=0;
    % Start updating the solutions.
    while FES<Max_FEs  
        % ----------------------------------------------------------------
        % 基于Beta分布的动态反向学习策略
        for j=1:dim
            s(j)=(1/2)*(ub(j)+lb(j)); % Eq.(14)
        end
        for i=1:pop
            for j=1:dim
                % -------------------------------------------------
                % Eq.(12)
                if abs(x(i,j)-s(j))/abs(ub(j)-lb(j))<phi_f(0,1)
                    r(i,j)=phi_f(ub(j)+lb(j)-x(i,j),s(j));
                else
                    r(i,j)=phi_f(lb(j),ub(j)+lb(j)-x(i,j));
                end
                % -------------------------------------------------
                % Eq.(13)
                if abs(x(i,j)-s(j))/abs(ub(j)-lb(j))<phi_f(0,1)
                    p(i,j)=phi_f(s(j),ub(j)+lb(j)-x(i,j));
                else
                    p(i,j)=phi_f(ub(j)+lb(j)-x(i,j),ub(j));
                end
                % -------------------------------------------------
                % Eq.(11)
                if x(i,j)>=s(j)
                    u(i,j)=r(i,j);
                else
                    u(i,j)=p(i,j);
                end
                % -------------------------------------------------
            end
        end
        for i=1:pop
            fit_u(i)=fobj(u(i,:));
            FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            Convergence_curve(FES) = fMin;
            if FES>=Max_FEs
                return;
            end
            
            
            if fit_u(i)<fit(i)
                pX(i,:)=u(i,:);
            end
        end
        % ----------------------------------------------------------------
        [fmax,B]=max(fit);
        worse=x(B,:);   
        r2=rand(1);
        for i=1:pNum    
            if(r2<0.9)
                a=rand(1,1);
                if(a>0.1)
                    a=1;
                else
                    a=-1;
                end
                x(i,:)=pX(i,:)+0.8*abs(pX(i,:)-worse)+a*0.1*(XX(i,:)); % Equation (1)
            else
                aaa= randperm(180,1);
                if(aaa==0||aaa==90||aaa==180)
                    x(i,:)=pX(i,:);   
                end
                theta=aaa*pi/180;   
                x(i,:)=rand*pX(i,:)+tan(theta).*abs(pX(i,:)-XX(i,:));    % Equation (2)
            end
            x(i,:)=Bounds(x(i,:),lb,ub);    
            fit(i)=fobj(x(i,:));
            FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            Convergence_curve(FES) = fMin;
            if FES>=Max_FEs
                return;
            end
            
        end 
        [fMMin,bestII]=min(fit);      % fMin denotes the current optimum fitness value
        bestXX=x(bestII,:);             % bestXX denotes the current optimum position 
        R=1-FES/Max_FEs;             
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
            x(i,:)=Bounds(x(i,:),Xnew1,Xnew2);
            fit(i)=fobj(x(i,:)) ;
            FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            Convergence_curve(FES) = fMin;
            if FES>=Max_FEs
                return;
            end
            
        end
        for i=13:19                  % Equation (6)
            x(i,:)=pX(i,:)+((randn(1)).*(pX(i,:)-Xnew11)+((rand(1,dim)).*(pX(i,:)-Xnew22)));
            x(i,:)=Bounds(x(i,:),lb,ub);
            fit(i)=fobj(x(i,:));
             FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            Convergence_curve(FES) = fMin;
            if FES>=Max_FEs
                return;
            end
        end
        for i=20:pop                 % Equation (7)
            x(i,:)=bestX+randn(1,dim).*((abs((pX(i,:)-bestXX)))+(abs((pX(i,:)-bestX))))./2;
            
            %-----------------------------------------------------------------------------
            x(i,:)=Bounds(x(i,:),lb,ub);
            fit(i)=fobj(x(i,:));
             FES = FES + 1;  % 每调用一次目标函数，增加评价计数
            Convergence_curve(FES) = fMin;
            if FES>=Max_FEs
                return;
            end
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
        Convergence_curve(FES)=fMin;
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
% -----------------------------------------------------------------------------------------------------------
% function s = Bounds(s, Lb, Ub)
%         temp = s;
%         I = temp < Lb;
%         temp(I) = Lb(I);
%         J = temp > Ub;
%         temp(J) = Ub(J);
%         s = temp;
%     end

function Phi=phi_f(a,b)
    Phi=a+(b-a)*betarnd(0.5,0.5); % Eq.(15)
end
% -----------------------------------------------------------------------------------------------------------