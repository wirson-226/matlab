
% L-SHADE

function [kk,gbest,Convergence_curve]=LSHADE(SearchAgents_no,MaxFES,lb, ub, dim,fobj )

MaxFES = SearchAgents_no.*MaxFES;
Nmin=4;


H=SearchAgents_no;
MCR=0.5*ones(H,1);
MF=0.5*ones(H,1);
A=[];
Fsigma=0.1;

x=initialization(SearchAgents_no,dim,ub,lb);
for i=1:SearchAgents_no
    fitness(i)=fobj(x(i,:));
end
N=SearchAgents_no;
FES=N;
k=1;
q=1;
countiter = 1;
% p0=ceil(0.05*N);
while FES<=MaxFES
    [~,I]=sort(fitness);
    SCR=[];
    SF=[];
    pmin=2/SearchAgents_no;
    for i=1:SearchAgents_no
        r=randi(H);
        CR(i)=MCR(r)+0.1*randn;
        CR(i)=max(0,CR(i));
        CR(i)=min(1,CR(i));
        F(i)=MF(r)+Fsigma*tan(pi*(rand-0.5));
        F(i)=min(1,F(i));
        while(F(i)<=0)
            F(i)=MF(r)+Fsigma*tan(pi*(rand-0.5));
            F(i)=min(1,F(i));
        end
        p=unifrnd(pmin,0.2);
        p0=round(p*SearchAgents_no);
        temp1=floor(rand*p0)+1;
        r1=floor(rand*SearchAgents_no)+1;
        while r1==i
            r1=floor(rand*SearchAgents_no)+1;
        end
        S=[x;A];
        r=size(S,1);
        r2=floor(rand*r)+1;
        while r2==i||r2==r1
            r2=floor(rand*r)+1;
        end
        v(i,:)=x(i,:)+F(i)*(x(I(temp1),:)-x(i,:))+F(i)*(x(r1,:)-S(r2,:));
        jrand=floor(rand*dim)+1;
        for j=1:dim
            if  v(i,j)>ub(j)
                v(i,j)=max(ub(j),2*ub(j)-v(i,j));
            end
            if  v(i,j)<lb(j)
                v(i,j)=min(lb(j),2*lb(j)-v(i,j));
            end
            if (rand<CR(i))||(j==jrand)
                newx(i,j)=v(i,j);
            else
                newx(i,j)=x(i,j);
            end
        end
        fnew(i)=fobj(newx(i,:));
        if fnew(i)<fitness(i)
            A=[A;x(i,:)];
            SCR=[SCR;CR(i)];
            SF=[SF;F(i)];
            x(i,:)=newx(i,:);
            fitness(i)=fnew(i);
        end

    end
    a=size(A,1);
    while a>SearchAgents_no
        temp3=floor(rand(a-SearchAgents_no,1)*a)+1;
        A(temp3,:)=[];
        a=size(A,1);
    end
    b=abs(fnew(q)-fitness(q));
    sumf=0;
    for q=1:numel(SCR)
        sumf=sumf+b;
    end

    if isempty(SCR)&&  isempty(SF)
        w=b/sumf;
        MCR(q)=sum(w*SCR(q));
        MF(q)=sum(w*(SF(q).^2))/sum(w*SF(q));
        q=q+1;
        if q>H
            q=1;
        end
    end
    Nnextg=round(((Nmin-N)/MaxFES)*FES+N);
    [~,I1]=sort(fitness,'descend');
    temp4=Nnextg-SearchAgents_no;
    while temp4>0
        temp5=randi(temp4);
        x(I1(temp5,:))=[];
        temp4=temp4-1;
    end
    SearchAgents_no=size(x,1);

    FES=FES+SearchAgents_no;
    Convergence_curve(1,countiter)=min(fitness);
    countiter = countiter + 1;
end
[kk,ll]=min(fitness);
gbest=x(ll,:);
end

function Positions=initialization(SearchAgents_no,dim,ub,lb)

% Boundary_no= size(ub,2); % numnber of boundaries

% If the boundaries of all variables are equal and user enter a signle
% number for both ub and lb
% if Boundary_no==1
    Positions=rand(SearchAgents_no,dim).*(ub-lb)+lb;
% end
end

