
function [fMin , bestX,Convergence_curve ] = NTSSA(pop, M,c,d,dim,fobj  )
% pop=200;
% M=100;
% c=1;
% d=-1;
% dim=2;
% fobj= evaluate_objective(x);
P_percent = 0.2;    % The population size of producers accounts for "P_percent" percent of the total population size
pNum = round( pop *  P_percent );    % The population size of the producers
lb= c.*ones( 1,dim );    % Lower limit/bounds/     a vector
ub= d.*ones( 1,dim );    % Upper limit/bounds/     a vector
%Initialization

x = repmat(lb,pop,1)+rand.* repmat((ub-lb),pop,1);%����ѡ��1��ΪLogistic����ӳ��

for i = 1 : pop
    
    fit( i ) = fobj( x( i, : ) ) ;
end
pFit = fit;
pX = x;                            % The individual's best position corresponding to the pFit
[ fMin, bestI ] = min( fit );      % fMin denotes the global optimum fitness value
bestX = x( bestI, : );             % bestX denotes the global optimum position corresponding to fMin

% Start updating the solutions.
for t = 1 : M
    [ ans, sortIndex ] = sort( pFit );% Sort.
    [fmax,B]=max( pFit );
    worse= x(B,:);
    %% �Ľ���2�������ںϱ�����ӥ�Ŀ�̽�׶β��ԡ���
    % ����Ҫ���뱱����ӥ�㷨������Ҫ�Ƚ�������ӥ�㷨��Ҫ�Ĳ���׼����
    FP = pFit(sortIndex(1:pNum));
    XP = x(sortIndex(1:pNum),:);

    r2=rand(1);
    if(r2<0.8)
        for i = 1 : pNum 
            %����򵥸Ľ�һ�±�����ӥ�Ŀ�̽�׶Σ�ѡ��һ���ȽϺõ�λ��P��֮ǰ������ӥ��Pֵ�����ѡ�ġ�
            ssa_position=find(FP<FP(i));
            if size(ssa_position,2)==0
                P = XP(1);
                F_P = pFit(1);
            else
                if rand <0.9
                    P=XP(1);
                    F_P = pFit(1);
                else
                    k=randperm(size(ssa_position,2),1);
                    P=XP(ssa_position(k),:);
                    F_P = pFit(ssa_position(k));
                end
            end
    
            I=round(1+rand); 
            %������ӥ��̽�׶εĹ�ʽ��
            if pFit(sortIndex(i))> F_P
                x( sortIndex(i),:)=pX(sortIndex(i),:)+rand(1,1).*(P-I.*pX(sortIndex(i),:)); % Eq. (4)
            else
                x( sortIndex(i),:)=pX(sortIndex(i),:)+rand(1,1).*(pX(sortIndex(i),:)-P); % Eq. (4)
            end
    
            x( sortIndex( i ), : ) = Bounds( x( sortIndex( i ), : ), lb, ub );
            fit( sortIndex( i ) ) = fobj( x( sortIndex( i ), : ) );
        end
    else
        for i = 1 : pNum
            x( sortIndex( i ), : ) = pX( sortIndex( i ), : )+randn(1)*ones(1,dim);
            x( sortIndex( i ), : ) = Bounds( x( sortIndex( i ), : ), lb, ub );
            fit( sortIndex( i ) ) = fobj( x( sortIndex( i ), : ) );
        end
    end
    
    [fMMin, bestII ] = min( fit );
    bestXX = x( bestII, : );
    %% �Ľ���3�������ں�����Ӧt�ֲ�����
    freen = exp(4.*(t/M).^2);  %����Ӧt�ֲ�
    for i = ( pNum + 1 ) : pop                     % Equation (4)
        if rand>0.5
            A=floor(rand(1,dim)*2)*2-1;
            if( i>(pop/2))
                x( sortIndex(i ), : )=randn(1)*exp((worse-pX( sortIndex( i ), : ))/(i)^2);
            else
                x( sortIndex( i ), : )=bestXX+(abs(( pX( sortIndex( i ), : )-bestXX)))*(A'*(A*A')^(-1))*ones(1,dim);
    
            end
        else  %�����ֵС��0.5ʱ��ʹ������Ӧt�ֲ�����
            x( sortIndex(i ), : )=   bestXX+ trnd(freen)*bestXX;
        end
        x( sortIndex( i ), : ) = Bounds( x( sortIndex( i ), : ), lb, ub );
        fit( sortIndex( i ) ) = fobj( x( sortIndex( i ), : ) );

    end

    c=randperm(numel(sortIndex));
    b=sortIndex(c(1:20));
    
    for j =  1  : length(b)      % Equation (5)
        if( pFit( sortIndex( b(j) ) )>(fMin) )
            x( sortIndex( b(j) ), : )=bestX+(randn(1,dim)).*(abs(( pX( sortIndex( b(j) ), : ) -bestX)));
        else
            x( sortIndex( b(j) ), : ) =pX( sortIndex( b(j) ), : )+(2*rand(1)-1)*(abs(pX( sortIndex( b(j) ), : )-worse))/ ( pFit( sortIndex( b(j) ) )-fmax+1e-50);
        end
        x( sortIndex(b(j) ), : ) = Bounds( x( sortIndex(b(j) ), : ), lb, ub );
        fit( sortIndex( b(j) ) ) = fobj( x( sortIndex( b(j) ), : ) );
    end
    for i = 1 : pop
        if ( fit( i ) < pFit( i ) )
            pFit( i ) = fit( i );
            pX( i, : ) = x( i, : );
        end
        if( pFit( i ) < fMin )
            fMin= pFit( i );
            bestX = pX( i, : );
        end
    end
    Convergence_curve(t)=fMin;
end


% Application of simple limits/bounds
function s = Bounds( s, Lb, Ub)
% Apply the lower bound vector
temp = s;
I = temp < Lb;
temp(I) = Lb(I);

% Apply the upper bound vector
J = temp > Ub;
temp(J) = Ub(J);
% Update this new move
s = temp;

%---------------------------------------------------------------------------------------------------------------------------