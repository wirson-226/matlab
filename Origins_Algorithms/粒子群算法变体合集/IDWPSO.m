%% ��̬��������Ȩ�ظĽ�����Ⱥ�㷨��IDWPSO��
function [Gbest,g,gb] = IDWPSO(N,Max_iteration,lb,ub,dim,fobj)


% N = N;                  % Ⱥ�����Ӹ���
D = dim;                    % ����ά��
T = Max_iteration;                  % ����������
c1 = 1.5;                 % ѧϰ����1
c2 = 1.5;                 % ѧϰ����2
F = 0.5;                  % ��������
p1 = 1;
q1 = 3;
sigma = 0.1;              % ���Ե�������
CR = 0.1;                 % �������
Wmax = 0.8;               % ����Ȩ�����ֵ
Wmin = 0.4;               % ����Ȩ����Сֵ
Xmax = ub;                 % λ�����ֵ
Xmin = lb;                % λ����Сֵ
Vmax = 1;                 % �ٶ����ֵ
Vmin = -1;                % �ٶ���Сֵ
tic;
%% ��ʼ����Ⱥ���壨�޶�λ�ú��ٶȣ�
x = rand(N,D) .* (Xmax-Xmin)+Xmin;
v = rand(N,D) .* (Vmax-Vmin)+Vmin;
%% ��ʼ����������λ�ú�����ֵ
p = x;                          % ��������λ��
pbest = ones(N,1);
for i = 1:N
    pbest(i) = fobj(x(i,:));   % ��������ֵ   һ�� 1*N �ľ���
end
%% ��ʼ��ȫ������λ�ú�����ֵ
g = ones(1,D);
gbest = inf;
for i = 1:N
    if pbest(i) < gbest         % ����Сֵ����ѡȡ��С����
        g = p(i,:);             % ȫ������λ��
        gbest = pbest(i);       % ��������ֵ  һ��ֵ
    end
end
gb = ones(1,T);
%% ���չ�ʽ���ε���ֱ�����㾫�Ȼ��ߵ�������
for i = 1:T                     % ��������
    for j = 1:N                 % ά��
        %% ���¸�������λ�ú�����ֵ
        if fobj(x(j,:)) < pbest(j)       % ����Сֵ����ѡȡ��С����
            p(j,:) = x(j,:);
            pbest(j) = fobj(x(j,:));     % һ�� 1*N �ľ���
        end
        %% ����ȫ������λ�ú�����ֵ
        if pbest(j) < gbest
            g = p(j,:);
            gbest = pbest(j);             % һ��ֵ
        end
        %% ���㶯̬����Ȩ��ֵ
        w = Wmin+(Wmax-Wmin)*exp(-i/T)+sigma*betarnd(p1,q1);
        v(j,:) = w*v(j,:)+c1*rand*(p(j,:)-x(j,:))+c2*rand*(g-x(j,:));
        %% ����λ�ú��ٶ�ֵ
        if rand < CR
            for ii = 1:D
                r1 = randperm(N,1);
                while r1 == j
                    r1 = randperm(N,1);
                end         % r1 
                
                r2 = randperm(N,1);
                while (r2==j) & (r2==r1)
                    r2 = randperm(N,1)
                end         % r2
                
                r3 = randperm(N,1);
                while (r3==j) & (r3==r1) & (r3==r2)
                    r3 = randperm(N,1)
                end         % r3
                
                x(j,ii) = x(r1,ii)+F*(x(r2,ii)-x(r3,ii));
            end             % ii 
        else
            x(j,:) = x(j,:)+v(j,:);
        end
        %% �߽���������
        % for ii = 1:D
        %     if (v(j,ii)>Vmax)  |  (v(j,ii)< Vmin)    % �ٶ�����
        %         v(j,ii)=rand * (Vmax-Vmin)+Vmin;
        %     end
        %     if (x(j,ii)>Xmax)  |  (x(j,ii)< Xmin)    % x ����
        %         x(j,ii)=rand * (Xmax-Xmin)+Xmin;
        %     end
        % end       % �߽�����
        
    end        % ά��ѭ��
    %% ��¼����ȫ������ֵ
    gb(i)=gbest;
end        % ����ѭ��
time =toc;
g;                         %���Ÿ���
Gbest = gb(end);           %����ֵ
row = find(gb == gb(end));
N = row(1);


