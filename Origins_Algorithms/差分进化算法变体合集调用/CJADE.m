function [bestScore, bestSolution, curve] = CJADE(N, Max_iteration, lb, ub, dim, fobj)
% JADE with Chaos Search encapsulated similar to PSO function
% Inputs:
%   N - Population size
%   Max_iteration - Maximum number of iterations
%   lb - Lower bound (scalar or vector)
%   ub - Upper bound (scalar or vector)
%   dim - Problem dimension
%   fobj - Objective function handle
%   Strategy - Strategy index for Chaos Search
% Outputs:
%   bestScore - Best fitness value found
%   bestSolution - Best solution vector
%   curve - Vector of best scores at each iteration
    Strategy=3;
    switch Strategy
        case 0
            circle=5000;
        case 1
            circle=2971; % Maximum numbef of iterations 4839 & 3572
        case 2
            circle=2679;
        case 3
            circle=5883;
    end
    
% Initialize parameters
if numel(lb) == 1
    lb = lb * ones(1, dim);
    ub = ub * ones(1, dim);
end
lu = [lb; ub];

popsize = N;
n = dim;

% Initialize the main population
popold = repmat(lu(1, :), popsize, 1) + rand(popsize, n) .* (repmat(lu(2, :) - lu(1, :), popsize, 1));

% Evaluate initial population
valParents = zeros(popsize, 1);
for i = 1:popsize
    valParents(i) = fobj(popold(i, :));
end

% Initialize parameters
c = 1/10;
p = 0.05;

CRm = 0.5;
Fm = 0.5;

Afactor = 1;

archive.NP = Afactor * popsize;     % The maximum size of the archive
archive.pop = zeros(0, n);          % The solutions stored in the archive
archive.funvalues = zeros(0, 1);    % The function values of the archived solutions

% Initialize best solution
[valBest, indBest] = sort(valParents, 'ascend');
bestScore = valBest(1);
bestSolution = popold(indBest(1), :);

curve = zeros(1, Max_iteration);

% Initialize variables for Chaos Search
Chaos_p = GenerateChaos(Max_iteration); % You need to define or import this function
radius = 0.0001;
k_circle = 1;
ns = [];
nf = [];
pfit = ones(1, 12);
LEP = 50;

% Main loop
iteration = 0;

while iteration < Max_iteration
    iteration = iteration + 1;
    % Chaotic Local Search
    if Strategy > 0
        lb = lu(1, :);
        ub = lu(2, :);
        z = 0.5;
        g = k_circle;

        switch Strategy
            case 1
                j = randi([1, 12], 1, 1);
                temp_X = popold(indBest(1), :) + radius * (ub - lb) .* (Chaos_p(j, g) - z);

                % Boundary check
                temp_X = max(min(temp_X, ub), lb);

                fit_temp = fobj(temp_X);

                if fit_temp < valBest(1)
                    popold(indBest(1), :) = temp_X;
                    valBest(1) = fit_temp;
                end

            case 2
                temp_X = zeros(12, n);
                for j = 1:12
                    temp_X(j, :) = popold(indBest(1), :) + radius * (ub - lb) .* (Chaos_p(j, g) - z);

                    % Boundary check
                    temp_X(j, :) = max(min(temp_X(j, :), ub), lb);
                end
                fit_temp = zeros(12, 1);
                for j = 1:12
                    fit_temp(j) = fobj(temp_X(j, :));
                end

                [num_Fbest, num_X] = min(fit_temp);

                if num_Fbest < valBest(1)
                    popold(indBest(1), :) = temp_X(num_X, :);
                    valBest(1) = num_Fbest;
                end

            case 3
                rr = rand;
                j = 1;
                partsum = 0;
                normfit = pfit / sum(pfit);
                while partsum < rr
                    partsum = partsum + normfit(j);
                    j = j + 1;
                end
                select = j - 1;

                lpcount = [];
                npcount = [];
                temp_X = popold(indBest(1), :) + radius * (ub - lb) .* (Chaos_p(select, g) - z);

                % Boundary check
                temp_X = max(min(temp_X, ub), lb);

                fit_temp = fobj(temp_X);

                if fit_temp < valBest(1)
                    popold(indBest(1), :) = temp_X;
                    valBest(1) = fit_temp;

                    tlpcount = zeros(1, 12);
                    tlpcount(select) = 1;
                    lpcount = [lpcount; tlpcount];

                    tnpcount = ones(1, 12);
                    tnpcount(select) = 0;
                    npcount = [npcount; tnpcount];
                else
                    tlpcount = zeros(1, 12);
                    lpcount = [lpcount; tlpcount];
                    tnpcount = ones(1, 12);
                    npcount = [npcount; tnpcount];
                end

                ns = [ns; sum(lpcount, 1)];
                nf = [nf; sum(npcount, 1)];

                % Update pfit
                if k_circle + 1 >= LEP
                    for i_pfit = 1:12
                        if (sum(ns(:, i_pfit)) + sum(nf(:, i_pfit))) == 0
                            pfit(i_pfit) = 0.01; % Avoid zero success rates
                        else
                            pfit(i_pfit) = sum(ns(:, i_pfit)) / (sum(ns(:, i_pfit)) + sum(nf(:, i_pfit))) + 0.01;
                        end
                    end
                    if size(ns, 1) > LEP
                        ns(1, :) = [];
                    end
                    if size(nf, 1) > LEP
                        nf(1, :) = [];
                    end
                end
        end
        radius = radius * 0.986; % Optional radius decay
    end

    % Update CRm and Fm
    if iteration > 1 && ~isempty(goodCR) && sum(goodF) > 0 % If goodF and goodCR are empty, skip the update
        CRm = (1 - c) * CRm + c * mean(goodCR);
        Fm = (1 - c) * Fm + c * sum(goodF .^ 2) / sum(goodF); % Lehmer mean
    end

    % Generate CR and F
    [F, CR] = randFCR(popsize, CRm, 0.1, Fm, 0.1);
    popAll = [popold; archive.pop];
    [r1, r2] = gnR1R2(popsize, size(popAll, 1));

    % Find the p-best solutions
    [~, indBest] = sort(valParents, 'ascend');
    pNP = max(round(p * popsize), 2);            % Choose at least two best solutions
    randindex = ceil(rand(1, popsize) * pNP);    % Select from [1, 2, ..., pNP]
    randindex = max(1, randindex);               % Ensure at least index 1
    pbest = popold(indBest(randindex), :);       % Randomly choose one of the top p% solutions

    % Mutation
    vi = popold + F(:, ones(1, n)) .* (pbest - popold + popold(r1, :) - popAll(r2, :));
    vi = boundConstraint(vi, popold, lu);

    % Crossover
    mask = rand(popsize, n) > CR(:, ones(1, n));                      % Mask for crossover
    rows = (1 : popsize)'; cols = floor(rand(popsize, 1) * n) + 1;    % Ensure at least one element from 'vi'
    jrand = sub2ind([popsize n], rows, cols); mask(jrand) = false;
    ui = vi; ui(mask) = popold(mask);

    % Evaluate offspring
    valOffspring = zeros(popsize, 1);
    for i = 1:popsize
        valOffspring(i) = fobj(ui(i, :));
    end

    % Selection
    pop = popold; % Copy of the current population
    [valParentsNew, I] = min([valParents, valOffspring], [], 2);

    % Update archive
    archive = updateArchive(archive, popold(I == 2, :), valParents(I == 2));

    % Update population and fitness values
    popold(I == 2, :) = ui(I == 2, :);
    valParents = valParentsNew;

    % Collect successful CR and F
    goodCR = CR(I == 2);
    goodF = F(I == 2);

    % Update best solution
    [valBest, indBest] = min(valParents);
    if valBest < bestScore
        bestScore = valBest;
        bestSolution = popold(indBest, :);
    end

    % Record the best score
    curve(iteration) = bestScore;

    k_circle = k_circle + 1;
end

end

% % Sub-functions (same as previous answer, include them here)
% function Chaos_p = GenerateChaos(Max_iteration)
%     % This function generates chaos sequences needed for the chaos search
%     % For simplicity, we'll use a simple logistic map as an example
%     % You should replace this with your actual chaos sequence generator
%     Chaos_p = zeros(12, Max_iteration);
%     r = 4; % Logistic map parameter
%     for j = 1:12
%         x = rand; % Initial value
%         for i = 1:Max_iteration
%             x = r * x * (1 - x);
%             Chaos_p(j, i) = x;
%         end
%     end
% end


% Include the other sub-functions: randFCR, gnR1R2, boundConstraint, updateArchive
function [F,CR] = randFCR(NP, CRm, CRsigma, Fm,  Fsigma)
    % Generate CR
    CR = CRm + CRsigma * randn(NP, 1);
    CR = min(1, max(0, CR));                % Truncated to [0, 1]

    % Generate F
    F = Fm + Fsigma * tan(pi * (rand(NP, 1) - 0.5)); % Cauchy distribution
    F = min(1, F);                          % Truncation

    % Ensure F > 0
    pos = find(F <= 0);
    while ~isempty(pos)
        F(pos) = Fm + Fsigma * tan(pi * (rand(length(pos), 1) - 0.5));
        F = min(1, F);
        pos = find(F <= 0);
    end
end

function [r1, r2] = gnR1R2(NP1, NP2)
    % Generate r1 and r2 for mutation
    r1 = zeros(NP1, 1);
    r2 = zeros(NP1, 1);
    for i = 1 : NP1
        idxs = [1:i-1, i+1:NP1];
        r1(i) = idxs(randi(NP1 - 1));
        idxs_all = 1:NP2;
        idxs_all([i, r1(i)]) = [];
        r2(i) = idxs_all(randi(NP2 - 2));
    end
end

function vi = boundConstraint(vi, pop, lu)
    % Handle boundary constraints
    [NP, D] = size(pop);
    xl = repmat(lu(1, :), NP, 1);
    xu = repmat(lu(2, :), NP, 1);

    % Lower bound
    pos = vi < xl;
    vi(pos) = (pop(pos) + xl(pos)) / 2;

    % Upper bound
    pos = vi > xu;
    vi(pos) = (pop(pos) + xu(pos)) / 2;
end

function archive = updateArchive(archive, pop, funvalue)
    % Update the archive with new solutions
    if archive.NP == 0, return; end

    % Add new solutions
    archive.pop = [archive.pop; pop];
    archive.funvalues = [archive.funvalues; funvalue];

    % Remove duplicates
    [archive.pop, ia] = unique(archive.pop, 'rows', 'stable');
    archive.funvalues = archive.funvalues(ia, :);

    % Maintain archive size
    if size(archive.pop, 1) > archive.NP
        rndpos = randperm(size(archive.pop, 1));
        rndpos = rndpos(1 : archive.NP);
        archive.pop = archive.pop(rndpos, :);
        archive.funvalues = archive.funvalues(rndpos, :);
    end
end


function Chaos_p=GenerateChaos(max_it)
    % [~,dim]=size(X);
    Chaos_p=zeros(15,max_it);
    Chaos=zeros(1,max_it);
    
    for ChaosSystemName_p=1:15;
    
    if ChaosSystemName_p==1;%Logisticmap
        z1=0.152;
        a=4.0;%a=3.6 3.7 3.8 3.9
        Chaos(1)=z1;
        for j=1:max_it;
            Chaos(j+1)=a*Chaos(j)-Chaos(j)*a*Chaos(j);
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==2;%PWLCMmap
        z1=0.152;
        Chaos(1)=z1;
        a=0.7;
        for j=1:max_it;
            if Chaos(j)<a &&Chaos(j)>0
               Chaos(j+1)=Chaos(j)/a;
            elseif Chaos(j)<1 &&Chaos(j)>=a
               Chaos(j+1)=(1-Chaos(j))/(1-a); 
            end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==3;%Singermap
        z1=0.002;
        a=1.073;%0.9-1.08
        Chaos(1)=z1;
        for j=1:max_it;
            Chaos(j+1)=a*(7.86*Chaos(j)-23.31*Chaos(j)^2+28.75*Chaos(j)^3-13.302875*Chaos(j)^4);
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==4;%Sinemap
        z1=0.152;
        Chaos(1)=z1;
        for j=1:max_it;
            Chaos(j+1)=sin(pi*Chaos(j));
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==5;%Gaussmap;
        z1=0.152;%%%%%%%0.84,0.85,0.86  0.3  0.4  0.5  0.6d   ܲ  ã  󲿷   0    ʼ     Ҫ  
                  %%%    0.344  0.3456 õ  Ľ⻹     0.34 Ͳ    ԡ       ֵ õ  Ľ  ܶ࣬
        Chaos(1)=z1;
        for j=1:max_it;
        if Chaos(j)==0;
           Chaos(j+1)=0;
        else
           val=1/Chaos(j);  
           Chaos(j+1)=val-floor(val);
        end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==6;%Tentmap
        z1=0.152;
        a=0.4;
        Chaos(1)=z1;
        for j=1:max_it;
            if Chaos(j)<=a&&Chaos(j)>0
               Chaos(j+1)=Chaos(j)/a;
            elseif Chaos(j)>a&&Chaos(j)<=1
                   Chaos(j+1)=(1-Chaos(j))/(1-a); 
            end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==7;%Bernoullimap
        z1=0.152;
        a=0.4;%%%%% ˴   a  ȡֵ  Ϊ0.4      0.5          0.5 Ļ       þ     0  
        Chaos(1)=z1;
        for j=1:max_it;
            if Chaos(j)<=(1-a)&&Chaos(j)>0
        Chaos(j+1)=Chaos(j)/(1-a);
            elseif Chaos(j)>(1-a)&&Chaos(j)<1
                Chaos(j+1)=(Chaos(j)-(1-a))/a; 
            end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==8;%Chebyshevmap
        z1=0.152;
        a=5;
        Chaos(1)=z1;
        for j=1:max_it;
         Chaos(j+1)=cos(a*cos(Chaos(j))^(-1));
         if Chaos(j+1)<0;
             Chaos(j+1)=abs(Chaos(j+1));
         else Chaos(j+1)=Chaos(j+1);
         end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==9;%Circlemap
        z1=0.152;
        b=0.5;
        a=2.2;%%%%   ݱ     ף  ı   a  ֵ       ԡ ԭ  Ϊ0.2      ܲ  á 
        Chaos(1)=z1;
        for j=1:max_it;
         val=(a/(2.0*pi))*sin(2*pi*Chaos(j));
	     Chaos(j+1)=Chaos(j)+b-(val-floor(val));
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==10;%Cubicmap
        z1=0.242;
        a=2.59;%%%%
        Chaos(1)=z1;
        for j=1:max_it;
        Chaos(j+1)=a*Chaos(j)*(1-Chaos(j)^2);
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==11;%sinusoidalmap
        z1=0.74;%  ʼֵ  ͬ   õ   ͼ   ܴ  ر     С  0.44ʱ
        a=2.3;
        Chaos(1)=z1;
        for j=1:max_it;
            Chaos(j+1)=a*(Chaos(j)^2)*sin(pi*Chaos(j));  
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==12;%ICMICmap
        z1=0.152;
        Chaos(1)=z1;
        a=70;%%%a=1  4  11  14  ʱͼ   Ǽ  ˵ģ     ֵʱ          a      ͼ     ı仯    a         ʱ  ͼ 仯  С  С  
        for j=1:max_it;
           Chaos(j+1)=sin(a/Chaos(j));
            if Chaos(j+1)<0;
             Chaos(j+1)=abs(Chaos(j+1));
         else Chaos(j+1)=Chaos(j+1);
            end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==13;%Iterativemap
        z1=0.152;
        a=0.7;
        Chaos(1)=z1;
        for j=1:max_it;
            Chaos(j+1)=sin(a*pi/Chaos(j));
            if Chaos(j+1)<0
                Chaos(j+1)=abs(Chaos(j+1));
            end
        end
    Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==14;%Intermittency Map
        z1=0.152;
        Chaos(1)=z1;
        for j=1:max_it;
            if Chaos(j)<=0.7 && Chaos(j)>0
                Chaos(j+1)=0.0001+Chaos(j)+(0.2999*(Chaos(j))^2/0.49);
            elseif Chaos(j)>0.7 && Chaos(j)<1
                Chaos(j+1)=(Chaos(j)-0.7)/0.3;
            end
        end
        Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    if ChaosSystemName_p==15;%Zaslavsky Map
        Chaos(1)=0.1;
        y(1)=0.1;
        e=0.3;
        r=5;
        omega=100;
        k=9;
        a=1.885;
        Chaos(1)=z1;
        for i=2:max_it
            Chaos(i)=mod(Chaos(i-1)+omega/(2*pi)+(a*omega)/(2*pi*r)*(1-exp(-r))*y(i-1)+(k/r)*(1-exp(-r))*cos(2*pi*Chaos(i-1)),1);
            y(i)=exp(-r)*(y(i-1)+e*cos(2*pi*Chaos(i-1)));
        end
        Chaos_p(ChaosSystemName_p,:)=Chaos(2:max_it+1);
    end
    
    
    % if ChaosSystemName_p==13;%uniform distribution
    %     Chaos_p(ChaosSystemName_p,:)=rand(1,max_it);
    % end
    % 
    % if ChaosSystemName_p==14;%normal random
    %     Chaos_p(ChaosSystemName_p,:)=randn(1,max_it);
    % end
    % 
    % 
    end

end
    
    
    
 

