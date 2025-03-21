function [bestScore, bestSolution, curve] = SaDE(N, Max_iteration, lb, ub, dim, fobj)
    % SaDE algorithm encapsulated similar to PSO function for uniform calling
    % N: Population size
    % Max_iteration: Maximum number of iterations
    % lb: Lower bounds (vector or scalar)
    % ub: Upper bounds (vector or scalar)
    % dim: Dimension of the problem
    % fobj: Objective function handle

    % Initialize parameters
    NP = N;
    D = dim;
    Max_Gen = Max_iteration;
    Max_FES = NP * Max_Gen;
    Lbound = lb;
    Ubound = ub;
    if numel(Lbound) == 1
        Lbound = Lbound * ones(1, D);
        Ubound = Ubound * ones(1, D);
    end
    XRmin = Lbound;
    XRmax = Ubound;
    err = 0;
    F = 0.5;       % Mutation factor
    CR = 0.9;      % Crossover probability
    strategy = 1;  % Strategy index

    global nfeval
    nfeval = 0;

    rand('state', sum(100 * clock));
    F0 = F;
    ccm = CR;

    pop = zeros(NP, D); % Initialize population

    for i = 1:NP
        pop(i, :) = XRmin + (XRmax - XRmin) .* rand(1, D);
    end

    popold = zeros(size(pop));     % Toggle population
    val = zeros(NP, 1);            % Cost array
    bestSolution = zeros(1, D);    % Best solution ever

    numst = 4;
    learngen = 20;
    localflag = zeros(1, NP);
    aaaa = [];
    lpcount = [];
    npcount = [];
    pfit = [1, 1, 1, 1];
    curve = zeros(1, Max_Gen);     % Best score at each iteration

    % Evaluate the initial population
    for i = 1:NP
        val(i) = fobj(pop(i, :));
        nfeval = nfeval + 1;
    end

    [bestScore, ibest] = min(val);
    bestSolution = pop(ibest, :);    % Best member of current iteration

    % Main loop
    for iter = 1:Max_Gen
        popold = pop;                   % Save the old population

        % Mutation and crossover
        [pop, val, bestSolution, bestScore, ccm, pfit, aaaa, lpcount, npcount] = ...
            mutation_crossover(pop, popold, val, bestSolution, bestScore, ...
            F0, ccm, numst, learngen, iter, fobj, XRmin, XRmax, Lbound, Ubound, Max_FES, pfit, aaaa, lpcount, npcount);

        % Update curve
        curve(iter) = bestScore;

        % Early stopping if error criterion is met
        if bestScore <= err
            break;
        end

        if nfeval >= Max_FES
            break;
        end
    end
end

function [pop, val, bestSolution, bestScore, ccm, pfit, aaaa, lpcount, npcount] = ...
    mutation_crossover(pop, popold, val, bestSolution, bestScore, ...
    F0, ccm, numst, learngen, iter, fobj, XRmin, XRmax, Lbound, Ubound, Max_FES, pfit, aaaa, lpcount, npcount)

    NP = size(pop, 1);
    D = size(pop, 2);
    global nfeval

    ind = randperm(4);              % Index pointer array
    rot = (0:1:NP - 1);
    rotd = (0:1:D - 1);

    a1 = randperm(NP);             % Shuffle locations of vectors
    rt = rem(rot + ind(1), NP);
    a2 = a1(rt + 1);
    rt = rem(rot + ind(2), NP);
    a3 = a2(rt + 1);
    rt = rem(rot + ind(3), NP);
    a4 = a3(rt + 1);
    rt = rem(rot + ind(4), NP);
    a5 = a4(rt + 1);

    pm1 = popold(a1, :);             % Shuffled population 1
    pm2 = popold(a2, :);             % Shuffled population 2
    pm3 = popold(a3, :);             % Shuffled population 3
    pm4 = popold(a4, :);             % Shuffled population 4
    pm5 = popold(a5, :);             % Shuffled population 5

    bm = repmat(bestSolution, NP, 1);

    if rem(iter, learngen) == 0
        if iter ~= 0 && ~isempty(aaaa)
            ccm = median(aaaa);
        end
        aaaa = [];
    end

    if rem(iter, 5) == 0
        cc = [];
        for k = 1:NP
            tt = normrnd(ccm, 0.1);
            while tt > 1 || tt < 0
                tt = normrnd(ccm, 0.1);
            end
            cc = [cc; tt];
        end
    else
        cc = ccm * ones(NP, 1);
    end

    F = [];
    for k = 1:NP
        tt = normrnd(F0, 0.3);
        while tt > 2 || tt < 0
            tt = normrnd(F0, 0.3);
        end
        F = [F; tt];
    end

    aa = rand(NP, D) < repmat(cc, 1, D);
    for k = 1:NP
        if all(aa(k, :) == 0)
            bb = ceil(D * rand);
            aa(k, bb) = 1;
        end
    end

    mui = aa;
    mpo = mui < 0.5;

    % Stochastic universal sampling
    rr = rand;
    spacing = 1 / NP;
    randnums = sort(mod(rr:spacing:1 + rr - 0.5 * spacing, 1));

    normfit = pfit / sum(pfit);
    partsum = 0;
    count(1) = 0;
    stpool = [];

    for i = 1:length(pfit)
        partsum = partsum + normfit(i);
        count(i + 1) = length(find(randnums < partsum));
        select(i, 1) = count(i + 1) - count(i);
        stpool = [stpool; ones(select(i, 1), 1) * i];
    end
    stpool = stpool(randperm(NP));

    % Apply strategies
    ui = zeros(size(pop));
    aaa = cell(numst, 1);
    for i = 1:numst
        atemp = zeros(1, NP);
        aaa{i} = atemp;
        index = find(stpool == i);
        if ~isempty(index)
            aaa{i}(index) = 1;
            switch i
                case 1
                    ui(index, :) = pm3(index, :) + repmat(F(index, :), 1, D) .* (pm1(index, :) - pm2(index, :));
                    ui(index, :) = popold(index, :) .* mpo(index, :) + ui(index, :) .* mui(index, :);
                case 2
                    ui(index, :) = popold(index, :) + repmat(F(index, :), 1, D) .* (bm(index, :) - popold(index, :)) + ...
                        repmat(F(index, :), 1, D) .* (pm1(index, :) - pm2(index, :) + pm3(index, :) - pm4(index, :));
                    ui(index, :) = popold(index, :) .* mpo(index, :) + ui(index, :) .* mui(index, :);
                case 3
                    ui(index, :) = popold(index, :) + repmat(F(index, :), 1, D) .* (pm5(index, :) - popold(index, :)) + ...
                        repmat(F(index, :), 1, D) .* (pm1(index, :) - pm2(index, :));
                case 4
                    ui(index, :) = pm5(index, :) + repmat(F(index, :), 1, D) .* (pm1(index, :) - pm2(index, :) + pm3(index, :) - pm4(index, :));
                    ui(index, :) = popold(index, :) .* mpo(index, :) + ui(index, :) .* mui(index, :);
            end
        end
    end

    % Boundary check
    for i = 1:NP
        ui(i, :) = max(ui(i, :), Lbound);
        ui(i, :) = min(ui(i, :), Ubound);
    end

    % Selection
    for i = 1:NP
        tempval = fobj(ui(i, :));
        nfeval = nfeval + 1;

        if tempval <= val(i)
            pop(i, :) = ui(i, :);
            val(i) = tempval;
            localflag(i) = 0;
            aaaa = [aaaa; cc(i, 1)];

            % Update success count
            tlpcount = zeros(1, numst);
            for j = 1:numst
                if aaa{j}(i) == 1
                    tlpcount(j) = 1;
                end
            end
            lpcount = [lpcount; tlpcount];
            if iter >= learngen
                lpcount(1, :) = [];
            end

            if tempval < bestScore
                bestScore = tempval;
                bestSolution = ui(i, :);
            end
        else
            % Update failure count
            tnpcount = zeros(1, numst);
            for j = 1:numst
                if aaa{j}(i) == 1
                    tnpcount(j) = 1;
                end
            end
            npcount = [npcount; tnpcount];
            if iter >= learngen
                npcount(1, :) = [];
            end
        end

        if nfeval >= Max_FES
            break;
        end
    end

    % Update pfit
    if iter >= learngen
        if isempty(npcount)
            npcount = zeros(1, numst);
        end
        for i = 1:numst
            if (sum(lpcount(:, i)) + sum(npcount(:, i))) == 0
                pfit(i) = 0.01;
            else
                pfit(i) = sum(lpcount(:, i)) / (sum(lpcount(:, i)) + sum(npcount(:, i))) + 0.01;
            end
        end
    end
end
