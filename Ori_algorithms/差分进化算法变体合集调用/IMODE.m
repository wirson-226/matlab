function [BestScore, BestPos, BestCost] = IMODE(nPop, MaxIt, VarMin, VarMax, nVar, CostFunction)
    % Initialization of parameters
    VarSize = [1 nVar];   
    beta_min = 0.2;  % Lower bound of scaling factor
    beta_max = 0.8;  % Upper bound of scaling factor
    pCR = 0.2;       % Crossover probability

    % Diversity and quality-related parameters
    numOperators = 3;   % Number of DE operators

    % Initialize operator populations as an array
    operatorPop = ones(numOperators, 1) * floor(nPop / numOperators);
    extraIndividuals = nPop - sum(operatorPop);

    % Distribute extra individuals among operators
    operatorPop(1:extraIndividuals) = operatorPop(1:extraIndividuals) + 1;

    % Compute cumulative sums for indexing
    operatorStartIndex = [0; cumsum(operatorPop)];

    % Initialize population
    empty_individual.Position = [];
    empty_individual.Cost = [];
    pop = repmat(empty_individual, nPop, 1);

    % Initialize BestSol
    BestSol.Position = [];
    BestSol.Cost = inf;

    for i = 1:nPop
        pop(i).Position = unifrnd(VarMin, VarMax, VarSize);
        pop(i).Cost = CostFunction(pop(i).Position);
        if pop(i).Cost < BestSol.Cost
            BestSol = pop(i);
        end
    end

    BestCost = zeros(MaxIt, 1);

    % Main loop of IMODE
    for it = 1:MaxIt
        % Recompute operatorStartIndex in case operatorPop has changed
        operatorStartIndex = [0; cumsum(operatorPop)];

        for i = 1:numOperators
            if operatorPop(i) > 0
                % Evolve each operator separately
                for j = 1:operatorPop(i)
                    currentPopIndex = operatorStartIndex(i) + j;
                    if currentPopIndex > nPop
                        continue; % Skip if index exceeds population size
                    end

                    x = pop(currentPopIndex).Position;
                    [NewSol, Operator] = MutateAndCrossover(x, i, VarMin, VarMax, beta_min, beta_max, pCR, pop);
                    NewSol.Cost = CostFunction(NewSol.Position);

                    if NewSol.Cost < pop(currentPopIndex).Cost
                        pop(currentPopIndex) = NewSol;
                        if NewSol.Cost < BestSol.Cost
                            BestSol = NewSol;
                        end
                    end
                end
            end
        end

        % Update diversity and quality calculations
        [D_op, Q_op] = CalculateDiversityAndQuality(pop, operatorPop, BestSol.Position, numOperators);

        % Update operator populations (Equation 11)
        operatorPop = UpdateOperatorPopulations(nPop, D_op, Q_op, numOperators);

        % Ensure operatorPop sums to nPop
        operatorPop = floor(operatorPop);
        totalPop = sum(operatorPop);
        diff = nPop - totalPop;
        while diff ~= 0
            for op = 1:numOperators
                if diff > 0
                    operatorPop(op) = operatorPop(op) + 1;
                    diff = diff - 1;
                elseif diff < 0 && operatorPop(op) > 1
                    operatorPop(op) = operatorPop(op) - 1;
                    diff = diff + 1;
                end
                if diff == 0
                    break;
                end
            end
        end

        % Apply local search if needed (last 15% iterations)
        if it > 0.85 * MaxIt
            if rand <= 0.1 % Local search probability
                [BestSol, ~] = LocalSearch(BestSol, CostFunction, VarMin, VarMax, 1);
            end
        end

        % Store best cost in each iteration
        BestCost(it) = BestSol.Cost;
        BestScore = BestSol.Cost;
        BestPos = BestSol.Position;
    end
end

function [NewSol, Operator] = MutateAndCrossover(x, operatorIndex, VarMin, VarMax, beta_min, beta_max, pCR, pop)
    % Ensure integer input for randperm
    popSize = length(pop); % Total number of individuals in the population
    indices = randperm(popSize);
    indices(indices == operatorIndex) = []; % Exclude current index if necessary
    A = indices(1:3); % Select three different random indices

    % Extract the three randomly chosen individuals
    a = A(1);
    b = A(2);
    c = A(3);
    
    % Different DE operators
    switch operatorIndex
        case 1
            beta = unifrnd(beta_min, beta_max, size(x));
            y = pop(a).Position + beta .* (pop(b).Position - pop(c).Position);
        case 2
            beta = unifrnd(beta_min, beta_max, size(x));
            y = pop(a).Position + beta .* (pop(b).Position - pop(c).Position);
        case 3
            beta = unifrnd(beta_min, beta_max, size(x));
            y = beta .* pop(a).Position + (1 - beta) .* (pop(b).Position - pop(c).Position);
    end

    % Crossover
    z = zeros(size(x));
    j0 = randi([1 numel(x)]);
    for j = 1:numel(x)
        if j == j0 || rand <= pCR
            z(j) = y(j);
        else
            z(j) = x(j);
        end
    end

    NewSol.Position = max(min(z, VarMax), VarMin);
    Operator = operatorIndex;
end

function [BestSol, P_ls] = LocalSearch(BestSol, CostFunction, VarMin, VarMax, CFE_ls)
    % Perform local search using SQP
    P_ls = 0.1;
    x = BestSol.Position;
    options = optimoptions('fmincon', 'Display', 'off');
    [x_opt, fval] = fmincon(CostFunction, x, [], [], [], [], VarMin, VarMax, [], options);
    
    if fval < BestSol.Cost
        BestSol.Position = x_opt;
        BestSol.Cost = fval;
        P_ls = 0.1; % Reset local search probability after successful use
    else
        P_ls = P_ls + 0.0001; % Gradually increase probability if unsuccessful
    end
end

function [D_op, Q_op] = CalculateDiversityAndQuality(pop, operatorPop, BestPos, numOperators)
    D_op = zeros(numOperators, 1); % Diversity of each operator
    Q_op = zeros(numOperators, 1); % Quality of each operator

    idx = 0;
    % Loop through each operator
    for op = 1:numOperators
        % Get the sub-population for the current operator
        subPop = pop(idx + 1 : idx + operatorPop(op));
        idx = idx + operatorPop(op);

        if ~isempty(subPop)
            % Calculate the diversity D_op based on Eq. (7)
            sumDist = 0;
            for i = 1:length(subPop)
                dist_i = norm(subPop(i).Position - BestPos); % Euclidean distance
                sumDist = sumDist + dist_i;
            end
            D_op(op) = sumDist / length(subPop);

            % Calculate the quality Q_op based on Eq. (9)
            costs = [subPop.Cost];
            f_best = min(costs); % Best objective value in sub-population
            if f_best == 0
                f_best = eps; % Avoid division by zero
            end
            sumCostRatios = sum(costs / f_best);
            Q_op(op) = sumCostRatios / length(subPop);
        else
            % Assign default values or handle empty subPop
            D_op(op) = 0;
            Q_op(op) = 1;
        end
    end
end

function operatorPop = UpdateOperatorPopulations(nPop, D_op, Q_op, numOperators)
    % Calculate improvement rate (IRV_op) for each operator using Eq. (10)
    IRV_op = (1 - Q_op) + D_op;

    % Avoid division by zero
    sumIRV = sum(IRV_op);
    if sumIRV == 0
        IRV_op = ones(numOperators, 1);
        sumIRV = numOperators;
    end

    % Update the number of solutions for each operator using Eq. (11)
    operatorPop = zeros(numOperators, 1);
    for op = 1:numOperators
        operatorPop(op) = max(1, round( (IRV_op(op) / sumIRV) * nPop ));
    end

    % Adjust to ensure total population size remains the same
    totalPop = sum(operatorPop);
    diff = nPop - totalPop;
    while diff ~= 0
        for op = 1:numOperators
            if diff > 0
                operatorPop(op) = operatorPop(op) + 1;
                diff = diff - 1;
            elseif diff < 0 && operatorPop(op) > 1
                operatorPop(op) = operatorPop(op) - 1;
                diff = diff + 1;
            end
            if diff == 0
                break;
            end
        end
    end
end
