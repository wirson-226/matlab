function [gBestScore, gBest, cg_curve] = SLPSO(N, Max_iteration, lb, ub, dim, fobj)
    %% Problem Definition
    MaxVelocity =6;
    CostFunction = fobj;  % Cost Function
    nVar = dim;           % Number of Unknown (Decision) Variables
    VarSize = [1, nVar];  % Matrix Size of Decision Variables
    VarMin = lb;          % Lower Bound of Decision Variables
    VarMax = ub;          % Upper Bound of Decision Variables

    %% Parameters of PSO
    MaxIt = Max_iteration; % Maximum Number of Iterations
    nPop = N;              % Population Size (Swarm Size)
    phi1 = 2.05;
    phi2 = 2.05;
    phi = phi1 + phi2;
    kappa = 1;
    chi = 2 * kappa / abs(2 - phi - sqrt(phi^2 - 4 * phi));
    w = chi;               % Inertia Coefficient
    wdamp = 1;             % Damping Ratio of Inertia Coefficient
    c1 = chi * phi1;       % Personal Acceleration Coefficient
    c2 = chi * phi2;       % Social Acceleration Coefficient

    %% Parameters of SLPSO
    proSTR = [0.25, 0.25, 0.25, 0.25]; % Probabilities of each strategy
    a = 1 / 6;                          % Learning coefficient
    proSTR2 = zeros(4, 1);
    Gs = 10;                             % Learning period
    S = zeros(4, 1);                     % Accumulators of 4 strategies
    wei = zeros(1, nPop);

    %% Initialization
    empty_particle.Position = [];
    empty_particle.Velocity = [];
    empty_particle.Cost = [];
    empty_particle.Best.Position = [];
    empty_particle.Best.Cost = [];

    particle = repmat(empty_particle, nPop, 1);
    GlobalBest.Cost = inf;

    MinVelocity = -0.2 * (VarMax - VarMin); % Set MinVelocity here

    for i = 1:nPop
        particle(i).Position = unifrnd(VarMin, VarMax, VarSize);
        particle(i).Velocity = zeros(VarSize);
        particle(i).Cost = CostFunction(particle(i).Position);
        particle(i).Best.Position = particle(i).Position;
        particle(i).Best.Cost = particle(i).Cost;
        if particle(i).Best.Cost < GlobalBest.Cost
            GlobalBest = particle(i).Best;
        end
    end

    BestCosts = zeros(MaxIt, 1);
    cg_curve = zeros(MaxIt, 1);

    %% Main Loop of PSO
    for it = 1:MaxIt
        % Update Strategy Probabilities
        if mod(it, Gs) == 0
            for j = 1:4
                proSTR2(j) = (1 - a) * proSTR(j) + a * S(j) / Gs;
                S(j) = 0;
            end
            proSTR = proSTR2 / sum(proSTR2);
        end

        for i = 1:nPop
            wei(i) = log(nPop - i + 1) / log(factorial(nPop));
            % Select Strategy
            index = find(rand() <= cumsum(proSTR), 1);
            % Update Velocity based on Strategy
            switch index
                case 1
                    particle(i).Velocity = w * particle(i).Velocity + ...
                        c1 * rand(VarSize) .* (particle(i).Best.Position - particle(i).Position) + ...
                        c2 * rand(VarSize) .* (GlobalBest.Position - particle(i).Position);
                case 2
                    % Update velocity for strategy 2
                case 3
                    % Update velocity for strategy 3
                case 4
                    % Update velocity for strategy 4
            end

            % Apply Velocity Limits
            particle(i).Velocity = max(particle(i).Velocity, MinVelocity);
            particle(i).Velocity = min(particle(i).Velocity, MaxVelocity);

            % Update Position
            particle(i).Position = particle(i).Position + particle(i).Velocity;

            % Apply Lower and Upper Bound Limits
            particle(i).Position = max(particle(i).Position, VarMin);
            particle(i).Position = min(particle(i).Position, VarMax);

            % Evaluation
            particle(i).Cost = CostFunction(particle(i).Position);

            % Update Personal Best
            if particle(i).Cost < particle(i).Best.Cost
                particle(i).Best.Position = particle(i).Position;
                particle(i).Best.Cost = particle(i).Cost;
                if particle(i).Best.Cost < GlobalBest.Cost
                    GlobalBest = particle(i).Best;
                end
            end
        end

        % Update Best Cost
        BestCosts(it) = GlobalBest.Cost;
        cg_curve(it) = GlobalBest.Cost;

        % Display Iteration Information
        % disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);

        % Damping Inertia Coefficient
        w = w * wdamp;
    end

    gBestScore = GlobalBest.Cost;
    gBest = GlobalBest.Position;
end
