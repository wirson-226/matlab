%% Optical Microscope Algorithm (OMA)
%  Author and programmer:
%         Professor        Min-Yuan Cheng
%         Ph.D. Student    Moh Nur Sholeh
%  Written by Moh Nur Sholeh
%  Computer Integrated Construction (CIC) Lab
%  National Taiwan University of Science and Technology, Taipei, Taiwan
%  Paper : Cheng, M. Y., & Sholeh, M. N. (2023). Optical microscope algorithm: a new metaheuristic inspired by microscope magnification
%          for solving engineering optimization problems. Knowledge-Based Systems.
%  DOI   : https://doi.org/10.1016/j.knosys.2023.110939 
%---------------------------------------------------------------------------------------------------------------------------%

function [bestMagnification,bestSolution, bestMagnificationHistory] = OMA(NP,NI, Lb, Ub, dim,ZoomFunction)

% The OMA function takes the following input parameters:
% - ZoomFunction : A function handle representing the objective function to be optimized
% - funnum       : An identifier for the objective function
% - Lb           : The lower bounds of the decision variables
% - Ub           : The upper bounds of the decision variables
% - dim          : The number of decision variables
% - para         : A vector containing two parameters: NI (number of iterations) and NP (population size)

% The output of the OMA function is as follows:
% - bestSolution             : The best solution found by the algorithm
% - bestMagnification        : The corresponding magnification value of the best solution
% - numEval                  : The total number of function evaluations performed during the algorithm
% - bestMagnificationHistory : A history of the best magnification values found at each iteration
    
    % Problem Definition
    nVar = dim;                 % Number of decision variables
    varSize = [1, nVar];        % Decision variable matrix size   
    if numel(Lb) == 1
        varLow = Lb * ones(1, dim);   % Lower bound of variables
        varUp = Ub * ones(1, dim);    % Upper bound of variables
    else
        varLow = Lb;
        varUp = Ub;
    end

   
    % Naked Eyes: Initialization of population (target objects)
    population = initializePopulation(NP, dim, varUp, varLow);

    % Evaluate population (target objects)
    objectiveMagnifications = zeros(1, NP);
    for i = 1:NP
        objectiveMagnifications(i) = ZoomFunction(population(i,:));
    end
    
    % OMA Main Loop
    bestSolution = [];
    bestMagnification = Inf;
    bestMagnificationHistory = [];    
    for iter = 1:NI
        [sortedMagnifications, sortedIndex] = sort(objectiveMagnifications);
        bestSolution = population(sortedIndex(1), :);
        bestMagnification = sortedMagnifications(1);
        
        % Objective Lens Phase
        for i = 1:NP
            % Modify target object
            newSolution = population(i, :) + rand(1, nVar) .* (1.40 * bestSolution);
            % Check the boundary
            newSolution = checkBounds(newSolution, varLow, varUp);
            % Evaluation of eyepiece
            newMagnification = ZoomFunction(newSolution);
            % Magnification result comparison
            if newMagnification < objectiveMagnifications(i)
                population(i, :) = newSolution;
                objectiveMagnifications(i) = newMagnification;
                if objectiveMagnifications(i) < bestMagnification
                    bestMagnification = objectiveMagnifications(i);
                    bestSolution = population(i, :);
                end
            end
        end
        
        % Eyepiece Phase
        for i = 1:NP
            j = i;
            while j == i
                j = floor(rand * NP) + 1;
            end

            % Calculate local search space
            space = population(i, :).*rand() - population(j, :);
            if objectiveMagnifications(j) < objectiveMagnifications(i)
                space = -space;
            end

            % Modify target object
            newSolution = population(i, :) + rand(1, nVar) .* (0.55 * space);
            % Check the boundary
            newSolution = checkBounds(newSolution, varLow, varUp);
            % Evaluation of objective lens
            newMagnification = ZoomFunction(newSolution);
            % Magnification result comparison
            if newMagnification < objectiveMagnifications(i)
                population(i, :) = newSolution;
                objectiveMagnifications(i) = newMagnification;
                if objectiveMagnifications(i) < bestMagnification
                    bestMagnification = objectiveMagnifications(i);
                    bestSolution = population(i, :);
                end
            end
        end
        
        % Save current iteration
        bestMagnificationHistory(iter) = bestMagnification;
        if iter >= 2000 && abs(bestMagnificationHistory(iter) - bestMagnificationHistory(iter - 100)) < 1e-350
            break;
        end
    end
    
    numEval = iter * NP;
end

function population = initializePopulation(NP, dim, Ub, Lb)
    if numel(Lb) == 1
        Lb = Lb * ones(1, dim);
        Ub = Ub * ones(1, dim);
    end
    x = zeros(NP, dim);
    x(1, :) = rand(1, dim);
    for i = 1:(NP-1)
        x(i+1, :) = x(i, :) .* (1 - x(i, :));
    end
    population = zeros(NP, dim);
    for k = 1:dim
        for i = 1:NP
            population(i, k) = Lb(k) + x(i, k) * (Ub(k) - Lb(k));
        end
    end
end

function s = checkBounds(s, Lb, Ub)
    % Apply to lower bound
    s(s < Lb) = Ub(s < Lb) + (s(s < Lb) - Lb(s < Lb));
    % Apply to upper bound
    s(s > Ub) = Lb(s > Ub) + (s(s > Ub) - Ub(s > Ub));
    % Check results
    s = s;
end