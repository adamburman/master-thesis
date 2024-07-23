N = 500;
if isempty(initialCurrentGuessDP)   
    load("initialCurrentGuessDP.mat");
end
initialGuess = initialCurrentGuessDP; %78/50*ones(1,N); %[1 0 Tf Tf 1]';
A = [];
b = [];
Aeq = [];
beq = [];
lb = zeros(1, N);
ub = 40*ones(1, N);
nonLinCon = @DPConstraints;
populationSize = 100;
% Construct A matrix for rate of change constraints
A = zeros(2*(N-1), N);
b = ones(2*(N-1), 1);

% Create a custom initial population matrix
customInitialPopulation = repmat(initialGuess,populationSize,1) + rand(populationSize, N) - rand(populationSize, N);

% Ensure bounds are respected
customInitialPopulation = min(max(customInitialPopulation, 0), 40);

% % Populate A for constraints: I(k+1) - I(k) <= 1
% for k = 1:(N-1)
%     % For I(k+1) - I(k) <= 1
%     A(k, k) = -1;
%     A(k, k+1) = 1;
% 
%     % For I(k) - I(k+1) <= 1
%     A(N-1+k, k) = 1;
%     A(N-1+k, k+1) = -1;
% end

% options = optimoptions('fmincon','Display','iter', 'MaxFunctionEvaluations', 1e5, 'MaxIterations', 1e4);
% optVars = fmincon(@DPCostFunction, initialGuess, A, b, Aeq, beq, lb, ub, nonLinCon);

% Set up GA options
options = optimoptions('ga', ...
    'Display', 'iter', ...
    'MaxGenerations', 5, ...
    'MaxStallGenerations', 50, ...
    'PopulationSize', populationSize, ...
    'EliteCount', 10, ...
    'CrossoverFraction', 0.8, ...
    'FunctionTolerance', 1e-8, ...
    'UseParallel', true, ...
    'InitialPopulationMatrix', customInitialPopulation);  % Set custom initial population

% Run the genetic algorithm
optVars = ga(@DPCostFunction, N, A, b, Aeq, beq, lb, ub, @DPConstraints, options);

%  % For the state space and SoE development
% x0 = [1 1e-3 20 20 1]';
% x = x0;
% for k = 2:3600
%     x(:,k) = DPBatteryDynamics(x(:,k-1), initialGuess(k), 1);
% end
% 
% figure(1);clf;titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
% for i = 1:5
%     subplot(5,1,i)
%     plot(x(i,:))
%     title(titleStrings{i})
% end