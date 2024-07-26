N = 500;
initialGuess = zeros(1, N);
if isempty(initialCurrentGuessDP)   
    load("initialCurrentGuessDP.mat");
end
if length(initialCurrentGuessDP) > N
    initialGuess = initialCurrentGuessDP(1:N); %78/50*ones(1,N); %[1 0 Tf Tf 1]';
else
    initialGuess(1:length(initialCurrentGuessDP)) = initialCurrentGuessDP;
end
initialGuess = 30.3413*ones(1,N);
initialGuess(1:7) = 40;
% initialGuess = zeros(1,500);
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
% clear customInitialPopulation;
% customInitialPopulation(:,1) = 40*rand(100,1);
% for i = 2:500
%     customInitialPopulation(:,i) = customInitialPopulation (:,k-1) + 2*(rand(100,1) - 0.5*ones(100,1));
%     customInitialPopulation(:,i) = min(max(customInitialPopulation(:,i), 0), 40);
% end

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
% x0Vec = zeros(5,3);
% x0Vec(:,1) = [1 0 20 20 1]';
x0 = [1 0 20 20 1]';
% x0 = [0.939924640506796 0.0089513790517327 33.5428728458853 39.9882734799299 0.939329768253312]; % after first 500 steps
DPSolution = [];%zeros(1,N*1);
lastI = [];
for i = 1:1
    options = optimoptions('fmincon','Display','iter', 'MaxFunctionEvaluations', 100*N, 'MaxIterations', 1e4,'UseParallel',true);
    optVars = fmincon(@(I) DPCostFunction(I,x0, lastI), initialGuess, A, b, Aeq, beq, lb, ub, @(I) DPConstraints(I, x0, lastI), options);
    DPSolution = [DPSolution optVars];%(N*(i-1)+1:N*i) = optVars;
    for k = 1:N
        x0 = DPBatteryDynamics(x0, optVars(k), 1);
    end
    lastI = optVars(end-49:end);
    % initialGuess=optVars;
end

% Set up GA options
% options = optimoptions('ga', ...
%     'Display', 'iter', ...
%     'MaxGenerations', 200, ...
%     'MaxStallGenerations', 20, ...
%     'PopulationSize', populationSize, ...
%     'EliteCount', 10, ...
%     'CrossoverFraction', 0.8, ...
%     'FunctionTolerance', 1e-8, ...
%     'UseParallel', true, ...
%     'InitialPopulationMatrix', customInitialPopulation);  % Set custom initial population
% 
% % Run the genetic algorithm
% optVars = ga(@DPCostFunction, N, A, b, Aeq, beq, lb, ub, @DPConstraints, options);

% % For the state space and SoE development
% x0 = [1 0 20 20 1]';
% x = x0;
% for k = 2:length(DPSolution)
%     x(:,k) = DPBatteryDynamics(x(:,k-1), DPSolution(k), 1);
%     % temp(k) = (x(1,k).^(4:-1:0))*soc_ocv_coefficients - initialCurrentGuessDP(k)*R0 - x(2,k);
% end
% 
% figure(2);clf;titleStrings = {'SoC', 'V1', 'Ts', 'Tc', 'SoE'};
% for i = 1:5
%     subplot(5,1,i)
%     plot(x(i,:))
%     title(titleStrings{i})
% end

%%
figure(1);clf;hold on
plot(repmat(initialGuess,1,3),'--k','linewidth',1)
plot(DPSolution, 'r','LineWidth',1)
legend('Initial guess', 'Solver output')